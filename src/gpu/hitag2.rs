//! GPU driver for the HiTag2 Hell search (cargo feature `gpu`).
//!
//! [`lowest_hit`] runs `hitag2.wgsl` over the layer-0 candidates in increasing order, in chunks
//! small enough to stay under the GPU's watchdog, and returns the lowest candidate index that
//! yields a state verifying any request. [`super::super::protocols::hitag2_hell`] then finalises
//! that one candidate on the CPU (its exact `states` + `verify_state`), so the recovered key equals
//! the CPU-only search. Honours `cancel` and advances `progress` like the CPU path.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use wgpu::util::DeviceExt;

/// Candidates per dispatch: 128 × 1024 i1 / 64 = 2048 workgroups, ~0.1 s on a modest GPU — well
/// under the ~10 s watchdog, and fine-grained enough to react to a cancel.
const CHUNK: u32 = 128;
/// Survivor slots; a verified candidate is rare (false rate ≈ 2^-15 per extra frame), so this is
/// ample. Overflow would only drop hits within one chunk, never below the true lowest.
const CAP: u32 = 4096;

/// What the GPU search found.
pub enum GpuSearch {
    /// The lowest layer-0 candidate index with a verifying state.
    Hit(u32),
    NoHit,
    Cancelled,
    /// No usable GPU (or a device error): the caller runs the CPU search.
    Unavailable,
}

/// One request for the kernel (matches `Req` in hitag2.wgsl).
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GpuReq {
    pub uid: u32,
    pub cnt: u32,
    pub btn: u32,
    pub has_second: u32,
    pub s_hop: u32,
    pub s_cnt: u32,
    pub s_btn: u32,
    pub _pad: u32,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Params {
    n_cands: u32,
    hop: u32,
    poly_lo: u32,
    poly_hi: u32,
    n_reqs: u32,
    base_idx: u32,
    _p0: u32,
    _p1: u32,
}

/// The lowest candidate index in `cands` (searched over `range`) with a state verifying any of
/// `reqs`. `cancel` stops it between chunks; `progress` counts candidates scanned.
pub fn lowest_hit(
    hop: u32,
    reqs: &[GpuReq],
    cands: &[u64],
    range: std::ops::Range<u32>,
    cancel: &AtomicBool,
    progress: &AtomicU32,
) -> GpuSearch {
    let Some(ctx) = super::context() else { return GpuSearch::Unavailable };
    if reqs.is_empty() || range.start >= range.end {
        return GpuSearch::NoHit;
    }
    match pollster::block_on(run(ctx, hop, reqs, cands, range, cancel, progress)) {
        Ok(Some(idx)) => GpuSearch::Hit(idx),
        Ok(None) if cancel.load(Ordering::Relaxed) => GpuSearch::Cancelled,
        Ok(None) => GpuSearch::NoHit,
        Err(()) => GpuSearch::Unavailable,
    }
}

async fn run(
    ctx: &super::Context,
    hop: u32,
    reqs: &[GpuReq],
    cands: &[u64],
    range: std::ops::Range<u32>,
    cancel: &AtomicBool,
    progress: &AtomicU32,
) -> Result<Option<u32>, ()> {
    let device = &ctx.device;
    let queue = &ctx.queue;
    let poly = (0xCE00_44C1_01CDu64 >> 1) | (1 << 47);

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("hitag2-hell"),
        source: wgpu::ShaderSource::Wgsl(include_str!("hitag2.wgsl").into()),
    });
    let reqs_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("reqs"),
        contents: bytemuck::cast_slice(reqs),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let out_size = (1 + CAP as u64) * 4;
    let out = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("out"),
        size: out_size,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let staging = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("staging"),
        size: out_size,
        usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let params_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("params"),
        size: std::mem::size_of::<Params>() as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    let end = range.end.min(cands.len() as u32);
    let mut base = range.start;
    while base < end {
        if cancel.load(Ordering::Relaxed) {
            return Ok(None);
        }
        let n = CHUNK.min(end - base);
        // this chunk's candidates as (lo, hi) pairs
        let mut cbuf: Vec<u32> = Vec::with_capacity(n as usize * 2);
        for &c in &cands[base as usize..(base + n) as usize] {
            cbuf.push(c as u32);
            cbuf.push((c >> 32) as u32);
        }
        let cands_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("cands"),
            contents: bytemuck::cast_slice(&cbuf),
            usage: wgpu::BufferUsages::STORAGE,
        });
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[uniform(0), storage(1, true), storage(2, true), storage(3, false)],
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: params_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: cands_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: reqs_buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 3, resource: out.as_entire_binding() },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&bgl],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: None,
            layout: Some(&layout),
            module: &shader,
            entry_point: "main",
            compilation_options: Default::default(),
        });

        let params = Params { n_cands: n, hop, poly_lo: poly as u32, poly_hi: (poly >> 32) as u32, n_reqs: reqs.len() as u32, base_idx: base, _p0: 0, _p1: 0 };
        queue.write_buffer(&params_buf, 0, bytemuck::bytes_of(&params));
        queue.write_buffer(&out, 0, &[0u8; 4]); // reset count
        device.poll(wgpu::Maintain::Wait);

        let threads = n * 1024;
        let wg = (threads + 63) / 64;
        let mut enc = device.create_command_encoder(&Default::default());
        {
            let mut pass = enc.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(wg, 1, 1);
        }
        enc.copy_buffer_to_buffer(&out, 0, &staging, 0, out_size);
        queue.submit(Some(enc.finish()));

        let slice = staging.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        device.poll(wgpu::Maintain::Wait);
        rx.recv().map_err(|_| ())?.map_err(|_| ())?;
        let data = slice.get_mapped_range();
        let words: &[u32] = bytemuck::cast_slice(&data);
        let count = words[0].min(CAP) as usize;
        let lowest = words[1..1 + count].iter().copied().min();
        drop(data);
        staging.unmap();

        progress.fetch_add(n, Ordering::Relaxed);
        if let Some(idx) = lowest {
            return Ok(Some(idx));
        }
        base += n;
    }
    Ok(None)
}

fn uniform(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
        count: None,
    }
}
fn storage(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only }, has_dynamic_offset: false, min_binding_size: None },
        count: None,
    }
}
