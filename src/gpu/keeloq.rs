//! GPU filter for the KeeLoq magic-serial search (cargo feature `gpu`).
//!
//! [`filter`] runs the bitsliced `keeloq.wgsl` kernel over a candidate range and returns the
//! manufacturer keys that pass every frame's button/discriminator check — a superset of what the
//! CPU search accepts. [`crate::protocols::keeloq_bf::search_range`] then runs the authoritative
//! `verify` on the survivors, so the final candidates and their order are identical to the CPU-only
//! search. Returns `None` (CPU fallback) when no GPU is present, the range is too small to be worth
//! it, or the survivor buffer overflows.

use std::sync::atomic::Ordering;

use wgpu::util::DeviceExt;

use crate::protocols::keeloq_bf::{device_key, layout, Control, Frame};

/// Below this many candidates the CPU is used (setup cost, and it keeps the unit tests GPU-free).
const MIN_RANGE: u64 = 1 << 24;
/// Survivor slots in the output buffer; must match `CAP` in `keeloq.wgsl`.
const CAP: u32 = 4096;
/// Workgroups per dispatch (×64 threads ×32 lanes = candidates per dispatch). Kept well under the
/// 65535-per-dimension floor Vulkan guarantees.
const WORKGROUPS: u32 = 32768;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Params {
    shift: u32,
    width: u32,
    n_frames: u32,
    base_lo: u32,
    base_hi: u32,
    threads: u32,
    _pad0: u32,
    _pad1: u32,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct FrameGpu {
    fix: u32,
    hop: u32,
    known_lo: u32,
    known_hi: u32,
}

/// Candidate manufacturer keys (`man`, i.e. candidate `<< shift`) the GPU keeps for `range`, or
/// `None` to fall back to the CPU. `ctl.cancel` stops it; `ctl.tested` tracks progress.
pub fn filter(frames: &[Frame], learning: u32, range: std::ops::Range<u64>, ctl: &Control) -> Option<Vec<u64>> {
    let (shift, width) = layout(learning);
    let end = range.end.min(1 << width);
    if range.start >= end || end - range.start < MIN_RANGE {
        return None;
    }
    let ctx = super::context()?;

    let gpu_frames: Vec<FrameGpu> = frames
        .iter()
        .map(|f| {
            let known = device_key(learning, f.fix, 0);
            FrameGpu { fix: f.fix, hop: f.hop, known_lo: known as u32, known_hi: (known >> 32) as u32 }
        })
        .collect();

    match pollster::block_on(run(ctx, &gpu_frames, shift, width, range.start, end, ctl)) {
        Ok(cands) => {
            // Map surviving candidate c to man = c << shift, keep those inside [start, end).
            let mut out = Vec::with_capacity(cands.len());
            for c in cands {
                if c >= range.start && c < end {
                    out.push(c << shift);
                }
            }
            Some(out)
        }
        Err(_) => None, // overflow or device error: let the caller use the CPU
    }
}

async fn run(
    ctx: &super::Context,
    frames: &[FrameGpu],
    shift: u32,
    width: u32,
    start: u64,
    end: u64,
    ctl: &Control,
) -> Result<Vec<u64>, ()> {
    let device = &ctx.device;
    let queue = &ctx.queue;

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("keeloq-bf"),
        source: wgpu::ShaderSource::Wgsl(include_str!("keeloq.wgsl").into()),
    });

    let frames_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("frames"),
        contents: bytemuck::cast_slice(frames),
        usage: wgpu::BufferUsages::STORAGE,
    });
    let out_len_u32 = 1 + (CAP as u64) * 2;
    let out_size = out_len_u32 * 4;
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

    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: None,
        entries: &[
            uniform_entry(0),
            storage_entry(1, true),
            storage_entry(2, false),
        ],
    });
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &bgl,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: params_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: frames_buf.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 2, resource: out.as_entire_binding() },
        ],
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: None,
        bind_group_layouts: &[&bgl],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
        label: Some("keeloq-bf"),
        layout: Some(&layout),
        module: &shader,
        entry_point: "main",
        compilation_options: Default::default(),
    });

    queue.write_buffer(&out, 0, &vec![0u8; out_size as usize]);
    device.poll(wgpu::Maintain::Wait);
    if ctx.has_failed() {
        return Err(());
    }

    // The kernel iterates gt = candidate >> 5 (32 candidates per thread). Cover every gt block that
    // overlaps [start, end); the host drops survivors outside the range.
    let start_gt = start >> 5;
    let end_gt = (end + 31) >> 5;
    let threads_per: u64 = WORKGROUPS as u64 * 64;
    let mut done: u64 = 0;
    let total_gt = end_gt - start_gt;

    while done < total_gt {
        if ctl.cancel.load(Ordering::Relaxed) {
            break;
        }
        if ctx.has_failed() {
            return Err(());
        }
        let threads = threads_per.min(total_gt - done);
        let base = start_gt + done;
        let params = Params {
            shift,
            width,
            n_frames: frames.len() as u32,
            base_lo: base as u32,
            base_hi: (base >> 32) as u32,
            threads: threads as u32,
            _pad0: 0,
            _pad1: 0,
        };
        queue.write_buffer(&params_buf, 0, bytemuck::bytes_of(&params));
        let wg = threads.div_ceil(64) as u32;
        let mut enc = device.create_command_encoder(&Default::default());
        {
            let mut pass = enc.begin_compute_pass(&Default::default());
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &bind, &[]);
            pass.dispatch_workgroups(wg, 1, 1);
        }
        queue.submit(Some(enc.finish()));
        device.poll(wgpu::Maintain::Wait);
        if ctx.has_failed() {
            return Err(());
        }

        done += threads;
        // Progress: candidates = gt threads × 32 lanes, clamped to the real range.
        ctl.tested.fetch_add((threads * 32).min(end - start), Ordering::Relaxed);
    }

    // Read the survivor buffer once at the end (the atomic count accumulated across dispatches).
    let mut enc = device.create_command_encoder(&Default::default());
    enc.copy_buffer_to_buffer(&out, 0, &staging, 0, out_size);
    queue.submit(Some(enc.finish()));
    let slice = staging.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |r| {
        let _ = tx.send(r);
    });
    device.poll(wgpu::Maintain::Wait);
    if ctx.has_failed() {
        return Err(());
    }
    rx.recv().map_err(|_| ())?.map_err(|_| ())?;
    let data = slice.get_mapped_range();
    let words: &[u32] = bytemuck::cast_slice(&data);
    let count = words[0];
    if count > CAP {
        drop(data);
        staging.unmap();
        return Err(()); // more survivors than the buffer holds: fall back to the CPU
    }
    let mut cands = Vec::with_capacity(count as usize);
    for i in 0..count as usize {
        let c_lo = words[1 + i * 2] as u64;
        let c_hi = words[1 + i * 2 + 1] as u64;
        cands.push((c_hi << 32) | c_lo);
    }
    drop(data);
    staging.unmap();
    Ok(cands)
}

fn uniform_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Storage { read_only },
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}
