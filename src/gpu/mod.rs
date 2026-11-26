//! Optional GPU compute backend for key recovery (cargo feature `gpu`).
//!
//! KAT's key searches ([`crate::protocols::keeloq_bf`], later the HiTag2 Hell attack) are
//! bitsliced boolean kernels over a candidate range — the shape a GPU runs well. This module hosts
//! the device and the WGSL kernels; a search asks [`keeloq::filter`] for the candidates a GPU keeps,
//! then finishes on the CPU exactly as the CPU-only path does, so the result is identical.
//!
//! The backend is [`wgpu`]: one WGSL source runs on Vulkan (Linux/Windows), Metal (macOS) and DX12
//! (Windows), compiled in-process by `naga` with no external shader toolchain. It is optional and
//! off by default; the CPU search is always built and is the oracle the GPU is tested against
//! ([`keeloq`] tests). Without a usable adapter [`context`] is `None` and callers fall back to the
//! CPU.
//!
//! A GPU kernel must bitslice like the CPU (32 candidates per thread, lane = bit): a scalar
//! one-key-per-thread port is slower than KAT's AVX2 search. See `keeloq.wgsl`.

pub mod hitag2;
pub mod keeloq;

use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, OnceLock,
};

/// A ready GPU device and queue, plus the adapter's name for display.
pub struct Context {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub name: String,
    pub backend: wgpu::Backend,
    failed: Arc<AtomicBool>,
}

impl Context {
    /// True once wgpu reports an uncaptured validation/OOM error or device loss.
    pub fn has_failed(&self) -> bool {
        self.failed.load(Ordering::SeqCst)
    }
}

static CONTEXT: OnceLock<Option<Context>> = OnceLock::new();

/// The shared GPU context, initialised once. `None` when no adapter could be opened (no GPU, no
/// driver, a headless CI machine) or after a device error: callers then use the CPU path.
pub fn context() -> Option<&'static Context> {
    CONTEXT.get_or_init(|| pollster::block_on(init())).as_ref().filter(|c| !c.has_failed())
}

async fn init() -> Option<Context> {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::PRIMARY, // Vulkan, Metal, DX12 — the platforms KAT ships
        ..Default::default()
    });
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            ..Default::default()
        })
        .await?;
    let info = adapter.get_info();
    // A software rasteriser (llvmpipe/lavapipe, WARP) is slower than the CPU search and only shows
    // up as a GPU by accident; decline it so the CPU path is used instead.
    if info.device_type == wgpu::DeviceType::Cpu {
        return None;
    }
    let (device, queue) = adapter
        .request_device(
            &wgpu::DeviceDescriptor { label: Some("kat-recovery"), ..Default::default() },
            None,
        )
        .await
        .ok()?;
    let failed = Arc::new(AtomicBool::new(false));
    let err_flag = failed.clone();
    device.on_uncaptured_error(Box::new(move |err| {
        tracing::warn!("GPU error (falling back to CPU): {err}");
        err_flag.store(true, Ordering::SeqCst);
    }));
    let lost_flag = failed.clone();
    device.set_device_lost_callback(move |reason, msg| {
        tracing::warn!("GPU device lost ({reason:?}: {msg}); falling back to CPU");
        lost_flag.store(true, Ordering::SeqCst);
    });
    Some(Context { device, queue, name: info.name, backend: info.backend, failed })
}

/// Whether a usable GPU is present. Used by the GPU equivalence tests and the TUI recovery view.
#[allow(dead_code)]
pub fn available() -> bool {
    context().is_some()
}

/// Whether the GPU is up already, without starting it: for the UI thread, which must not wait
/// for a driver (a search brings the GPU up on its own thread, the first time one runs).
pub fn is_up() -> bool {
    CONTEXT.get().is_some_and(|c| c.as_ref().is_some_and(|ctx| !ctx.has_failed()))
}

/// A short description of the GPU KAT would use, e.g. for a startup line or `--help`.
pub fn description() -> Option<String> {
    context().map(|c| format!("{} ({:?})", c.name, c.backend))
}
