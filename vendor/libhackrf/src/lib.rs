use std::{
    any::Any,
    ffi::c_void,
    ptr, slice,
    sync::{Arc, Mutex},
};

mod enums;
pub mod error;
pub mod ffi;
pub use enums::DeviceType;
mod transfer;
pub mod util;

use error::{HackrfError, Result};
use ffi::SerialNumber;
use transfer::{rx_callback, tx_callback, ReceiveCallback, TransferContext, TransmitCallback};

pub mod exports {
    pub use num_complex;
}

/// Open devices (and device lists being read). libhackrf is initialised while there are any:
/// `hackrf_init` with the first, `hackrf_exit` after the last. Opening and closing happen under
/// this lock, so two threads opening HackRFs at once (two receivers starting) cannot race the
/// library's set-up; a failed open no longer leaves the count up (it used to, so the library was
/// never shut down again).
static LIBRARY_USERS: Mutex<usize> = Mutex::new(0);

fn library_users() -> std::sync::MutexGuard<'static, usize> {
    LIBRARY_USERS.lock().unwrap_or_else(|e| e.into_inner())
}

/// Run `f` with libhackrf initialised; it stays initialised if `f` returns `keep` = true (a device
/// was opened, counted as a user until it is dropped).
fn with_library<T>(f: impl FnOnce() -> Result<(T, bool)>) -> Result<T> {
    let mut users = library_users();
    if *users == 0 {
        unsafe { HackrfError::from_id(ffi::hackrf_init())? }
    }
    let result = f();
    match &result {
        Ok((_, true)) => *users += 1,
        _ if *users == 0 => {
            let _ = unsafe { ffi::hackrf_exit() };
        }
        _ => {}
    }
    result.map(|(value, _)| value)
}

/// A HackRF on USB, as [HackRf::list] finds it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ListedDevice {
    /// Its serial number (32 hex digits); None when it could not be read (libusb may not open the
    /// device: missing USB permissions, or another program has it on Windows).
    pub serial: Option<String>,
}

/// A HackRf device.
#[derive(Clone)]
pub struct HackRf {
    // Dropped only when all user clones leave scope, stopping any forgotten stream before closing.
    // Callback handles hold `None` here so dropping a callback clone cannot self-join libhackrf.
    owner: Option<Arc<HackRfOwner>>,
    inner: Arc<HackRfInner>,
}

struct HackRfOwner {
    inner: Arc<HackRfInner>,
}

struct ActiveTransfer {
    ctx: *mut c_void,
    drop_ctx: unsafe fn(*mut c_void),
    is_tx: bool,
}

#[cfg(test)]
type MockStartFn = Arc<dyn Fn(*mut ffi::HackrfDevice, *mut c_void) -> std::ffi::c_int + Send + Sync>;
#[cfg(test)]
type MockStopFn = Arc<dyn Fn(*mut ffi::HackrfDevice) -> std::ffi::c_int + Send + Sync>;

#[cfg(test)]
#[derive(Clone, Default)]
pub(crate) struct MockHooks {
    pub(crate) users: Arc<Mutex<usize>>,
    pub(crate) events: Arc<Mutex<Vec<&'static str>>>,
    pub(crate) start_rx: Option<MockStartFn>,
    pub(crate) stop_rx: Option<MockStopFn>,
    pub(crate) start_tx: Option<MockStartFn>,
    pub(crate) stop_tx: Option<MockStopFn>,
}

struct HackRfInner {
    device: *mut ffi::HackrfDevice,
    stream_op: Mutex<()>,
    active: Mutex<Option<ActiveTransfer>>,
    #[cfg(test)]
    hooks: Option<MockHooks>,
}

impl HackRf {
    /// Connects to a HackRF device (the first one libhackrf finds).
    pub fn open() -> Result<HackRf> {
        Self::open_with(|device| unsafe { ffi::hackrf_open(device) })
    }

    /// Connects to the HackRF whose serial number ends with `serial` (all 32 digits, or the last
    /// ones, as `hackrf_transfer -d` takes it).
    pub fn open_by_serial(serial: &str) -> Result<HackRf> {
        let wanted = std::ffi::CString::new(serial).map_err(|_| HackrfError::InvalidParam)?;
        Self::open_with(|device| unsafe { ffi::hackrf_open_by_serial(wanted.as_ptr(), device) })
    }

    fn open_with(open: impl FnOnce(*mut *mut ffi::HackrfDevice) -> std::ffi::c_int) -> Result<HackRf> {
        with_library(|| {
            let mut device = std::ptr::null_mut();
            HackrfError::from_id(open(&mut device))?;
            if device.is_null() {
                return Err(HackrfError::NotFound);
            }
            let inner = Arc::new(HackRfInner {
                device,
                stream_op: Mutex::new(()),
                active: Mutex::new(None),
                #[cfg(test)]
                hooks: None,
            });
            let hackrf = Self {
                owner: Some(Arc::new(HackRfOwner {
                    inner: Arc::clone(&inner),
                })),
                inner,
            };
            Ok((hackrf, true))
        })
    }

    pub(crate) fn callback_handle(&self) -> Self {
        Self {
            owner: None,
            inner: Arc::clone(&self.inner),
        }
    }

    #[cfg(test)]
    pub(crate) fn mock_for_test() -> (Self, MockHooks) {
        Self::mock_with_hooks(MockHooks::default())
    }

    #[cfg(test)]
    pub(crate) fn mock_with_hooks(hooks: MockHooks) -> (Self, MockHooks) {
        *hooks.users.lock().unwrap_or_else(|e| e.into_inner()) += 1;
        let inner = Arc::new(HackRfInner {
            device: ptr::null_mut(),
            stream_op: Mutex::new(()),
            active: Mutex::new(None),
            hooks: Some(hooks.clone()),
        });
        (
            Self {
                owner: Some(Arc::new(HackRfOwner {
                    inner: Arc::clone(&inner),
                })),
                inner,
            },
            hooks,
        )
    }

    /// The HackRFs on USB, in libusb's order.
    pub fn list() -> Result<Vec<ListedDevice>> {
        with_library(|| {
            let list = unsafe { ffi::hackrf_device_list() };
            if list.is_null() {
                return Err(HackrfError::NoMem);
            }
            let devices = unsafe {
                let l = &*list;
                let count = usize::try_from(l.devicecount).unwrap_or(0);
                (0..count)
                    .map(|i| {
                        let serial = if l.serial_numbers.is_null() { ptr::null_mut() } else { *l.serial_numbers.add(i) };
                        ListedDevice {
                            serial: (!serial.is_null()).then(|| {
                                std::ffi::CStr::from_ptr(serial).to_string_lossy().trim().to_string()
                            }),
                        }
                    })
                    .collect()
            };
            unsafe { ffi::hackrf_device_list_free(list) };
            Ok((devices, false))
        })
    }

    /// Gets the internial representation of the HackRF device. This can be used
    /// with unsafe FFI functions if needed.
    #[inline(always)]
    pub fn device(&self) -> *mut ffi::HackrfDevice {
        self.inner.device
    }

    /// Gets the device serial number.
    pub fn get_serial_number(&self) -> Result<SerialNumber> {
        let mut serial_number = SerialNumber::default();
        unsafe {
            HackrfError::from_id(ffi::hackrf_board_partid_serialno_read(
                self.device(),
                &mut serial_number,
            ))?
        }
        Ok(serial_number)
    }

    /// Read hackrf_board_id from a device and convert it to a DeviceType.
    pub fn get_device_type(&self) -> Result<DeviceType> {
        let mut value = 0;
        unsafe { HackrfError::from_id(ffi::hackrf_board_id_read(self.device(), &mut value)) }?;
        Ok(DeviceType::from_id(value))
    }

    /// Read HackRF firmware version as a string.
    pub fn version(&self) -> String {
        let mut version = vec![0; 32];

        unsafe {
            ffi::hackrf_version_string_read(
                self.device(),
                version.as_mut_ptr(),
                version.len() as u8,
            );
        }

        let end = version
            .iter()
            .position(|&x| x == 0)
            .unwrap_or(version.len());

        // Use from_raw_parts to handle both c_char=i8 (x86) and c_char=u8 (ARM)
        let version = unsafe { slice::from_raw_parts(version.as_ptr() as *const u8, end) };
        String::from_utf8_lossy(version).into_owned()
    }

    /// Sets the center frequency in Hz.
    pub fn set_freq(&self, freq: u64) -> Result<()> {
        unsafe { HackrfError::from_id(ffi::hackrf_set_freq(self.device(), freq)) }
    }

    /// Sets the sample rate in Hz.
    pub fn set_sample_rate(&self, sample_rate: u32) -> Result<()> {
        unsafe {
            HackrfError::from_id(ffi::hackrf_set_sample_rate_manual(
                self.device(),
                sample_rate,
                1,
            ))
        }
    }

    /// Sets the state of the externial amplifier.
    pub fn set_amp_enable(&self, enable: bool) -> Result<()> {
        unsafe { HackrfError::from_id(ffi::hackrf_set_amp_enable(self.device(), enable as u8)) }
    }

    /// Low noise amplifier gain.
    /// Between 0d and 40d in steps of 8dB.
    pub fn set_lna_gain(&self, gain: u32) -> Result<()> {
        unsafe { HackrfError::from_id(ffi::hackrf_set_lna_gain(self.device(), gain)) }
    }

    /// Variable gain amplifier. Range 0-62 (step 2dB).
    pub fn set_rxvga_gain(&self, gain: u32) -> Result<()> {
        unsafe { HackrfError::from_id(ffi::hackrf_set_vga_gain(self.device(), gain)) }
    }

    /// Transmit variable gain amplifier. Range 0-47 (step 1dB).
    pub fn set_txvga_gain(&self, gain: u32) -> Result<()> {
        unsafe { HackrfError::from_id(ffi::hackrf_set_txvga_gain(self.device(), gain)) }
    }

    pub fn set_baseband_filter_bandwidth(&self, bandwidth_hz: u32) -> Result<()> {
        unsafe {
            HackrfError::from_id(ffi::hackrf_set_baseband_filter_bandwidth(
                self.device(),
                ffi::hackrf_compute_baseband_filter_bw(bandwidth_hz),
            ))
        }
    }

    /// Starts transmitting samples from the device.
    pub fn start_tx(&self, callback: TransmitCallback, user_data: impl Any) -> Result<()> {
        self.start_transfer::<TransmitCallback>(true, callback, Box::new(user_data))
    }

    /// Stops the current transmit operation.
    pub fn stop_tx(&self) -> Result<()> {
        // Stop first: libhackrf cancels the transfers and waits for them, so the callback can no
        // longer run when the context is freed.
        self.stop_transfer(true)
    }

    /// Starts receiving samples from the device.
    pub fn start_rx(&self, callback: ReceiveCallback, user_data: impl Any + Sync) -> Result<()> {
        self.start_transfer::<ReceiveCallback>(false, callback, Box::new(user_data))
    }

    /// Stops the current receive operation.
    pub fn stop_rx(&self) -> Result<()> {
        // Stop first (see `stop_tx`), then free the callback context.
        self.stop_transfer(false)
    }

    fn start_transfer<Callback>(&self, is_tx: bool, callback: Callback, user_data: Box<dyn Any>) -> Result<()>
    where
        Callback: 'static,
    {
        if self.owner.is_none() {
            return Err(HackrfError::InvalidParam);
        }

        let mut failed_ctx = None;
        let result = {
            let _op = self.inner.stream_op.lock().unwrap_or_else(|e| e.into_inner());
            let mut active = self.inner.active.lock().unwrap_or_else(|e| e.into_inner());
            if active.is_some() {
                return Err(HackrfError::Busy);
            }
            let context = TransferContext::new(callback, self.callback_handle(), user_data);
            let ctx_ptr = Box::into_raw(Box::new(context)) as *mut c_void;
            *active = Some(ActiveTransfer {
                ctx: ctx_ptr,
                drop_ctx: drop_transfer_context::<Callback>,
                is_tx,
            });
            drop(active);

            let res = HackrfError::from_id(start_stream_ffi(&self.inner, is_tx, ctx_ptr));
            if res.is_err() {
                let _ = stop_stream_ffi(&self.inner, is_tx);
                failed_ctx = self.take_transfer_context(Some(is_tx));
            }
            res
        };
        if let Some(transfer) = failed_ctx {
            unsafe { (transfer.drop_ctx)(transfer.ctx) };
        }
        result
    }

    fn stop_transfer(&self, is_tx: bool) -> Result<()> {
        let (result, taken) = {
            let _op = self.inner.stream_op.lock().unwrap_or_else(|e| e.into_inner());
            let result = HackrfError::from_id(stop_stream_ffi(&self.inner, is_tx));
            let taken = self.take_transfer_context(Some(is_tx));
            (result, taken)
        };
        if let Some(transfer) = taken {
            unsafe { (transfer.drop_ctx)(transfer.ctx) };
        }
        result
    }

    fn take_transfer_context(&self, expect_is_tx: Option<bool>) -> Option<ActiveTransfer> {
        let mut active = self.inner.active.lock().unwrap_or_else(|e| e.into_inner());
        match (&*active, expect_is_tx) {
            (Some(t), Some(expected)) if t.is_tx != expected => None,
            (Some(_), _) => active.take(),
            (None, _) => None,
        }
    }

    /// Returns true if the device is currently streaming samples (transmitting or receiving).
    /// libhackrf returns HACKRF_TRUE (1) while streaming and a negative error once the stream has
    /// stopped (hackrf_transfer loops `while (hackrf_is_streaming(device) == HACKRF_TRUE)`).
    pub fn is_streaming(&self) -> bool {
        unsafe { ffi::hackrf_is_streaming(self.device()) == 1 }
    }

    /// Ok while streaming; the error libhackrf reports once the stream stopped on its own
    /// (`StreamingStopped` / `StreamingExitCalled` after a USB error, e.g. the device was unplugged).
    pub fn streaming_status(&self) -> Result<()> {
        unsafe { HackrfError::from_id(ffi::hackrf_is_streaming(self.device())) }
    }
}

unsafe fn drop_transfer_context<Callback>(ctx: *mut c_void) {
    if !ctx.is_null() {
        drop(Box::from_raw(ctx as *mut TransferContext<Callback>));
    }
}

fn start_stream_ffi(inner: &HackRfInner, is_tx: bool, ctx: *mut c_void) -> std::ffi::c_int {
    #[cfg(test)]
    if let Some(h) = &inner.hooks {
        h.events
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(if is_tx { "start_tx" } else { "start_rx" });
        let f = if is_tx { &h.start_tx } else { &h.start_rx };
        return f.as_ref().map_or(0, |cb| cb(inner.device, ctx));
    }
    unsafe {
        if is_tx {
            ffi::hackrf_start_tx(inner.device, tx_callback, ctx)
        } else {
            ffi::hackrf_start_rx(inner.device, rx_callback, ctx)
        }
    }
}

fn stop_stream_ffi(inner: &HackRfInner, is_tx: bool) -> std::ffi::c_int {
    #[cfg(test)]
    if let Some(h) = &inner.hooks {
        h.events
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(if is_tx { "stop_tx" } else { "stop_rx" });
        let f = if is_tx { &h.stop_tx } else { &h.stop_rx };
        return f.as_ref().map_or(0, |cb| cb(inner.device));
    }
    unsafe {
        if is_tx {
            ffi::hackrf_stop_tx(inner.device)
        } else {
            ffi::hackrf_stop_rx(inner.device)
        }
    }
}

unsafe impl Send for HackRfInner {}
unsafe impl Sync for HackRfInner {}

impl Drop for HackRfOwner {
    fn drop(&mut self) {
        let taken = {
            let _op = self.inner.stream_op.lock().unwrap_or_else(|e| e.into_inner());
            let taken = self.inner.active.lock().unwrap_or_else(|e| e.into_inner()).take();
            if let Some(ref transfer) = taken {
                let _ = stop_stream_ffi(&self.inner, transfer.is_tx);
            }
            taken
        };
        if let Some(transfer) = taken {
            unsafe { (transfer.drop_ctx)(transfer.ctx) };
        }
    }
}

impl Drop for HackRfInner {
    fn drop(&mut self) {
        #[cfg(test)]
        if let Some(h) = &self.hooks {
            let mut users = h.users.lock().unwrap_or_else(|e| e.into_inner());
            h.events.lock().unwrap_or_else(|e| e.into_inner()).push("close");
            *users = users.saturating_sub(1);
            if *users == 0 {
                h.events.lock().unwrap_or_else(|e| e.into_inner()).push("exit");
            }
            return;
        }
        let mut users = library_users();
        let _ = unsafe { HackrfError::from_id(ffi::hackrf_close(self.device)) };
        *users = users.saturating_sub(1);
        if *users == 0 {
            let _ = unsafe { HackrfError::from_id(ffi::hackrf_exit()) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct DropProbe {
        events: Arc<Mutex<Vec<&'static str>>>,
    }

    impl Drop for DropProbe {
        fn drop(&mut self) {
            self.events.lock().unwrap().push("drop_user");
        }
    }

    #[test]
    fn clones_close_and_exit_only_once_on_last_drop() {
        let (h1, hooks) = HackRf::mock_for_test();
        let h2 = h1.clone();
        assert_eq!(*hooks.users.lock().unwrap(), 1);
        drop(h2);
        assert!(hooks.events.lock().unwrap().is_empty(), "dropping clone must not close device");
        assert_eq!(*hooks.users.lock().unwrap(), 1);
        drop(h1);
        assert_eq!(*hooks.events.lock().unwrap(), ["close", "exit"]);
        assert_eq!(*hooks.users.lock().unwrap(), 0);
    }

    #[test]
    fn start_rx_error_frees_context_and_allows_retry() {
        let hooks = MockHooks {
            start_rx: Some(Arc::new(|_, _| HackrfError::Libusb as i32)),
            ..Default::default()
        };
        let (h, hooks) = HackRf::mock_with_hooks(hooks);
        let cb: ReceiveCallback = |_, _, _| {};
        let err = h.start_rx(cb, DropProbe { events: hooks.events.clone() }).unwrap_err();
        assert!(matches!(err, HackrfError::Libusb));
        assert_eq!(*hooks.events.lock().unwrap(), ["start_rx", "stop_rx", "drop_user"]);
        assert!(h.inner.active.lock().unwrap().is_none());

        drop(h);
        assert_eq!(
            *hooks.events.lock().unwrap(),
            ["start_rx", "stop_rx", "drop_user", "close", "exit"]
        );
    }

    #[test]
    fn stop_frees_only_matching_transfer_and_second_start_refuses() {
        let (h, hooks) = HackRf::mock_for_test();
        let rx_cb: ReceiveCallback = |_, _, _| {};
        let tx_cb: TransmitCallback = |_, _, _| {};

        h.start_rx(rx_cb, DropProbe { events: hooks.events.clone() }).unwrap();
        assert!(matches!(h.start_rx(rx_cb, ()), Err(HackrfError::Busy)));
        assert!(matches!(h.start_tx(tx_cb, ()), Err(HackrfError::Busy)));

        // Calling stop_tx while RX is active must not free RX's TransferContext.
        h.stop_tx().unwrap();
        assert_eq!(*hooks.events.lock().unwrap(), ["start_rx", "stop_tx"]);

        h.stop_rx().unwrap();
        assert_eq!(*hooks.events.lock().unwrap(), ["start_rx", "stop_tx", "stop_rx", "drop_user"]);

        drop(h);
        assert_eq!(
            *hooks.events.lock().unwrap(),
            ["start_rx", "stop_tx", "stop_rx", "drop_user", "close", "exit"]
        );
    }

    #[test]
    fn drop_while_streaming_stops_before_closing_and_refuses_callback_handle_start() {
        let (h, hooks) = HackRf::mock_for_test();
        let cb_handle = h.callback_handle();
        let tx_cb: TransmitCallback = |_, _, _| {};
        assert!(matches!(cb_handle.start_tx(tx_cb, ()), Err(HackrfError::InvalidParam)));

        h.start_tx(tx_cb, DropProbe { events: hooks.events.clone() }).unwrap();
        drop(cb_handle);
        assert_eq!(*hooks.events.lock().unwrap(), ["start_tx"]);

        drop(h);
        assert_eq!(
            *hooks.events.lock().unwrap(),
            ["start_tx", "stop_tx", "drop_user", "close", "exit"]
        );
    }
}
