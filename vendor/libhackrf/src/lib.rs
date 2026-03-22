use std::{
    any::Any,
    ffi::c_void,
    mem, ptr, slice,
    sync::{
        atomic::{AtomicPtr, AtomicUsize, Ordering},
        Arc,
    },
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

static DEVICE_COUNT: AtomicUsize = AtomicUsize::new(0);

/// A HackRf device.
#[derive(Clone)]
pub struct HackRf {
    inner: Arc<HackRfInner>,
}

struct HackRfInner {
    device: *mut ffi::HackrfDevice,
    user_data: AtomicPtr<c_void>,
}

impl HackRf {
    /// Connects to a HackRF device.
    pub fn open() -> Result<HackRf> {
        if DEVICE_COUNT.fetch_add(1, Ordering::Relaxed) == 0 {
            unsafe { HackrfError::from_id(ffi::hackrf_init())? }
        }

        let mut device = std::ptr::null_mut();
        unsafe { HackrfError::from_id(ffi::hackrf_open(&mut device))? }

        Ok(Self {
            inner: Arc::new(HackRfInner {
                device,
                user_data: AtomicPtr::new(ptr::null_mut()),
            }),
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
        let context = TransferContext::new(callback, self.clone(), Box::new(user_data));
        let callback = Box::leak(Box::new(context)) as *mut _ as *mut _;
        self.inner.user_data.store(callback, Ordering::Relaxed);

        unsafe { HackrfError::from_id(ffi::hackrf_start_tx(self.device(), tx_callback, callback)) }
    }

    /// Stops the current transmit operation.
    pub fn stop_tx(&self) -> Result<()> {
        let user_data = &self.inner.user_data;
        let callback = user_data.swap(ptr::null_mut(), Ordering::Relaxed);
        if !callback.is_null() {
            let callback = unsafe { Box::from_raw(callback as *mut fn(*mut ffi::HackrfTransfer)) };
            drop(callback);
        }

        unsafe { HackrfError::from_id(ffi::hackrf_stop_tx(self.device())) }
    }

    /// Starts receiving samples from the device.
    pub fn start_rx(&self, callback: ReceiveCallback, user_data: impl Any + Sync) -> Result<()> {
        let context = TransferContext::new(callback, self.clone(), Box::new(user_data));
        let callback = Box::leak(Box::new(context)) as *mut _ as *mut _;
        self.inner.user_data.store(callback, Ordering::Relaxed);

        unsafe { HackrfError::from_id(ffi::hackrf_start_rx(self.device(), rx_callback, callback)) }
    }

    /// Stops the current receive operation.
    pub fn stop_rx(&self) -> Result<()> {
        let user_data = &self.inner.user_data;
        let callback = user_data.swap(ptr::null_mut(), Ordering::Relaxed);
        if !callback.is_null() {
            let callback = unsafe { Box::from_raw(callback as *mut fn(*mut ffi::HackrfTransfer)) };
            drop(callback);
        }

        unsafe { HackrfError::from_id(ffi::hackrf_stop_rx(self.device())) }
    }

    /// Returns true if the device is currently streaming samples (transmitting or receiving).
    pub fn is_streaming(&self) -> bool {
        unsafe { ffi::hackrf_is_streaming(self.device()) != 0 }
    }
}

unsafe impl Send for HackRfInner {}
unsafe impl Sync for HackRfInner {}

impl Drop for HackRf {
    fn drop(&mut self) {
        let _ = unsafe { HackrfError::from_id(ffi::hackrf_close(self.device())) };

        if DEVICE_COUNT.fetch_sub(1, Ordering::Relaxed) == 1 {
            let _ = unsafe { HackrfError::from_id(ffi::hackrf_exit()) };
        }
    }
}
