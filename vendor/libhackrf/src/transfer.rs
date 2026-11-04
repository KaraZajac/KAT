use std::{any::Any, mem::ManuallyDrop, slice};

use num_complex::Complex;

use super::{ffi, HackRf};

pub type TransmitCallback = fn(hack_rf: &HackRf, samples: &mut [Complex<i8>], user: &dyn Any);
pub type ReceiveCallback = fn(hack_rf: &HackRf, samples: &[Complex<i8>], user: &dyn Any);

/// What libhackrf's transfer thread hands back to the callbacks. `HackRf`'s `Drop` closes the
/// device, so the handle passed to callbacks is a clone that is never dropped (`ManuallyDrop`);
/// freeing a context drops only the user data.
pub struct TransferContext<Callback> {
    callback: Callback,
    hackrf: ManuallyDrop<HackRf>,
    user_data: Box<dyn Any>,
}

impl<Callback> TransferContext<Callback> {
    pub(super) fn new(callback: Callback, hackrf: HackRf, user_data: Box<dyn Any>) -> Self {
        Self {
            callback,
            hackrf: ManuallyDrop::new(hackrf),
            user_data,
        }
    }
}

pub(super) extern "C" fn tx_callback(transfer: *mut ffi::HackrfTransfer) -> i32 {
    unsafe {
        let transfer = &mut *transfer;
        let context = &*(transfer.tx_ctx as *mut TransferContext<TransmitCallback>);

        let buffer = slice::from_raw_parts_mut(
            transfer.buffer as *mut Complex<i8>,
            transfer.valid_length as usize / 2,
        );
        (context.callback)(&*context.hackrf, buffer, &*context.user_data);
    }

    0
}

pub(super) extern "C" fn rx_callback(transfer: *mut ffi::HackrfTransfer) -> i32 {
    unsafe {
        let transfer = &*transfer;
        let context = &*(transfer.rx_ctx as *mut TransferContext<ReceiveCallback>);

        let buffer = slice::from_raw_parts(
            transfer.buffer as *const Complex<i8>,
            transfer.valid_length as usize / 2,
        );
        (context.callback)(&*context.hackrf, buffer, &*context.user_data);
    }

    0
}
