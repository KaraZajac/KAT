use std::{
    any::Any,
    panic::{self, AssertUnwindSafe},
    slice,
};

use num_complex::Complex;

use super::{ffi, HackRf};

pub type TransmitCallback = fn(hack_rf: &HackRf, samples: &mut [Complex<i8>], user: &dyn Any);
pub type ReceiveCallback = fn(hack_rf: &HackRf, samples: &[Complex<i8>], user: &dyn Any);

/// What libhackrf's transfer thread hands back to the callbacks. `hackrf` is a non-owning handle
/// (`owner: None`) so freeing the context neither prematurely closes the device nor deadlocks if
/// `Drop` runs on the outer handle; `user_data` is dropped when transfers stop or `start_*` fails.
pub struct TransferContext<Callback> {
    callback: Callback,
    hackrf: HackRf,
    user_data: Box<dyn Any>,
}

impl<Callback> TransferContext<Callback> {
    pub(super) fn new(callback: Callback, hackrf: HackRf, user_data: Box<dyn Any>) -> Self {
        Self {
            callback,
            hackrf,
            user_data,
        }
    }
}

fn valid_sample_count(transfer: &ffi::HackrfTransfer) -> Option<usize> {
    if transfer.buffer.is_null() || transfer.buffer_length < 0 || transfer.valid_length < 0 {
        return None;
    }
    if transfer.valid_length > transfer.buffer_length {
        return None;
    }
    Some(transfer.valid_length as usize / 2)
}

pub(super) extern "C" fn tx_callback(transfer: *mut ffi::HackrfTransfer) -> i32 {
    if transfer.is_null() {
        return -1;
    }
    panic::catch_unwind(AssertUnwindSafe(|| unsafe {
        let transfer = &mut *transfer;
        if transfer.tx_ctx.is_null() {
            return -1;
        }
        let Some(samples) = valid_sample_count(transfer) else {
            return -1;
        };
        let context = &*(transfer.tx_ctx as *const TransferContext<TransmitCallback>);
        let mut empty = [];
        let buffer = if samples == 0 {
            &mut empty[..]
        } else {
            slice::from_raw_parts_mut(transfer.buffer as *mut Complex<i8>, samples)
        };
        (context.callback)(&context.hackrf, buffer, &*context.user_data);
        0
    }))
    .unwrap_or(-1)
}

pub(super) extern "C" fn rx_callback(transfer: *mut ffi::HackrfTransfer) -> i32 {
    if transfer.is_null() {
        return -1;
    }
    panic::catch_unwind(AssertUnwindSafe(|| unsafe {
        let transfer = &*transfer;
        if transfer.rx_ctx.is_null() {
            return -1;
        }
        let Some(samples) = valid_sample_count(transfer) else {
            return -1;
        };
        let context = &*(transfer.rx_ctx as *const TransferContext<ReceiveCallback>);
        let empty = [];
        let buffer = if samples == 0 {
            &empty[..]
        } else {
            slice::from_raw_parts(transfer.buffer as *const Complex<i8>, samples)
        };
        (context.callback)(&context.hackrf, buffer, &*context.user_data);
        0
    }))
    .unwrap_or(-1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ptr;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;

    #[test]
    fn rx_callback_guards_invalid_transfers_and_panics() {
        assert_eq!(rx_callback(ptr::null_mut()), -1);

        let handle = HackRf::mock_for_test().0;
        let seen = Arc::new(AtomicUsize::new(0));
        let cb: ReceiveCallback = |_, samples, user| {
            let count = user.downcast_ref::<Arc<AtomicUsize>>().unwrap();
            count.store(samples.len(), Ordering::SeqCst);
        };
        let ctx = Box::new(TransferContext::new(cb, handle.callback_handle(), Box::new(seen.clone())));
        let ctx_ptr = Box::into_raw(ctx);

        let mut buf = [1u8, 2, 3, 4, 5, 6];
        let mut xfer = ffi::HackrfTransfer {
            device: ptr::null_mut(),
            buffer: ptr::null_mut(),
            buffer_length: 6,
            valid_length: 6,
            rx_ctx: ptr::null_mut(),
            tx_ctx: ptr::null_mut(),
        };
        assert_eq!(rx_callback(&mut xfer), -1, "null rx_ctx rejected");

        xfer.rx_ctx = ctx_ptr.cast();
        assert_eq!(rx_callback(&mut xfer), -1, "null buffer rejected");

        xfer.buffer = buf.as_mut_ptr();
        xfer.valid_length = -4;
        assert_eq!(rx_callback(&mut xfer), -1, "negative valid_length rejected");

        xfer.valid_length = 8;
        assert_eq!(rx_callback(&mut xfer), -1, "valid_length > buffer_length rejected");

        xfer.valid_length = 0;
        assert_eq!(rx_callback(&mut xfer), 0, "zero valid_length passes empty slice");
        assert_eq!(seen.load(Ordering::SeqCst), 0);

        xfer.valid_length = 6;
        assert_eq!(rx_callback(&mut xfer), 0, "valid transfer passes samples");
        assert_eq!(seen.load(Ordering::SeqCst), 3);

        unsafe { drop(Box::from_raw(ctx_ptr)) };

        let panic_cb: ReceiveCallback = |_, _, _| panic!("boom in rx callback");
        let pctx = Box::into_raw(Box::new(TransferContext::new(
            panic_cb,
            handle.callback_handle(),
            Box::new(()),
        )));
        xfer.rx_ctx = pctx.cast();
        assert_eq!(rx_callback(&mut xfer), -1, "panic caught instead of unwinding across FFI");
        unsafe { drop(Box::from_raw(pctx)) };
    }

    #[test]
    fn tx_callback_guards_invalid_transfers_and_panics() {
        assert_eq!(tx_callback(ptr::null_mut()), -1);

        let handle = HackRf::mock_for_test().0;
        let cb: TransmitCallback = |_, samples, _| {
            for (i, s) in samples.iter_mut().enumerate() {
                *s = Complex::new(i as i8, -(i as i8));
            }
        };
        let ctx_ptr = Box::into_raw(Box::new(TransferContext::new(
            cb,
            handle.callback_handle(),
            Box::new(()),
        )));

        let mut buf = [0u8; 4];
        let mut xfer = ffi::HackrfTransfer {
            device: ptr::null_mut(),
            buffer: buf.as_mut_ptr(),
            buffer_length: 4,
            valid_length: -2,
            rx_ctx: ptr::null_mut(),
            tx_ctx: ctx_ptr.cast(),
        };
        assert_eq!(tx_callback(&mut xfer), -1, "negative valid_length rejected");

        xfer.valid_length = 6;
        assert_eq!(tx_callback(&mut xfer), -1, "oversized valid_length rejected");

        xfer.valid_length = 4;
        assert_eq!(tx_callback(&mut xfer), 0);
        assert_eq!(buf, [0, 0, 1, 255u8]);

        unsafe { drop(Box::from_raw(ctx_ptr)) };

        let panic_cb: TransmitCallback = |_, _, _| panic!("boom in tx callback");
        let pctx = Box::into_raw(Box::new(TransferContext::new(
            panic_cb,
            handle.callback_handle(),
            Box::new(()),
        )));
        xfer.tx_ctx = pctx.cast();
        assert_eq!(tx_callback(&mut xfer), -1, "panic caught cleanly");
        unsafe { drop(Box::from_raw(pctx)) };
    }
}
