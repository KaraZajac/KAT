#![allow(improper_ctypes)]

use std::ffi::{c_char, c_double, c_int, c_uchar, c_uint, c_ulonglong, c_void};

#[repr(C)]
pub struct HackrfDevice;

#[repr(C)]
pub struct HackrfTransfer {
    pub device: *mut HackrfDevice,
    pub buffer: *mut c_uchar,
    pub buffer_length: c_int,
    pub valid_length: c_int,
    pub rx_ctx: *mut c_void,
    pub tx_ctx: *mut c_void,
}

#[derive(Default)]
#[repr(C)]
pub struct SerialNumber {
    pub part_id: [c_uint; 2],
    pub serial_no: [c_uint; 4],
}

#[link(name = "hackrf")]
extern "C" {
    pub fn hackrf_init() -> c_int;
    pub fn hackrf_exit() -> c_int;

    pub fn hackrf_open(device: *mut *mut HackrfDevice) -> c_int;
    pub fn hackrf_close(device: *mut HackrfDevice) -> c_int;

    pub fn hackrf_start_rx(
        device: *mut HackrfDevice,
        callback: extern "C" fn(*mut HackrfTransfer) -> c_int,
        rx_ctx: *mut c_void,
    ) -> c_int;
    pub fn hackrf_stop_rx(device: *mut HackrfDevice) -> c_int;

    pub fn hackrf_start_tx(
        device: *mut HackrfDevice,
        callback: extern "C" fn(*mut HackrfTransfer) -> c_int,
        tx_ctx: *mut c_void,
    ) -> c_int;
    pub fn hackrf_stop_tx(device: *mut HackrfDevice) -> c_int;

    pub fn hackrf_is_streaming(device: *mut HackrfDevice) -> c_int;

    pub fn hackrf_set_baseband_filter_bandwidth(
        device: *mut HackrfDevice,
        bandwidth_hz: c_uint,
    ) -> c_int;

    pub fn hackrf_board_id_read(device: *mut HackrfDevice, value: *mut c_uchar) -> c_int;
    pub fn hackrf_version_string_read(
        device: *mut HackrfDevice,
        version: *mut c_char,
        length: c_uchar,
    ) -> c_int;
    pub fn hackrf_board_partid_serialno_read(
        device: *mut HackrfDevice,
        read_partid_serialno: *mut SerialNumber,
    ) -> c_int;

    pub fn hackrf_set_freq(device: *mut HackrfDevice, freq_hz: c_ulonglong) -> c_int;
    pub fn hackrf_set_freq_explicit(
        device: *mut HackrfDevice,
        if_freq_hz: c_ulonglong,
        lo_freq_hz: c_ulonglong,
        path: c_uint,
    ) -> c_int;

    pub fn hackrf_set_sample_rate_manual(
        device: *mut HackrfDevice,
        freq_hz: c_uint,
        divider: c_uint,
    ) -> c_int;
    pub fn hackrf_set_sample_rate(device: *mut HackrfDevice, freq_hz: c_double) -> c_int;

    pub fn hackrf_set_amp_enable(device: *mut HackrfDevice, value: c_uchar) -> c_int;

    pub fn hackrf_set_lna_gain(device: *mut HackrfDevice, value: c_uint) -> c_int;
    pub fn hackrf_set_vga_gain(device: *mut HackrfDevice, value: c_uint) -> c_int;
    pub fn hackrf_set_txvga_gain(device: *mut HackrfDevice, value: c_uint) -> c_int;

    pub fn hackrf_set_antenna_enable(device: *mut HackrfDevice, value: c_uchar) -> c_int;

    pub fn hackrf_error_name(errcode: c_int) -> *const c_char;
    pub fn hackrf_board_id_name(hackrf_board_id: c_uchar) -> *const c_char;
    pub fn hackrf_filter_path_name(path: c_uint) -> *const c_char;

    pub fn hackrf_compute_baseband_filter_bw_round_down_lt(bandwidth_hz: c_uint) -> c_uint;
    pub fn hackrf_compute_baseband_filter_bw(bandwidth_hz: c_uint) -> c_uint;
}
