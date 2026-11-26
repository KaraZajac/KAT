//! SoapySDR's C API, loaded at run time from the system's library (only the helper process
//! loads it). ABI 0.7, 0.8 and 0.9 are the same for what KAT uses but `setupStream`, which 0.8
//! changed to return the stream.

use libloading::Library;
use std::ffi::{c_char, c_int, c_long, c_void, CStr, CString};
use std::ptr;

const RX: c_int = 1;
/// readStream: no samples within the timeout.
pub const TIMEOUT: c_int = -1;
/// readStream: samples were dropped (the reader was late).
pub const OVERFLOW: c_int = -4;
/// SOAPY_SDR_INFO: the level of messages passed on.
const LOG_INFO: c_int = 6;

#[repr(C)]
struct Kwargs {
    size: usize,
    keys: *mut *mut c_char,
    vals: *mut *mut c_char,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Range {
    pub minimum: f64,
    pub maximum: f64,
    pub step: f64,
}

#[repr(C)]
struct RawDevice {
    _private: [u8; 0],
}

#[repr(C)]
struct RawStream {
    _private: [u8; 0],
}

type LogHandler = extern "C" fn(level: c_int, message: *const c_char);
type SetupStream08 =
    unsafe extern "C" fn(*mut RawDevice, c_int, *const c_char, *const usize, usize, *const Kwargs) -> *mut RawStream;
type SetupStream07 = unsafe extern "C" fn(
    *mut RawDevice,
    *mut *mut RawStream,
    c_int,
    *const c_char,
    *const usize,
    usize,
    *const Kwargs,
) -> c_int;

enum SetupStream {
    Abi07(SetupStream07),
    Abi08(SetupStream08),
}

/// The library names tried, after `KAT_SOAPYSDR` (a path) when it is set.
#[cfg(target_os = "linux")]
const LIBRARIES: &[&str] = &["libSoapySDR.so.0.9", "libSoapySDR.so.0.8", "libSoapySDR.so.0.7", "libSoapySDR.so"];
#[cfg(target_os = "macos")]
const LIBRARIES: &[&str] = &[
    "libSoapySDR.0.9.dylib",
    "libSoapySDR.0.8.dylib",
    "libSoapySDR.0.7.dylib",
    "libSoapySDR.dylib",
    "/opt/homebrew/lib/libSoapySDR.dylib",
    "/usr/local/lib/libSoapySDR.dylib",
];
#[cfg(windows)]
const LIBRARIES: &[&str] = &["SoapySDR.dll", "libSoapySDR.dll"];
#[cfg(not(any(target_os = "linux", target_os = "macos", windows)))]
const LIBRARIES: &[&str] = &["libSoapySDR.so"];

/// SoapySDR, loaded.
pub struct Soapy {
    /// Its library file, its version, its ABI ("0.8-3").
    pub path: String,
    pub version: String,
    pub abi: String,
    enumerate: unsafe extern "C" fn(*const Kwargs, *mut usize) -> *mut Kwargs,
    kwargs_list_clear: unsafe extern "C" fn(*mut Kwargs, usize),
    make: unsafe extern "C" fn(*const Kwargs) -> *mut RawDevice,
    unmake: unsafe extern "C" fn(*mut RawDevice) -> c_int,
    last_error: unsafe extern "C" fn() -> *const c_char,
    err_to_str: unsafe extern "C" fn(c_int) -> *const c_char,
    register_log_handler: unsafe extern "C" fn(Option<LogHandler>),
    set_log_level: unsafe extern "C" fn(c_int),
    get_driver_key: unsafe extern "C" fn(*const RawDevice) -> *mut c_char,
    get_hardware_key: unsafe extern "C" fn(*const RawDevice) -> *mut c_char,
    set_sample_rate: unsafe extern "C" fn(*mut RawDevice, c_int, usize, f64) -> c_int,
    get_sample_rate: unsafe extern "C" fn(*const RawDevice, c_int, usize) -> f64,
    get_sample_rate_range: Option<unsafe extern "C" fn(*const RawDevice, c_int, usize, *mut usize) -> *mut Range>,
    list_sample_rates: unsafe extern "C" fn(*const RawDevice, c_int, usize, *mut usize) -> *mut f64,
    set_frequency: unsafe extern "C" fn(*mut RawDevice, c_int, usize, f64, *const Kwargs) -> c_int,
    has_gain_mode: unsafe extern "C" fn(*const RawDevice, c_int, usize) -> bool,
    set_gain_mode: unsafe extern "C" fn(*mut RawDevice, c_int, usize, bool) -> c_int,
    set_gain: unsafe extern "C" fn(*mut RawDevice, c_int, usize, f64) -> c_int,
    get_gain_range: unsafe extern "C" fn(*const RawDevice, c_int, usize) -> Range,
    setup_stream: SetupStream,
    close_stream: unsafe extern "C" fn(*mut RawDevice, *mut RawStream) -> c_int,
    activate_stream: unsafe extern "C" fn(*mut RawDevice, *mut RawStream, c_int, i64, usize) -> c_int,
    deactivate_stream: unsafe extern "C" fn(*mut RawDevice, *mut RawStream, c_int, i64) -> c_int,
    read_stream:
        unsafe extern "C" fn(*mut RawDevice, *mut RawStream, *const *mut c_void, usize, *mut c_int, *mut i64, c_long) -> c_int,
    /// `SoapySDR_free` (from 0.8): memory the library hands out goes back to its own allocator.
    /// Without it (0.7) that little memory is left, rather than freed by a C runtime that may
    /// not be the library's (Windows).
    free: Option<unsafe extern "C" fn(*mut c_void)>,
    // Last: the functions above point into it.
    _lib: Library,
}

/// The ABI's major and minor version ("0.8-3" → (0, 8)).
pub fn abi_version(abi: &str) -> Option<(u32, u32)> {
    let mut parts = abi.split(['.', '-']);
    Some((parts.next()?.trim().parse().ok()?, parts.next()?.trim().parse().ok()?))
}

/// A C string from the library, as Rust's (None for a null pointer).
unsafe fn text(p: *const c_char) -> Option<String> {
    (!p.is_null()).then(|| CStr::from_ptr(p).to_string_lossy().into_owned())
}

/// SoapySDR's log handler: the message on stderr. Never panics across the `extern "C"` boundary
/// (for instance if stderr is closed while KAT shuts down).
extern "C" fn forward_log(level: c_int, message: *const c_char) {
    let _ = std::panic::catch_unwind(|| {
        use std::io::Write;
        let level = match level {
            1..=3 => "error",
            4 => "warning",
            5 | 6 => "info",
            _ => "debug",
        };
        // SAFETY: SoapySDR passes a NUL-terminated message (or NULL).
        let message = unsafe { text(message) }.unwrap_or_default();
        let _ = writeln!(std::io::stderr(), "[{level}] {message}");
    });
}

impl Soapy {
    /// Load the system's SoapySDR; Err: the libraries tried, and why each failed.
    pub fn load() -> Result<Soapy, String> {
        let mut tried = Vec::new();
        let custom = std::env::var("KAT_SOAPYSDR").ok().filter(|p| !p.trim().is_empty());
        for name in custom.iter().map(String::as_str).chain(LIBRARIES.iter().copied()) {
            // SAFETY: loading SoapySDR runs its initialisers, which register its built-in parts.
            match unsafe { Library::new(name) } {
                Ok(lib) => return unsafe { Soapy::bind(lib, name) },
                Err(e) => tried.push(format!("{name}: {e}")),
            }
        }
        Err(tried.join("; "))
    }

    unsafe fn bind(lib: Library, path: &str) -> Result<Soapy, String> {
        /// A function of the library, as the type it is assigned to.
        unsafe fn get<T: Copy>(lib: &Library, path: &str, name: &[u8]) -> Result<T, String> {
            lib.get::<T>(name).map(|s| *s).map_err(|e| {
                format!("{path}: {}: {e}", String::from_utf8_lossy(name.strip_suffix(b"\0").unwrap_or(name)))
            })
        }
        macro_rules! sym {
            ($name:literal) => {
                get(&lib, path, $name)?
            };
        }
        macro_rules! opt {
            ($name:literal) => {
                get(&lib, path, $name).ok()
            };
        }
        let get_abi: unsafe extern "C" fn() -> *const c_char = sym!(b"SoapySDR_getABIVersion\0");
        let get_lib: unsafe extern "C" fn() -> *const c_char = sym!(b"SoapySDR_getLibVersion\0");
        let abi = text(get_abi()).unwrap_or_default();
        let version = text(get_lib()).unwrap_or_default();
        let setup_stream = match abi_version(&abi) {
            Some((0, minor)) if minor < 8 => SetupStream::Abi07(sym!(b"SoapySDRDevice_setupStream\0")),
            Some(_) => SetupStream::Abi08(sym!(b"SoapySDRDevice_setupStream\0")),
            None => return Err(format!("{path}: an ABI version KAT cannot read ({abi:?})")),
        };
        Ok(Soapy {
            path: path.to_string(),
            version,
            abi,
            enumerate: sym!(b"SoapySDRDevice_enumerate\0"),
            kwargs_list_clear: sym!(b"SoapySDRKwargsList_clear\0"),
            make: sym!(b"SoapySDRDevice_make\0"),
            unmake: sym!(b"SoapySDRDevice_unmake\0"),
            last_error: sym!(b"SoapySDRDevice_lastError\0"),
            err_to_str: sym!(b"SoapySDR_errToStr\0"),
            register_log_handler: sym!(b"SoapySDR_registerLogHandler\0"),
            set_log_level: sym!(b"SoapySDR_setLogLevel\0"),
            get_driver_key: sym!(b"SoapySDRDevice_getDriverKey\0"),
            get_hardware_key: sym!(b"SoapySDRDevice_getHardwareKey\0"),
            set_sample_rate: sym!(b"SoapySDRDevice_setSampleRate\0"),
            get_sample_rate: sym!(b"SoapySDRDevice_getSampleRate\0"),
            get_sample_rate_range: opt!(b"SoapySDRDevice_getSampleRateRange\0"),
            list_sample_rates: sym!(b"SoapySDRDevice_listSampleRates\0"),
            set_frequency: sym!(b"SoapySDRDevice_setFrequency\0"),
            has_gain_mode: sym!(b"SoapySDRDevice_hasGainMode\0"),
            set_gain_mode: sym!(b"SoapySDRDevice_setGainMode\0"),
            set_gain: sym!(b"SoapySDRDevice_setGain\0"),
            get_gain_range: sym!(b"SoapySDRDevice_getGainRange\0"),
            setup_stream,
            close_stream: sym!(b"SoapySDRDevice_closeStream\0"),
            activate_stream: sym!(b"SoapySDRDevice_activateStream\0"),
            deactivate_stream: sym!(b"SoapySDRDevice_deactivateStream\0"),
            read_stream: sym!(b"SoapySDRDevice_readStream\0"),
            free: opt!(b"SoapySDR_free\0"),
            _lib: lib,
        })
    }

    /// SoapySDR's messages, and its modules', on stderr ([forward_log]): never on KAT's screen,
    /// as the helper's stderr goes to kat.log.
    pub fn log_to_stderr(&self) {
        // SAFETY: plain calls into the library; the handler lives as long as the process.
        unsafe {
            (self.register_log_handler)(Some(forward_log));
            (self.set_log_level)(LOG_INFO);
        }
    }

    fn free(&self, p: *mut c_void) {
        if let (Some(free), false) = (self.free, p.is_null()) {
            // SAFETY: p came from this library.
            unsafe { free(p) }
        }
    }

    fn last_error(&self) -> String {
        // SAFETY: a static string of the library's (thread-local), copied at once.
        unsafe { text((self.last_error)()) }.filter(|e| !e.is_empty()).unwrap_or_else(|| "no reason given".into())
    }

    /// A stream error code in words ("TIMEOUT", "OVERFLOW" …).
    pub fn err_str(&self, code: c_int) -> String {
        // SAFETY: a static string of the library's.
        unsafe { text((self.err_to_str)(code)) }.unwrap_or_else(|| format!("error {code}"))
    }

    /// Every radio SoapySDR's modules find: each as its arguments.
    pub fn enumerate(&self) -> Result<Vec<Vec<(String, String)>>, String> {
        let mut n = 0usize;
        // SAFETY: a null Kwargs pointer means no filter; the list goes back with its own clear.
        unsafe {
            let list = (self.enumerate)(ptr::null(), &mut n);
            if list.is_null() {
                return if n == 0 { Ok(Vec::new()) } else { Err(self.last_error()) };
            }
            let found = std::slice::from_raw_parts(list, n).iter().map(|k| pairs(k)).collect();
            (self.kwargs_list_clear)(list, n);
            Ok(found)
        }
    }

    /// Open the radio the arguments name.
    pub fn make<'a>(&'a self, args: &[(String, String)]) -> Result<Device<'a>, String> {
        let owned: Vec<(CString, CString)> = args
            .iter()
            .map(|(k, v)| Ok((CString::new(k.as_str())?, CString::new(v.as_str())?)))
            .collect::<Result<_, std::ffi::NulError>>()
            .map_err(|e| format!("an argument holds a NUL byte ({e})"))?;
        let mut keys: Vec<*mut c_char> = owned.iter().map(|(k, _)| k.as_ptr() as *mut c_char).collect();
        let mut vals: Vec<*mut c_char> = owned.iter().map(|(_, v)| v.as_ptr() as *mut c_char).collect();
        let kwargs = Kwargs { size: owned.len(), keys: keys.as_mut_ptr(), vals: vals.as_mut_ptr() };
        // SAFETY: SoapySDR copies the arguments; `owned` outlives the call.
        let raw = unsafe { (self.make)(&kwargs) };
        if raw.is_null() {
            return Err(self.last_error());
        }
        Ok(Device { soapy: self, raw, stream: ptr::null_mut(), active: false })
    }
}

/// The pairs of a Kwargs.
unsafe fn pairs(k: &Kwargs) -> Vec<(String, String)> {
    if k.size == 0 || k.keys.is_null() || k.vals.is_null() {
        return Vec::new();
    }
    (0..k.size)
        .map(|i| (text(*k.keys.add(i)).unwrap_or_default(), text(*k.vals.add(i)).unwrap_or_default()))
        .collect()
}

/// An open radio, and its receive stream once set up; closed when dropped.
pub struct Device<'a> {
    soapy: &'a Soapy,
    raw: *mut RawDevice,
    stream: *mut RawStream,
    active: bool,
}

impl Device<'_> {
    fn check(&self, what: &str, rc: c_int) -> Result<(), String> {
        if rc == 0 {
            Ok(())
        } else {
            Err(format!("{what}: {} ({})", self.soapy.last_error(), self.soapy.err_str(rc)))
        }
    }

    fn owned_text(&self, p: *mut c_char) -> String {
        // SAFETY: a string the library allocated, copied, then handed back.
        let s = unsafe { text(p) }.unwrap_or_default();
        self.soapy.free(p.cast());
        s
    }

    pub fn driver_key(&self) -> String {
        self.owned_text(unsafe { (self.soapy.get_driver_key)(self.raw) })
    }

    pub fn hardware_key(&self) -> String {
        self.owned_text(unsafe { (self.soapy.get_hardware_key)(self.raw) })
    }

    /// The receive sample rates channel 0 takes, as ranges (single rates as ranges of one).
    pub fn sample_rate_ranges(&self) -> Vec<Range> {
        let mut n = 0usize;
        // SAFETY: arrays the library allocated, copied, then handed back.
        unsafe {
            if let Some(get) = self.soapy.get_sample_rate_range {
                let p = get(self.raw, RX, 0, &mut n);
                if !p.is_null() {
                    let ranges = std::slice::from_raw_parts(p, n).to_vec();
                    self.soapy.free(p.cast());
                    if !ranges.is_empty() {
                        return ranges;
                    }
                }
            }
            n = 0;
            let p = (self.soapy.list_sample_rates)(self.raw, RX, 0, &mut n);
            if p.is_null() {
                return Vec::new();
            }
            let rates: Vec<Range> =
                std::slice::from_raw_parts(p, n).iter().map(|&r| Range { minimum: r, maximum: r, step: 0.0 }).collect();
            self.soapy.free(p.cast());
            rates
        }
    }

    pub fn set_sample_rate(&mut self, rate: f64) -> Result<(), String> {
        let rc = unsafe { (self.soapy.set_sample_rate)(self.raw, RX, 0, rate) };
        self.check("setting the sample rate", rc)
    }

    pub fn sample_rate(&self) -> f64 {
        unsafe { (self.soapy.get_sample_rate)(self.raw, RX, 0) }
    }

    pub fn set_frequency(&mut self, hz: f64) -> Result<(), String> {
        let rc = unsafe { (self.soapy.set_frequency)(self.raw, RX, 0, hz, ptr::null()) };
        self.check("tuning", rc)
    }

    pub fn gain_range(&self) -> Range {
        unsafe { (self.soapy.get_gain_range)(self.raw, RX, 0) }
    }

    /// A manual gain in dB (taken into the radio's range), or its automatic gain (None) when it
    /// has one.
    pub fn set_gain(&mut self, gain: Option<f64>) -> Result<(), String> {
        let has_agc = unsafe { (self.soapy.has_gain_mode)(self.raw, RX, 0) };
        match gain {
            None if has_agc => self.check("automatic gain", unsafe { (self.soapy.set_gain_mode)(self.raw, RX, 0, true) }),
            None => Ok(()),
            Some(db) => {
                if has_agc {
                    self.check("manual gain", unsafe { (self.soapy.set_gain_mode)(self.raw, RX, 0, false) })?;
                }
                let r = self.gain_range();
                let db = if r.maximum >= r.minimum { db.clamp(r.minimum, r.maximum) } else { db };
                self.check("setting the gain", unsafe { (self.soapy.set_gain)(self.raw, RX, 0, db) })
            }
        }
    }

    /// Set up and start channel 0's receive stream, complex f32 samples.
    pub fn start_stream(&mut self) -> Result<(), String> {
        if !self.stream.is_null() {
            return Err("stream already started".into());
        }
        let format = c"CF32".as_ptr();
        let channel: usize = 0;
        // SAFETY: the format and channel list are read during the call only.
        let stream = unsafe {
            match self.soapy.setup_stream {
                SetupStream::Abi08(setup) => setup(self.raw, RX, format, &channel, 1, ptr::null()),
                SetupStream::Abi07(setup) => {
                    let mut s = ptr::null_mut();
                    if setup(self.raw, &mut s, RX, format, &channel, 1, ptr::null()) != 0 {
                        s = ptr::null_mut();
                    }
                    s
                }
            }
        };
        if stream.is_null() {
            return Err(format!("setting up the stream: {}", self.soapy.last_error()));
        }
        self.stream = stream;
        let rc = unsafe { (self.soapy.activate_stream)(self.raw, stream, 0, 0, 0) };
        self.check("starting the stream", rc)?;
        self.active = true;
        Ok(())
    }

    /// Read up to `buf.len() / 2` samples (I, Q pairs) within `timeout_us`: the number read, or
    /// a stream error code ([TIMEOUT], [OVERFLOW] …).
    pub fn read(&mut self, buf: &mut [f32], timeout_us: i64) -> c_int {
        let buffs = [buf.as_mut_ptr().cast::<c_void>()];
        let (mut flags, mut time_ns) = (0, 0);
        let timeout = c_long::try_from(timeout_us).unwrap_or(c_long::MAX);
        // SAFETY: one channel, one buffer of buf.len() / 2 complex samples.
        unsafe {
            (self.soapy.read_stream)(
                self.raw,
                self.stream,
                buffs.as_ptr(),
                buf.len() / 2,
                &mut flags,
                &mut time_ns,
                timeout,
            )
        }
    }
}

impl Drop for Device<'_> {
    fn drop(&mut self) {
        // SAFETY: the stream and device were made by this library and are released once.
        unsafe {
            if !self.stream.is_null() {
                if self.active {
                    (self.soapy.deactivate_stream)(self.raw, self.stream, 0, 0);
                }
                (self.soapy.close_stream)(self.raw, self.stream);
            }
            (self.soapy.unmake)(self.raw);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abi_versions_are_read() {
        assert_eq!(abi_version("0.8-3"), Some((0, 8)));
        assert_eq!(abi_version("0.7"), Some((0, 7)));
        assert_eq!(abi_version("0.9"), Some((0, 9)));
        assert_eq!(abi_version("junk"), None);
    }

    #[test]
    fn forward_log_handles_null_and_valid_messages() {
        forward_log(6, ptr::null());
        forward_log(3, c"soapy test error".as_ptr());
    }

    #[test]
    fn pairs_ignores_null_key_or_value_arrays() {
        let bad = Kwargs { size: 3, keys: ptr::null_mut(), vals: ptr::null_mut() };
        assert!(unsafe { pairs(&bad) }.is_empty());
    }
}
