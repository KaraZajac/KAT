//! Offline analysis of captured signals (works on a capture's level/duration pairs; no radio).
//!
//! - [timing]: KAT's port of ProtoPirate's Timing Tuner (received pulse timing vs a protocol's
//!   `te_short` / `te_long` / `te_delta`), plus pulse clustering, a base-TE estimate and a
//!   line-coding guess for captures no decoder recognised.
//! - [signal]: the "Analyze" report of any capture with raw pulses: modulation, pulse widths, rows
//!   and repeats, a trial decode to bits, a waveform and the protocols with similar timing.

pub mod signal;
pub mod timing;
