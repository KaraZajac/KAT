//! Offline analysis of captured signals (works on a capture's level/duration pairs; no radio).
//!
//! - [timing]: KAT's port of ProtoPirate's Timing Tuner (received pulse timing vs a protocol's
//!   `te_short` / `te_long` / `te_delta`), plus pulse clustering, a base-TE estimate and a
//!   line-coding guess for captures no decoder recognised.
//! - [compare]: "Compare", the bits of several presses of one remote lined up: which stay fixed,
//!   which count, which follow the button and which change with no pattern.
//! - [signal]: the "Analyze" report of any capture with raw pulses: modulation, pulse widths, rows
//!   and repeats, a trial decode to bits, a waveform and the protocols with similar timing.

pub mod compare;
pub mod signal;
pub mod timing;
