//! Transmit queue, per-transmission modulation and TX power (the app side of KAT's transmitter).
//!
//! Every transmission is queued with the modulation its capture calls for
//! ([crate::capture::Capture::tx_modulation]: the capture's own preset, its receive path, or its
//! protocol's preset) and sent with the TX power settings (`tx_vga_gain` / `tx_amp` in
//! `config.ini`, `:txpower`, the TX settings tab). The queue runs first in, first out, so
//! `:replay 1-5` goes out in the order given.

use std::collections::VecDeque;

use anyhow::Result;

use super::{App, RadioState};
use crate::radio::preset::TxModulation;
use crate::radio::tx::TxSettings;
use crate::radio::LevelDuration;

/// Highest HackRF TX VGA gain (dB).
pub const TX_VGA_MAX: u32 = 47;

/// TX power choices offered in the settings dropdown: (TX VGA dB, RF amp).
pub const TX_POWER_STEPS: [(u32, bool); 9] = [
    (47, true),
    (40, true),
    (30, true),
    (47, false),
    (40, false),
    (30, false),
    (20, false),
    (10, false),
    (0, false),
];

/// One queued transmission.
pub(super) struct PendingTransmit {
    pub(super) signal: Vec<LevelDuration>,
    pub(super) frequency: u32,
    pub(super) modulation: TxModulation,
}

/// The transmit queue (first in, first out).
pub(super) type TransmitQueue = VecDeque<PendingTransmit>;

/// "47 dB + amp" / "30 dB".
pub fn tx_power_label(vga_gain: u32, amp: bool) -> String {
    format!("{vga_gain} dB{}", if amp { " + amp" } else { "" })
}

/// Parse `:txpower` arguments into (gain, amp), keeping the current value of whatever is not
/// given: `<gain>`, `<gain> amp on|off`, `<gain> on|off`, `amp on|off`. None = usage error (the
/// gain range is checked when it is applied).
fn parse_txpower(args: &[&str], gain: u32, amp: bool) -> Option<(u32, bool)> {
    let mut words: Vec<String> = args.iter().map(|w| w.to_ascii_lowercase()).collect();
    let mut gain = gain;
    let mut amp = amp;
    if let Some(g) = words.first().and_then(|w| w.parse::<u32>().ok()) {
        gain = g;
        words.remove(0);
    }
    let words: Vec<&str> = words.iter().map(String::as_str).collect();
    match words.as_slice() {
        [] if !args.is_empty() => {}
        ["amp", v] | [v] => match *v {
            "on" | "1" | "true" => amp = true,
            "off" | "0" | "false" => amp = false,
            _ => return None,
        },
        _ => return None,
    }
    Some((gain, amp))
}

impl App {
    /// Queue a transmission; the main loop draws the TX state, then runs it.
    pub(super) fn queue_transmit(
        &mut self,
        signal: Vec<LevelDuration>,
        frequency: u32,
        modulation: TxModulation,
    ) {
        self.pending_transmit_queue.push_back(PendingTransmit { signal, frequency, modulation });
        if self.pending_transmit_restore.is_none() {
            self.pending_transmit_restore = Some(self.radio_state);
            self.radio_state = RadioState::Transmitting;
        }
    }

    /// Settings for one transmission: its modulation at the current TX power.
    pub fn tx_settings(&self, modulation: TxModulation) -> TxSettings {
        TxSettings { modulation, vga_gain: self.tx_vga_gain, amp: self.tx_amp }
    }

    /// True if there are queued transmits (UI should draw then call run_one_pending_transmit).
    pub fn has_pending_transmit(&self) -> bool {
        !self.pending_transmit_queue.is_empty()
    }

    /// Run the oldest queued transmit; restores radio_state when the queue is empty. Call after
    /// drawing. A failed transmission is reported in the status line (it no longer ends KAT).
    pub fn run_one_pending_transmit(&mut self) -> Result<()> {
        let Some(tx) = self.pending_transmit_queue.pop_front() else {
            return Ok(());
        };
        let settings = self.tx_settings(tx.modulation);
        let result = match self.radio.as_mut() {
            Some(radio) => radio.transmit(&tx.signal, tx.frequency, &settings),
            None => Ok(()),
        };
        if self.pending_transmit_queue.is_empty() {
            if let Some(prev) = self.pending_transmit_restore.take() {
                self.radio_state = prev;
            }
        }
        if let Err(e) = result {
            self.last_error = Some(format!("Transmit failed: {e:#}"));
        }
        Ok(())
    }

    /// Set the TX power used for every transmission (VGA 0–47 dB, RF amp on/off).
    pub(super) fn set_tx_power(&mut self, vga_gain: u32, amp: bool) {
        if vga_gain > TX_VGA_MAX {
            self.last_error = Some(format!("TX VGA gain must be 0-{TX_VGA_MAX} dB"));
            return;
        }
        self.tx_vga_gain = vga_gain;
        self.tx_amp = amp;
        self.status_message = Some(format!("TX power: {}", tx_power_label(vga_gain, amp)));
    }

    /// `:txpower [<0-47>] [amp on|off]` (arguments after the command word). Without arguments it
    /// shows the current setting; `amp on|off` alone keeps the gain, a gain alone keeps the amp.
    pub(super) fn txpower_command(&mut self, args: &[&str]) {
        if args.is_empty() {
            self.status_message =
                Some(format!("TX power: {}", tx_power_label(self.tx_vga_gain, self.tx_amp)));
            return;
        }
        match parse_txpower(args, self.tx_vga_gain, self.tx_amp) {
            Some((gain, amp)) => self.set_tx_power(gain, amp),
            None => self.last_error = Some("Usage: :txpower <0-47> [amp on|off]".to_string()),
        }
    }

    /// Settings dropdown: index of the current TX power in [TX_POWER_STEPS] (0 if not listed).
    pub(super) fn tx_power_value_index(&self) -> usize {
        TX_POWER_STEPS
            .iter()
            .position(|&(g, a)| g == self.tx_vga_gain && a == self.tx_amp)
            .unwrap_or(0)
    }

    /// Settings dropdown: apply [TX_POWER_STEPS]\[index\].
    pub(super) fn apply_tx_power_value(&mut self, index: usize) {
        if let Some(&(gain, amp)) = TX_POWER_STEPS.get(index) {
            self.set_tx_power(gain, amp);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn txpower_arguments() {
        assert_eq!(parse_txpower(&["30"], 47, true), Some((30, true)));
        assert_eq!(parse_txpower(&["30", "amp", "off"], 47, true), Some((30, false)));
        assert_eq!(parse_txpower(&["0", "AMP", "ON"], 47, false), Some((0, true)));
        assert_eq!(parse_txpower(&["20", "off"], 47, true), Some((20, false)));
        assert_eq!(parse_txpower(&["amp", "off"], 40, true), Some((40, false)));
        assert_eq!(parse_txpower(&["99"], 47, true), Some((99, true)), "range checked when applied");
        for bad in [&["amp"][..], &["loud"], &["30", "amp"], &["30", "amp", "maybe"], &["30", "40"], &[]] {
            assert_eq!(parse_txpower(bad, 47, true), None, "{bad:?}");
        }
    }

    #[test]
    fn tx_power_steps_are_valid_and_labelled() {
        assert_eq!(TX_POWER_STEPS[0], (TX_VGA_MAX, true), "first entry = KAT ≤ 1.4 default");
        assert!(TX_POWER_STEPS.iter().all(|&(g, _)| g <= TX_VGA_MAX));
        assert_eq!(tx_power_label(47, true), "47 dB + amp");
        assert_eq!(tx_power_label(30, false), "30 dB");
    }
}
