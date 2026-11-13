//! Frames in an endless pulse stream.
//!
//! KAT's SDR receivers hand the decoders bursts: a squelch opens on a carrier and closes after it,
//! and [ProtocolRegistry::process_signal_stream] decodes one burst at a time. A Flipper Zero
//! streaming its CC1101's demodulator over USB (`subghz rx_raw`, [crate::radio::flipper]) never
//! goes quiet: its AGC turns every pause into noise pulses, tens to thousands a second, so there are no
//! bursts to take. As on the Flipper itself, the decoders read the stream as it comes:
//! [StreamLocator::feed] gives every pulse once to the decoders flagged for the receive modulation
//! that support the frequency, in registry order; the first to complete a frame wins and the
//! decoders are reset, exactly as [ProtocolRegistry::process_signal_stream_inner] treats a burst's
//! pulses (an rtl_433 port's frame resets only the ports; `autobf`'s unchecked PSA frames reset
//! nothing). [StreamLocator::reads] then finds where the frame starts: the receiver cuts its pulses
//! out of the noise and hands them to the app as a burst, which decodes it like any other.

use super::{frequency_supported, DecodedSignal, ProtocolRegistry, PSA_UNCHECKED_NAME};
use crate::capture::RfModulation;
use crate::radio::LevelDuration;

/// A frame a decoder completed on a pulse of the stream.
#[derive(Debug, Clone)]
pub struct LocatedFrame {
    /// The decoder (its index in the registry), for [StreamLocator::reads].
    pub decoder: usize,
    /// Protocol name, as a burst decode would report it.
    pub name: String,
    pub frame: DecodedSignal,
    /// `autobf`'s unchecked PSA frame: it reset no decoder.
    pub unchecked: bool,
}

/// The registry's decoders reading one pulse stream (see the module docs).
pub struct StreamLocator {
    registry: ProtocolRegistry,
    /// Decoders that take part: those flagged for the receive modulation.
    allowed: Vec<bool>,
    frequency: u32,
    /// Feed the opposite level: FSK, where which tone is "on" depends on the fob.
    invert: bool,
}

impl StreamLocator {
    /// Decoders for pulses received at `frequency` with `rf` modulation; `invert` feeds them the
    /// opposite level. `report_unchecked`: report the PSA frames `autobf` searches a key for
    /// ([ProtocolRegistry::set_auto_key_search]), so that the app sees them.
    pub fn new(frequency: u32, rf: RfModulation, invert: bool, report_unchecked: bool) -> Self {
        let mut registry = ProtocolRegistry::new();
        registry.set_auto_key_search(report_unchecked);
        let allowed = registry.allowed(Some(rf), invert, rf == RfModulation::FM);
        Self { registry, allowed, frequency, invert }
    }

    /// Feed one pulse: the frames it completes (the winner, and an unchecked PSA frame).
    pub fn feed(&mut self, level: bool, duration_us: u32) -> Vec<LocatedFrame> {
        self.step(Some((level != self.invert, duration_us)))
    }

    /// The stream ended or broke (pulses were lost): the frames decoders held back for another
    /// pulse ([super::ProtocolDecoder::flush]); then every decoder starts over.
    pub fn flush(&mut self) -> Vec<LocatedFrame> {
        let out = self.step(None);
        for d in &mut self.registry.decoders {
            d.reset();
        }
        out
    }

    fn step(&mut self, pulse: Option<(bool, u32)>) -> Vec<LocatedFrame> {
        let reg = &mut self.registry;
        let mut hit: Option<LocatedFrame> = None;
        let mut unchecked: Option<LocatedFrame> = None;
        for k in 0..reg.decoders.len() {
            if !self.allowed[k] || !frequency_supported(reg.decoders[k].supported_frequencies(), self.frequency) {
                continue;
            }
            let decoded = match pulse {
                Some((level, duration_us)) => reg.decoders[k].feed(level, duration_us),
                None => reg.decoders[k].flush(),
            };
            let Some(frame) = decoded else { continue };
            if hit.is_some() {
                continue;
            }
            let name = frame.protocol_display_name.as_deref().unwrap_or_else(|| reg.decoders[k].name()).to_string();
            if name == PSA_UNCHECKED_NAME {
                unchecked.get_or_insert(LocatedFrame { decoder: k, name, frame, unchecked: true });
                continue;
            }
            // As on a Flipper: the winner's callback resets the receiver before the decoders after
            // it see the pulse (see process_signal_stream_inner).
            let rtl = reg.rtl[k];
            for (d, &r) in reg.decoders.iter_mut().zip(&reg.rtl) {
                if r || !rtl {
                    d.reset();
                }
            }
            hit = Some(LocatedFrame { decoder: k, name, frame, unchecked: false });
        }
        hit.into_iter().chain(unchecked).collect()
    }

    /// Whether `located`'s decoder alone, fresh and flushed at the end, reads its frame from
    /// `pulses` (as received: this locator's inversion is applied). Run on the registry's probe
    /// copy of the decoder, so the decoders reading the stream keep their state.
    pub fn reads(&mut self, located: &LocatedFrame, pulses: &[LevelDuration]) -> bool {
        let Some(dec) = self.registry.probe.get_mut(located.decoder) else { return false };
        dec.reset();
        let want = &located.frame;
        let same = |d: &DecodedSignal| d.data == want.data && d.data_count_bit == want.data_count_bit;
        let mut found = pulses.iter().any(|p| dec.feed(p.level != self.invert, p.duration_us).is_some_and(|d| same(&d)));
        if !found {
            found = dec.flush().is_some_and(|d| same(&d));
        }
        dec.reset();
        found
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocols::FlipperKey;

    /// A made-up Princeton key's transmission, as its encoder sends it.
    fn princeton_upload() -> (Vec<LevelDuration>, DecodedSignal) {
        let key = FlipperKey::parse("Protocol: Princeton\nBit: 24\nKey: 00 00 00 00 00 5A 3C 81\nTE: 400\n").unwrap();
        let reg = ProtocolRegistry::new();
        let (name, d) = reg.decode_key(&key).unwrap();
        let up = reg.get(&name).unwrap().encode(&d, d.button.unwrap_or(0)).unwrap();
        (up, d)
    }

    /// Noise pulses as a Flipper's CC1101 gives them between transmissions: short, irregular.
    fn noise(n: usize, seed: &mut u64, level: &mut bool) -> Vec<LevelDuration> {
        (0..n)
            .map(|_| {
                *seed ^= *seed << 13;
                *seed ^= *seed >> 7;
                *seed ^= *seed << 17;
                *level = !*level;
                LevelDuration::new(*level, 15 + (*seed % 240) as u32)
            })
            .collect()
    }

    /// Frames in noise are found once each, on the pulse that completes them, and `reads` tells
    /// the frame's own pulses from the noise before them.
    #[test]
    fn finds_frames_in_noise() {
        let (up, d) = princeton_upload();
        let (mut seed, mut level) = (0x2545_F491_4F6C_DD1Du64, false);
        let mut stream = noise(3000, &mut seed, &mut level);
        let start = stream.len();
        stream.extend_from_slice(&up);
        stream.extend(noise(3000, &mut seed, &mut level));
        let mut loc = StreamLocator::new(433_920_000, RfModulation::AM, false, true);
        let mut hits = Vec::new();
        for (i, p) in stream.iter().enumerate() {
            hits.extend(loc.feed(p.level, p.duration_us).into_iter().map(|f| (i, f)));
        }
        hits.extend(loc.flush().into_iter().map(|f| (stream.len() - 1, f)));
        assert!(!hits.is_empty(), "no frame found");
        assert!(hits.iter().all(|(_, f)| f.name == "Princeton" && f.frame.data == d.data), "{hits:?}");
        let (end, first) = &hits[0];
        assert!(*end > start && *end < start + up.len() + 2, "frame completed at {end}, upload {start}..{}", start + up.len());
        // Its pulses from where the upload starts read; its last ten bits alone do not.
        assert!(loc.reads(first, &stream[start..=*end]));
        assert!(loc.reads(first, &stream[..=*end]));
        assert!(!loc.reads(first, &stream[*end - 20..=*end]));
    }
}
