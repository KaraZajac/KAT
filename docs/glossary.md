---
layout: default
---

# Glossary

The words the pages of this site use, in the sense they use them. Each entry is short on purpose; where a page
says more, the entry links to it.

## Radio

### Band

The part of the spectrum a remote transmits in. Car remotes sold in North America and Japan mostly use
**315 MHz** (Japanese ones also 312 to 314 MHz), those sold in Europe and most other markets **433.92 MHz**, and
some newer European ones **868 MHz**. Garage door openers also use 300, 310, 318 and 390 MHz. A recording's file
says the frequency it was tuned to, which may be a little off the remote's own: a receiver's filter is wide
enough to hold a signal 50 kHz to the side.

### AM, ASK, OOK

On-off keying: the transmitter is on for one level and off for the other. Flipper firmware and this site call it
**AM**. It is the simplest signal to make and to receive; the remote's bits are in how long the carrier stays on
and off.

### FM, FSK, 2-FSK

Frequency-shift keying: the transmitter stays on and moves between two frequencies, one per level. Flipper
firmware and this site call it **FM**. A receiver has to know how far apart the two frequencies are, which is the
deviation.

### Deviation

How far each of the two frequencies of an FSK signal is from the middle. A remote "at 433.92 MHz with ±47.6 kHz"
sends 433.872 and 433.968 MHz. A receiver set for a much smaller or much larger deviation hears little or
nothing, which is why some recordings of FSK remotes hold no data ([Recording a remote](recording.md)).

### Baud, bit rate

Baud is level changes per second at most; bits per second is the data. With Manchester coding one bit is two
half bits, so a 2 kbit/s remote is a 4 kBaud signal. A receiver's filter is set for a rate, and a signal much
faster than that comes out with its pulses stretched or lost.

### Preset

A named radio setting of the Flipper Zero, written into every `.sub` file: keying, deviation, filter width and
data rate. The four every firmware has:

| Preset | Keying | Deviation | Filter | Notes |
|---|---|---|---|---|
| `AM650` | OOK | – | 650 kHz | the default; holds any on-off-keyed remote that is not too fast |
| `AM270` | OOK | – | 270 kHz | narrower, for a crowded band |
| `FM238` | 2-FSK | ±2.38 kHz | 270 kHz | for narrow FSK |
| `FM476` | 2-FSK | ±47.6 kHz | 270 kHz | the usual FM preset for car remotes |

Firmwares add custom presets, which a file stores as a list of radio registers. [Transmit](transmit.md) lists
the presets KAT sends each protocol with.

### CC1101

The radio chip in a Flipper Zero (Texas Instruments). A preset is a set of its register values. It delivers the
signal as levels and durations, not as samples, so what it filtered away is gone from the recording.

### SDR: HackRF One, RTL-SDR

Software-defined radios: they deliver raw samples of a slice of spectrum and leave the demodulating to the
computer. KAT receives with either and transmits with a HackRF One ([Receive chain](radio.md)). What an RTL-SDR
hears depends on its tuner chip ([RTL-SDR tuners](radio.md#rtl-sdr-tuners)); other SDRs reach KAT through
SoapySDR, to receive ([SoapySDR radios](radio.md#soapysdr-radios)).

### IQ recording

The raw samples of an SDR, saved (`.cs8`, `.cu8`, `.cf32`; a PortaPack's `.C8`, `.C16`). Unlike a `.sub` file it keeps everything the radio
heard, so it can be demodulated again another way. KAT decodes such files and can record one while it receives.

### Skew

A receiver that turns a carrier into levels sees a pulse's two edges at slightly different delays, so every
carrier pulse comes out a little long and every pause a little short, or the other way round. Tens of
microseconds are usual in a Flipper recording of an on-off-keyed remote. It matters when the remote's pulses are
short: at 118 µs a half bit, a skew of 50 µs makes a long pause as long as a short carrier pulse
([GM RFA](gm_rfa.md), [Chevrolet V0](chevrolet_v0.md)).

## Line coding

### Pulse, TE

A pulse is a stretch of one level. Most protocols are built on one time unit, written TE in Flipper code: a
short pulse is one TE and a long one two or three. The protocol table gives the short and the long pulse each
decoder is built around.

### PWM

Pulse-width coding: every bit is a carrier pulse and a pause, and the bit's value is which of the two is the
long one (short carrier and long pause for a 0, the reverse for a 1, or the other way round). Common in gate
remotes and older car remotes.

### PPM

Pulse-position coding: the carrier pulses are all alike and the bit's value is the length of the pause between
them.

### Manchester

Every bit is two half bits of opposite level, so there is a level change in the middle of every bit and never
more than two half bits of one level in a row. Which of the two orders is a 1 is a convention, and the pages say
which they use. A Manchester signal is HIGH exactly as long as it is LOW, which is what lets a decoder measure
and remove [skew](#skew).

### Differential Manchester

The level changes between all bit cells, and once more in the middle of a cell for one of the two bit values.
Only the changes carry data, so the signal reads the same upside down: it does not matter which FSK tone is
called HIGH ([Isuzu Rolling](isuzu_rolling.md)).

### Miller

Delay coding: a 1 is a level change in the middle of its cell, a 0 is none, and two 0s in a row have a change
between them. Pulses are one, one and a half or two cells long ([Honda V6](honda_v6.md)).

### NRZ

The level is the bit, for the length of a bit. Without a change in every bit the receiver needs a good clock, so
remotes that use it send a preamble to set it.

### Preamble

What a remote sends before the frame so that the receiver can settle: usually a run of alternating short pulses,
or of one bit value, tens to hundreds of bits long.

### Sync word

A fixed pattern between the preamble and the data that marks where the frame begins. A decoder searches for it.

### Frame, burst, press

A **frame** is one message: the bits between the sync and the end. A **burst** is one unbroken transmission,
which may hold a preamble and one or several frames. A **press** of a button sends one or more bursts: many
remotes repeat the frame a few times, and some send a new one for as long as the button is held.

## Codes

### Fixed code

The remote sends the same frame every time a button is pressed. A recording of it opens the door again. Most
gate remotes with DIP switches and many older ones are fixed code.

### Rolling code

The remote counts its presses and the frame changes with the count, usually through a cipher, so that a
recorded frame is used up once the receiver has heard it. The receiver accepts counts a little ahead of the last
one it saw, which is why pressing a remote out of range a few times does no harm, and pressing it very many
times can put it out of step. Also called hopping code.

### Serial, counter, button

What a rolling-code frame carries, in the clear or encrypted: the remote's identity (**serial**, or fob ID), the
number of the press (**counter**), and which **button** it was. A page's frame table says where each is and
whether KAT can read it.

### Checksum, CRC

Bits computed from the rest of the frame, so that a receiver notices a damaged frame. For a decoder a checksum is
also the proof that it read a frame and not noise. Protocols without one are read on the shape of the whole burst
alone, and their pages say so.

### KeeLoq

Microchip's cipher and the family of encoder chips (HCS200, HCS300, HCS361 and others) that use it: a 32-bit
block under a 64-bit key. Each remote has its own key, derived from its serial and a secret **manufacturer key**
by one of several **learning** rules, so a frame can be read only with the manufacturer's key. See
[KeeLoq](keeloq.md).

### HITAG2

NXP's 48-bit stream cipher, used in immobiliser transponders and in the remote function of many keys of the
2000s and 2010s. Its key can be recovered from a few frames ([Recover key](hitag2_hell.md)).

### AUT64, TEA, XTEA, AES

Block ciphers found in remotes: AUT64 in the Volkswagen group's keys ([VAG](vag.md)), TEA in Peugeot and Citroën
keys ([PSA](psa.md)), XTEA in Bosch remotes ([Opel V1](opel_v1.md)), AES-128 in newer smart keys
([Kia V6](kia_v6.md)).

### LFSR

A linear feedback shift register: a row of bits that moves one place at each step while the XOR of a few of them
(its taps, written as a polynomial) is fed back in. It runs through a long but fixed sequence, so a remote can use
one in place of a counter, and with no key involved one recorded press tells all the presses after it: the oldest
Volkswagen keys ([VW-1](vw_1.md)) send a 24-bit register that moves 23 steps a press. The HITAG2 cipher is a
48-bit one with a non-linear output.

### Replay

Sending a recorded frame again as it was. It works on fixed-code remotes. On a rolling-code receiver it works
only for a frame the receiver has not heard.

### Encoder

On this site: KAT can build a remote's next valid frame itself (the next count, any button) instead of only
replaying a recording. That needs the whole format, the checksum and, for an encrypted protocol, the key. The
protocol table marks these protocols **encode**.

## Remotes

### Key fob, remote-head key, smart key

A **key fob** is a remote on the key ring. A **remote-head key** has the remote built into the head of a key
with a blade. A **smart key** stays in the pocket: the car and the key exchange a low-frequency challenge and a
radio answer when a door handle is touched. Its buttons send ordinary remote frames as well, and those are what
this site covers.

### RKE, PEPS

Remote keyless entry: locking and unlocking with a button. PEPS, passive entry and passive start, is the smart
key's handshake.

### FCC ID

The identifier the US Federal Communications Commission gives every radio transmitter sold in the United States,
printed on the remote: a grantee code (the maker) and a product code. Behind it is a public filing with test
reports and photos. See [FCC IDs](https://kat.karazajac.io/fcc.html).

### Model-year

One make, model and year, as a parts catalogue lists them: the unit [Coverage](https://kat.karazajac.io/coverage.html)
counts in.

## Files and tools

### .sub file

The Flipper Zero's file for a sub-GHz signal. A **RAW** file holds the levels and durations the radio delivered,
as signed microseconds; a **key file** holds one decoded frame and the protocol's name
([Flipper key files](flipper_key_files.md)). The [recordings](https://kat.karazajac.io/captures.html) of this site
are `.sub` files.

### .fob file

KAT's own file for a saved remote: the decoded frame and what KAT knows about the car
([Sessions and saved fobs](session.md)).

### Reference

A program whose decoder a KAT protocol follows and is checked against: ProtoPirate, the Flipper firmwares
Flipper-ARF, doctor-firmware, Momentum, RogueMaster and FoxFW, and rtl_433. [About this site](about.md) says what
"checked against" means.

### rtl_433

A program that decodes hundreds of devices from an RTL-SDR. KAT carries ports of its decoders for remotes no
Flipper firmware reads; they are receive only.
