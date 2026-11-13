---
layout: default
---

# A Flipper Zero as KAT's radio

**Rust module:** `src/radio/flipper.rs` (and `src/protocols/stream_locator.rs`)

KAT can use a Flipper Zero plugged in over USB as its radio, for receiving and transmitting. It
works through the Flipper's command line, the USB serial console that qFlipper also uses, so there
is no Flipper app to install.

## Setting it up

- Plug the Flipper in with a data cable. Only one program at a time can use its console: close
  qFlipper and any terminal open on it before you receive or transmit with it in KAT. Starting KAT
  does not open it; KAT connects the first time you press `r` (or `R`) or transmit with the
  Flipper, and keeps the console until it quits.
- On Linux, your user needs access to the serial port: `sudo usermod -aG dialout $USER`, then log
  in again.
- The Flipper needs an SD card: KAT writes each transmission to it before playing it.
- KAT uses every radio it finds (`[radio] device` in `config.ini`; `auto`, the default, takes all
  of them), so a Flipper works beside a HackRF or an RTL-SDR, each on its own frequency: select it
  with its number key (`1`–`9`) or `:radio flipper`, and make it the transmitter with
  `:txradio flipper` or `tx_radio = flipper` ([several radios](radio.md#several-radios)). Without a
  HackRF, a Flipper is the transmitter. `device = flipper` uses only Flippers.
- KAT finds the Flipper by its USB name ("Flipper Devices Inc."; on Linux
  `/dev/serial/by-id/usb-Flipper_Devices_Inc._*`). If it does not, give the port:
  `flipper_port = /dev/ttyACM0` (macOS `/dev/cu.usbmodemflip_<name>1`, Windows `COM5`).

The header shows "Flipper Zero" as the device and the receive preset in place of the SDR gains.
The `Tab` settings have its frequency only, and `:lna`, `:vga` and `:amp` answer "Flipper has no
gain settings (its receive preset sets them)".

## What works on which firmware

KAT uses two console commands, `subghz rx_raw` and `subghz tx_from_file`. Every firmware among
KAT's references has both:

| Firmware | Receive | FM receive | Transmit |
| --- | --- | --- | --- |
| Momentum, RogueMaster | OOK, 270 kHz filter | no | OOK and 2-FSK |
| Flipper-ARF, FoxFW | OOK, 650 kHz filter | no | OOK and 2-FSK |
| doctor-firmware | the preset you choose (`flipper_preset`) | yes | OOK and 2-FSK |

The official firmware is not among KAT's references; Momentum's two commands come from it. A
firmware with a region lock refuses to transmit outside its bands, and KAT shows its message.

`rx_raw` receives with the firmware's own AM preset. Only doctor-firmware's takes a preset
(`subghz rx_raw <Hz> <device> <preset>`), so FM (2-FSK) remotes can only be received there:
set `flipper_preset = FM476` (or `FM238`, `AM270`, `AM650`, …, as doctor-firmware names them).
KAT checks the firmware's usage text when it connects and refuses a preset the firmware would
ignore.

## Receiving

`subghz rx_raw <Hz>` streams the CC1101's demodulated output: `+350 -700 …` (HIGH and LOW
durations in µs; `.` where the Flipper's buffer overran) until Ctrl+C. Unlike an SDR's, this stream
never goes quiet: between transmissions the receiver's AGC turns up the noise, tens to thousands of
pulses a second (a Flipper with Momentum gave roughly 60 a second at 433.92 MHz and 650 at 315 MHz;
the Flipper recordings in `IMPORTS` made without a threshold show 3,000–7,000 a second).
There are no bursts to cut it into, so KAT reads it the way a Flipper's own Sub-GHz "Read" does:

1. **The decoders read the stream as it comes.** Every pulse goes once to the decoders flagged for
   the receive modulation that support the frequency, in registry order; the first to complete a
   frame wins and the decoders are reset, exactly as for a burst
   ([ProtocolRegistry::process_signal_stream](radio.md)).
2. **The frame is cut out of the noise.** The decoders read from their last reset, through the
   noise before the frame, so KAT looks for the shortest stretch of pulses ending at the frame that
   the decoder alone still reads (a binary search), then adds the regular pulses right before it:
   the rest of a preamble the decoder did not need (a pulse as long as the one two later, within
   20 %, which noise rarely is twice in a row).
3. **The app gets it as a burst**, with two more pulses, the frequency, AM (or FM) and its on-air
   time, and decodes, lists and de-duplicates it like an SDR's burst.

What this means in use:

- Only signals a protocol decodes are listed. With no carrier to cut bursts by, there are no
  "unknown" signals from a Flipper.
- A capture's pulses start where its frame (and its preamble) does, so Replay sends the frame and
  not the noise before it. A remote whose first frame cannot be read on its own (Princeton's
  first copy has no sync before it) starts at the first copy that can.
- There is no signal meter (the Flipper gives no level), no `:record` (it gives pulses, not IQ)
  and no hopping. Changing the frequency restarts `rx_raw`.
- With an FM preset, the stream is read in both polarities, as KAT reads FSK from an SDR.

## Transmitting

KAT builds the transmission exactly as it does for a HackRF (its own encoders, copies per press
and rolling-code advance), writes it as a RAW `.sub` to `/ext/apps_data/kat/tx.sub` (with
`storage write_chunk`, out of the Sub-GHz app's list of saved signals) and plays it once with
`subghz tx_from_file /ext/apps_data/kat/tx.sub 1 0`. Receiving pauses for it and resumes. It counts
as sent once the Flipper prints the line every firmware starts sending with ("Listening at …" and
"Press CTRL+C to stop"); without it KAT reports what the Flipper said instead. (Up to 1.13.0 any
answer without a known error counted as sent.)

The file's preset sets the modulation:

| Transmission | Preset in the file |
| --- | --- |
| OOK | `FuriHalSubGhzPresetOok650Async` (AM650) |
| 2-FSK at ±2.38 kHz or ±47.6 kHz | `FuriHalSubGhzPreset2FSKDev238Async` / `…Dev476Async` |
| 2-FSK at another preset's deviation (FM12K, FM20K, HONDA1, …) | that preset's register image |
| 2-FSK at any other deviation | FM476's register image with the nearest DEVIATN |

The transmit power is the Flipper's (its preset's PA table); `:txpower` does not apply. When the
Flipper refuses ("Frequency not supported", a region lock, no SD card), the status line shows its
words. The command line of current firmwares (official 1.x, Momentum, RogueMaster) also refuses
`subghz` while an app is open on the Flipper, and when it cannot load that command from the SD card;
KAT says so in words (see Troubleshooting).

## Troubleshooting

| Message | Meaning |
| --- | --- |
| `Flipper Zero: not found` | No Flipper on USB (or a charge-only cable). Set `flipper_port` if it has an unusual name. |
| `… is busy: qFlipper or another program has the Flipper's console` | Close qFlipper, a terminal or `scripts/flipper_replay.py`. |
| `… is not accessible` | Linux: the `dialout` group (above). |
| `no answer from its console` | Not a Flipper's console, or it hangs: unplug it and plug it in again. |
| `flipper_preset = …: this firmware's rx_raw takes no preset` | Remove the key, or use doctor-firmware. |
| `the Flipper did not transmit: …` | The Flipper's own reason (region, frequency, SD card, one of the two below), or `it did not start sending` when it gave no sign of sending. |
| `… an app is open on the Flipper; close it …` | Leave the app on the Flipper's screen: its console runs no Sub-GHz command while an app is open. |
| `… its subghz command did not load from the SD card …` | Current firmwares load `subghz` from `/ext/apps_data/cli/plugins` on the SD card, and refuse files built for another firmware version (after flashing the firmware alone, for instance). Install the firmware together with its SD card files: qFlipper, the firmware's web updater or its update package. |
| `Receiver stopped: Flipper: … (r restarts)` | The console closed (unplugged); `r` tries again. |
| `could not connect: …` in `:radios` | The last try to open the console failed (the reason follows); the next `r` or transmission tries again. |

## How it is tested

- On a real Flipper Zero (Momentum dev `d3f89dfe`, US region, 2026-11-13): KAT opens it on the first
  `r`, receives at 433.92 MHz and at 315 MHz (retuned while receiving), cuts a car key fob's presses
  out of the stream as whole frames (decoded, CRC valid; repeats inside a press are recognized),
  replays one (the file read back off the SD card decodes as the captured frame) and resumes
  receiving, and on quitting leaves the console free at once. Before that the same Flipper ran an
  Unleashed build whose SD card held console commands for another firmware version: `subghz` did
  not load, receiving failed with the console's words, and a transmission would have counted as
  sent (fixed: see Transmitting).

- A simulated Flipper on a pseudo-terminal pair answers KAT's commands as the firmwares do: a
  Princeton press in noise reaches the app, a transmission is written and played and receiving
  resumes, a region refusal comes back as the error, a receive preset goes to doctor-firmware
  only, and nothing is sent to the console before the Flipper is first used.
- The 127 AM Flipper recordings in `IMPORTS`, streamed pulse by pulse through the receive path,
  give exactly the 329 distinct frames KAT decodes from the files themselves: none missing, none
  extra (`cargo test --release imports_through_the_stream -- --ignored --nocapture`).
- Frames in synthetic noise come out where the transmission starts; Subaru's whole preamble is
  kept.
