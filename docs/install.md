---
layout: default
---

# Install and run

## Overview

KAT is a single program. Each [release](https://github.com/KaraZajac/KAT/releases/latest) has an archive for
64-bit Linux on Intel, AMD and ARM, for Apple Silicon Macs and for 64-bit Windows, each with a `.sha256` file
beside it; other platforms build from source. On Linux and macOS, KAT needs the HackRF library at runtime, even
if you only use an RTL-SDR or no radio at all. This page goes from the download to the first keys to press.

## The HackRF library

KAT needs it at runtime, even if you only use an RTL-SDR. Installing it brings libusb along.

```sh
sudo apt install libhackrf0     # Debian / Ubuntu / Raspberry Pi OS
sudo dnf install hackrf         # Fedora
sudo pacman -S hackrf           # Arch Linux
brew install hackrf             # macOS
```

On Windows there is nothing to install here: the `.zip` carries the HackRF library and libusb next to
`kat.exe`. The radio needs a WinUSB driver instead ([Radios](#radios), below).

## Download, check and unpack

Get the archive for your machine, and its `.sha256` file, from the
[latest release](https://github.com/KaraZajac/KAT/releases/latest):

| Archive | For |
|---|---|
| `kat-v1.27.1-x86_64-linux.tar.gz` | 64-bit Intel or AMD Linux with glibc 2.35 or newer (Ubuntu 22.04, Debian 12 and later) |
| `kat-v1.27.1-aarch64-linux.tar.gz` | 64-bit ARM Linux, such as 64-bit Raspberry Pi OS bookworm |
| `kat-v1.27.1-aarch64-macos.tar.gz` | Apple Silicon Macs. It isn't notarized; `xattr -d com.apple.quarantine kat` clears the block |
| `kat-v1.27.1-x86_64-windows.zip` | 64-bit Windows 10 and 11. It isn't signed, so SmartScreen may stop its first start: “More info”, then “Run anyway” |

Intel Macs and other platforms: [build from source](#build-from-source).

```sh
sha256sum -c kat-v1.27.1-x86_64-linux.tar.gz.sha256    # macOS: shasum -a 256 -c …
tar xzf kat-v1.27.1-x86_64-linux.tar.gz
./kat-v1.27.1-x86_64-linux/kat
```

On Windows, in PowerShell (Windows Terminal):

```powershell
$want = (Get-Content kat-v1.27.1-x86_64-windows.zip.sha256).Split(' ')[0]
(Get-FileHash kat-v1.27.1-x86_64-windows.zip).Hash -eq $want    # True
Expand-Archive kat-v1.27.1-x86_64-windows.zip .
.\kat-v1.27.1-x86_64-windows\kat.exe
```

## Run it

`kat` uses every HackRF, RTL-SDR, Flipper Zero and SoapySDR radio it finds, each on its own frequency, or runs with
files only.
Its settings, captures and log are in `~/.config/KAT` (on Windows `C:\Users\<you>\.config\KAT`).

| Key or command | What it does |
|---|---|
| `r` | Receive, on 433.92 MHz to start with |
| `:freq 315` | Change the frequency |
| `:hop on` | Hop the common fob frequencies |
| `Enter` | A capture's actions: transmit, export, analyze, key recovery |
| `a` | Analyze a capture, decoded or not: OOK or FSK, pulse widths, TE, coding, bits ([Analyze](analyze.md)) |
| `c` | Compare the presses of a remote: which bits are fixed, count or follow the button ([Compare](compare.md)) |
| `/` and `s` | Filter the list; sort it by a column |
| `Tab` | Radio settings |
| `:radios` | Your radios; `1` to `9` picks the one to tune, `R` starts them all |
| `:load` | Open `.sub` files and IQ recordings |
| `?` and `q` | Help; quit |

![KAT's main screen with fourteen decoded captures and the action menu open on capture 4, a Ford V0 Lock press: Replay, TX Lock, TX Unlock, TX Trunk, TX Panic, Export .fob, Export .sub (Flipper), Export Flipper key (.sub), Analyze and Delete Signal](img/kat-actions.webp#wide "`Enter` on a capture opens its actions (KAT 1.18): here a Ford V0 press, which KAT can re-encode, so it offers the four buttons as well as a replay. A protocol whose key KAT can recover adds an entry for that. Sending needs a HackRF or a Flipper Zero, and the owner's written permission.")

## Radios

- A **HackRF One** (or compatible) receives and transmits; an **RTL-SDR** only receives. A dongle may carry any of
  six tuners: the R820T, R820T2 or R828D of the dongles sold now (RTL-SDR Blog V3 and V4), or an older E4000,
  FC0012, FC0013 or FC2580; an FC2580 does not reach 315 or 433.92 MHz ([RTL-SDR tuners](radio.md#rtl-sdr-tuners)).
- **Other SDRs** receive through SoapySDR: an Airspy, SDRplay, LimeSDR, PlutoSDR, USRP, bladeRF or any radio with a
  SoapySDR module, when SoapySDR and that module are installed (Debian and Ubuntu package them, such as
  `sudo apt install soapysdr-module-airspy`; on Windows, PothosSDR with its `bin` folder on the `PATH`). KAT loads
  SoapySDR when it runs and finds those radios at startup and with `:radios`; without it, KAT runs as before
  ([SoapySDR radios](radio.md#soapysdr-radios)).
- A **Flipper Zero** on USB receives and transmits through its console, with no app to install: decoded signals
  only, FM reception with doctor-firmware ([Flipper as the radio](flipper_radio.md)). KAT opens it when you first
  receive or transmit with it: close qFlipper before that; on Linux, join the `dialout` group.
- **Several at once**: each radio has its own frequency, gains and hopping; one transmits while the others
  listen, and each capture says which radio heard it ([several radios](radio.md#several-radios)).
- Linux: for USB permission, install the udev rules from the hackrf or rtl-sdr package. The DVB-T driver can
  claim an RTL-SDR first: `sudo rmmod dvb_usb_rtl28xxu`, or blacklist it.
- Windows: libusb only opens a radio that has the WinUSB driver. Install it once with
  [Zadig](https://zadig.akeo.ie): Options → List All Devices, select the HackRF One (an RTL-SDR shows as
  “Bulk-In, Interface (Interface 0)”), WinUSB, Install Driver.
- The Linux binaries link libhackrf, libusb-1.0 (and libudev through it) and glibc; the Windows `.zip` carries
  libhackrf.dll, libusb-1.0.dll and libwinpthread-1.dll. No librtlsdr is needed: KAT drives the RTL-SDR itself.
  SoapySDR is never needed: KAT loads it only when it is installed.

## Build from source

Rust 1.88 or newer, and the HackRF library with its headers:

```sh
brew install hackrf                                          # macOS
sudo apt install libhackrf-dev pkg-config libusb-1.0-0-dev   # Debian / Ubuntu / Raspberry Pi OS
sudo dnf install hackrf-devel pkg-config libusb1-devel       # Fedora
sudo pacman -S hackrf libusb                                 # Arch Linux
pacman -S --needed mingw-w64-ucrt-x86_64-{toolchain,rust,pkgconf,libusb,hackrf}   # Windows: an MSYS2 UCRT64 shell
```

```sh
git clone https://github.com/KaraZajac/KAT.git
cd KAT
cargo build --release
```

The binary is `target/release/kat`. Use a release build for live receive: a debug build is about 20× slower.

## Decode files from the command line

`kat --decode` prints the decodes of files and whole folders without the terminal interface, for scripts and for
checking capture collections: Flipper `.sub` captures and key files, and IQ recordings from hackrf_transfer
(`.cs8`), rtl_sdr and rtl_433 (`.cu8`), a PortaPack's Capture app (`.C8`, `.C16`), or as `.cf32`. In the
terminal interface, `:load` opens the same files, and `:record` saves the raw IQ while receiving
([Receive chain and IQ files](radio.md)).

```sh
kat --decode captures/ fob.sub              # .sub files and IQ recordings (.cs8/.cu8/.cf32/.C8/.C16)
kat --decode gfile001.cu8 --rate 250k       # a file whose name carries no sample rate needs --rate
kat --iq fob.cs8 --freq 433.92 --rate 2M    # start the terminal interface with an IQ recording decoded
kat --keeloq-bf captures/ --save            # recover a magic-serial KeeLoq key
```

One line per frame: the file, the frequency, the kind of file, the protocol, how often the frame repeats, and
its fields. Six `.sub` files and three of rtl_433's test recordings:

```
$ kat --decode captures/ recordings/
captures/audi_a3.sub | 434420000 | key | VAG | x1 | data=C129194B7ABCFEC2/80 extra=8000000101032F1D sn=3BF8E083 btn=1 cnt=9C61 crc=1 tx=1
captures/doorhan.sub | 433920000 | key | KeeLoq (DoorHan) | x1 | data=4850F07233789514/64 sn=8A91ECC btn=2 cnt=4 crc=1 tx=1
captures/ford_focus_lock.sub | 433920000 | raw | Ford V0 | x3 | data=556C29917F3FF82F/64 extra=6DD2 sn=4306BE50 btn=1 cnt=877F crc=1 tx=1
captures/ford_focus_lock.sub | 433920000 | raw | Ford V0 | x4 | data=55C6833BD5950785/64 extra=6EF9 sn=4306BE50 btn=1 cnt=8780 crc=1 tx=1
captures/ford_focus_lock.sub | 433920000 | raw | Ford V0 | x4 | data=55C0853DD3938306/64 extra=6F90 sn=4306BE50 btn=1 cnt=8781 crc=1 tx=1
captures/hyundai_v4.sub | 433920000 | key | Kia V4 | x1 | data=C3B1790C8FC0C864/68 extra=0 sn=61303F1 btn=2 cnt=BDE9 crc=1 tx=1
captures/liftmaster.sub | 433920000 | key | Security+ 2.0 | x1 | data=3D290FB9BEEE/62 extra=3C016B19DD60 sn=4CB8D282 btn=68 cnt=1 crc=1 tx=1
captures/subaru_forester.sub | 433920000 | raw | Subaru | x1 | data=E2324AFE5BE7500A/64 sn=324AFE btn=2 cnt=32 crc=1 tx=1
recordings/audiovox_unlock_303.3M_250k.cu8 | 303300000 | am | Audiovox-PROOE3B | x13 | data=1A2ACA2/25 sn=2EA9 btn=2 cnt=- crc=0 tx=0
recordings/elero_869.4M_2048k.cu8 | 869400000 | fm | Elero | x1 | data=1B6A451005011184/240 extra=74E332C89E2FDFEE sn=84D351 btn=20 cnt=6A crc=1 tx=0
recordings/hcs361_lock_315.1M_250k.cu8 | 315100000 | am | Microchip-HCS361 [KeeLoq (Unknown)] | x4 | data=51D1AB6BDA280502/67 extra=5 sn=A0145B btn=4 cnt=- crc=1 tx=0
```

The last three lines are receive-only rtl_433 ports: Audiovox, Elero (FM, 869.4 MHz) and Microchip HCS361.
