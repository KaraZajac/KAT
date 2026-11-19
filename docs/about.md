---
layout: default
---

# About this site

What this site is, where its facts come from, and how sure each kind of statement is.

## Overview

kat.karazajac.io is two things. It is the manual of KAT, the Keyfob Analysis Toolkit: a terminal program that
receives, decodes and sends the signals of car key fobs and of gate, garage and home remotes. And it is a
reference work on those signals: what each kind of remote sends, bit by bit, with the recordings and the public
filings that show it.

The two are one because the program is the reference. A protocol's page describes the decoder KAT runs; the
box of facts beside it is written out by KAT itself; the recordings listed under it are the ones KAT's tests
read on every change.

## What is on a page

| Part | Where it comes from |
|---|---|
| The text | Written by hand, kept beside the code in the repository's [`docs`](https://github.com/KaraZajac/KAT/tree/main/docs) folder. The foot of every page links its file |
| The box of facts | The program: its name for the protocol, the line coding and modulation it expects, the pulse widths and frame length its decoder is built around, whether it can also send. `docs/data/protocols.json`, written by a test in KAT's own source |
| Recordings | The [library](https://kat.karazajac.io/captures.html): Flipper `.sub` files of real remotes in the repository's `IMPORTS` folder. What KAT reads from each is pinned in two lists that the tests compare on every change |
| Example frames | The program again: frames it reads from the recordings, with the digits that hold the serial, the button and the counter marked. A field is marked only when the same bits give KAT's value in every frame of the protocol in the library (`docs/data/frames.json`); for a few protocols the whole frame is mapped by hand |
| FCC IDs | The remotes' public filings at the US Federal Communications Commission, read one by one, and parts catalogues for the cars each remote fits ([FCC IDs](https://kat.karazajac.io/fcc.html)) |
| Remotes | A page per remote ([Remotes](https://kat.karazajac.io/remotes.html)): pictures of the remote and its circuit board, notes on what its filing says, the chips on it ([Chips](https://kat.karazajac.io/chips.html)) |

## The references

Most of KAT's decoders are ports: they follow the decoder of an existing open program line for line, and the
page says which.

- [ProtoPirate](https://protopirate.net/ProtoPirate/ProtoPirate), the Flipper Zero application for car remotes,
  comes first where it has the protocol.
- The Flipper firmwares [Flipper-ARF](https://github.com/D4C1-Labs/Flipper-ARF),
  [doctor-firmware](https://github.com/thedoctorz/doctor-firmware),
  [Momentum](https://github.com/Next-Flip/Momentum-Firmware),
  [RogueMaster](https://github.com/RogueMaster/flipperzero-firmware-wPlugins) and FoxFW, for the protocols
  ProtoPirate does not carry and for the gate and garage remotes.
- [rtl_433](https://github.com/merbanan/rtl_433), for remotes no Flipper firmware reads. These are receive
  only.

A few decoders are KAT's own, worked out from recordings where no reference reads the signal:
[Honda V6](honda_v6.md), [Kia Mobis](kia_mobis.md), [Kia Denso](kia_denso.md), [Kia Forte](kia_forte.md),
[GM RFA](gm_rfa.md), [Isuzu Rolling](isuzu_rolling.md), [Isuzu Static](isuzu_static.md),
[Suzuki TS002](suzuki_ts002.md), [Opel Rolling](opel_rolling.md), [Renault 1-Button](renault_one_button.md),
[PSA 2-Button](psa_two_button.md) and [VW-1](vw_1.md).
Their pages say
what each field is known from, and what is not known.

## How sure a statement is

**Checked against the reference.** A ported decoder is run beside the reference's own C code, built for a
desktop computer, on the same recordings; the two have to list the same frames. Where KAT differs on purpose,
the page has a section on the deviation and the recordings that justify it.

**Read from recordings.** A statement like "the counter goes up by one with every press" is made from the
recordings listed on the page. When it rests on one remote, the page says so: a field that is constant in one
remote's recordings may be the remote's ID or part of the protocol, and one remote cannot tell.

**No checksum.** Some frames carry no check KAT knows. Their decoders accept a frame on its shape alone: the
lead, the sync, the exact length, the burst ending where the frame ends, sometimes the same frame twice. The
page lists that gate, and such a protocol sits behind every protocol with a real check in the decoder list.

**FCC ties.** A filing is tied to a protocol by a recording of that remote, by the protocol's own source naming
the FCC ID, or by the filing's figures matching the protocol. The last is a judgement and is marked as one
([FCC IDs](https://kat.karazajac.io/fcc.html)).

**Pictures.** The pictures of a remote with an FCC ID are from the internal photos of its filing, a public
record; each names its exhibit and opens it. The pictures of a remote without one were sent in by its owner,
with the recordings.

**Notes on a filing.** A remote's page quotes its filing: the frequency, the modulation, pulse widths, the
parts it names. The notes were taken by reading the exhibits. Where every quotation in a note is found again in
the text of the exhibits, the note is marked *quoted*; many filings are scans, and their notes cannot be
checked that way. The exhibit is one click away.

**Chips.** A chip is listed for a remote when a picture of the board shows its marking or the filing names it.
A marking that is a customer's own number says no more than that.

**Not known.** The pages say "not known" where that is the case. A cipher that has not been identified, a
button nobody pressed in a recording, a remote whose FCC ID nobody read off its back.

## What is not here

- The keyless handshake of smart keys and the immobiliser. The site covers what a button press sends.
- Remotes without a recording or a reference decoder. [Coverage](https://kat.karazajac.io/coverage.html) counts
  what that leaves out for North America and lists the remotes a recording is wanted of.
- Counts for other markets. Many of the protocols and recordings are of European, Asian and Australian cars,
  but only North America has public filings and catalogues to count against.

## Adding to it

A recording of a remote KAT does not read is the most useful thing to add
([Recording a remote](recording.md)); so is the FCC ID of a remote in the library whose ID is not known, and a
correction to anything a page says. The repository's
[issues](https://github.com/KaraZajac/KAT/issues) take all three.

## Licence and use

KAT is released under the BSD 3-Clause licence. Use the program and what this site describes only on systems
and vehicles you own or have explicit, written permission to test
([the full notice](https://kat.karazajac.io/#legal)).
