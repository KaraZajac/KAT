---
layout: default
---

# Compare

**Rust modules:** `src/analysis/compare.rs` (the comparison and its report); overlay:
`src/app/compare_view.rs`, `src/ui/compare_overlay.rs`  
**Reference:** none, this is KAT's own

Compare lines up the frames of several presses of one remote, bit under bit, and says what each bit
does from one press to the next: it stays fixed, it is part of a counter, it follows the button, or
it changes with no pattern. [Analyze](analyze.html) says how a signal is coded; Compare says what
its bits mean. It needs no decoder, so it is the next step with a signal KAT does not know: record a
few presses of each button, and the report shows where the serial, the counter and the button are
and how much of the frame is encrypted.

## Use

- `c` on a capture in the list compares the presses of its remote.
- Command: `:compare` does the same for the selected capture. `:compare 1-5` compares the captures
  named, in the order named (`:compare 3 1 2`); `:cmp` is short for it.
- `:compare 1-5 / 6-10` puts the captures in groups, here A and B, which take the place of the
  buttons. That is for a signal whose buttons KAT cannot know.

The report opens in an overlay. Up/Down, PgUp/PgDn and Home/End scroll, `Tab` changes between the
decoded data and the bits as sent, and Esc, Enter or `q` close. The selection in the list stays
where it was.

### Which captures

`c` takes the captures of the selected capture's remote that the list shows, in the order they
arrived:

- a decoded capture: the captures of the same protocol and serial;
- an unknown capture: the unknown captures on the same frequency, received the same way (AM or FM).

The list's filter (`/`) narrows this: with `unlock` as the filter, `c` compares the Unlock presses
alone. `:compare` with IDs takes any captures, also of different remotes, which shows what two
remotes of one kind have in common (the bits that are fixed in both). At most 64 captures are
compared.

### Buttons

A frame's button is the **Command** the capture was given (`i`), and without one what its decoder
read. So an unknown capture gets its button from you: name the button of each press as you record
it. Captures with neither have no button: the report then finds the fixed bits and the counter, and
says how to name the buttons. Groups given with `/` replace the buttons for that comparison.

### Decoded data and bits as sent

There are up to two comparisons, and `Tab` changes between them:

- **Decoded data**: the frame as the decoder keeps it (the `Key` of a Flipper key file), up to its
  first 64 bits. The frames of the protocol and length most of the captures have are compared; the
  report names the captures left out.
- **Bits as sent**: the rows of bits Analyze's trial decode reads from each capture's pulses. This
  is all an unknown capture has. For a decoded one it shows what the decoder leaves out: the
  preamble, a frame longer than 64 bits, the bits before they are inverted or put in order.

Which rows of the pulses are a capture's frames:

- A decoded capture gives its last row. The capture ends with the frame that decoded, and what
  comes before it can be left over from the press before.
- An unknown capture gives every different row it holds. A press that sends two different frames
  gives two, named `#26/1` and `#26/2`.
- Rows differ in length by a few bits, because the receiver catches the preamble at another bit
  each time. Rows within 8 bits of one length are one kind; the kind most captures have is
  compared. Each row is then moved by up to 8 bits to where it agrees best with the first, and the
  bits every row has are compared. The report says so: `The rows are 375 to 381 bits long: they
  are lined up by what they hold`.

A side with a single frame is left out when the other has frames to compare.

## The report

| Section | What it holds |
|---|---|
| SUMMARY | how many frames of how many bits, how many bits are fixed, the counters, the bits that follow the button, the bits that change; then what the frames cannot show and what to record next |
| FIELDS | the frame cut into fields: neighbouring bits that do the same thing, with their place, size and values |
| BITS | every frame's bits, one frame per row, with a letter under each column for what it does |

Bit 0 is the first bit sent. A field's value is read with its first bit as the high bit. The BITS
section shows 64 columns in a block where the overlay is wide enough and 32 otherwise; a block in
which nothing changes takes one line (`bits 0-31  fixed: 0x5A223837`).

### What the words mean

| In the report | Letter | The column of bits… |
|---|---|---|
| fixed | `.` | is the same in every frame: a serial, a preamble, a constant, or the high bits of a counter |
| counts up / down by N | `C` | is part of a field that, read as a number, goes up or down by a small step from each frame to the next |
| follows the button | `B` | is the same in every frame of one button and differs between two buttons |
| changes rarely | `r` | changes far less often than a coin toss would: a flag, a battery bit |
| changes with every frame | `e` | flips from each frame to the next: a toggle bit, or the low bit of a counter whose other bits are elsewhere |
| no pattern | `?` | changes, and nothing above fits: encrypted or hashed bits (a rolling code, a checksum) |

### How sure it is

Random bits look like a counter or like the button's bits now and then, and with few frames that
happens often. So every claim is weighed against how often chance alone would give it, over all
the places in the frame where it was looked for, and the report has two levels:

- **counts** / **may count.** A counter is looked for in every run of 2 to 24 neighbouring columns
  that change, with the high bit first and with the low bit first, counting up and down, in steps
  of up to 15. "Counts" means random bits would pass as well in about one comparison
  in five hundred, "may count" in about one in sixteen. Five presses of one button are enough for
  "counts" in a frame that is otherwise fixed. Beside 32 encrypted bits, five give "may count" and
  seven "counts": there are more places for chance to look like a counter.
- **follow the button** / **may follow the button.** Each further frame of a button that agrees
  with the others of that button halves the chance. "Follow" stands for about one in 32, "may
  follow" for one in four. With too few frames the report says how many would do (`about 4
  different ones of each would do`).

A counter that skips is still a counter: `counts up by 1, 2 jumps` is a field whose steps fit but
for two, which are presses that were not captured or frames out of order.

## Example

Twelve presses of a made-up remote, six of Lock and then six of Unlock. Its frame has a 28-bit
serial, a 2-bit button, a 10-bit counter and 24 encrypted bits:

```
 Some Protocol V2  serial 0A5C3F0  433.92 MHz  Captures #01-#12
 Tab: the bits as sent

 SUMMARY
 Compared     12 frames of 64 bits: the decoded data of Some Protocol V2
 Fixed        34 of 64 bits
 Counter      bits 36-39 count up by 1
 Button       bits 28-29 follow the button (Lock, Unlock)
 Changing     24 bits with no pattern
 Bits with no pattern are encrypted or hashed (a rolling code, a checksum),
 or follow something these frames do not show.

 FIELDS   bit 0 is sent first, values high bit first
 Bits   Size  What they do
 0-27     28  fixed: 0xA5C3F01
 28-29     2  follows the button: Lock 0x1, Unlock 0x2
 30-35     6  fixed: 0x15
 36-39     4  counts up by 1: 0x0 to 0xB
 40-63    24  no pattern: 12 values in 12 frames
 A counter's high bits do not change in a few presses: they are among the
 fixed bits beside it.

 BITS   . fixed  C counter  B button  ? no pattern
 bits 0-31
 #01 Lock   10100101 11000011 11110000 00010101
 #02 Lock   10100101 11000011 11110000 00010101
 #03 Lock   10100101 11000011 11110000 00010101
 #04 Lock   10100101 11000011 11110000 00010101
 #05 Lock   10100101 11000011 11110000 00010101
 #06 Lock   10100101 11000011 11110000 00010101
 #07 Unlock 10100101 11000011 11110000 00011001
 #08 Unlock 10100101 11000011 11110000 00011001
 #09 Unlock 10100101 11000011 11110000 00011001
 #10 Unlock 10100101 11000011 11110000 00011001
 #11 Unlock 10100101 11000011 11110000 00011001
 #12 Unlock 10100101 11000011 11110000 00011001
            ........ ........ ........ ....BB..

 bits 32-63
 #01 Lock   01010000 00100100 10110110 00111010
 #02 Lock   01010001 10011010 01110100 10101011
 #03 Lock   01010010 11100001 10110011 10101100
 #04 Lock   01010011 00010111 01000110 00100110
 #05 Lock   01010100 10101101 10111011 10101111
 #06 Lock   01010101 11010111 01010001 00100011
 #07 Unlock 01010110 10010100 01010000 00011010
 #08 Unlock 01010111 01110101 00011010 01110001
 #09 Unlock 01011000 01010111 00111111 01101100
 #10 Unlock 01011001 11101010 01110001 10010001
 #11 Unlock 01011010 11010010 11101010 01011101
 #12 Unlock 01011011 10110100 11001111 00110001
            ....CCCC ???????? ???????? ????????
```

What the report got right without knowing the layout: the serial, the button's two bits with the
code of each button, the counter and the encrypted part. What it could not see is the counter's
size. Twelve presses change only its four low bits, so its six high bits are the fixed `0x15`
beside it, and the note under the fields says so.

## Checked against decoders

The tests compare recordings in `IMPORTS/` and check the result against what KAT's decoders read
from the same captures. The comparison is given the buttons' names and nothing of the layout.

| Recording | Presses | What Compare finds |
|---|---|---|
| Kia V1 (`KIA/KiaV1_5cl_5op.sub`) | 5 Lock, 5 Unlock | the counter and the button's bits, on both sides; the four bits left over are the frame's checksum |
| Kia V3 (`KIA/Kia3_5cl_5op_5tr.sub`) | 15, read as two buttons | the first 32 bits have no pattern and no counter (the KeeLoq part, where the counter is encrypted); the button's bits beside them |
| Suzuki V0 (`SUZUKI/Suzuki_v0_kd.sub`) | 5 each of Lock, Unlock, Trunk | the counter and the button's bits, also in rows of 375 to 381 bits lined up by what they hold |

"The counter" means: one counter is found, and from frame to frame it counts as the decoder's
counter does. "The button's bits" means: they are equal for two frames exactly when the decoder
reads the same button for both.

## Limits

- The frames are compared in the order given, which for `c` is the order the captures arrived in.
  A counter shows only when the captures are in the order of the presses.
- A counter's size is not known from a few presses: only the bits that changed are called a
  counter. Fields whose bits are not neighbours, and steps larger than 15, are not found.
- A checksum over a frame with a counter can count too. Five Unlock presses of the Kia V1
  recording give `may count down by 1` for three bits of its checksum. More presses, or a second
  button, end that.
- What is encrypted shows no pattern, whatever is in it. A rolling code's counter and a button
  that is part of the encrypted block cannot be seen this way; the report says where such a block
  is and how long.
- The decoded data is at most the frame's first 64 bits, which is what KAT keeps as a frame's
  data. The bits as sent have the whole frame.
- The bits as sent are read by the trial decode's convention, as in Analyze. Where that is the
  inverse of the protocol's, the fields are in the same places, but a counter counts down.
- Two frames tell only which bits differ. The report says what to record next: five or more
  presses of one button for a counter, then a few of another for the button's bits.
- The comparison is not saved. It is worked out again from the captures each time.
