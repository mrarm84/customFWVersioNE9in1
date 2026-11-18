# VersionBeatMidiSendTrigs

This is **VersioBeatSend** with a **trigger drum machine** in the last slot (SW0 right, SW1 right), in place of the Beat Machine. The other 8 modes are the same as in VersioBeatSend (see its README).

Firmware: `build/VersionBeatMidiSendTrigs.bin` (bootloader build, flashed like VersioBeatSend). Choose the mode as usual: set both switches right, hold the button 4 s, then press once. After that, the switches have new jobs (see below).

## TRIG DRUMS — 8 voices, 8 trigger inputs

| Input | Voice |
|---|---|
| jack 1 (under K0) | **BD** kick |
| jack 2 (K1) | **SD** snare |
| jack 3 (K2) | **OH** open hat (choked by CH) |
| jack 4 (K3) | **CH** closed hat; its triggers also **clock** the pattern |
| jack 5 (K4) | **CP** clap |
| jack 6 (K5) | **MULTI**: tom / conga / rim / cowbell |
| jack 7 (K6) | **SYNTH A**: dub-techno **chord stab** |
| gate | **SYNTH B**: **resonator chord** (ringing combs) |

A fast jump on a jack (trigger / gate, more than 15 % of the range within ~1 ms) plays that jack's voice. Its height is the velocity. Keep a knob below about 85 % if 5 V triggers go into its jack: the knob and the jack are summed in hardware, so a trigger on top of a fully-up knob can't be seen.

### Knobs: all 7 edit ONE voice
**Tap** the button to step to the next voice: BD → SD → OH → CH → CP → MULTI → SYNTH A → SYNTH B → BD … **LED 3** shows which voice you are editing, and flashes when that voice plays.

| LED 3 | Voice |
|---|---|
| red | BD |
| orange | SD |
| yellow | OH |
| pale yellow | CH |
| cyan | CP |
| green | MULTI |
| violet | SYNTH A |
| blue | SYNTH B |

Every voice has the same layout:

| K0 | K1 | K2 | K3 | K4 | K5 | K6 |
|---|---|---|---|---|---|---|
| TUNE | DECAY | CHARACTER A | CHARACTER B | **FILTER** | **DRIVE** | **SEND** |

The last three knobs each do two things, one leading into the other:
- **K4 FILTER:** at noon the filter is open. **CCW** is a **low-pass** that closes down to ~70 Hz; **CW** is a **high-pass** that opens up to ~11 kHz. In both directions the **resonance** rises as you turn away from noon, peaks halfway and eases off at the ends, so it stays musical and never screams.
- **K5 DRIVE:** fully CCW is clean. The first half is **warm saturation**; the second half turns into a **folding distortion**. Level-matched, so it gets dirtier, not louder.
- **K6 SEND:** the first ¾ adds **reverb**; the last ¼ adds the **echo** on top.

Every hit is **panned at random a little around the centre**; the kick stays in the middle. There is no level knob: each voice is balanced, and velocity sets how hard it hits.

| Voice | TUNE | CHARACTER A | CHARACTER B |
|---|---|---|---|
| BD | 36–94 Hz | click | drive |
| SD | 160–336 Hz | snappy | noise tail |
| OH / CH | metal tone | brightness | noise |
| CP | tone | burst spread | width (band) |
| MULTI | sweeps **tom → conga → rim → cowbell**, each with its own pitch range | pitch bend | noise / slap |
| SYNTH A (stab) | chord root C2–C7 | **CHORD** | **FILTER**: dark and soft → bright and resonant |
| SYNTH B (resonator) | chord root C2–C7 | **CHORD** | **BRIGHTNESS** |

**Chords (CHARACTER A), mostly minor, left to right:** m · m7 · m9 · m11 · m7sus4 · m(add9) · maj7 · open fifths.
- **SYNTH A (stab):** two detuned saws per chord note through a resonant low-pass. Its filter envelope opens more on accents.
- **SYNTH B (resonator):** a short noise burst rings one comb resonator per chord note. **DECAY = ring time**, 0.2–6 s.
- **Defaults:** A minor, a m9 stab (A3) and a m7 resonator (A2), both already sent to the echo and reverb.
- **Pitch:** TUNE follows a slow CV (1 V/oct-ish). USB MIDI channel 2 / 3 notes set the root.

### CTRL-ALL (both switches right)
With **SW0 right and SW1 right**, the knobs stop editing one voice and work like **Elektron's control-all**: turning a knob moves that parameter on **all 8 voices at once** (K0 tunes the whole kit, K1 shortens or lengthens every decay, K6 sends everything to the reverb / echo, and so on).
- It is **relative**: each voice moves by the amount you turn, so the voices keep their differences. A voice that reaches the end of its range stops there.
- **K0 TUNE moves the drums only a little** (a quarter of the turn) and **never the synths' pitch**: the chord roots of SYNTH A / B stay where they are.
- Like a normal page, a knob does nothing until you turn it, so entering CTRL-ALL changes nothing by itself.
- **LED 3 is white** in CTRL-ALL and flashes on any hit.
- This position also plays the **INDUSTRIAL** kit (see Kits). Taps still step the voice, which you'll edit again when you move a switch away.

- **Jump on move:** after you change voice, a parameter keeps its value until you turn that knob.
- **Triggers are never affected by which voice you're editing:** all 8 jacks keep playing their own voices.
- **A slow CV on a jack** (an LFO, say) moves that knob's parameter of the voice you are editing.

### Echo + reverb (the send)
The reverb takes each voice's SEND, plus the echo. The **echo** is a tempo-synced ping-pong at a dotted 8th of the beat, darkened and thinned in its feedback. It gets the top quarter of SEND, plus DUB. SYNTH A / B default to a send in the echo range: dub techno lives in the echo.

### DUB (hold the button)
**Hold** the button and DUB fades in over 0.15 s. It is momentary, and it starts after 0.2 s, so taps only change the voice.
- Every decay gets **20 % longer**.
- All voices go into the **dub echo and reverb**. The echo feedback and the reverb tail both grow.
- **Release** and it fades out over 1.5 s, so the reverb rings on.
- LEDs 0–2 turn blue while you dub.
- A hold longer than 4 s still arms a mode change, as in every mode. If you don't confirm, it is dropped after 2 s.

### ALT firmware: hold + turn = momentary tweak
`build_alt/VersionBeatMidiSendTrigsAlt.bin` (`make ALT=1`, flash with `make ALT=1 program-dfu`). Everything is the same, plus:
- **Hold the button and turn knobs:** the knobs move their parameters on the voice you are editing, or on **all 8** in CTRL-ALL. Every parameter, including the FX (FILTER, DRIVE, SEND), moves by the amount you turn.
- **Release the button:** every parameter **snaps back** to where it was when you pressed. Knobs then change nothing until you turn them again.
- **DUB only comes in if you hold without turning.** As soon as a knob turns during the hold, that hold is just the tweak: no DUB (if DUB had already started, it cuts out in 0.15 s).
- Like CTRL-ALL, the tweak never moves the synths' pitch, and moves drum TUNE only a little.
- The normal `build/VersionBeatMidiSendTrigs.bin` doesn't have this.

### Kits (the switches inside the mode)
Each switch position is a different **drum machine**: the same 8 voices and knobs, with a different character (pitch drop, click, decays, noise, hat metal, bus drive, bit depth). Left / left is the **SEQ**.

| SW0 ↓ / SW1 → | left | mid | right |
|---|---|---|---|
| **left** | **SEQ** (pattern) | **ANALOG**: 808-ish, round, long pitch drop | **PUNCH**: 909-ish, clicky kick, noisy hats, bus drive |
| **mid** | **MINIMAL**: 606-ish, short, high, thin, bright tight hats | **RHYTHM BOX**: CR-78-ish soft blips, no click, metallic hats | **DIGITAL**: early sample drums, hard click, 12 bit / 28 kHz |
| **right** | **CRUNCH**: gritty sampler, fat and driven, 10 bit / 22 kHz | **DEEP**: dub, low, long, dark | **INDUSTRIAL**: huge pitch drop, noise, heavy drive, 8 bit / 16 kHz, **+ CTRL-ALL** |

- The kits only change the drums. SYNTH A / B sound the same in every kit.
- The kits are balanced to play about as loud as each other.

**Double-tap:**
- **In a kit:** next **audio in** mode, THRU → SKIN → CHOP → THRU (THRU at power-up):

| Audio in | |
|---|---|
| **THRU** | The input passes at 50 %, drums on top |
| **SKIN** | The input **becomes the noise** of snare, clap, hats and multi. The kick ducks the input |
| **CHOP** | The input is heard **only through the hits**: BD opens the lows, SD the mids, hats the highs, CP its band, MULTI / SYNTH A the full input |

- **In SEQ:** next pattern. HOUSE · TECHNO · BOOM BAP · BREAK · ELECTRO · PERC · **DUB TECHNO** (four-on-the-floor, soft off-beat hats, an off-beat chord stab and one ringing resonator chord per bar).

### SEQ (SW0 left, SW1 left)
The built-in pattern plays, on top of the triggers, with **the kit you used last**. A kit only counts as "used" once its position has been held for **0.5 s**, so the positions you pass while moving the switches to left / left don't count. The audio-in mode stays as you set it in the kits.
- **Tempo,** in order: USB MIDI clock or **closed-hat triggers on jack 4**; the beat tracked from audio in; otherwise the last tempo (120 BPM at power-up).
- **Patched voices take over:** a voice that gets external triggers (jack or MIDI) drops out of the pattern for 4 s, so one patched trigger replaces just that part.
- **Closed hat as clock:** a steady stream of quarters, 8ths or 16ths. The division is automatic, picked to land in 75–150 BPM. Hits are counted, so swing doesn't upset it. It settles after about 4 hits, and the first hit after 2 s of silence is beat 1.

### USB MIDI in
- **GM drum notes:** 35/36 BD · 38/40 SD · 42/44 CH · 46 OH · 39 CP · 37, 41, 43, 45, 47, 48, 50, 56 MULTI. Velocity is used.
- **Channel 2 / 3** notes play SYNTH A / SYNTH B at the note's pitch.
- **MIDI clock** (24 ppq) sets the tempo; Start resets to beat 1.

## Listening on a computer
`tools/drum_render.cpp` compiles the **real drum engine** (the same `common/TrigDrumsEngine.h` the firmware uses) for the PC. `tools/host/` holds two tiny stand-ins for the Daisy headers. It renders WAVs to `renders/` in a few seconds:
- every voice on each of the 8 kits, the same groove on every kit (`groove_kit*.wav`), and all 7 patterns
- DUB TECHNO with a dub throw
- every chord on SYNTH A / B
- sweeps of FILTER, DRIVE and SEND

```
cd tools
g++ -O2 -std=gnu++17 -I host -I ../../common drum_render.cpp -o drum_render.exe
drum_render.exe ../renders
```
g++ comes from MinGW-w64 (winget: `BrechtSanders.WinLibs.POSIX.UCRT`). Open a new terminal after installing it, so it is on PATH.

## Build
```
make
```

## Status
Builds clean. The drum sounds were checked as WAV renders of the real engine code (tools/drum_render.cpp). **Not yet tested on hardware:** trigger detection with real triggers and knob positions, and MIDI input.
