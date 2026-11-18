# Custom Versio firmware: 9 beat-locked modes + TRIG DRUMS

Custom firmware for the Noise Engineering **Versio** (Daisy Seed): 8 beat-locked send effects plus a **trigger drum machine** in the last slot (SW0 right, SW1 right).

## Firmware (ready to flash)
| File | |
|---|---|
| `firmware/VersioBeatMidiSendTrigs.bin` | Normal |
| `firmware/VersioBeatMidiSendTrigsAlt.bin` | ALT: in TRIG DRUMS, hold the button + turn = momentary tweak that snaps back on release (DUB only when you hold without turning) |

Both run through the Daisy bootloader (USB MIDI build). See `VersioBeatMidiSendTrigs/README.md` for the full manual.

## TRIG DRUMS in short
- 8 voices, 8 trigger inputs (7 CV jacks + gate), USB MIDI notes and clock.
- **8 kits** on the switches (ANALOG, PUNCH, MINIMAL, RHYTHM BOX, DIGITAL, CRUNCH, DEEP, INDUSTRIAL). **SW0 left + SW1 left = SEQ**, the built-in patterns with the kit used last.
- **Both switches right = CTRL-ALL:** each knob moves its parameter on all 8 voices at once (relative, like Elektron).
- Tap = next voice, double-tap = audio in (THRU / SKIN / CHOP) or next pattern in SEQ, hold = DUB.

## Build
```
git clone --recursive https://github.com/mrarm84/customFWVersioNE9in1
cd customFWVersioNE9in1/libDaisy && make && cd ../DaisySP && make
cd ../VersioBeatMidiSendTrigs
make            # -> build/VersioBeatMidiSendTrigs.bin
make ALT=1      # -> build_alt/VersioBeatMidiSendTrigsAlt.bin
```
Needs the ARM GNU toolchain (`arm-none-eabi-gcc`). libDaisy and DaisySP are submodules pinned to the versions this was built with.

## Status
Builds clean. The drum sounds were checked as WAV renders of the real engine (`VersioBeatMidiSendTrigs/tools/drum_render.cpp`). Not yet tested on hardware.
