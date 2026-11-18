// VersionBeatMidiSendTrigs - VersioBeatSend with a trigger drum machine in
// the last slot (SW0 right, SW1 right): TRIG DRUMS, see TrigDrumsEngine.h.
//   jacks 1..7 (the CV jacks under K0..K6) + gate = triggers for 8 voices:
//   BD SD OH CH CP MULTI FM-SYNTH PLUCK-SYNTH. A fast jump on a jack plays
//   its voice; the knobs edit ONE voice at a time (tap = next voice):
//   K0 TUNE K1 DECAY K2/K3 CHARACTER K4 PAN K5 LEVEL K6 REVERB SEND.
//   Hold the button = DUB (momentary: decays +20 %, all into the reverb).
//   SW0 x SW1 = 8 KITS (drum machines); left / left = SEQ: the built-in
//   pattern with the kit used last (double-tap = next pattern). In a kit
//   position double-tap = audio in (THRU / SKIN / CHOP). Both right =
//   INDUSTRIAL kit + CTRL-ALL (the knobs move all 8 voices, relative).
//   USB MIDI clock and the closed-hat triggers on jack 4 clock the SEQ.
// ---------------------------------------------------------------------------
// Based on:
// VersioBeatSend - nine beat-locked send effects for Noise Engineering Versio
// ---------------------------------------------------------------------------
// MODES (Switch 0 = row, Switch 1 = column), selected with a 4 s hold:
//                 SW1 left        SW1 mid          SW1 right
//   SW0 left      DATA BENDER     TAPE MANGLER     BEAT SLICER
//   SW0 mid       RESONATOR       BEAT REPEATER    BEAT SATURATOR
//   SW0 right     GRANULAR CLOUD  WAVETABLE SYNTH  TRIG DRUMS
// Inside a mode the switches pick one of 9 SUBMODES (see README).
//
// SEND BUS (every mode)
//   in ------------------------------------------------ x (1 - MIX) --+--> out
//    \-- x SEND (layer 1 K0) -> LPF (layer 1 K3) -> mode (100 % wet) x MIX
//   MIX = DRY/WET (hold the button + turn K0, layer 2). It starts at 100 % wet on
//   every power-up, so by default no dry signal is heard. The SEND knob
//   sets how much of the input goes INTO the effect (CCW = nothing, CW =
//   full); tails keep ringing after the send is closed.
//
// BEAT (every mode): gate clock if patched (1 pulse = 1 beat), otherwise
//   the beat is tracked from the audio input (BeatTracker.h). The beat is
//   the clock of every mode.
//
// KNOB LAYERS
//   layer 1  normal                     (K0 = SEND, K3 = LPF in every mode;
//            K2 = the mode's K0 parameter)
//   layer 2  hold the button (+ turn)   (LED 3 violet; K0 = DRY/WET, K2 =
//            the mode's layer-1 K3 parameter; the mode's own layer-2 K0
//            parameter stays at its default)
//   The old layer-3 output strip (FILTER, WIDTH, DRIVE, CRUSH, GATE, SPACE)
//   is no longer reachable and stays neutral.
//   No soft takeover: after a layer change a knob keeps its stored value
//   until you move it, then the parameter jumps to the knob.
//
// BUTTON
//   tap                      MAIN MODE on / off (LED 3 cyan): opens the
//                            send LPF and lets every mode knob wander
//                            +-5 % at random; fades out over 1.5 s
//   hold                     MAIN MODE while held (fades out on release)
//                            + the knobs edit layer 2
//   double-tap               the mode's main toggle (freeze / infinite / ...)
//   hold 4 s                 arm a mode change (without turning); press
//                            again within 2 s of releasing to confirm. A
//                            transition effect carries you into the mode.
// GATE in = beat clock in every mode (overrides the audio beat tracking).
//
// MODE CHANGE: hold 4 s (LEDs pulse amber), then ONE press confirms. No
// stacking: one mode runs at a time.
//
// USB MIDI (USB_MIDI=1 build, the default): the Seed's USB port is a class-
// compliant MIDI device ("Daisy Seed Built In"). Independently of the mode
// and the knob layers it sends, on MIDI channel 1:
//   CC 20..26  = jack 0..6 (knob + CV, the value the ADC sees), 14-bit:
//                MSB on CC 20..26, LSB on CC 52..58
//   CC 27, 28  = audio in L, R level (envelope follower, 5 ms attack,
//                150 ms release, 0 = silence, 127 = full scale), 14-bit:
//                LSB on CC 59, 60
//   CC 29      = CPU load of the audio callback (peak over 250 ms),
//                14-bit: LSB on CC 61
//   note 60    = gate in (note on at the rising edge, note off at the fall)
// All seven CCs are resent once a second so a host that connects late
// (VCV Rack MIDI>CV-CC) picks up the current values.
// ---------------------------------------------------------------------------

#include "daisy_versio.h"
#include "util/CpuLoadMeter.h"
#include "Panel.h"
#include "BeatTracker.h"
#include "DataBenderEngine.h"
#include "SlicerEngine.h"
#include "ResonatorEngine.h"
#include "TapeEngine.h"
#include "RepeaterEngine.h"
#include "SaturatorEngine.h"
#include "CloudEngine.h"
#include "WaveEngine.h"
#include "TrigDrumsEngine.h"
#include "Strip.h"
#include "Transition.h"

using namespace daisy;

DaisyVersio hw;

#ifndef USB_MIDI
#define USB_MIDI 1
#endif

#if USB_MIDI
MidiUsbHandler midi;

namespace usbmidi
{
static constexpr uint8_t  kChannel  = 0;  // MIDI channel 1
static constexpr uint8_t  kCcBase   = 20; // jack k -> CC 20 + k (MSB)
static constexpr uint8_t  kLsbOff   = 32; // LSB on CC 52 + k
static constexpr uint8_t  kGateNote = 60;
static constexpr uint32_t kPeriodMs = 2;    // scan rate (500 Hz)
static constexpr uint32_t kResendMs = 1000; // full refresh
static constexpr int      kHyst     = 6;    // 14-bit steps of noise to ignore
static constexpr int      kValues   = DaisyVersio::KNOB_LAST + 3; // + in L/R, CPU

int      sent[kValues];
bool     gateSent = false;
uint32_t lastScan = 0, lastFull = 0;

static void Send3(uint8_t a, uint8_t b, uint8_t c)
{
    uint8_t msg[3] = {a, b, c};
    midi.SendMessage(msg, 3);
}

static void SendCc(int k, int v14)
{
    Send3(0xB0 | kChannel, kCcBase + k, (v14 >> 7) & 0x7F);
    Send3(0xB0 | kChannel, kCcBase + kLsbOff + k, v14 & 0x7F);
}

static void Init()
{
    MidiUsbHandler::Config cfg;
    cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
    // no retries: with no host attached a send just fails instead of
    // stalling the UI loop
    cfg.transport_config.tx_retry_count = 0;
    midi.Init(cfg);
    for(int k = 0; k < kValues; k++)
        sent[k] = -1000;
}

// main loop: vals = 7 smoothed jack values + in L/R level, all 0..1,
// gate = gate in
static void Update(uint32_t now, const float* vals, bool gate)
{
    if(now - lastScan < kPeriodMs)
        return;
    lastScan        = now;
    const bool full = now - lastFull >= kResendMs;
    if(full)
        lastFull = now;

    if(gate != gateSent)
    {
        gateSent = gate;
        Send3((gate ? 0x90 : 0x80) | kChannel, kGateNote, gate ? 127 : 0);
    }

    for(int k = 0; k < kValues; k++)
    {
        int v = (int)(vals[k] * 16383.f + 0.5f);
        v     = v < 0 ? 0 : (v > 16383 ? 16383 : v);
        // ends always reach 0 / 16383 despite the hysteresis
        const bool edge = (v == 0 || v == 16383) && v != sent[k];
        if(full || edge || abs(v - sent[k]) > kHyst)
        {
            sent[k] = v;
            SendCc(k, v);
        }
    }
}
} // namespace usbmidi
#endif

// --------------------------------------------------------- mode handling ---
enum Mode : uint8_t
{
    MODE_BENDER,
    MODE_TAPE,
    MODE_SLICER,
    MODE_RESO,
    MODE_REPEAT,
    MODE_SAT,
    MODE_CLOUD,
    MODE_WAVE,
    MODE_DRUMS,
    MODE_COUNT,
    MODE_EMPTY = 0xFF,
};

// slot = Sw(0) * 3 + Sw(1); positions 0 = left, 1 = mid, 2 = right
static const uint8_t kSlotMode[9] = {
    //            SW1: left          mid           right
    /* SW0 left  */ MODE_BENDER,  MODE_TAPE,    MODE_SLICER,
    /* SW0 mid   */ MODE_RESO,    MODE_REPEAT,  MODE_SAT,
    /* SW0 right */ MODE_CLOUD,   MODE_WAVE,    MODE_DRUMS,
};

// transition into each mode (see Transition.h)
static const trn::Recipe kTrans[MODE_COUNT] = {
    // stutMs stutEnd rateDip crush lp    hp    ring0 ring1  echoMs echoFb scatter
    {80.f, 0.25f, 0.f, 0.6f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, false},  // BENDER: crushed glitch roll
    {0.f, 0.f, 1.f, 0.f, 0.4f, 0.f, 0.f, 0.f, 0.f, 0.f, false},     // TAPE: tape stop -> start
    {125.f, 0.12f, 0.f, 0.f, 0.f, 0.6f, 0.f, 0.f, 0.f, 0.f, false}, // SLICER: roll + HP rise
    {0.f, 0.f, 0.f, 0.f, 0.3f, 0.f, 0.f, 0.f, 4.5f, 0.93f, false},  // RESO: metallic comb ring
    {100.f, 0.06f, 0.f, 0.f, 0.2f, 0.f, 0.f, 0.f, 0.f, 0.f, false}, // REPEAT: accelerating roll
    {0.f, 0.f, 0.5f, 0.5f, 0.3f, 0.f, 0.f, 0.f, 0.f, 0.f, false},   // SAT: crushed tape dip
    {60.f, 1.f, 0.3f, 0.f, 0.3f, 0.f, 0.f, 0.f, 0.f, 0.f, true},    // CLOUD: scattered grains
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 30.f, 1800.f, 60.f, 0.6f, false},// WAVE: ring sweep spiral
    {125.f, 0.25f, 0.f, 0.f, 0.f, 0.2f, 0.f, 0.f, 0.f, 0.f, false}, // DRUMS: clean roll
};

// send bus: linear dry/wet crossfade (layer 3 K0, 1 = 100 % wet). Linear,
// so dry + an effect that passes the send through stays at unity and the
// output limiter is not driven.

static constexpr uint32_t kModeHoldMs   = 4000;
static constexpr uint32_t kActionMs     = 600;
static constexpr uint32_t kTapMs        = 400; // longer press is not a tap
static constexpr uint32_t kDoubleMs     = 300; // gap for a double-tap
static constexpr uint32_t kLayer3Ms     = 250; // hold this long for layer 3
static constexpr uint32_t kConfirmMs    = 2000;
static constexpr float    kTurn         = 0.03f; // knob move that counts as turning
static constexpr float    kJump         = 0.02f; // knob move that takes over a parameter

struct Settings
{
    uint32_t magic;
    uint32_t mode;
    bool     operator==(const Settings& o) const
    {
        return magic == o.magic && mode == o.mode;
    }
    bool operator!=(const Settings& o) const { return !(*this == o); }
};
static constexpr uint32_t kMagic = 0xD7A65E4Du; // own mode numbering
// last 64 kB block of the 8 MB QSPI, far away from the program
static constexpr uint32_t kStoreOffset = 0x7F0000u;
PersistentStorage<Settings> store(hw.seed.qspi);

volatile uint8_t wantMode   = MODE_BENDER;
volatile uint8_t activeMode = MODE_BENDER;
// stacked mode playing before activeMode (MODE_EMPTY = no stack)
volatile uint8_t wantBottom = MODE_EMPTY;
volatile uint8_t bottomMode = MODE_EMPTY;
volatile uint8_t layerReq   = MODE_EMPTY; // main -> audio: stack this now
float            stackFade  = 1.f;        // new top fading in over the bottom
CpuLoadMeter     cpu;
float            modeGain   = 1.f;

// ------------------------------------------------------------- controls ---
static constexpr size_t kBlock = 8; // 6 kHz control rate
static constexpr int    kKnobs = DaisyVersio::KNOB_LAST;

struct Cv
{
    float v = 0.f;
    void  Process(float raw) { v += 0.25f * (raw - v); }
};
Cv cv[kKnobs];

inline float RawKnob(int i)
{
    float r = 1.f - hw.knobs[i].GetRawFloat(); // Versio knobs are flipped
    return r < 0.f ? 0.f : (r > 1.f ? 1.f : r);
}

inline int Sw(int i)
{
    switch(hw.sw[i].Read())
    {
        case Switch3::POS_UP: return 0;
        case Switch3::POS_CENTER: return 1;
        default: return 2;
    }
}

// ---------------------------------------------------------- knob layers ---
// layer 2 defaults per mode (layer 1 starts where the knobs are).
// K2 = send LPF in every mode, default fully open.
static const float kL2Def[MODE_COUNT][kKnobs] = {
    {0.684f, 0.f, 1.f, 0.5f, 0.f, 0.f, 0.f},     // BENDER
    {0.f, 0.f, 1.f, 0.5f, 0.f, 0.5f, 0.4f},      // TAPE
    {0.564f, 0.f, 1.f, 0.f, 0.f, 0.5f, 0.f},     // SLICER
    {0.5f, 0.f, 1.f, 0.3f, 0.f, 0.5f, 0.f},      // RESO
    {0.5f, 0.f, 1.f, 0.4f, 0.f, 0.f, 0.f},       // REPEAT
    {0.5f, 0.5f, 1.f, 0.3f, 0.3f, 0.f, 0.5f},    // SAT
    {0.f, 0.5f, 1.f, 0.f, 0.f, 0.5f, 1.f},       // CLOUD
    {0.5f, 0.f, 1.f, 0.f, 0.15f, 0.f, 0.f},      // WAVE
    {0.45f, 0.4f, 0.4f, 0.35f, 0.4f, 0.4f, 0.35f}, // DRUMS: decays (K2 OH, K3 CH)
};

float         L1[MODE_COUNT][kKnobs];
float         L2[MODE_COUNT][kKnobs];
float         L3[kKnobs];
volatile bool holdLayer = false; // button held: layer 2 + MAIN MODE
volatile bool mainOn    = false; // tap: MAIN MODE latched on
volatile bool turned    = false; // a knob moved while held
int           editLayer = 0;
bool          locked[kKnobs];
float         pressK[kKnobs];

static float* LayerVals(int layer, uint8_t m)
{
    return layer == 0 ? L1[m] : (layer == 1 ? L2[m] : L3);
}

// layer-1 K3 parameter of each mode, now on layer 2 K2 (BEND, WOW, SHUFFLE,
// DECAY, DECAY, POPS, DENSITY, DECAY, PITCH): its value at power-up
static const float kL1K3Def[MODE_COUNT]
    = {0.f, 0.2f, 0.f, 0.5f, 0.3f, 0.2f, 0.5f, 0.5f, 0.5f};

// physical knob -> stored parameter. Storage keeps the old layout (L1 K2 =
// SEND, L2 K2 = LPF) so the engines are unchanged; only the panel moves:
//   layer 1: K0 = SEND, K2 = mode's K0 parameter, K3 = LPF
//   layer 2: K2 = mode's layer-1 K3 parameter
//   layer 2: K0 = DRY/WET (stored in L3[0]; IN GAIN is gone, input x1)
static float* Target(int layer, uint8_t m, int k)
{
    if(m == MODE_DRUMS) // knob k = voice k: layer 1 tune, layer 2 decay
        return &LayerVals(layer, m)[k];
    if(layer == 0)
    {
        if(k == 0) return &L1[m][2]; // SEND
        if(k == 2) return &L1[m][0];
        if(k == 3) return &L2[m][2]; // LPF
    }
    else if(layer == 1)
    {
        if(k == 0) return &L3[0]; // DRY/WET
        if(k == 2) return &L1[m][3];
    }
    return &LayerVals(layer, m)[k];
}

// what a hold can change (layer 2), saved at the press: put back unless the
// press really was a hold + turn, so patched CV can't slip in on long holds
struct HoldSave
{
    uint8_t m;
    float   l2[kKnobs], l1k3, mix;
} holdSave;
static void SaveHold(uint8_t m)
{
    holdSave.m = m;
    memcpy(holdSave.l2, L2[m], sizeof(holdSave.l2));
    holdSave.l1k3 = L1[m][3];
    holdSave.mix  = L3[0];
}
static void RestoreHold()
{
    const uint8_t m = holdSave.m;
    memcpy(L2[m], holdSave.l2, sizeof(holdSave.l2));
    L1[m][3] = holdSave.l1k3;
    L3[0]    = holdSave.mix;
}

// ------------------------------------------------- TRIG DRUMS: the jacks --
// Every jack is knob + CV summed. A jump of > 0.15 within ~1 ms is a
// trigger (its height = velocity); the slow part is the knob value the
// drum voice gets (so triggers never move a parameter).
struct JackTrig
{
    float h[8];
    int   hi   = 0;
    bool  high = false;
    float base = 0.f, peak = 0.f;
    int   pend = 0;
};
static JackTrig jt[kKnobs];
static float    knobClean[kKnobs];
static bool     gateWasD = false;
volatile bool   midiGate = false; // USB MIDI clock as a beat gate

static void TrigsInit()
{
    for(int k = 0; k < kKnobs; k++)
    {
        const float x = RawKnob(k);
        for(int j = 0; j < 8; j++)
            jt[k].h[j] = x;
        jt[k].base   = x;
        knobClean[k] = x;
    }
}

// jack k -> voice: jack 3 = open hat, jack 4 = closed hat
static const int kJackVoice[kKnobs]
    = {tdr::BD, tdr::SD, tdr::OH, tdr::CH, tdr::CP, tdr::MT, tdr::SA};

// ---- closed hat (jack 4) as a clock: every hit is counted, every ppq-th
// hit is a beat. ppq (1, 2, 4) is the one that puts the tempo at 75..150
// BPM, from the median of the last 4 hit intervals; it only changes after
// 4 hits agree. Counting hits (not timing them) keeps swing in time.
static uint32_t blockN = 0;           // audio blocks since power-up
static uint32_t hatAt = 0, hatIv[4] = {0, 0, 0, 0};
static int      hatIvN = 0, hatPpq = 2, hatCand = 2, hatCandN = 0, hatCount = 0;
static int      hatGateT = 0;         // blocks the beat gate stays high
volatile bool   hatGate = false;

static void HatClock()
{
    const uint32_t iv = blockN - hatAt;
    hatAt             = blockN;
    if(iv > 6000 * 2) // > 2 s: a new run, this hit is beat 1
    {
        hatIvN   = 0;
        hatCount = 0;
    }
    else
    {
        hatIv[hatIvN & 3] = iv;
        hatIvN++;
    }
    if(hatIvN >= 2)
    {
        // median-ish of the last (up to) 4 intervals
        uint32_t a[4];
        const int n = hatIvN < 4 ? hatIvN : 4;
        for(int i = 0; i < n; i++)
            a[i] = hatIv[i];
        for(int i = 1; i < n; i++)
            for(int j = i; j > 0 && a[j] < a[j - 1]; j--)
            {
                const uint32_t t = a[j];
                a[j]             = a[j - 1];
                a[j - 1]         = t;
            }
        const float unit = (float)a[n / 2] / 6000.f; // seconds per hit
        int         want = hatPpq;
        for(int p = 1; p <= 4; p *= 2)
        {
            const float bpm = 60.f / (unit * (float)p);
            if(bpm >= 75.f && bpm < 150.f)
                want = p;
        }
        if(want != hatPpq)
        {
            hatCandN = want == hatCand ? hatCandN + 1 : 1;
            hatCand  = want;
            if(hatCandN >= 4)
            {
                hatPpq   = want;
                hatCount = 0; // re-align: this hit is a beat
            }
        }
        else
            hatCandN = 0;
    }
    if(hatCount % hatPpq == 0)
        hatGateT = 30; // 5 ms beat pulse
    hatCount++;
}

static void Fire(int v, float vel)
{
    tdr::reqExt[v] = true;
    tdr::req[v]    = vel;
}

// audio callback, every block (6 kHz)
static void ScanTrigs(bool drums)
{
    for(int k = 0; k < kKnobs; k++)
    {
        JackTrig&   j   = jt[k];
        const float x   = RawKnob(k);
        const float old = j.h[j.hi & 7]; // 8 blocks = 1.3 ms ago
        j.h[j.hi & 7]   = x;
        j.hi++;
        if(!j.high)
        {
            if(x - old > 0.15f && x - j.base > 0.15f)
            {
                j.high = true;
                j.pend = 2; // velocity: the peak over the next 2 blocks
                j.peak = x - j.base;
            }
            else
                j.base += (x - j.base) * 0.02f; // ~8 ms
        }
        else
        {
            j.peak = fmaxf(j.peak, x - j.base);
            if(j.pend > 0 && --j.pend == 0 && drums)
            {
                Fire(kJackVoice[k], pn::Clamp(j.peak / 0.6f, 0.25f, 1.f));
                if(k == 3)
                    HatClock(); // closed hat also clocks the pattern
            }
            if(x - j.base < 0.07f)
                j.high = false;
        }
        knobClean[k] = j.base;
    }
    blockN++;
    if(hatGateT > 0)
        hatGateT--;
    hatGate = hatGateT > 0;
    const bool g = hw.gate.State();
    if(drums && g && !gateWasD)
        Fire(tdr::SB, 1.f);
    gateWasD = g;
}

// the value a knob gives its layer: trigger-free in TRIG DRUMS
static inline float KnobVal(uint8_t m, int k)
{
    return m == MODE_DRUMS ? knobClean[k] : cv[k].v;
}

// TRIG DRUMS knob pages: the 7 knobs edit every parameter of ONE voice
// (tap = next voice, in jack order). Jump on move: after a page change a
// parameter keeps its value until its knob is turned. Triggers on the
// jacks keep playing their own voices whatever the page.
static const int kPageVoice[8] = {tdr::BD, tdr::SD, tdr::OH, tdr::CH,
                                  tdr::CP, tdr::MT, tdr::SA, tdr::SB};
volatile int     drumPage = 0;
static int       dPageWas = -1;
static bool      dLock[kKnobs];
static float     dPress[kKnobs];

// CTRL-ALL (both switches right, Elektron style): each knob moves its
// parameter on ALL 8 voices at once, by the amount it is turned (relative),
// so the voices keep their differences. Turning stops at 0 / 1 per voice.
static float dRef[kKnobs]; // CTRL-ALL: knob position already applied
static constexpr float kAllStep = 0.004f; // deadband: no drift from ADC noise

// relative move of parameter k on voice v. Moves over several voices
// (CTRL-ALL, hold-tweak on all) never touch the synths' pitch, and move
// drum TUNE only a little (1/4 of the turn), so the kit stays playable.
// A single-voice hold-tweak keeps the drum TUNE small too, synths fixed.
static inline void MoveRel(int v, int k, float d)
{
    if(k == tdr::P_TUNE)
    {
        if(v == tdr::SA || v == tdr::SB)
            return;
        d *= 0.25f;
    }
    tdr::prm[v][k] = pn::Clamp(tdr::prm[v][k] + d, 0.f, 1.f);
}

#ifndef HOLD_TWEAK
#define HOLD_TWEAK 0
#endif
#if HOLD_TWEAK
// ALT build: hold the button + turn = a MOMENTARY tweak. While the button is
// held the knobs move their parameters relative to where they were (the
// edited voice, or all 8 in CTRL-ALL); on release every parameter snaps
// back to the state it had when the button went down. DUB still runs.
volatile bool tweakHold   = false;         // set by the UI loop
volatile bool tweakTurned = false;         // a knob moved during this hold
static bool   tweakWas  = false;
static float  tweakSave[8][tdr::kP];       // parameters at button down
static float  tRef[kKnobs], tPress[kKnobs];
static bool   tLock[kKnobs];
#endif

static void DrumKnobs()
{
    const int  pg  = drumPage;
    const bool all = Sw(0) == 2 && Sw(1) == 2;
    const int  key = all ? 8 : pg; // CTRL-ALL counts as its own page
#if HOLD_TWEAK
    const bool tw = tweakHold;
    if(tw && !tweakWas)
    {
        memcpy(tweakSave, tdr::prm, sizeof(tweakSave));
        tweakTurned = false;
        for(int k = 0; k < kKnobs; k++)
        {
            tLock[k]  = false;
            tPress[k] = tRef[k] = knobClean[k];
        }
    }
    else if(!tw && tweakWas)
    {
        memcpy(tdr::prm, tweakSave, sizeof(tweakSave)); // back to init
        dPageWas = -1; // knobs re-arm: nothing jumps after the release
    }
    tweakWas = tw;
    if(tw)
    {
        for(int k = 0; k < kKnobs; k++)
        {
            const float kv = knobClean[k];
            if(!tLock[k])
            {
                if(fabsf(kv - tPress[k]) <= kJump)
                    continue;
                tLock[k] = true;
            }
            tweakTurned = true; // this hold is a tweak: no DUB
            const float d = kv - tRef[k];
            if(fabsf(d) < kAllStep)
                continue;
            tRef[k] = kv;
            for(int v = 0; v < 8; v++)
                if(all || v == kPageVoice[pg])
                    MoveRel(v, k, d);
        }
        tdr::page   = kPageVoice[pg];
        tdr::allEdt = all;
        return;
    }
#endif
    if(key != dPageWas)
    {
        dPageWas = key;
        for(int k = 0; k < kKnobs; k++)
        {
            dLock[k]  = false;
            dPress[k] = knobClean[k];
            dRef[k]   = knobClean[k];
        }
    }
    if(all)
    {
        for(int k = 0; k < kKnobs; k++)
        {
            const float kv = knobClean[k];
            if(!dLock[k])
            {
                if(fabsf(kv - dPress[k]) <= kJump)
                    continue;
                dLock[k] = true; // the first kJump of travel counts too
            }
            const float d = kv - dRef[k];
            if(fabsf(d) < kAllStep)
                continue;
            dRef[k] = kv;
            for(int v = 0; v < 8; v++)
                MoveRel(v, k, d);
        }
    }
    else
    {
        float* pr = tdr::prm[kPageVoice[pg]];
        for(int k = 0; k < kKnobs; k++)
        {
            const float kv = knobClean[k];
            if(!dLock[k] && fabsf(kv - dPress[k]) > kJump)
                dLock[k] = true;
            if(dLock[k])
                pr[k] = kv;
        }
    }
    tdr::page   = kPageVoice[pg];
    tdr::allEdt = all;
}

// called in the audio callback: route knobs to the edited layer
static void EditKnobs(uint8_t m)
{
    const int want = holdLayer ? 1 : 0;
    if(want != editLayer)
    {
        editLayer = want;
        for(int k = 0; k < kKnobs; k++)
        {
            locked[k] = false;
            pressK[k] = KnobVal(m, k);
        }
    }
    for(int k = 0; k < kKnobs; k++)
    {
        const float kv = KnobVal(m, k);
        if(holdLayer && fabsf(kv - pressK[k]) > kTurn)
            turned = true;
        // jump on move: the knob takes over as soon as it is turned
        if(!locked[k] && fabsf(kv - pressK[k]) > kJump)
            locked[k] = true;
        if(locked[k])
            *Target(editLayer, m, k) = kv;
    }
}

// new mode: layer 1 jumps to the knobs (SEND, LPF too), other layers jump
// when a knob moves. L1 K3 lives on layer 2 now and keeps its value.
static void LayersOnActivate(uint8_t m)
{
    for(int k = 0; k < kKnobs; k++)
    {
        *Target(0, m, k) = KnobVal(m, k);
        locked[k] = true;
        pressK[k] = KnobVal(m, k);
    }
    editLayer = 0;
}

// UI -> audio
volatile bool     wantA     = false; // double-tap toggle
volatile uint32_t actionReq = 0;     // hold + release counter

dbe::Controls ctl;
slc::Controls sctl;
Panel         pnl;  // top (controlled) mode
Panel         pnlB; // bottom (frozen) mode of a stack

// what an engine gets from the panel: live for the top mode, frozen for the
// bottom mode of a stack
struct Ctx
{
    int      row, col;
    bool     a;
    uint32_t action;
};
Ctx          botCtx = {0, 0, false, 0};
volatile int pendRow = 0, pendCol = 0; // bottom-to-be, set before a swap
volatile bool     pendA      = false;
volatile uint32_t pendAction = 0;
int           corruptType = dbe::C_DECIMATE;

// --------------------------------------------------------------- fills ---
// Every engine runs 100 % wet (the send bus does dry/wet), clocked by the
// beat (gate jack or audio beat tracking). Layer 1 K2 = SEND and layer 2
// K2 = LPF are used by the bus, not by the engines.
static void FillBender(const Ctx& cx)
{
    const float* a = L1[MODE_BENDER];
    const float* b = L2[MODE_BENDER];
    switch(cx.row)
    {
        case 0: ctl.mode = dbe::MACRO_TRAVERSE; break;
        case 1: ctl.mode = dbe::MACRO_SILENCE; break;
        default: ctl.mode = dbe::MICRO; break;
    }
    float pos = a[6] * (float)dbe::C_LAST;
    if(fabsf(pos - ((float)corruptType + 0.5f)) > 0.6f)
    {
        int z       = (int)pos;
        corruptType = z >= dbe::C_LAST ? dbe::C_LAST - 1 : z;
    }
    ctl.time          = a[0];
    ctl.repeats       = a[1];
    ctl.mix           = 1.f; // 100 % wet
    ctl.bend          = a[3];
    ctl.brk           = a[4];
    ctl.corrupt       = a[5];
    ctl.corruptType   = corruptType;
    ctl.windowMs      = 0.5f * powf(80.f, b[0]);
    ctl.reverseChance = b[1];
    ctl.breakShape    = 0.5f; // layer 2 K2 is the LPF
    ctl.corruptColor  = b[3];
    ctl.regen         = b[4] * 0.95f;
    ctl.shuffle       = b[5];
    ctl.randomRepeats = b[6];
    ctl.width         = 1.f;
    ctl.gate          = bt::st.pulse;
    ctl.freeze        = cx.a;
    ctl.reverse       = false;
    ctl.recapture     = cx.action;
    ctl.unique        = cx.col != 0;
    ctl.hardWindow    = cx.col == 2;
}

static void FillSlicer(const Ctx& cx)
{
    const int row = cx.row, col = cx.col;
    const float* a = L1[MODE_SLICER];
    const float* b = L2[MODE_SLICER];
    sctl.character = row;
    sctl.play      = col;
    sctl.rate      = a[0];
    sctl.steps     = a[1];
    sctl.mix       = 1.f; // 100 % wet
    sctl.shuffle   = a[3];
    sctl.stutter   = a[4];
    sctl.pitch     = a[5];
    sctl.gate      = a[6];
    sctl.fade      = b[0];
    sctl.stereo    = b[1];
    sctl.revChance = 0.f; // layer 2 K2 is the LPF
    sctl.swing     = b[3];
    sctl.texture   = b[4];
    sctl.ratPitch  = b[5];
    sctl.drop      = b[6];
    sctl.clockIn   = bt::st.pulse;
    sctl.freeze    = cx.a;
    sctl.reverse   = false;
    sctl.recapture = cx.action;
}

static void FillPanel(uint8_t m, Panel& p, const Ctx& cx)
{
    for(int i = 0; i < kKnobs; i++)
    {
        p.k[i] = L1[m][i];
        p.s[i] = L2[m][i];
    }
    if(m != MODE_DRUMS) // drums: K2 / layer 2 K2 are the closed hat
    {
        p.k[2] = 1.f; // engine mix: 100 % wet
        // layer 2 K2 is the LPF: the engine's own K2 parameter gets a fixed
        // value (resonator input level x1, wavetable scan depth half, tape
        // R-head spread / cloud feedback off)
        p.s[2] = (m == MODE_RESO || m == MODE_WAVE) ? 0.5f : 0.f;
    }
    p.row    = cx.row;
    p.col    = cx.col;
    p.gate   = bt::st.pulse;
    p.a      = cx.a;
    p.action = cx.action;
}

static Ctx LiveCtx()
{
    return {Sw(0), Sw(1), wantA, actionReq};
}


// run one mode: fill its controls from cx, process in -> out (100 % wet)
static void RunMode(uint8_t            m,
                    const Ctx&         cx,
                    Panel&             p,
                    const float* const* in,
                    float**            out,
                    size_t             n)
{
    switch(m)
    {
        case MODE_BENDER:
            FillBender(cx);
            dbe::Process(ctl, in, out, n);
            break;
        case MODE_SLICER:
            FillSlicer(cx);
            slc::Process(sctl, in, out, n);
            break;
        default:
            FillPanel(m, p, cx);
            switch(m)
            {
                case MODE_RESO: rez::Process(p, in, out, n); break;
                case MODE_TAPE: tape::Process(p, in, out, n); break;
                case MODE_REPEAT: rpt::Process(p, in, out, n); break;
                case MODE_SAT: sat::Process(p, in, out, n); break;
                case MODE_CLOUD: cld::Process(p, in, out, n); break;
                case MODE_WAVE: wav::Process(p, in, out, n); break;
                case MODE_DRUMS: tdr::Process(p, in, out, n); break;
                default:
                    for(size_t i = 0; i < n; i++)
                        out[0][i] = in[0][i], out[1][i] = in[1][i];
                    break;
            }
            break;
    }
}

static void TakePendingBottom()
{
    botCtx.row    = pendRow;
    botCtx.col    = pendCol;
    botCtx.a      = pendA;
    botCtx.action = pendAction;
}

static void ActivateEngine(uint8_t m)
{
    LayersOnActivate(m);
    FillPanel(m, pnl, LiveCtx());
    switch(m)
    {
        case MODE_SLICER: slc::Activate(); break;
        case MODE_RESO: rez::Activate(pnl); break;
        case MODE_TAPE: tape::Activate(pnl); break;
        case MODE_REPEAT: rpt::Activate(pnl); break;
        case MODE_SAT: sat::Activate(pnl); break;
        case MODE_CLOUD: cld::Activate(pnl); break;
        case MODE_WAVE: wav::Activate(pnl); break;
        case MODE_DRUMS: tdr::Activate(pnl); break;
        default: break;
    }
}

// ----------------------------------------------------------- MAIN MODE ---
// on (tap latch, or while the button is held): the send LPF opens and every
// mode knob (layers 1 and 2, SEND too; not the LPF or DRY/WET) wanders
// +-5 % at random. Off: all of it fades back over 1.5 s.
static constexpr float kMainRand = 0.05f;
volatile float  mainAmt = 0.f; // 0..1: in over 30 ms, out over 1.5 s
static float    rndCur[2][kKnobs], rndTgt[2][kKnobs];
static uint32_t rndNext = 0, rndSeed = 0x2545F491u;
static float    mainSave[2][kKnobs];

static float Rnd() // -1..1
{
    rndSeed ^= rndSeed << 13;
    rndSeed ^= rndSeed >> 17;
    rndSeed ^= rndSeed << 5;
    return (float)(rndSeed >> 8) * (2.f / 16777216.f) - 1.f;
}

static void MainUpdate(size_t n, float sr)
{
    const bool  on = (mainOn || holdLayer) && activeMode != MODE_DRUMS;
    const float st = (float)n / ((on ? 0.03f : 1.5f) * sr); // linear ramp
    float       a  = on ? mainAmt + st : mainAmt - st;
    mainAmt        = a < 0.f ? 0.f : (a > 1.f ? 1.f : a);
    // new random targets every 120..400 ms, glided (~80 ms): no zipper
    if(rndNext <= n)
    {
        for(int l = 0; l < 2; l++)
            for(int k = 0; k < kKnobs; k++)
                rndTgt[l][k] = Rnd();
        rndNext = (uint32_t)((0.26f + 0.14f * Rnd()) * sr);
    }
    else
        rndNext -= (uint32_t)n;
    float g = (float)n / (0.08f * sr);
    g       = g > 1.f ? 1.f : g;
    for(int l = 0; l < 2; l++)
        for(int k = 0; k < kKnobs; k++)
            rndCur[l][k] += (rndTgt[l][k] - rndCur[l][k]) * g;
}

static inline float Clamp01(float v)
{
    return v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
}

// add the random offsets in place for this block (RestoreMain undoes it)
static void ApplyMain(uint8_t m)
{
    memcpy(mainSave[0], L1[m], sizeof(mainSave[0]));
    memcpy(mainSave[1], L2[m], sizeof(mainSave[1]));
    const float d = kMainRand * mainAmt;
    for(int k = 0; k < kKnobs; k++)
    {
        L1[m][k] = Clamp01(L1[m][k] + d * rndCur[0][k]);
        if(k != 2) // L2 K2 = the LPF: opened instead
            L2[m][k] = Clamp01(L2[m][k] + d * rndCur[1][k]);
    }
}
static void RestoreMain(uint8_t m)
{
    memcpy(L1[m], mainSave[0], sizeof(mainSave[0]));
    memcpy(L2[m], mainSave[1], sizeof(mainSave[1]));
}

// --------------------------------------------------------------- audio ---
static float dryBuf[2][64]; // input
static float inBuf[2][64];  // send after the LPF = engine input
static float wetBuf[2][64]; // engine output
static float midBuf[2][64]; // bottom mode -> top mode of a stack
static daisysp::Svf sendLpf[2];
static float        sendG = 0.f, sendC = 0.002f;

#if USB_MIDI
// audio input level for USB MIDI CC 27/28 (raw input, before IN GAIN)
volatile float inLevel[2] = {0.f, 0.f};
static float   envAtt = 0.f, envRel = 0.f;
static void    InLevel(const float* x, size_t n, int ch)
{
    float e = inLevel[ch];
    for(size_t i = 0; i < n; i++)
    {
        const float a = fabsf(x[i]);
        e += (a > e ? envAtt : envRel) * (a - e);
    }
    inLevel[ch] = e > 1.f ? 1.f : e;
}
#endif

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    cpu.OnBlockStart();
    for(int i = 0; i < kKnobs; i++)
        cv[i].Process(RawKnob(i));
    ScanTrigs(activeMode == MODE_DRUMS && wantMode == MODE_DRUMS);

    // stack request from the UI (double-press confirm): the running mode
    // becomes the frozen bottom, the new one starts on top and fades in
    if(layerReq != MODE_EMPTY && activeMode == wantMode
       && bottomMode == wantBottom)
    {
        const uint8_t nm = layerReq;
        layerReq         = MODE_EMPTY;
        TakePendingBottom();
        bottomMode = wantBottom = activeMode;
        activeMode = wantMode = nm;
        ActivateEngine(nm);
        stackFade = 0.f;
    }

    const uint8_t m  = activeMode;
    const uint8_t bm = bottomMode;
    if(m == MODE_DRUMS)
        DrumKnobs();
    else
        EditKnobs(m);

#if USB_MIDI
    InLevel(in[0], size, 0);
    InLevel(in[1], size, 1);
#endif
    const size_t n = size > 64 ? 64 : size;
    for(size_t i = 0; i < n; i++)
    {
        dryBuf[0][i] = in[0][i];
        dryBuf[1][i] = in[1][i];
    }

    // beat: gate clock, or tracked from the input (before the send)
    // (TRIG DRUMS: the gate jack is voice 8; USB MIDI clock or the
    // closed-hat triggers on jack 4 clock the pattern)
    const bool drumsOn = m == MODE_DRUMS;
    bt::Process(dryBuf[0], dryBuf[1], n,
                drumsOn ? (bool)(midiGate || hatGate)
                        : (hw.gate.State() || midiGate));

    // MAIN MODE: random offsets on top of the knobs for this block
    MainUpdate(n, hw.AudioSampleRate());
    const bool mainFx = mainAmt > 0.f;
    if(mainFx)
        ApplyMain(m);

    // ---- send bus: input x SEND -> LPF -> engine input
    {
        const float sk  = L1[m][2];
        const float sT  = sk * sk; // audio taper, noon = -12 dB
        const float fk  = L2[m][2] + (1.f - L2[m][2]) * mainAmt; // MAIN opens
        const bool  thru = fk > 0.985f;
        const float hz  = 30.f * powf(666.7f, fk); // 30 Hz .. 20 kHz
        for(int c = 0; c < 2; c++)
            sendLpf[c].SetFreq(fminf(hz, 16000.f));
        for(size_t i = 0; i < n; i++)
        {
            sendG += (sT - sendG) * sendC;
            for(int c = 0; c < 2; c++)
            {
                const float x = dryBuf[c][i] * sendG;
                sendLpf[c].Process(x); // always runs: no click on bypass
                inBuf[c][i] = thru ? x : sendLpf[c].Low();
            }
        }
    }
    // TRIG DRUMS gets the raw input (it handles THRU / SKIN / CHOP itself)
    const float* ip[2] = {drumsOn ? dryBuf[0] : inBuf[0],
                          drumsOn ? dryBuf[1] : inBuf[1]};
    float*       wp[2] = {wetBuf[0], wetBuf[1]};

    const Ctx top = LiveCtx();
    if(bm == MODE_EMPTY)
        RunMode(m, top, pnl, ip, wp, n);
    else
    {
        // stack: send -> bottom (frozen) -> top (live) -> wet
        float*       mo[2] = {midBuf[0], midBuf[1]};
        const float* mi[2] = {midBuf[0], midBuf[1]};
        RunMode(bm, botCtx, pnlB, ip, mo, n);
        RunMode(m, top, pnl, mi, wp, n);
        if(stackFade < 1.f)
        {
            const float st = 1.f / (0.03f * hw.AudioSampleRate()); // 30 ms
            for(size_t i = 0; i < n; i++)
            {
                stackFade = stackFade + st > 1.f ? 1.f : stackFade + st;
                for(int c = 0; c < 2; c++)
                    wetBuf[c][i] = midBuf[c][i]
                                   + (wetBuf[c][i] - midBuf[c][i]) * stackFade;
            }
        }
    }
    if(mainFx)
        RestoreMain(m); // the knobs keep their own values
    if(m != MODE_SLICER && bm != MODE_SLICER)
        slc::Idle(ip, n); // keep the slicer's buffer fresh

    // ---- mode change: the WET path goes through the target's transition
    // effect, engines swap at its peak (the dry signal is never touched)
    const bool changing = activeMode != wantMode || bottomMode != wantBottom;
    if(changing && !trn::Active())
        trn::Start(kTrans[wantMode]);
    const float gTarget = (changing && trn::Progress() > 0.485f) ? 0.f : 1.f;
    const float step    = 1.f / (0.004f * hw.AudioSampleRate());
    for(size_t i = 0; i < n; i++)
    {
        modeGain += gTarget > modeGain ? step : -step;
        modeGain = modeGain < 0.f ? 0.f : (modeGain > 1.f ? 1.f : modeGain);
        wetBuf[0][i] *= modeGain;
        wetBuf[1][i] *= modeGain;
    }
    trn::Process(wetBuf[0], wetBuf[1], n);

    // ---- dry/wet (layer 3 K0; 1 = wet only, no dry at all)
    {
        // TRIG DRUMS: its output already holds the input it wants
        const float w = drumsOn ? 1.f
                        : (L3[0] > 0.985f ? 1.f : (L3[0] < 0.015f ? 0.f : L3[0]));
        const float d = 1.f - w;
        for(size_t i = 0; i < n; i++)
        {
            out[0][i] = dryBuf[0][i] * d + wetBuf[0][i] * w;
            out[1][i] = dryBuf[1][i] * d + wetBuf[1][i] * w;
        }
    }

    // layer 3 output strip (its trance gate follows the beat)
    strip::Process(L3, bt::st.pulse, out[0], out[1], n);

    if(changing && trn::MidPassed() && modeGain <= 0.f)
    {
        const bool newTop = activeMode != wantMode;
        TakePendingBottom();
        bottomMode = wantBottom;
        activeMode = wantMode;
        if(newTop) // removing only the bottom keeps the top running as is
            ActivateEngine(activeMode);
        stackFade = 1.f;
    }
    cpu.OnBlockEnd();
}

// -------------------------------------------------------------- LEDs -------
static void LedsBender(float pf, float sf)
{
    auto&       st = dbe::status;
    const float b0 = 0.25f + 0.75f * pf;
    switch(ctl.mode)
    {
        case dbe::MACRO_TRAVERSE: hw.SetLed(0, 0.f, 0.f, b0); break;
        case dbe::MACRO_SILENCE: hw.SetLed(0, 0.f, b0 * 0.7f, b0); break;
        default: hw.SetLed(0, 0.f, b0, 0.f); break;
    }
    const bool blink = (System::GetNow() / 80) & 1;
    if(st.capturing)
        hw.SetLed(1, blink, blink, blink);
    else if(st.frozen)
        hw.SetLed(1, 0.8f, 0.9f, 1.f);
    else
        hw.SetLed(1, 0.f, st.outLevel * 0.8f, 0.f);

    float r, g, b;
    dbe::CorruptColor(corruptType, r, g, b);
    float amt = corruptType == dbe::C_DJFILTER ? dbe::DjAmount() : ctl.corrupt;
    float br  = 0.08f + 0.92f * amt;
    hw.SetLed(2, r * br, g * br, b * br);

    if(st.clocked)
        hw.SetLed(3, sf, sf * 0.7f, 0.f);
    else
        hw.SetLed(3, sf, sf, sf);
}

// inverted scheme: everything glows, events punch dark holes
static void LedsSlicer(float pf, float sf)
{
    auto& st = slc::status;

    static const float kChar[3][3] = {
        {1.f, 0.45f, 0.f}, // LOCKED  amber
        {1.f, 0.f, 0.35f}, // DRIFT   pink
        {0.8f, 0.f, 1.f},  // CHAOS   magenta
    };
    const int   ch = sctl.character;
    const float b0 = 0.8f * (1.f - pf);
    hw.SetLed(0, kChar[ch][0] * b0, kChar[ch][1] * b0, kChar[ch][2] * b0);

    const bool blink = (System::GetNow() / 80) & 1;
    if(st.capturing)
        hw.SetLed(1, 0.f, blink, blink);
    else if(st.frozen)
        hw.SetLed(1, 0.f, 1.f, 1.f);
    else
    {
        // play style colour, brightness = level
        static const float kP[3][3] = {{0.5f, 0.f, 1.f}, {1.f, 0.3f, 0.f}, {0.f, 1.f, 0.3f}};
        const float l = fminf(st.outLevel, 1.f);
        const int   p = sctl.play;
        hw.SetLed(1, kP[p][0] * l, kP[p][1] * l, kP[p][2] * l);
    }

    const uint8_t kd = st.stepKind;
    float         r = 0.f, g = 0.f, b = 0.f;
    if(kd == 0)
        r = g = b = 0.06f;
    else
    {
        if(kd & slc::K_SHUF)
            r += 1.f, g += 0.35f;
        if(kd & slc::K_REV)
            r += 1.f;
        if(kd & slc::K_RAT)
            b += 1.f, g += 0.2f;
        if(kd & slc::K_OCT)
            r += 0.6f, g += 0.6f, b += 0.6f;
    }
    hw.SetLed(2, fminf(r, 1.f), fminf(g, 1.f), fminf(b, 1.f));

    const float b3 = 0.7f * (1.f - sf);
    if(st.clocked)
        hw.SetLed(3, b3, b3 * 0.6f, 0.f);
    else
        hw.SetLed(3, b3, b3, b3);
}

#if USB_MIDI
// ------------------------------------------------------- USB MIDI input --
// GM drum notes -> voices; channel 2 / 3 notes -> SA / SB at that pitch;
// MIDI clock (24 ppq) -> a gate for the beat (beat = 24 ticks)
static uint32_t midiTicks = 0, midiClockAt = 0;
static int      GmVoice(int note)
{
    switch(note)
    {
        case 35:
        case 36: return tdr::BD;
        case 38:
        case 40: return tdr::SD;
        case 42:
        case 44: return tdr::CH;
        case 46: return tdr::OH;
        case 39: return tdr::CP;
        case 37:
        case 41:
        case 43:
        case 45:
        case 47:
        case 48:
        case 50:
        case 56: return tdr::MT;
        default: return -1;
    }
}
static void MidiIn(uint32_t now)
{
    midi.Listen();
    while(midi.HasEvents())
    {
        MidiEvent e = midi.PopEvent();
        if(e.type == NoteOn)
        {
            const NoteOnEvent n = e.AsNoteOn();
            if(n.velocity == 0 || activeMode != MODE_DRUMS)
                continue;
            const float vel = (float)n.velocity / 127.f;
            int         v   = -1;
            if(n.channel == 1)
                tdr::midiNote[0] = (float)n.note, v = tdr::SA;
            else if(n.channel == 2)
                tdr::midiNote[1] = (float)n.note, v = tdr::SB;
            else
                v = GmVoice(n.note);
            if(v >= 0)
                Fire(v, vel);
        }
        else if(e.type == SystemRealTime)
        {
            if(e.srt_type == TimingClock)
            {
                midiTicks++;
                midiClockAt = now;
            }
            else if(e.srt_type == Start)
                midiTicks = 0;
        }
    }
    // high for the first half of every beat while the clock is running
    midiGate = (now - midiClockAt < 500) && (midiTicks % 24) < 12;
}
#endif

static void SaveMode(uint32_t m)
{
    Settings& s = store.GetSettings();
    s.magic     = kMagic;
    s.mode      = m;
    store.Save(); // only erases/writes if it changed
}

// big buffers of the engine about to start are wiped while it is not running
static void PrepareEngine(uint8_t m)
{
    switch(m)
    {
        case MODE_RESO: rez::Clear(); break;
        case MODE_TAPE: tape::Clear(); break;
        case MODE_REPEAT: rpt::Clear(); break;
        case MODE_SAT: sat::Clear(); break;
        case MODE_CLOUD: cld::Clear(); break;
        case MODE_WAVE: wav::Clear(); break;
        case MODE_DRUMS: tdr::Clear(); break;
        default: break;
    }
}

// Hold the front button while powering up (keep holding ~1 s, LEDs turn
// blue): the Seed restarts into its bootloader and waits for a new firmware
// in flash.daisy.audio / dfu-util, no BOOT/RESET buttons needed.
static void BootloaderGesture()
{
    uint32_t held = 0;
    for(int i = 0; i < 1500; i++)
    {
        hw.tap.Debounce();
        if(!hw.tap.RawState())
            return; // not held at power-up: normal start
        held++;
        if(held > 1000)
        {
            for(int k = 0; k < 4; k++)
                hw.SetLed(k, 0.f, 0.f, 1.f);
            hw.UpdateLeds();
            System::Delay(300);
#if USB_MIDI
            // Daisy bootloader (this build runs through it), wait forever
            System::ResetToBootloader(System::BootloaderMode::DAISY_INFINITE_TIMEOUT);
#else
            // internal-flash build: the chip's own DFU mode
            System::ResetToBootloader(System::BootloaderMode::STM);
#endif
        }
        System::Delay(1);
    }
}

int main(void)
{
    hw.Init(true); // 480 MHz
    BootloaderGesture();
    hw.SetAudioBlockSize(kBlock);
    const float sr = hw.AudioSampleRate();

    Settings defaults = {kMagic, MODE_BENDER};
    store.Init(defaults, kStoreOffset);
    {
        const Settings& s = store.GetSettings();
        const uint8_t   m = (s.magic == kMagic && s.mode < MODE_COUNT)
                                ? (uint8_t)s.mode
                                : (uint8_t)MODE_BENDER;
        wantMode = activeMode = m;
    }

    for(int m = 0; m < MODE_COUNT; m++)
        for(int k = 0; k < kKnobs; k++)
            L2[m][k] = kL2Def[m][k];
    for(int k = 0; k < kKnobs; k++)
        L3[k] = strip::kDefault[k];
    L3[0] = 1.f; // DRY/WET: always 100 % wet at power-up
    for(int m = 0; m < MODE_COUNT; m++)
        L1[m][3] = kL1K3Def[m];

    hw.StartAdc();
    System::Delay(20);
    for(int i = 0; i < kKnobs; i++)
        cv[i].v = RawKnob(i);
    TrigsInit();

    const uint32_t seed = hw.knobs[0].GetRawValue();
    dbe::Init(sr, seed);
    slc::Init(sr, seed + 12345u);
    rez::Init(sr);
    tape::Init(sr);
    rpt::Init(sr);
    sat::Init(sr);
    cld::Init(sr);
    wav::Init(sr); // builds the wavetables (~2 MB, a moment at power-up)
    tdr::Init(sr);
    bt::Init(sr);
    strip::Init(sr);
    trn::Init(sr, 1.f); // 1 s transition, engines swap at 0.5 s
    dbe::SetInitialMix(1.f); // engines are 100 % wet, the send bus mixes
    slc::SetInitialMix(1.f);
    for(int c = 0; c < 2; c++)
    {
        sendLpf[c].Init(sr);
        sendLpf[c].SetRes(0.15f);
        sendLpf[c].SetDrive(0.f); // clean: DaisySP's Svf saturates by default
        sendLpf[c].SetFreq(16000.f);
    }
    sendC = 1.f - expf(-1.f / (0.01f * sr)); // send knob smoothing 10 ms
    sendG = cv[DaisyVersio::KNOB_0].v * cv[DaisyVersio::KNOB_0].v; // SEND
    ActivateEngine(activeMode);
    cpu.Init(sr, kBlock);
    hw.StartAudio(AudioCallback);
#if USB_MIDI
    envAtt = 1.f - expf(-1.f / (0.005f * sr)); // 5 ms
    envRel = 1.f - expf(-1.f / (0.150f * sr)); // 150 ms
    usbmidi::Init();
#endif

    // gesture state
    uint32_t pressMs = 0, relMs = 0;
    bool     wasPressed  = false;
    bool     tapPending  = false; // single tap waiting for a possible 2nd
    bool     consumed    = false; // this press is spent (double-tap / confirm)
    bool     modeHandled = false; // the 4 s hold happened on this press

    // mode change waiting for confirmation
    bool     armed     = false;
    uint8_t  armedMode = MODE_BENDER;
    uint32_t armedAt   = 0;
    int      armSw[2]  = {0, 0};

    uint32_t blinkUntil = 0;
    bool     blinkRed   = false;

    // CPU: peak load of the audio callback over the last 250 ms
    uint32_t cpuAt = 0, cpuWarnUntil = 0;
    float    cpuPeak = 0.f;
    float    pf = 0.f, sf = 0.f, bf = 0.f;
    uint32_t beatAt   = 0;
    bool     beatDown = false;
    while(1)
    {
        const uint32_t now = System::GetNow();
        bt::Update(); // tempo / phase search (outside the audio callback)
#if USB_MIDI
        MidiIn(now);
#endif

        if(now - cpuAt >= 250)
        {
            cpuAt         = now;
            const float l = cpu.GetMaxCpuLoad();
            cpuPeak       = (l == l) ? l : 0.f; // NaN right after Reset()
            cpu.Reset();
            if(cpuPeak > 0.95f)
                cpuWarnUntil = now + 1000;
        }

        // ------------------------------------------------------ button ----
        hw.tap.Debounce();
        const bool pressed = hw.tap.Pressed();
        const bool rise    = pressed && !wasPressed;
        const bool fall    = !pressed && wasPressed;
        wasPressed         = pressed;

        // armed mode change: dropped by timeout or moving a switch. Knobs
        // are not watched: each jack is knob + CV summed in hardware, so a
        // patched CV would look like a turned knob.
        if(armed)
        {
            if(pressed && !rise)
                armedAt = now; // still holding the 4 s press
            bool drop = (now - armedAt) > kConfirmMs;
            if(Sw(0) != armSw[0] || Sw(1) != armSw[1])
                drop = true;
            if(drop)
            {
                armed      = false;
                blinkUntil = now + 300;
                blinkRed   = true;
            }
        }

        if(rise)
        {
            if(armed)
            {
                // one press confirms: the new mode replaces the running one
                armed    = false;
                consumed = true;
                wantA    = false;
                if(armedMode != wantMode)
                {
                    PrepareEngine(armedMode);
                    wantMode = armedMode;
                }
                SaveMode(armedMode);
                blinkUntil = now + 900;
                blinkRed   = false;
            }
            else if(tapPending && now - relMs < kDoubleMs)
            {
                // double-tap: the mode's main toggle
                tapPending = false;
                consumed   = true;
                wantA      = !wantA;
            }
            else
            {
                consumed = false;
                pressMs  = now;
                turned   = false;
                // layer 3 as it was before this press: put back unless the
                // press really was a hold + turn
                SaveHold(activeMode);
            }
            modeHandled = false;
        }

        // 4 s hold arms even if a knob/CV moved (CV can't be told apart
        // from a knob); the confirming second press keeps it safe
        if(pressed && !consumed && !modeHandled
           && now - pressMs > kModeHoldMs)
        {
            modeHandled        = true;
            const uint8_t slot = (uint8_t)(Sw(0) * 3 + Sw(1));
            const uint8_t m    = kSlotMode[slot];
            if(m == wantMode || m == MODE_EMPTY)
            {
                blinkUntil = now + 900; // already there
                blinkRed   = m == MODE_EMPTY;
            }
            else
            {
                armed     = true;
                armedMode = m;
                armedAt   = now;
                armSw[0]  = Sw(0);
                armSw[1]  = Sw(1);
            }
        }

        // layer 3 only after 250 ms of holding (taps never touch it), and
        // not during the 4 s mode-change hold. Every jack is knob + CV
        // summed, so patched CV used to slip into layer 3 (CRUSH, DRIVE,
        // GATE ...) on every long hold and stay there in all modes.
        const bool l3Was = holdLayer;
        holdLayer = pressed && !consumed && !modeHandled
                    && now - pressMs >= kLayer3Ms
                    && activeMode != MODE_DRUMS; // drums: hold = DUB
        if(l3Was && !holdLayer && modeHandled)
            RestoreHold(); // the hold became a mode change

        if(fall && !consumed && !modeHandled && !turned)
        {
            const uint32_t held = now - pressMs;
            if(held < kTapMs)
            {
                tapPending = true;
                relMs      = now;
            }
        }
        if(fall)
        {
            // a press that wasn't a real hold + turn leaves layer 3 alone
            const uint32_t held = now - pressMs;
            if(!consumed && (modeHandled || !turned || held < kTapMs))
                RestoreHold();
            consumed = false;
        }

        // single tap (no second one came): MAIN MODE on / off; in TRIG
        // DRUMS: the knobs go to the next voice (8 pages, jack order)
        if(tapPending && !pressed && now - relMs >= kDoubleMs)
        {
            tapPending = false;
            if(activeMode == MODE_DRUMS)
                drumPage = (drumPage + 1) % 8;
            else
                mainOn = !mainOn;
        }

        // TRIG DRUMS: holding the button = DUB (momentary). Starts after
        // 0.2 s so a tap only changes the page; the engine fades it.
        tdr::dubReq = activeMode == MODE_DRUMS && pressed && !consumed
                      && !modeHandled && now - pressMs >= 200;
#if HOLD_TWEAK
        tweakHold = activeMode == MODE_DRUMS && pressed && !consumed
                    && !modeHandled;
        // ALT: a hold with a knob turn is only the tweak, no DUB (if DUB
        // had already come in, it cuts out quickly)
        tdr::dubCut = tweakHold && tweakTurned;
        if(tdr::dubCut)
            tdr::dubReq = false;
#endif

        // -------------------------------------------------------- LEDs ----
        const uint8_t am = activeMode;
        // beat (gate or tracked audio): drives LED 3 blips and the
        // repeater / saturator LEDs
        if(bt::flash > 0.5f)
        {
            bt::flash  = 0.f;
            beatAt     = now;
            beatDown   = bt::flashDown;
            bf         = 1.f;
        }
        bf *= 0.9f;
        {
            float pFlash = 0.f, sFlash = 0.f;
            switch(am)
            {
                case MODE_BENDER:
                    pFlash                  = dbe::status.periodFlash;
                    sFlash                  = dbe::status.sliceFlash;
                    dbe::status.periodFlash = 0.f;
                    dbe::status.sliceFlash  = 0.f;
                    break;
                case MODE_SLICER:
                    pFlash                = slc::status.loopFlash;
                    sFlash                = slc::status.stepFlash;
                    slc::status.loopFlash = 0.f;
                    slc::status.stepFlash = 0.f;
                    break;
                case MODE_RESO:
                    pFlash         = rez::pluckShow;
                    rez::pluckShow = 0.f;
                    break;
                case MODE_CLOUD:
                    pFlash          = cld::spawnFlash;
                    cld::spawnFlash = 0.f;
                    break;
                case MODE_WAVE:
                    pFlash          = wav::spawnFlash;
                    wav::spawnFlash = 0.f;
                    break;
                default: break;
            }
            if(pFlash > 0.5f)
                pf = 1.f;
            if(sFlash > 0.5f)
                sf = 1.f;
        }
        pf *= 0.93f;
        sf *= 0.85f;

        if(armed)
        {
            // waiting for confirmation: amber pulse, number of lit LEDs =
            // row (Switch 0), LED 3 colour = column (Switch 1)
            const float ph  = 0.5f + 0.5f * sinf((float)now * 0.008f);
            const int   row = armSw[0] + 1;
            for(int i = 0; i < 3; i++)
            {
                const float b = i < row ? ph : 0.08f;
                hw.SetLed(i, b, b * 0.45f, 0.f);
            }
            static const float kCol[3][3]
                = {{1.f, 1.f, 1.f}, {0.f, 1.f, 1.f}, {0.f, 0.2f, 1.f}};
            const int c = armSw[1];
            hw.SetLed(3, kCol[c][0] * ph, kCol[c][1] * ph, kCol[c][2] * ph);
        }
        else if((int32_t)(blinkUntil - now) > 0)
        {
            // white = mode selected, red = arm dropped
            const float b = ((blinkUntil - now) / 150) & 1 ? 1.f : 0.f;
            for(int i = 0; i < 4; i++)
                hw.SetLed(i, b, blinkRed ? 0.f : b, blinkRed ? 0.f : b);
        }
        else
        {
            LedOut lo;
            bool   generic = true;
            switch(am)
            {
                case MODE_BENDER: LedsBender(pf, sf), generic = false; break;
                case MODE_SLICER: LedsSlicer(pf, sf), generic = false; break;
                case MODE_RESO: rez::Leds(lo, now, pf); break;
                case MODE_TAPE: tape::Leds(lo, now, pf); break;
                case MODE_REPEAT: rpt::Leds(lo, now, bf); break;
                case MODE_SAT: sat::Leds(lo, now, bf); break;
                case MODE_CLOUD: cld::Leds(lo, now, pf, pnl); break;
                case MODE_WAVE: wav::Leds(lo, now, pf, pnl); break;
                case MODE_DRUMS: tdr::Leds(lo, now, bf); break;
                default:
                    for(int i = 0; i < 4; i++)
                        lo.Set(i, 0.f, 0.f, 0.f);
                    break;
            }
            if(generic)
                for(int i = 0; i < 4; i++)
                    hw.SetLed(i, lo.c[i][0], lo.c[i][1], lo.c[i][2]);

            // LED 3: violet = held (layer 2 + MAIN MODE), cyan = MAIN MODE
            // (dims with its fade-out)
            const bool l3 = holdLayer;
            if(l3)
                hw.SetLed(3, 0.7f, 0.f, 1.f);
            else if(mainAmt > 0.01f)
                hw.SetLed(3, 0.f, 0.8f * mainAmt, 0.8f * mainAmt);
            // beat blip on LED 3 (40 ms): green = beat tracked from the
            // audio, orange = gate clock, white = beat 1 of the bar
            const uint8_t src = bt::st.source;
            if(!l3 && src != bt::SRC_FREE && now - beatAt < 40)
            {
                if(beatDown)
                    hw.SetLed(3, 1.f, 1.f, 1.f);
                else if(src == bt::SRC_GATE)
                    hw.SetLed(3, 1.f, 0.4f, 0.f);
                else
                    hw.SetLed(3, 0.f, 1.f, 0.f);
            }

            // CPU overload (audio may crackle): LED 1 flashes red
            if((int32_t)(cpuWarnUntil - now) > 0 && (now / 100) & 1)
                hw.SetLed(1, 1.f, 0.f, 0.f);
        }

        hw.UpdateLeds();
#if USB_MIDI
        {
            float vals[usbmidi::kValues];
            for(int k = 0; k < kKnobs; k++)
                vals[k] = cv[k].v;
            vals[kKnobs]     = inLevel[0];
            vals[kKnobs + 1] = inLevel[1];
            vals[kKnobs + 2] = cpuPeak > 1.f ? 1.f : cpuPeak;
            usbmidi::Update(now, vals, hw.gate.State());
        }
#endif
        System::Delay(1);
    }
}
