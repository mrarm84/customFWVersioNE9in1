// TrigDrumsEngine.h - 8-voice trigger drum machine.
//
// VOICES (trigger input -> voice; the knob above the jack tunes it)
//   jack 1 (K0) BD   kick         jack 5 (K4) CP   clap
//   jack 2 (K1) SD   snare        jack 6 (K5) MT   multi drum:
//   jack 3 (K2) OH   open hat                      tom / conga / rim / cowbell
//                    (CH chokes)  jack 7 (K6) SA   chord stab (dub techno)
//   jack 4 (K3) CH   closed hat,  gate        SB   resonator chord (dub)
//                    its triggers also clock the pattern (see the firmware)
// Every jack is knob + CV summed: a fast jump (trigger / gate) plays the
// voice (its height = velocity), the slow part (knob + slow CV) is the
// voice's parameter. USB MIDI notes play the voices too.
//
// SW0 right: a pattern plays (locked to the beat: MIDI clock / closed-hat
// clock, the beat tracked from the audio, or the free tempo). A voice that
// gets external triggers drops out of the pattern for 4 s.
//
// SUBMODE  SW0 x SW1: 8 KITS (drum machines), plus the SEQ at left/left
//            SW0 / SW1  left        mid           right
//            left       SEQ         ANALOG 808    PUNCH 909
//            mid        MINIMAL 606 RHYTHM BOX    DIGITAL (12 bit)
//            right      CRUNCH      DEEP          INDUSTRIAL (8 bit)
//          SEQ: the built-in pattern plays (+ trigs) with the kit used
//          last (a kit counts once its position is held 0.5 s, so the
//          positions passed on the way to left/left do not count)
//          double-tap: SEQ = next pattern (7), kits = next AUDIO IN mode:
//                             THRU   input passes (50 %), drums on top
//                             SKIN   the input becomes the noise of SD, CP,
//                                    hats and MT; the kick ducks the input
//                             CHOP   input heard only through the hits: BD
//                                    opens the lows, SD the mids, hats the
//                                    highs, CP / MT / synths their bands
// KNOBS edit ONE voice at a time (the page; the firmware steps through the
// 8 voices with taps). The same layout on every page:
//   K0 TUNE   K1 DECAY   K2 CHARACTER A   K3 CHARACTER B
//   K4 FILTER  noon = open; CCW low-pass, CW high-pass, the resonance
//              rising as you move away from noon (peaks mid-way, eases off
//              at the ends so it never screams)
//   K5 DRIVE   CCW clean; first half warm saturation, second half
//              turns into a folding distortion (level-matched)
//   K6 SEND    reverb up to 3/4, the last quarter adds the echo
//   Every hit gets a small random pan around the centre (the kick stays
//   in the middle).
//   character A / B:  BD click / drive · SD snappy / noise tail ·
//   OH, CH brightness / noise · CP spread / width · MT bend / noise
//   (MT TUNE sweeps tom > conga > rim > cowbell) · SA CHORD / FILTER ·
//   SB CHORD / BRIGHTNESS. SA and SB TUNE = chord root, 1 V/oct-ish
//   (C2..C7). Chords (CHARACTER A): m · m7 · m9 · m11 · m7sus4 ·
//   m(add9) · maj7 · open fifths. SA: detuned saws through a resonant
//   low-pass with a filter envelope (CHARACTER B: dark & soft -> bright &
//   resonant). SB: a noise burst rings 4-5 comb resonators tuned to the
//   chord (DECAY = ring time). Both send to the echo + reverb by default.
// SEND bus: tempo-synced ping-pong echo (dotted 8th) -> reverb.
// DUB (button held, momentary, smooth): every decay 20 % longer and all
// voices into the reverb (its tail also grows); fades out over 1.5 s.
// USB MIDI: GM drum notes
// play the voices, channel 2 / 3 notes play SA / SB at that pitch, MIDI
// clock sets the tempo
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"
#include "BeatTracker.h"

namespace tdr
{
using namespace daisy;
using namespace daisysp;

enum Voice
{
    BD,
    SD,
    CH,
    OH,
    CP,
    MT,
    SA,
    SB,
    kV
};

// trigger requests (velocity 0..1), from the jacks / gate (audio thread)
// and USB MIDI (main loop); consumed at the start of each block
static volatile float req[kV];
static volatile bool  reqExt[kV];          // came from outside (mutes pattern)
static volatile float midiNote[2] = {-1.f, -1.f}; // SA / SB pitch from MIDI

// voice parameters, written by the firmware's knob pages:
//   [voice][0 TUNE, 1 DECAY, 2 CHAR A, 3 CHAR B, 4 FILTER, 5 DRIVE, 6 SEND]
enum Param
{
    P_TUNE,
    P_DECAY,
    P_A,
    P_B,
    P_FILT,
    P_DRIVE,
    P_SEND,
    kP
};
static float prm[8][kP];
static const float kPrmDef[8][kP] = {
    {0.5f, 0.45f, 0.5f, 0.45f, 0.5f, 0.1f, 0.f},   // BD
    {0.5f, 0.4f, 0.5f, 0.4f, 0.5f, 0.1f, 0.f},     // SD
    {0.5f, 0.4f, 0.5f, 0.5f, 0.5f, 0.f, 0.f},      // CH
    {0.5f, 0.35f, 0.5f, 0.5f, 0.5f, 0.f, 0.f},     // OH
    {0.5f, 0.4f, 0.5f, 0.5f, 0.5f, 0.1f, 0.2f},    // CP
    {0.3f, 0.4f, 0.5f, 0.5f, 0.5f, 0.1f, 0.f},     // MT
    {0.35f, 0.4f, 0.35f, 0.45f, 0.5f, 0.15f, 0.82f}, // SA: A3 m9 stab, echo
    {0.15f, 0.5f, 0.2f, 0.5f, 0.5f, 0.f, 0.78f},     // SB: A2 m7 resonator
};
static volatile bool dubReq = false; // button held (firmware)
static volatile bool dubCut = false; // ALT hold-tweak: DUB out fast
static float         dub    = 0.f;   // smoothed 0..1
static volatile int  page   = 0;     // voice the knobs edit (for the LEDs)
static volatile bool allEdt = false; // CTRL-ALL: knobs edit all 8 voices

// small dub reverb: 4 damped combs + 2 allpasses per side (Freeverb-ish)
static constexpr int kCombs = 4, kAps = 2;
static const int     kCombLen[kCombs] = {1694, 1759, 1622, 1547};
static const int     kApLen[kAps]     = {605, 480};
static float DSY_SDRAM_BSS rvComb[2][kCombs][1800];
static float DSY_SDRAM_BSS rvAp[2][kAps][640];
static int                 rvCi[2][kCombs], rvAi[2][kAps];
static float               rvLp[2][kCombs], rvInLp = 0.f;

static constexpr float kTwoPi = 6.2831853f;

// voice levels into the bus, set from measured peaks so a single hit
// peaks at about: BD 0.55, SD 0.45, hats 0.3, CP / MT 0.35, synths 0.3,
// leaving headroom for hits that land together
static constexpr float kGainBd = 0.53f, kGainSd = 0.36f, kGainCp = 0.25f;
static constexpr float kGainMt = 0.23f, kGainSa = 0.5f, kGainSb = 0.6f;

// ------------------------------------------------------------- patterns ---
// digit = velocity (9 = accent), '.' = rest. Order: BD SD CH OH CP MT SA SB
static constexpr int kPats = 7;
static const char* const kPat[kPats][kV] = {
    // HOUSE
    {"9...9...9...9...", "................", "...3...3...3...3", "..7...7...7...7.",
     "....9.......9...", "......5.......5.", "..........7.....", "7..5..7...5..7.."},
    // TECHNO
    {"9...9...9...9...", "................", "5.3.5.3.5.3.5.3.", "..7...7...7...7.",
     "....7.......7...", "...4..4....4..4.", "......5.......5.", "9.5.9.5.9.5.9.5."},
    // BOOM BAP
    {"9......5.9......", "....9.......9...", "7.5.7.5.7.5.7.5.", "..............5.",
     "............5...", "...........3....", "7...........5...", "9......5.9......"},
    // BREAK
    {"9.........9.5...", "....9..3.5..9..5", "7575757575757575", "......5.........",
     "................", "..3.....3....3..", "....5.....5.....", "9.........9.5..."},
    // ELECTRO
    {"9.....9...9..5..", "....9.......9...", "3535353535353535", "................",
     "....5.......5...", "..5...5...5...5.", "9..5..9..5..9.5.", "9.....9...9..5.."},
    // PERC
    {"9.....5...9.....", "......5.......5.", "5.55.5.55.5.55.5", "................",
     "....7.......7...", "7.5.5.7..57.5.5.", "..5....5..5....5", "9..5....9..5...."},
    // DUB TECHNO: off-beat stab, a ringing chord, soft hats
    {"9...9...9...9...", "................", "..4...4...4...4.", "................",
     "................", "................", "..9.......7.....", "......7........."},
};
// synth note offsets per step (semitones)
static const int8_t kNoteA[kPats][16] = {
    {0, 0, 7, 0, 12, 0, 7, 0, 0, 0, 10, 0, 7, 0, 5, 0},
    {0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 12, 0},
    {12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0},
    {0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0},
    {0, 0, 0, 12, 0, 0, 7, 0, 0, 10, 0, 0, 7, 0, 3, 0},
    {0, 0, 7, 0, 0, 0, 0, 5, 0, 0, 12, 0, 0, 0, 0, 7},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};
static const int8_t kNoteB[kPats][16] = {
    {0, 0, 0, 12, 0, 0, 10, 0, 0, 0, 0, 7, 0, 0, 5, 0},
    {0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12, 0},
    {0, 0, 0, 0, 0, 0, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 5, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0, 0, 3, 0, 0},
    {0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

// chords (semitones above the root), mostly minor; -99 = unused
static constexpr int kChordMax = 5;
static const int8_t  kChord[8][kChordMax] = {
    {0, 3, 7, 12, -99},  // m
    {0, 3, 7, 10, -99},  // m7
    {0, 3, 7, 10, 14},   // m9
    {0, 3, 7, 10, 17},   // m11
    {0, 5, 7, 10, -99},  // m7sus4
    {0, 7, 14, 15, -99}, // m(add9), open voicing
    {0, 4, 7, 11, -99},  // maj7
    {0, 7, 12, 19, -99}, // open fifths
};

// ---------------------------------------------------------------- state ---
static float   sr = 48000.f, isr = 1.f / 48000.f;
static pn::Rng rng;

inline float Coef(float sec) { return expf(-1.f / (sec * sr)); }
inline float Sq(float ph) { return ph < 0.5f ? 1.f : -1.f; }
inline float Wrap(float p) { return p >= 1.f ? p - 1.f : p; }

// small TPT state-variable filter (band / low / high)
struct Svf2
{
    float ic1 = 0.f, ic2 = 0.f, g = 0.1f, k = 1.f;
    void  Set(float hz, float q)
    {
        g = tanf(3.1415927f * fminf(hz, sr * 0.45f) * isr);
        k = 1.f / q;
    }
    // returns band, writes low / high
    float Run(float x, float& lo, float& hi)
    {
        const float a1 = 1.f / (1.f + g * (g + k));
        const float v3 = x - ic2;
        const float v1 = a1 * ic1 + g * a1 * v3;
        const float v2 = ic2 + g * v1;
        ic1            = 2.f * v1 - ic1;
        ic2            = 2.f * v2 - ic2;
        lo             = v2;
        hi             = x - k * v1 - v2;
        return v1;
    }
};

struct Bd
{
    float ph = 0.f, pe = 0.f, amp = 0.f, cl = 0.f;
} bd;
struct Sd
{
    float p1 = 0.f, p2 = 0.f, pe = 0.f, body = 0.f, nz = 0.f, lp = 0.f;
} sd;
struct Hat
{
    float ph[6] = {0, 0.13f, 0.37f, 0.61f, 0.29f, 0.83f};
    float amp = 0.f, h1 = 0.f, h2 = 0.f, x1 = 0.f, x2 = 0.f, choke = 1.f;
} ch, oh;
struct Cp
{
    float    b = 0.f, tail = 0.f, vel = 1.f;
    uint32_t t = 99999;
    Svf2     bp;
} cp;
struct Mt
{
    float ph = 0.f, ph2 = 0.f, pe = 0.f, amp = 0.f, fast = 0.f, slap = 0.f;
    int   type = 0;
    bool  ping = false;
    Svf2  bp;
} mt;
// SA chord stab: two detuned band-limited saws per chord note
struct Stab
{
    float ph[kChordMax][2];
    float inc[kChordMax][2];
    int   n = 4;
    float amp = 0.f, fenv = 0.f, vel = 1.f;
    Svf2  lp;
} sa;
// SB resonator chord: a noise burst rings one comb per chord note
static constexpr uint32_t kRc = 2048, kRcM = kRc - 1;
static float DSY_SDRAM_BSS rcBuf[kChordMax][kRc];
struct Reso
{
    uint32_t w = 0;
    float    len[kChordMax];
    float    lp[kChordMax];
    int      n = 4, exc = 0;
    float    amp = 0.f, nlp = 0.f;
    uint32_t act = 0; // samples left to run
} sb;

// SEND bus echo: tempo-synced ping-pong (dotted 8th), feeds the reverb
static constexpr uint32_t kEch = 1u << 16, kEchM = kEch - 1;
static float DSY_SDRAM_BSS ech[2][kEch];
static uint32_t            ew = 0;
static float               eLp[2] = {0, 0}, eHp[2] = {0, 0}, eDs = 18000.f;

// band-limited saw (polyBLEP)
inline float Blep(float t, float dt)
{
    if(t < dt)
    {
        t /= dt;
        return t + t - t * t - 1.f;
    }
    if(t > 1.f - dt)
    {
        t = (t - 1.f) / dt;
        return t * t + t + t + 1.f;
    }
    return 0.f;
}
inline int ChordOf(float a)
{
    int z = (int)(a * 7.999f);
    return z < 0 ? 0 : (z > 7 ? 7 : z);
}

// per-voice FILTER (TPT state-variable, low-pass or high-pass) and the
// random pan of the current hit
static Svf2  vf[kV];
static float hitPan[kV];

// DRIVE: saturation that turns into a folding distortion, level-matched
// at the peak of a typical hit (0.5)
inline float Drive(float x, float pre, float m, float comp)
{
    const float z   = x * pre;
    const float sat = tanhf(z);
    if(m <= 0.f)
        return sat * comp;
    // distortion: the saturated signal folds over more and more
    const float fold = sinf(sat * (1.6f + 2.4f * m));
    return (sat + (fold - sat) * m) * comp;
}

// ----------------------------------------------------------------- kits ---
// character of each drum machine (voices + bus); synths are the same in all
struct Kit
{
    float bdBend, bdPcS, click, bdDecM, bdTuneM; // kick: pitch drop, its time
    float sdHp, sdTuneM, sdDecM, snapAdd;        // snare
    float hatNoise, hatG, hatScaleM, hatHpM, hatDecM;
    float cpToneM, cpDecM, mtTuneM, mtDecM;
    float busDrive;
    float q, holdHz; // grit: q = quantiser steps (0 = off), sample-hold rate
    float lvl;       // output level, so the kits play about as loud
};
static constexpr int kKits = 8;
static const Kit kKit[kKits] = {
    // ANALOG (808-ish): round, long pitch drop
    {2.2f, 0.02f, 0.35f, 1.f, 1.f, 900.f, 1.f, 1.f, 0.f, 0.08f, 3.f, 1.f, 1.f, 1.f,
     1.f, 1.f, 1.f, 1.f, 1.f, 0.f, 0.f, 1.00f},
    // PUNCH (909-ish): clicky kick, noisy hats, bus drive
    {3.2f, 0.008f, 0.9f, 1.f, 1.f, 1800.f, 1.f, 1.f, 0.f, 0.45f, 1.2f, 1.f, 1.f, 1.f,
     1.f, 1.f, 1.f, 1.f, 1.4f, 0.f, 0.f, 1.00f},
    // MINIMAL (606-ish): short, higher, thin, bright tight hats
    {1.6f, 0.012f, 0.5f, 0.55f, 1.25f, 2500.f, 1.2f, 0.7f, 0.2f, 0.2f, 2.2f, 1.35f, 1.3f, 0.7f,
     1.2f, 0.7f, 1.15f, 0.7f, 1.1f, 0.f, 0.f, 1.00f},
    // RHYTHM BOX (CR-78-ish): soft blips, short, metallic hats, no click
    {0.6f, 0.01f, 0.1f, 0.45f, 1.4f, 1200.f, 1.3f, 0.6f, -0.15f, 0.f, 3.2f, 0.85f, 0.8f, 0.6f,
     1.4f, 0.5f, 1.2f, 0.6f, 1.f, 0.f, 0.f, 1.00f},
    // DIGITAL (early sample drums): hard click, 12 bit / 28 kHz
    {2.8f, 0.006f, 1.f, 0.8f, 1.f, 1500.f, 1.05f, 1.f, 0.1f, 0.7f, 1.f, 1.1f, 1.1f, 0.9f,
     1.f, 1.f, 1.f, 1.f, 1.3f, 2048.f, 28000.f, 0.85f},
    // CRUNCH (gritty sampler): fat, driven, 10 bit / 22 kHz
    {2.4f, 0.015f, 0.7f, 1.1f, 0.9f, 700.f, 0.9f, 1.2f, 0.f, 0.5f, 1.4f, 0.9f, 0.85f, 1.f,
     0.85f, 1.2f, 0.9f, 1.1f, 2.2f, 512.f, 22050.f, 1.00f},
    // DEEP (dub): low, long, dark
    {1.8f, 0.035f, 0.15f, 1.6f, 0.8f, 600.f, 0.8f, 1.4f, -0.1f, 0.15f, 2.4f, 0.8f, 0.7f, 1.3f,
     0.8f, 1.5f, 0.85f, 1.3f, 1.15f, 0.f, 0.f, 0.95f},
    // INDUSTRIAL: huge pitch drop, noise, heavy drive, 8 bit / 16 kHz
    {4.5f, 0.01f, 1.2f, 1.f, 1.f, 1200.f, 0.95f, 1.f, 0.25f, 0.9f, 1.f, 1.6f, 0.9f, 1.f,
     1.f, 1.f, 1.f, 1.f, 3.5f, 256.f, 16000.f, 1.55f},
};

// pattern / external triggers
static int      lastStep = -1, pat = 0;
static bool     lastA = false;
static int      kitBase = 0;  // kit the SEQ plays (the one used last)
static int      kitPos  = -1; // switch position (0 = SEQ, 1..8 = kits)
static uint32_t kitHeld = 0;  // samples the position has been held
static int      audMode = 0;  // AUDIO IN: 0 THRU, 1 SKIN, 2 CHOP
static uint32_t extHold[kV]; // samples left during which the pattern is muted
// input processing
static float inEnv = 0.f, lo1 = 0.f, lo2 = 0.f, hiLp = 0.f;
static Svf2  inMid, inCp;
// grit
static float holdPh = 0.f, held[2] = {0, 0}, gLp[2] = {0, 0};

static volatile float hitShow[kV];
static volatile int   showPat = 0;
static volatile bool  showRun = true;

inline void Clear()
{
    memset(rcBuf, 0, sizeof(rcBuf));
    memset(ech, 0, sizeof(ech));
    sb.act = 0;
    memset(rvComb, 0, sizeof(rvComb));
    memset(rvAp, 0, sizeof(rvAp));
    memset(rvCi, 0, sizeof(rvCi));
    memset(rvAi, 0, sizeof(rvAi));
    memset(rvLp, 0, sizeof(rvLp));
    bd.amp = sd.body = sd.nz = ch.amp = oh.amp = cp.b = cp.tail = 0.f;
    mt.amp = sa.amp = sb.amp = 0.f;
    for(int v = 0; v < kV; v++)
        req[v] = 0.f, extHold[v] = 0;
}
inline void Init(float s)
{
    sr  = s;
    isr = 1.f / s;
    memcpy(prm, kPrmDef, sizeof(prm));
    Clear();
}

// one stereo reverb sample; fb = comb feedback (tail length)
inline void Reverb(float x, float fb, float& l, float& r)
{
    rvInLp += (x - rvInLp) * 0.45f; // darker input (~6 kHz)
    float o[2];
    for(int c = 0; c < 2; c++)
    {
        const int sp = c ? 23 : 0; // right side a little longer
        float     s  = 0.f;
        for(int k = 0; k < kCombs; k++)
        {
            float* b   = rvComb[c][k];
            const int L = kCombLen[k] + sp;
            int& i      = rvCi[c][k];
            const float y = b[i];
            rvLp[c][k] += (y - rvLp[c][k]) * 0.6f; // damping
            b[i] = rvInLp * 0.1f + rvLp[c][k] * fb;
            if(++i >= L)
                i = 0;
            s += y;
        }
        for(int k = 0; k < kAps; k++)
        {
            float* b   = rvAp[c][k];
            const int L = kApLen[k] + sp;
            int& i      = rvAi[c][k];
            const float bo = b[i];
            b[i]           = s + bo * 0.5f;
            s              = bo - s;
            if(++i >= L)
                i = 0;
        }
        o[c] = s;
    }
    l = o[0];
    r = o[1];
}
inline void Activate(const Panel& p)
{
    lastA    = p.a;
    lastStep = -1;
}

// ----------------------------------------------------------------- hits ---
inline void Hit(int v, float vel, float noteOff)
{
    hitShow[v] = fmaxf(hitShow[v], vel);
    // a small random place around the centre for every hit (kick: centre)
    hitPan[v] = v == BD ? 0.f : 0.3f * rng.B();
    switch(v)
    {
        case BD:
            bd.ph = 0.f, bd.pe = 1.f, bd.amp = vel, bd.cl = vel;
            break;
        case SD:
            sd.p1 = sd.p2 = 0.f, sd.pe = 1.f, sd.body = vel, sd.nz = vel;
            break;
        case CH:
            ch.amp = vel, ch.choke = 1.f;
            oh.choke = 0.f; // closed hat chokes the open one
            break;
        case OH:
            oh.amp = vel, oh.choke = 1.f;
            break;
        case CP:
            cp.t = 0, cp.b = vel, cp.tail = 0.f, cp.vel = vel;
            break;
        case MT:
            mt.ph = mt.ph2 = 0.f, mt.pe = 1.f, mt.amp = vel, mt.fast = vel,
            mt.slap = vel, mt.ping = true;
            break;
        case SA: // noteOff carries the root frequency
        {
            const int8_t* ch = kChord[ChordOf(prm[SA][P_A])];
            int           n  = 0;
            for(int j = 0; j < kChordMax && ch[j] > -99; j++, n++)
            {
                const float f = noteOff * exp2f((float)ch[j] / 12.f);
                sa.inc[j][0]  = f * exp2f(-0.07f / 12.f) * isr; // +-7 cents
                sa.inc[j][1]  = f * exp2f(0.07f / 12.f) * isr;
                if(sa.amp < 1e-3f)
                    sa.ph[j][0] = rng.F(), sa.ph[j][1] = rng.F();
            }
            sa.n = n, sa.amp = vel, sa.fenv = 1.f, sa.vel = vel;
            break;
        }
        case SB:
        {
            const int8_t* ch = kChord[ChordOf(prm[SB][P_A])];
            int           n  = 0;
            for(int j = 0; j < kChordMax && ch[j] > -99; j++, n++)
                sb.len[j] = pn::Clamp(sr / (noteOff * exp2f((float)ch[j] / 12.f)),
                                      16.f, (float)(kRc - 8));
            sb.n = n, sb.amp = vel, sb.exc = (int)(0.004f * sr);
            sb.act = (uint32_t)(8.f * sr); // longest ring, then it sleeps
            break;
        }
    }
}

// ------------------------------------------------------------- process ----
// k = layer 1 (tune + character), s = layer 2 (decay + second); in = the
// raw audio input (not the send), out = finished mix (dry included)
inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    // SW0 x SW1: left/left = SEQ (pattern + trigs, the kit used last),
    // the other 8 positions = kits. A kit becomes "used last" once held
    // 0.5 s, so passing over positions on the way to SEQ does not count.
    // double-tap: SEQ = next pattern, kits = next AUDIO IN mode
    const int pos = p.row * 3 + p.col;
    if(pos != kitPos)
        kitPos = pos, kitHeld = 0;
    else if(kitHeld < (uint32_t)sr)
        kitHeld += (uint32_t)size;
    if(pos > 0 && kitHeld >= (uint32_t)(0.5f * sr))
        kitBase = pos - 1;
    const bool run = pos == 0;
    if(p.a != lastA)
    {
        lastA = p.a;
        if(run)
            pat = (pat + 1) % kPats;
        else
            audMode = (audMode + 1) % 3;
    }
    const int  aud  = audMode;
    const Kit& K    = kKit[run ? kitBase : pos - 1];
    const bool grit = K.q > 0.f;
    showPat = pat;
    showRun = run;

    // ---- DUB: smooth in (0.15 s) and out (1.5 s); decays +20 %
    {
        const float tgt = dubReq ? 1.f : 0.f;
        const float st  = (float)size / ((dubReq || dubCut ? 0.15f : 1.5f) * sr);
        dub += pn::Clamp(tgt - dub, -st, st);
    }
    const float dl = 1.f + 0.2f * dub;

    // ---- kit character
    const float bdBend   = K.bdBend;
    const float bdPc     = Coef(K.bdPcS);
    const float hatNoise = K.hatNoise;
    const float sdHpHz   = K.sdHp;
    const float busDrive = K.busDrive;
    // hats: metal is mostly removed by the high-pass, so metal-only kits
    // get more gain than the noisy ones
    const float hatG = K.hatG;

    // ---- per-voice mix: random pan, FILTER, DRIVE, SEND (+ DUB)
    float gL[kV], gR[kV], snd[kV], esnd[kV];
    float dPre[kV], dM[kV], dComp[kV];
    bool  fOn[kV], fHp[kV];
    float fGain[kV];
    for(int v = 0; v < kV; v++)
    {
        const float pan = hitPan[v];
        gL[v]           = fminf(1.f, 1.f - pan);
        gR[v]           = fminf(1.f, 1.f + pan);
        // SEND: reverb over the first 3/4, the echo comes in on the last
        // quarter. DUB throws into both (the kick only a little: a thrown
        // kick muddies the echo and eats the headroom)
        const float k   = prm[v][P_SEND];
        const float dt  = dub * (v == BD ? 0.18f : 0.6f);
        snd[v]          = pn::Clamp(k / 0.75f + dt, 0.f, 1.f);
        esnd[v]         = pn::Clamp((k - 0.7f) / 0.3f, 0.f, 1.f) * 0.8f + dt;
        // FILTER: distance from noon = how far it closes, resonance peaks
        // mid-way and eases off towards the end
        const float fd = prm[v][P_FILT] - 0.5f;
        const float t  = fminf(fabsf(fd) * 2.f, 1.f);
        fOn[v]         = t > 0.03f;
        fHp[v]         = fd > 0.f;
        if(fOn[v])
        {
            const float q  = 0.7f + 7.f * t * (1.f - t) + 1.5f * t;
            const float hz = fHp[v] ? 30.f * exp2f(t * 8.5f)      // up to ~11 kHz
                                    : 18000.f * exp2f(-t * 8.f);  // down to ~70 Hz
            vf[v].Set(hz, q);
            fGain[v] = 1.f / (1.f + 0.15f * (q - 0.7f));
        }
        // DRIVE: first half saturation, second half folding distortion
        const float u = prm[v][P_DRIVE];
        dPre[v]       = 1.f + 9.f * u * u;
        dM[v]         = pn::Clamp((u - 0.5f) * 2.f, 0.f, 1.f);
        dComp[v]      = u > 0.02f ? 0.5f / tanhf(dPre[v] * 0.5f) : 0.f;
    }
    const float rvFb = 0.8f + 0.12f * dub; // the tail grows while dubbing

    // ---- voice parameters
    const float* Pb = prm[BD];
    const float  bdF      = 36.f * powf(2.6f, Pb[P_TUNE]) * K.bdTuneM; // 36..94 Hz
    const float  bdAc     = Coef(0.07f * powf(20.f, Pb[P_DECAY]) * dl * K.bdDecM);
    const float  clickLvl = K.click * 1.25f * Pb[P_A];
    const float  bdDrv    = 1.f + 4.f * Pb[P_B] * Pb[P_B];
    const float* Ps = prm[SD];
    const float  sdF   = 160.f * powf(2.1f, Ps[P_TUNE]) * K.sdTuneM; // 160..336 Hz
    const float  sdBc  = Coef((0.05f + 0.18f * Ps[P_DECAY]) * dl * K.sdDecM);
    const float  snap  = pn::Clamp(0.3f + 0.65f * Ps[P_A] + K.snapAdd, 0.f, 1.f);
    const float  sdNc  = Coef(0.07f * powf(7.f, Ps[P_B]) * dl * K.sdDecM); // noise tail
    const float  sdHpC = pn::OnePole(sdHpHz, sr);
    const float* Pc = prm[CH];
    const float* Po = prm[OH];
    const float  chScale = (0.7f + 0.9f * Pc[P_TUNE]) * K.hatScaleM;
    const float  ohScale = (0.7f + 0.9f * Po[P_TUNE]) * K.hatScaleM;
    const float  chAc  = Coef(0.012f * powf(12.f, Pc[P_DECAY]) * dl * K.hatDecM); // 12..144 ms
    const float  ohAc  = Coef(0.12f * powf(12.f, Po[P_DECAY]) * dl * K.hatDecM);  // 0.12..1.4 s
    const float  chHpC = pn::OnePole((5000.f + 4000.f * Pc[P_A]) * K.hatHpM, sr);
    const float  ohHpC = pn::OnePole((5000.f + 4000.f * Po[P_A]) * K.hatHpM, sr);
    const float  chNz  = pn::Clamp(hatNoise * 2.f * Pc[P_B], 0.f, 1.f);
    const float  ohNz  = pn::Clamp(hatNoise * 2.f * Po[P_B], 0.f, 1.f);
    const float  chokeC = Coef(0.004f);
    const float* Pp = prm[CP];
    const float  cpTone = 800.f * powf(3.f, Pp[P_TUNE]) * K.cpToneM;
    cp.bp.Set(cpTone, 1.2f + 3.f * Pp[P_B]);
    const uint32_t cpGap = (uint32_t)((0.008f + 0.008f * Pp[P_A]) * sr);
    const float  cpBc  = Coef(0.0035f);
    const float  cpTc  = Coef(0.06f * powf(7.f, Pp[P_DECAY]) * dl * K.cpDecM);
    // multi: TUNE sweeps type and pitch
    const float* Pm = prm[MT];
    const float  m  = Pm[P_TUNE];
    int          mType;
    float        mPos;
    if(m < 0.45f)
        mType = 0, mPos = m / 0.45f;
    else if(m < 0.7f)
        mType = 1, mPos = (m - 0.45f) / 0.25f;
    else if(m < 0.85f)
        mType = 2, mPos = (m - 0.7f) / 0.15f;
    else
        mType = 3, mPos = (m - 0.85f) / 0.15f;
    mt.type          = mType;
    const float mDec = 0.05f * powf(14.f, Pm[P_DECAY]) * dl * K.mtDecM;
    const float mBend = 2.f * Pm[P_A], mNoise = 2.f * Pm[P_B];
    float       mF;
    switch(mType)
    {
        case 0: mF = 70.f * powf(3.2f, mPos); break;    // tom
        case 1: mF = 180.f * powf(2.3f, mPos); break;   // conga
        case 2: mF = 1500.f * powf(1.8f, mPos); break;  // rim
        default: mF = 540.f * (0.8f + 0.5f * mPos); break; // cowbell
    }
    mF *= K.mtTuneM;
    if(mType == 2)
        mt.bp.Set(mF, 14.f);
    else if(mType == 3)
        mt.bp.Set(mF * 4.8f, 2.f);
    const float mAc   = Coef(mType == 1 ? mDec * 0.45f : (mType == 2 ? mDec * 0.25f : mDec));
    const float mFastC = Coef(0.015f);
    const float mPc   = Coef(mType == 0 ? 0.03f : 0.008f);
    // synths: TUNE 1 V/oct-ish over C2..C7
    const float* Pa = prm[SA];
    const float  synF  = 65.41f * exp2f(Pa[P_TUNE] * 5.f);
    // SA stab: amp decay 80 ms..2 s, filter envelope half as long;
    // CHARACTER B = filter: base cutoff, envelope depth, resonance together
    const float  saDec = 0.08f * powf(25.f, Pa[P_DECAY]) * dl;
    const float  saAc  = Coef(saDec);
    const float  saFc  = Coef(saDec * 0.45f);
    const float  fB    = Pa[P_B];
    const float  saCut = 140.f * (1.f + 3.f * fB)
                        * exp2f((2.f + 4.5f * fB) * sa.fenv * sa.vel);
    sa.lp.Set(fminf(saCut, 16000.f), 0.7f + 6.f * fB * fB);
    const float saNorm = 1.f / (2.f * (float)sa.n);
    // SB resonator: ring 0.2..6 s, CHARACTER B = brightness (comb damping
    // and the colour of the burst)
    const float* Pz = prm[SB];
    const float  sbF    = 65.41f * exp2f(Pz[P_TUNE] * 5.f);
    const float  sbRing = 0.2f * powf(30.f, Pz[P_DECAY]) * dl;
    const float  sbDmp  = 0.15f + 0.8f * Pz[P_B];
    float        sbFb[kChordMax];
    for(int j = 0; j < kChordMax; j++)
        sbFb[j] = powf(0.001f, sb.len[j] / (sbRing * sr));
    const float sbNorm = 1.f / (float)sb.n;
    // echo: dotted 8th of the beat, feedback grows with DUB
    const float eD  = pn::Clamp(bt::st.period * 0.75f, 480.f, (float)(kEch - 64));
    const float eFb = 0.42f + 0.26f * dub;
    const float eLc = pn::OnePole(3200.f, sr), eHc = pn::OnePole(120.f, sr);

    // ---- triggers: external first (they also mute the pattern), then the
    // pattern on 16th steps
    for(int v = 0; v < kV; v++)
    {
        const float r = req[v];
        if(r > 0.f)
        {
            req[v] = 0.f;
            if(reqExt[v])
                extHold[v] = (uint32_t)(4.f * sr);
            float f = 0.f;
            if(v == SA || v == SB)
            {
                const int   mi = v == SA ? 0 : 1;
                const float mn = midiNote[mi];
                f = mn >= 0.f ? 440.f * exp2f((mn - 69.f) / 12.f)
                              : (v == SA ? synF : sbF);
                midiNote[mi] = -1.f;
            }
            Hit(v, r, f);
        }
        if(extHold[v] > size)
            extHold[v] -= (uint32_t)size;
        else
            extHold[v] = 0;
    }
    {
        const float pos16 = bt::Pos(0, 4) * 4.f;
        int         step  = (int)pos16;
        if(step > 15)
            step = 15;
        if(step != lastStep)
        {
            lastStep = step;
            if(run)
                for(int v = 0; v < kV; v++)
                {
                    if(extHold[v] > 0)
                        continue;
                    const char c = kPat[pat][v][step];
                    if(c < '1' || c > '9')
                        continue;
                    const float vel = (float)(c - '0') / 9.f;
                    float       f   = 0.f;
                    if(v == SA)
                        f = synF * exp2f((float)kNoteA[pat][step] / 12.f);
                    if(v == SB)
                        f = sbF * exp2f((float)kNoteB[pat][step] / 12.f);
                    Hit(v, vel, f);
                }
        }
    }

    // ---- audio in
    const float inC = pn::OnePole(30.f, sr);
    const float c150 = pn::OnePole(150.f, sr), c6k = pn::OnePole(6000.f, sr);
    inMid.Set(sdF * 4.f, 1.2f);
    inCp.Set(cpTone, 2.f);

    // ---- grit (kits with a bit depth): quantiser + sample hold + smoothing
    const float q    = grit ? K.q : 2048.f;
    const float hInc = (grit ? K.holdHz : 24000.f) / sr;
    const float gC   = pn::OnePole(grit ? fminf(K.holdHz * 0.38f, 12000.f) : 9000.f, sr);
    // unity for small signals (peaks rounded), plus make-up so the three
    // kits play about as loud
    const float dN = (grit ? 1.75f : 1.35f) / busDrive;

    for(size_t i = 0; i < size; i++)
    {
        const float xin = 0.5f * (in[0][i] + in[1][i]);
        const float ax  = fabsf(xin);
        inEnv += (ax - inEnv) * inC;
        // noise source: white, or the input (level-normalised) in SKIN
        float nz = rng.B();
        if(aud == 1)
        {
            const float bl = pn::Clamp(inEnv * 30.f, 0.f, 1.f);
            const float xn = pn::Clamp(xin / (inEnv * 2.5f + 1e-3f), -2.f, 2.f);
            nz += (xn - nz) * bl;
        }

        // BD: pitch-dropping sine + click
        float oL = 0.f, oR = 0.f, rv = 0.f, ev = 0.f, oBd = 0.f;
        auto  mix = [&](int v, float x) {
            if(dComp[v] > 0.f)
                x = Drive(x, dPre[v], dM[v], dComp[v]);
            if(fOn[v])
            {
                float lo, hi;
                vf[v].Run(x, lo, hi);
                x = (fHp[v] ? hi : lo) * fGain[v];
            }
            oL += x * gL[v];
            oR += x * gR[v];
            rv += x * snd[v];
            ev += x * esnd[v];
        };
        if(bd.amp > 1e-4f || bd.cl > 1e-4f)
        {
            const float f = bdF * (1.f + bdBend * bd.pe);
            bd.pe *= bdPc;
            bd.ph = Wrap(bd.ph + f * isr);
            float y = sinf(kTwoPi * bd.ph) * bd.amp;
            bd.amp *= bdAc;
            y += rng.B() * bd.cl * clickLvl * 0.5f;
            bd.cl *= 0.97f; // ~0.7 ms click
            oBd = tanhf(y * bdDrv) / tanhf(bdDrv);
            mix(BD, oBd * kGainBd);
        }
        // SD: two body modes + high-passed noise
        float oSd = 0.f;
        if(sd.body > 1e-4f || sd.nz > 1e-4f)
        {
            const float f = sdF * (1.f + 0.5f * sd.pe);
            sd.pe *= 0.995f;
            sd.p1 = Wrap(sd.p1 + f * isr);
            sd.p2 = Wrap(sd.p2 + f * 1.62f * isr);
            const float body = (0.6f * sinf(kTwoPi * sd.p1) + 0.35f * sinf(kTwoPi * sd.p2))
                               * sd.body;
            sd.body *= sdBc;
            sd.lp += (nz - sd.lp) * sdHpC;
            const float n = (nz - sd.lp) * sd.nz;
            sd.nz *= sdNc;
            oSd = body * (1.f - 0.45f * snap) + n * snap * 1.1f;
            mix(SD, oSd * kGainSd);
        }
        // hats: six detuned squares (808 ratios) + noise, high-passed twice
        static const float kMetal[6] = {205.3f, 304.4f, 369.6f, 522.7f, 540.f, 800.f};
        Hat* hats[2]     = {&ch, &oh};
        const float sc[2] = {chScale, ohScale}, hc[2] = {chHpC, ohHpC};
        const float ac[2] = {chAc, ohAc}, hn[2] = {chNz, ohNz};
        for(int h = 0; h < 2; h++)
        {
            Hat& hh = *hats[h];
            if(hh.amp < 1e-4f)
                continue;
            float sum = 0.f;
            for(int j = 0; j < 6; j++)
            {
                hh.ph[j] = Wrap(hh.ph[j] + kMetal[j] * sc[h] * isr);
                sum += Sq(hh.ph[j]);
            }
            float x = sum * (1.f / 6.f) * (1.f - hn[h]) + nz * hn[h];
            hh.h1 += (x - hh.h1) * hc[h];
            const float y1 = x - hh.h1;
            hh.h2 += (y1 - hh.h2) * hc[h];
            const float y2 = y1 - hh.h2;
            hh.amp *= ac[h];
            if(hh.choke < 1.f)
                hh.amp *= chokeC;
            const float yh = y2 * hh.amp * hatG;
            mix(h == 0 ? CH : OH, yh);
        }
        // CP: three quick bursts, then a tail, band-passed noise
        float oCp = 0.f;
        if(cp.t < (uint32_t)(3.f * sr))
        {
            if(cp.t == cpGap || cp.t == 2 * cpGap)
                cp.b = fmaxf(cp.b, 0.85f * cp.vel);
            if(cp.t == 3 * cpGap)
                cp.tail = 0.75f * cp.vel;
            const float env = fmaxf(cp.b, cp.tail);
            cp.b *= cpBc;
            cp.tail *= cpTc;
            float lo, hi;
            oCp = cp.bp.Run(nz, lo, hi) * env * 2.2f;
            cp.t++;
            if(env < 1e-4f && cp.t > 3 * cpGap)
                cp.t = 99999;
            oCp *= kGainCp;
            mix(CP, oCp);
        }
        // MT: tom / conga / rim / cowbell
        float oMt = 0.f;
        if(mt.amp > 1e-4f)
        {
            float y;
            switch(mt.type)
            {
                case 0: // tom: sine with a deep pitch drop
                case 1: // conga: higher, tighter, a slap of noise
                {
                    const float bend = (mt.type == 0 ? 0.6f : 0.15f) * mBend;
                    mt.ph = Wrap(mt.ph + mF * (1.f + bend * mt.pe) * isr);
                    y     = sinf(kTwoPi * mt.ph);
                    if(mt.type == 1)
                    {
                        y += nz * mt.slap * 0.4f * mNoise;
                        mt.slap *= 0.993f;
                    }
                    else
                        y += nz * 0.04f * mNoise;
                    mt.pe *= mPc;
                    break;
                }
                case 2: // rim: ringing band-pass ping
                {
                    float lo, hi;
                    const float ex = mt.ping ? 1.f : 0.f;
                    mt.ping        = false;
                    y              = mt.bp.Run(ex * 8.f + nz * mt.fast * 0.3f * mNoise, lo, hi);
                    break;
                }
                default: // cowbell: two squares through a band-pass
                {
                    mt.ph  = Wrap(mt.ph + mF * isr);
                    mt.ph2 = Wrap(mt.ph2 + mF * 1.48f * isr);
                    float lo, hi;
                    y = mt.bp.Run(0.5f * (Sq(mt.ph) + Sq(mt.ph2)), lo, hi) * 1.6f;
                    y *= 0.35f + 0.65f * mt.fast;
                    break;
                }
            }
            mt.fast *= mFastC;
            oMt = y * mt.amp;
            mt.amp *= mAc;
            oMt *= kGainMt;
            mix(MT, oMt);
        }
        // SA: chord stab, detuned saws -> resonant low-pass
        if(sa.amp > 1e-4f)
        {
            float sum = 0.f;
            for(int j = 0; j < sa.n; j++)
                for(int u = 0; u < 2; u++)
                {
                    float&      ph = sa.ph[j][u];
                    const float dt = sa.inc[j][u];
                    ph             = Wrap(ph + dt);
                    sum += 2.f * ph - 1.f - Blep(ph, dt);
                }
            float lo, hi;
            sa.lp.Run(sum * saNorm, lo, hi);
            const float y = tanhf(lo * 1.4f) * sa.amp;
            sa.amp *= saAc;
            sa.fenv *= saFc;
            mix(SA, y * kGainSa);
        }
        // SB: resonator chord (one comb per chord note)
        if(sb.act > 0)
        {
            float x = 0.f;
            if(sb.exc > 0)
            {
                sb.nlp += (rng.B() - sb.nlp) * sbDmp;
                x = sb.nlp * sb.amp;
                sb.exc--;
            }
            float sum = 0.f;
            for(int j = 0; j < sb.n; j++)
            {
                const float y = pn::ReadFrac(rcBuf[j], kRcM, sb.w, sb.len[j]);
                sb.lp[j] += (y - sb.lp[j]) * sbDmp;
                rcBuf[j][sb.w & kRcM] = pn::LoopClip(x + sb.lp[j] * sbFb[j]);
                sum += y;
            }
            sb.w++;
            sb.act--;
            mix(SB, sum * sbNorm * kGainSb);
        }

        // ---- audio in: THRU / SKIN (ducked by the kick) / CHOP
        float dry = 0.f;
        if(aud == 0)
            dry = xin * 0.5f;
        else if(aud == 1)
            dry = xin * 0.5f * (1.f - 0.7f * fminf(bd.amp, 1.f));
        else
        {
            lo1 += (xin - lo1) * c150;
            lo2 += (lo1 - lo2) * c150;
            hiLp += (xin - hiLp) * c6k;
            float l, h;
            const float mid  = inMid.Run(xin, l, h);
            const float cpb  = inCp.Run(xin, l, h);
            const float hatE = ch.amp + oh.amp;
            dry = lo2 * 1.6f * fminf(bd.amp, 1.f) + mid * 1.4f * fmaxf(sd.body, sd.nz)
                  + (xin - hiLp) * 1.6f * fminf(hatE, 1.f)
                  + cpb * 1.4f * fmaxf(cp.b, cp.tail) + xin * 0.8f * fmaxf(mt.amp, sa.amp);
            dry *= 0.6f; // headroom: the chopped input sits under the drums
        }

        // ---- bus: drive, LO-FI grit, then the (clean) reverb
        float yo[2] = {oL, oR};
        if(busDrive > 1.01f)
            for(int c = 0; c < 2; c++)
                yo[c] = tanhf(yo[c] * busDrive) * dN;
        yo[0] *= K.lvl;
        yo[1] *= K.lvl;
        if(grit)
        {
            holdPh += hInc;
            if(holdPh >= 1.f)
            {
                holdPh -= 1.f;
                held[0] = roundf(yo[0] * q) / q;
                held[1] = roundf(yo[1] * q) / q;
            }
            for(int c = 0; c < 2; c++)
            {
                gLp[c] += (held[c] - gLp[c]) * gC;
                yo[c] = gLp[c];
            }
        }
        // SEND bus: ping-pong echo, then echo + send into the reverb
        eDs += (eD - eDs) * 0.0005f; // glide on tempo changes
        const float eL = pn::ReadFrac(ech[0], kEchM, ew, eDs);
        const float eR = pn::ReadFrac(ech[1], kEchM, ew, eDs);
        float       fbk[2] = {eR * eFb, eL * eFb}; // cross = ping-pong
        for(int c = 0; c < 2; c++)
        {
            eLp[c] += (fbk[c] - eLp[c]) * eLc;
            eHp[c] += (eLp[c] - eHp[c]) * eHc;
            fbk[c] = eLp[c] - eHp[c];
        }
        ech[0][ew & kEchM] = pn::LoopClip(ev + fbk[0]);
        ech[1][ew & kEchM] = pn::LoopClip(fbk[1]);
        ew++;
        float rl, rr;
        Reverb(rv + 0.5f * (eL + eR), rvFb, rl, rr);
        rl += 0.4f * eL;
        rr += 0.4f * eR;
        // DUB pulls the dry drums down a little (like a desk throw), so the
        // reverb can come up without hitting the limiter
        const float dd = 1.f - 0.3f * dub;
        out[0][i] = pn::Lim(yo[0] * dd + rl * 0.45f + dry);
        out[1][i] = pn::Lim(yo[1] * dd + rr * 0.45f + dry);
    }
}

inline void Leds(LedOut& l, uint32_t now, float bf)
{
    // 0 kick / snare, 1 hats, 2 clap / multi, 3 synths; dim glow = pattern
    // running (colour per pattern), dark = stopped
    auto d = [](int v) { float x = hitShow[v]; hitShow[v] = x * 0.8f; return x; };
    const float bdv = d(BD), sdv = d(SD), chv = d(CH), ohv = d(OH);
    const float cpv = d(CP), mtv = d(MT), sav = d(SA), sbv = d(SB);
    const float g   = showRun ? 0.06f : 0.f;
    l.Set(0, fminf(1.f, bdv + g), fminf(1.f, sdv * 0.6f + g), g);
    l.Set(1, fminf(1.f, chv * 0.8f + ohv), fminf(1.f, chv * 0.8f + ohv), fminf(1.f, chv * 0.3f + g));
    l.Set(2, fminf(1.f, cpv), fminf(1.f, mtv + g), fminf(1.f, cpv * 0.5f + mtv * 0.6f));
    // LED 3 = the voice the knobs edit (page), hit of that voice = flash
    static const float kPg[kV][3] = {
        {1.f, 0.f, 0.f},    // BD red
        {1.f, 0.4f, 0.f},   // SD orange
        {1.f, 1.f, 0.6f},   // CH pale yellow
        {1.f, 0.8f, 0.f},   // OH yellow
        {0.f, 1.f, 1.f},    // CP cyan
        {0.f, 1.f, 0.f},    // MT green
        {0.7f, 0.f, 1.f},   // SA violet
        {0.f, 0.3f, 1.f},   // SB blue
    };
    const int   pg = page;
    const float hv[kV] = {bdv, sdv, chv, ohv, cpv, mtv, sav, sbv};
    if(allEdt) // CTRL-ALL: white, flashes on any hit
    {
        float h = 0.f;
        for(int v = 0; v < kV; v++)
            h = fmaxf(h, hv[v]);
        const float b = 0.35f + 0.65f * fminf(h * 2.f, 1.f);
        l.Set(3, b, b, b);
    }
    else
    {
        const float b3 = 0.35f + 0.65f * fminf(hv[pg] * 2.f, 1.f);
        l.Set(3, kPg[pg][0] * b3, kPg[pg][1] * b3, kPg[pg][2] * b3);
    }
    // DUB: blue wash over LEDs 0..2
    const float dw = dub * 0.6f;
    for(int k = 0; k < 3; k++)
        l.Set(k, l.c[k][0], l.c[k][1], fminf(1.f, l.c[k][2] + dw));
    (void)now;
    (void)bf;
}

} // namespace tdr
