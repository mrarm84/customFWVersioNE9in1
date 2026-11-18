// Transition.h - the mode-change "send": for ~1 s the output passes
// through a transition effect whose depth rises to 100 % at the midpoint
// (where the engines are swapped, hidden inside the effect) and falls back.
// Every target mode has its own recipe built from a few primitives:
// stutter roll, tape-speed dip, bit crush, LP/HP sweep, ring sweep, echo.
#pragma once
#include "daisy_seed.h"
#include <string.h>
#include "Panel.h"

namespace trn
{
using namespace daisy;

struct Recipe
{
    float stutMs;  // stutter loop length at the start (0 = off)
    float stutEnd; // loop length factor at the peak (<1 = accelerating roll)
    float rateDip; // tape speed dip at the peak (1 = full stop)
    float crush;   // bit/rate crush at the peak
    float lp;      // low-pass closing at the peak
    float hp;      // high-pass opening at the peak
    float ring0;   // ring-mod sweep start Hz (0 = off)
    float ring1;   //               end Hz
    float echoMs;  // echo / comb send (0 = off)
    float echoFb;  //               feedback at the peak
    bool  scatter; // stutter jumps to random spots
};

static constexpr uint32_t kLen = 65536, kM = kLen - 1;
static constexpr uint32_t kEch = 16384, kEM = kEch - 1;
static float DSY_SDRAM_BSS buf[2][kLen];
static float DSY_SDRAM_BSS ech[2][kEch];

static Recipe   rc;
static float    sr = 48000.f, T = 48000.f;
static uint32_t w = 0, t = 0, ew = 0;
static bool     active = false, midPassed = false, stutOn = false;
static float    d = 1.f, ph = 0.f, ringPh = 0.f, holdPh = 0.f, hold[2] = {0, 0};
static float    lpS[2] = {0, 0}, hpS[2] = {0, 0}, stutLen = 1.f;
static uint32_t stutStart = 0;
static pn::Rng  rng;
static volatile float depth = 0.f; // for LEDs

inline void Init(float s, float seconds)
{
    sr = s;
    T  = seconds * s;
    memset(buf, 0, sizeof(buf));
    memset(ech, 0, sizeof(ech));
}

inline void Start(const Recipe& r)
{
    rc        = r;
    active    = true;
    midPassed = false;
    stutOn    = false;
    t         = 0;
    d         = 1.f;
    ph        = 0.f;
    lpS[0] = lpS[1] = hpS[0] = hpS[1] = 0.f;
}

inline bool  Active() { return active; }
inline bool  MidPassed() { return midPassed; }
inline float Progress() { return active ? (float)t / T : 0.f; }

inline void Process(float* L, float* R, size_t size)
{
    if(!active)
    {
        for(size_t i = 0; i < size; i++)
        {
            buf[0][w & kM] = L[i];
            buf[1][w & kM] = R[i];
            w++;
        }
        depth = 0.f;
        return;
    }

    // block-rate parameters from the depth envelope
    const float p = (float)t / T;
    const float e = sinf(3.1415927f * fminf(p, 1.f));
    depth         = e;
    const float rate  = 1.f - rc.rateDip * e;
    const float cr    = rc.crush * e;
    const float hInc  = 1.f - cr * 0.97f;
    const float q     = exp2f(15.f - cr * 12.f);
    const float lpC   = pn::OnePole(80.f + 18000.f * powf(1.f - rc.lp * e, 3.f), sr);
    const float hpC   = pn::OnePole(20.f + rc.hp * e * 3000.f, sr);
    const float rHz   = rc.ring0 + (rc.ring1 - rc.ring0) * p;
    const float eFb   = rc.echoFb * e;
    const float eD    = rc.echoMs * 0.001f * sr;
    const bool  stut  = rc.stutMs > 0.f && e > 0.15f;
    if(stut && !stutOn)
    {
        stutOn    = true;
        stutLen   = rc.stutMs * 0.001f * sr;
        stutStart = w - (uint32_t)stutLen;
        ph        = 0.f;
    }

    float* io[2] = {L, R};
    for(size_t i = 0; i < size; i++)
    {
        buf[0][w & kM] = L[i];
        buf[1][w & kM] = R[i];

        // source: stutter loop, or the tape-speed read head
        d += 1.f - rate;
        if(d > (float)(kLen - 64))
            d = (float)(kLen - 64);
        float win = 1.f;
        if(stutOn)
        {
            ph += rate; // the loop also sinks with the speed dip
            if(ph >= stutLen)
            {
                ph      = 0.f;
                stutLen = rc.stutMs * 0.001f * sr * (1.f + (rc.stutEnd - 1.f) * e);
                if(stutLen < 64.f)
                    stutLen = 64.f;
                if(rc.scatter)
                    stutStart = w - (uint32_t)stutLen
                                - (uint32_t)(rng.F() * 0.5f * sr);
            }
            win = fminf(1.f, fminf(ph, stutLen - ph) / 48.f);
        }
        holdPh += hInc;
        const bool take = holdPh >= 1.f;
        if(take)
            holdPh -= 1.f;
        const float s  = sinf(6.2831853f * ringPh);
        ringPh += rHz / sr;
        ringPh -= floorf(ringPh);

        for(int c = 0; c < 2; c++)
        {
            float y = stutOn ? pn::ReadAt(buf[c], kM, stutStart, ph) * win
                             : pn::ReadFrac(buf[c], kM, w + 1, d);
            if(take)
                hold[c] = y;
            y = cr > 0.001f ? roundf(hold[c] * q) / q : y;
            lpS[c] += (y - lpS[c]) * lpC;
            hpS[c] += (lpS[c] - hpS[c]) * hpC;
            y = lpS[c] - hpS[c];
            if(rc.ring0 > 0.f)
                y *= s;
            if(rc.echoMs > 0.f)
            {
                // first pass: nothing written yet this transition
                const float r = (float)(t + i) < eD
                                    ? 0.f
                                    : pn::ReadFrac(ech[c], kEM, ew, eD);
                ech[c][ew & kEM] = pn::LoopClip(y + r * eFb);
                y += r;
            }
            io[c][i] = pn::Lim(io[c][i] * (1.f - e) + y * e);
        }
        w++;
        ew++;
    }

    t += size;
    if(!midPassed && t >= (uint32_t)(T * 0.5f))
        midPassed = true;
    if(t >= (uint32_t)T)
        active = false;
}

// one signature transition per mode (order = Mode enum)
static const Recipe kRecipe[9] = {
    // stutMs stutEnd rateDip crush lp    hp    ring0 ring1  echoMs echoFb scatter
    {80.f, 0.25f, 0.f, 0.6f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, false},     // BENDER: crushed glitch roll
    {125.f, 0.12f, 0.f, 0.f, 0.f, 0.6f, 0.f, 0.f, 0.f, 0.f, false},    // SLICER: accelerating beat-repeat + HP rise
    {40.f, 0.5f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, true},        // ALT: crushed scatter
    {0.f, 0.f, 0.f, 0.f, 0.3f, 0.f, 0.f, 0.f, 4.5f, 0.93f, false},     // RESO: metallic comb ring
    {0.f, 0.f, 0.f, 0.f, 0.85f, 0.5f, 0.f, 0.f, 0.f, 0.f, false},      // FORMANT: band-pass squeeze
    {0.f, 0.f, 1.f, 0.f, 0.4f, 0.f, 0.f, 0.f, 0.f, 0.f, false},        // TAPE: tape stop -> tape start
    {0.f, 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 110.f, 0.85f, false},    // REVERB: dark echo wash
    {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 30.f, 1800.f, 60.f, 0.6f, false},   // SHIFTER: ring sweep spiral
    {60.f, 1.f, 0.3f, 0.f, 0.3f, 0.f, 0.f, 0.f, 0.f, 0.f, true},       // CLOUD: scattered grains, sinking
};

} // namespace trn
