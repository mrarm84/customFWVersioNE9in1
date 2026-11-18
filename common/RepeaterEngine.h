// RepeaterEngine.h - beat repeater locked to the beat tracker (bt::).
//
// The send is recorded all the time. At every WINDOW boundary on the beat
// grid the repeater decides (REPEAT knob) whether to grab a slice of
// LENGTH beats and loop it until the next window boundary. Loops restart
// exactly on the grid, so they stay in time. Outside a repeat the wet path
// is the send itself.
//
// SUBMODE  row (SW0) = GRID    plain repeat of the captured slice
//                      ROLL    the loop halves every 2 repeats (build-up)
//                      SCATTER every repeat plays a random slice of the
//                              last bar
//          col (SW1) = FORWARD | REVERSE (after the capture) | PING-PONG
// LAYER 1  K0 LENGTH 1/8, 1/6, 1/4, 1/3, 1/2, 1 beat, 2 beats, 1 bar
//          K1 REPEAT  chance per window, top 10 % = always (CV gate here
//                     holds the repeat)
//          K2 SEND    K3 DECAY per repeat   K4 PITCH STEP per repeat
//                     (+-7 semitones, noon none)
//          K5 GATE (slice length heard)     K6 WINDOW 1, 2, 4, 8 beats
// LAYER 2  K0 SPEED 0.5..2x (noon 1x)       K1 REVERSE chance
//          K2 LPF   K3 FADE 0.5..20 ms      K4 PING-PONG PAN
//          K5 DROP chance                   K6 DRIVE on the repeats
// double-tap = LATCH (repeat forever), hold+release = repeat at the next
// window
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"
#include "BeatTracker.h"

namespace rpt
{
using namespace daisy;
using namespace daisysp;

static constexpr uint32_t kRing = 1u << 19, kMask = kRing - 1;
static float DSY_SDRAM_BSS ringL[kRing];
static float DSY_SDRAM_BSS ringR[kRing];
static float* const        ring[2] = {ringL, ringR};

static const float kLen[8] = {0.125f, 0.1666667f, 0.25f, 0.3333333f,
                              0.5f,   1.f,        2.f,   4.f};
static const float kWin[4] = {1.f, 2.f, 4.f, 8.f};

static float    sr = 48000.f;
static uint32_t w = 0, start = 0, lastAction = 0;
static bool     on = false, shot = false, dropped = false;
static float    lenB = 1.f, lenS = 24000.f, pos = 0.f, speed = 1.f, gain = 1.f;
static float    dir = 1.f, xf = 0.f, lastBp = -1.f;
static int      reps = 0, lenZ = 5, winZ = 0, row = 0, col = 0;
static pn::Rng  rng;

static volatile float level = 0.f, repFlash = 0.f;
static volatile bool  showOn = false;
static volatile int   showRow = 0, showLen = 5;

inline void Clear()
{
    memset(ringL, 0, sizeof(ringL));
    memset(ringR, 0, sizeof(ringR));
    on = false;
    xf = 0.f;
}
inline void Init(float s)
{
    sr = s;
    Clear();
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    on         = false;
    shot       = false;
    xf         = 0.f;
    lastBp     = -1.f;
}

inline void SetLen()
{
    lenS = pn::Clamp(lenB * bt::st.period, 64.f, (float)(kRing / 4));
}

inline void Scatter()
{
    // a random slice of the last bar (already recorded: plays at once)
    int n = (int)(4.f / lenB + 0.5f);
    if(n < 1)
        n = 1;
    const uint32_t k = 1u + rng.Next() % (uint32_t)n;
    start            = w - (uint32_t)(lenS * (float)k);
}

inline void StartEvent(float baseLen, float winB)
{
    on      = true;
    reps    = 0;
    pos     = 0.f;
    gain    = 1.f;
    speed   = 1.f;
    dir     = 1.f;
    dropped = false;
    lenB    = baseLen > winB ? winB : baseLen;
    SetLen();
    start = w; // capture starts now: the first pass is the live send
    if(row == 2)
        Scatter();
    repFlash = 1.f;
}

inline void NextRep(float decay, float stepMul, float revCh, float dropCh)
{
    reps++;
    pos = 0.f;
    gain *= decay;
    speed = pn::Clamp(speed * stepMul, 0.125f, 4.f);
    if(row == 1 && (reps & 1) == 0 && lenB > 0.04f)
        lenB *= 0.5f; // roll: 1/4 -> 1/8 -> 1/16 -> 1/32 ...
    SetLen();
    if(col == 1)
        dir = -1.f;
    else if(col == 2)
        dir = (reps & 1) ? -1.f : 1.f;
    else
        dir = 1.f;
    if(revCh > 0.01f && rng.F() < revCh)
        dir = -dir;
    dropped = dropCh > 0.01f && rng.F() < dropCh;
    if(row == 2)
        Scatter();
    repFlash = 1.f;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    row = p.row;
    col = p.col;

    const float baseLen = kLen[pn::Zone(p.k[0], 8, lenZ)];
    const float k1      = p.k[1];
    const bool  always  = k1 > 0.9f || p.a;
    const float prob    = k1 / 0.9f;
    const float decay   = 1.f - p.k[3] * 0.6f;
    float       semi    = (p.k[4] - 0.5f) * 14.f;
    if(fabsf(semi) < 0.4f)
        semi = 0.f;
    const float stepMul = exp2f(semi / 12.f);
    const float duty    = p.k[5] > 0.98f ? 1.f : 0.05f + 0.95f * p.k[5];
    const float winB    = kWin[pn::Zone(p.k[6], 4, winZ)];

    float oct = (p.s[0] - 0.5f) * 2.f;
    if(fabsf(oct) < 0.05f)
        oct = 0.f;
    const float baseSpeed = exp2f(oct);
    const float revCh     = p.s[1];
    const float fadeS     = 0.5f * powf(40.f, p.s[3]) * 0.001f * sr;
    const float spread    = p.s[4];
    const float dropCh    = p.s[5];
    const float drv       = 1.f + p.s[6] * p.s[6] * 8.f;
    const float drvN      = 1.f / (1.f + (drv - 1.f) * 0.3f);

    if(p.action != lastAction)
        lastAction = p.action, shot = true;

    const float xfC = pn::OnePole(60.f, sr);
    float       pk  = 0.f;

    for(size_t i = 0; i < size; i++)
    {
        ringL[w & kMask] = in[0][i];
        ringR[w & kMask] = in[1][i];

        // ---- grid
        const float bp      = bt::Pos(i, 8);
        const bool  wrapped = lastBp >= 0.f && bp < lastBp;
        const bool  winX
            = lastBp >= 0.f
              && (wrapped || floorf(bp / winB) != floorf(lastBp / winB));
        const bool gridX
            = on && lastBp >= 0.f
              && (wrapped || floorf(bp / lenB) != floorf(lastBp / lenB));
        lastBp = bp;

        if(winX)
        {
            const bool trig = always || shot || (prob > 0.001f && rng.F() < prob);
            shot            = false;
            if(!trig)
                on = false;
            else if(!(on && always && row != 1))
                StartEvent(baseLen, winB); // latched GRID/SCATTER keep looping
            else
                NextRep(decay, stepMul, revCh, dropCh);
        }
        else if(gridX)
            NextRep(decay, stepMul, revCh, dropCh);

        // ---- play
        float rep[2] = {0.f, 0.f};
        if(on)
        {
            const bool  capture = reps == 0 && row != 2;
            const float rp      = dir > 0.f ? pos : lenS - 1.f - pos;
            float       env     = 1.f;
            if(!capture)
                env = fminf(env, pos / fadeS);
            env = fminf(env, (lenS - pos) / fadeS);
            env = fminf(env, (lenS * duty - pos) / fadeS);
            env = pn::Clamp(env, 0.f, 1.f) * gain * (dropped ? 0.f : 1.f);
            float gL = 1.f, gR = 1.f;
            if(spread > 0.01f && reps > 0)
            {
                const float pp = (reps & 1) ? -spread : spread;
                gL             = fminf(1.f, 1.f - pp);
                gR             = fminf(1.f, 1.f + pp);
            }
            const float rpc = pn::Clamp(rp, 0.f, lenS - 1.f);
            rep[0] = pn::ReadAt(ringL, kMask, start, rpc) * env * gL;
            rep[1] = pn::ReadAt(ringR, kMask, start, rpc) * env * gR;
            if(drv > 1.01f)
            {
                rep[0] = SoftClip(rep[0] * drv) * drvN;
                rep[1] = SoftClip(rep[1] * drv) * drvN;
            }
            pos += capture ? 1.f : speed * baseSpeed;
            if(pos >= lenS)
                pos -= lenS;
        }
        xf += ((on ? 1.f : 0.f) - xf) * xfC;
        const float oL = in[0][i] + (rep[0] - in[0][i]) * xf;
        const float oR = in[1][i] + (rep[1] - in[1][i]) * xf;
        out[0][i]      = oL;
        out[1][i]      = oR;
        pk             = fmaxf(pk, fabsf(oL) + fabsf(oR));
        w++;
    }
    level   = fmaxf(pk * 0.5f, level * 0.95f);
    showOn  = on;
    showRow = row;
    showLen = lenZ;
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    static const float kR[3][3] = {
        {1.f, 0.3f, 0.f},  // GRID orange
        {1.f, 0.f, 0.3f},  // ROLL red-pink
        {0.4f, 0.f, 1.f},  // SCATTER violet
    };
    const int   r  = showRow;
    const float b0 = 0.25f + 0.75f * pf;
    l.Set(0, kR[r][0] * b0, kR[r][1] * b0, kR[r][2] * b0);
    // 1: repeating (bright, flashes on every repeat) / level
    if(showOn)
    {
        const float f = 0.4f + 0.6f * repFlash;
        l.Set(1, f, f, f);
    }
    else
    {
        const float lv = fminf(level * 1.5f, 1.f);
        l.Set(1, 0.f, lv * 0.8f, lv * 0.3f);
    }
    repFlash = repFlash * 0.8f;
    // 2: length, dim = short, bright = whole bar
    const float b = 0.1f + 0.9f * (float)showLen / 7.f;
    l.Set(2, b * 0.2f, b * 0.6f, b);
    l.Set(3, pf, pf * 0.5f, 0.f);
    (void)now;
}

} // namespace rpt
