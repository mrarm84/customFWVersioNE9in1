// BeatMachineEngine.h - turns the send into a grainy, sampled beat.
//
// The send is listened to all the time. Every hit (onset) is cut out of
// the audio (350 ms, mono, 3 ms pre-roll) and sorted by its sound into one
// of 4 kit slots: KICK (lots of lows), HAT (lots of highs), SNARE / PERC
// (the rest, taking turns). A 16-step pattern locked to the beat (gate
// clock or tracked from the audio) plays the kit, then a sampler-style
// grit stage (sample rate + bits + drive + low-pass) makes it grainy.
// Until a slot has caught a sound from the input it plays a built-in
// synthetic sound, so a beat is there from the start.
//
// SUBMODE  row (SW0) = STYLE  BOOM BAP | FOUR ON THE FLOOR | BROKEN
//          col (SW1) = GRIT   SP (12 bit, 26 kHz) | TAPE (saturated, dark,
//                             hiss) | CRUSHED (8 bit, 11 kHz, fold)
// LAYER 1  K0 PATTERN (4 per style, the last one re-rolls every bar)
//          K1 DENSITY (core hits -> ghost notes)   K2 SEND
//          K3 PITCH of the kit (noon 1x, 1 V/oct)  K4 DECAY 30 ms..full
//          K5 GRIT amount   K6 SWING
// LAYER 2  K0 KIT SWAP (CCW: only empty slots fill, CW: sounds swap often)
//          K1 HUMANIZE (timing + velocity)   K2 LPF (send)
//          K3 FILL (rolls at the end of the bar)   K4 REVERSE chance
//          K5 RATCHET chance   K6 VINYL (crackle)
// double-tap = FREEZE the kit, hold+release = grab a fresh kit (every slot
// takes the next hit that fits)
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"
#include "BeatTracker.h"

namespace bm
{
using namespace daisy;
using namespace daisysp;

enum Slot
{
    KICK,
    SNARE,
    HAT,
    PERC,
    kSlots
};

static constexpr uint32_t kRing = 1u << 16, kMask = kRing - 1; // 1.36 s
static constexpr uint32_t kLen  = 16800;                       // 350 ms
static constexpr uint32_t kPre  = 144;                         // 3 ms
static float DSY_SDRAM_BSS ring[kRing];
static float DSY_SDRAM_BSS pool[kSlots + 1][kLen];

// 16-step patterns: digit = priority (1 = core hit, 9 = faint ghost),
// '.' = never. A hit plays when its priority fits under DENSITY.
// [style][pattern][slot]
static const char* const kPat[3][4][kSlots] = {
    {   // BOOM BAP
        {"1.....3.1.4.....", "....1.......1...", "2.3.2.3.2.3.2.3.", ".......6......7."},
        {"1..4..2...1.5...", "....1..6....1..5", "2.4.2.4.2.4.2.4.", "...7.....6......"},
        {"1.....1.4.2.....", "....1.......1.6.", "3.2.3.2.3.2.3.2.", ".5.......5..7..."},
        {"1.....3.1.4.....", "....1.......1...", "2.3.2.3.2.3.2.3.", ".......6......7."},
    },
    {   // FOUR ON THE FLOOR
        {"1...1...1...1...", "....1.......1...", "5.1.5.1.5.1.5.1.", "...6..5....6.5.."},
        {"1...1...1...1..5", "....1.......1.6.", "..1...1...1...1.", "7.4.7.4.7.4.7.4."},
        {"1...1...1...1...", "....1..7....1...", "3.1.3.1.3.1.3.1.", "..6.....6..5..4."},
        {"1...1...1...1...", "....1.......1...", "5.1.5.1.5.1.5.1.", "...6..5....6.5.."},
    },
    {   // BROKEN
        {"1.....2...1.4...", "....1..5.3..1..6", "2323232323232323", "..6...5..6...4.."},
        {"1..3......1..4..", "....1.6.....1.5.", "2.3.2.32.3.2.3.3", ".7....6.....5..."},
        {"1.4...1..2......", "....1.....6.1.4.", "3232323232323232", "...5.6.....5..6."},
        {"1.....2...1.4...", "....1..5.3..1..6", "2323232323232323", "..6...5..6...4.."},
    },
};

struct Voice
{
    float* buf   = nullptr;
    bool   cap   = false; // holds a sound from the input
    bool   play  = false;
    float  pos   = 0.f, speed = 1.f, vel = 1.f, env = 1.f, dir = 1.f;
    int    rat = 0;       // ratchet hits left
    float  ratT = 0.f, ratGap = 0.f;
};
static Voice v[kSlots];
static float* spare = nullptr; // the pool buffer being filled

static float    sr = 48000.f;
static uint32_t w = 0;
// onset detection (power followers, full band + high band)
static float    envF = 0.f, envS = 0.f, hF = 0.f, hS = 0.f, lpLo = 0.f, lpHi = 0.f;
// background band power just before a hit, and the hit's own energy
static float    bLo = 0.f, bHi = 0.f, bAll = 0.f, g0[3] = {0, 0, 0};
static float    eLo = 0.f, eHi = 0.f, eAll = 0.f, mRl = 0.5f, mRh = 0.2f;
static uint32_t refr = 0, clsT = 0, onsetAt = 0;
static bool     classifying = false;
static int      nextMid = SNARE;
// caught hits waiting until all 350 ms are recorded, then copied
struct Job
{
    uint32_t start;
    int      slot;
};
static Job      jobs[4];
static int      nJobs = 0;
static bool     copying = false;
static uint32_t copyAt = 0, capStart = 0;
static int      copySlot = 0;
static bool     want[kSlots] = {false, false, false, false};
static uint32_t lastAction = 0;
// sequencer
static int      lastStep = -1, patZ = 0;
static char     gen[kSlots][17]; // the re-rolled pattern
static bool     fillBar = false;
static pn::Rng  rng;
// grit
static float holdPh = 0.f, held[2] = {0, 0}, gLp[2] = {0, 0}, crk = 0.f;

static volatile float level = 0.f, stepFlash = 0.f, swapFlash = 0.f;
static volatile int   showStyle = 0, showCaught = 0;

// ------------------------------------------------- built-in sounds --------
inline void SynthKit()
{
    pn::Rng r;
    for(uint32_t i = 0; i < kLen; i++)
    {
        const float t = (float)i / sr;
        // kick: pitch-dropping sine
        const float ph = 6.2831853f * (48.f * t + 90.f * (1.f - expf(-t * 25.f)) / 25.f);
        v[KICK].buf[i] = sinf(ph) * expf(-t * 9.f) * 0.9f;
        // snare: body + noise
        v[SNARE].buf[i] = (0.5f * sinf(6.2831853f * 185.f * t) * expf(-t * 22.f)
                           + 0.6f * r.B() * expf(-t * 16.f))
                          * 0.8f;
        // hat: high noise
        static float hp = 0.f, x1 = 0.f;
        const float  n  = r.B();
        hp              = 0.6f * (hp + n - x1);
        x1              = n;
        v[HAT].buf[i]   = hp * expf(-t * 60.f) * 0.7f;
        // perc: wood block
        v[PERC].buf[i] = sinf(6.2831853f * 720.f * t) * expf(-t * 35.f) * 0.6f;
    }
    for(int s = 0; s < kSlots; s++)
        v[s].cap = false;
}

inline void Clear()
{
    memset(ring, 0, sizeof(ring));
    for(int s = 0; s < kSlots; s++)
        v[s].buf = pool[s], v[s].play = false;
    spare = pool[kSlots];
    SynthKit();
    classifying = copying = false;
    nJobs       = 0;
    showCaught  = 0;
}
inline void Init(float s)
{
    sr = s;
    Clear();
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    lastStep   = -1;
    for(int s = 0; s < kSlots; s++)
        want[s] = false;
}

// --------------------------------------------------------- sequencer ------
inline void Reroll(float density)
{
    // generative pattern: core hits from pattern 0, the rest at random
    const char* const* base = kPat[showStyle][0];
    for(int s = 0; s < kSlots; s++)
    {
        for(int k = 0; k < 16; k++)
        {
            const char c = base[s][k];
            if(c == '1')
                gen[s][k] = '1';
            else if(rng.F() < 0.12f + 0.25f * density)
                gen[s][k] = (char)('3' + rng.Next() % 6);
            else
                gen[s][k] = '.';
        }
        gen[s][16] = 0;
    }
}

inline void Hit(int s, float vel, float speed, float revCh, float ratCh, float stepS)
{
    Voice& x = v[s];
    x.play   = true;
    x.dir    = (revCh > 0.01f && rng.F() < revCh) ? -1.f : 1.f;
    x.pos    = x.dir > 0.f ? 0.f : (float)(kLen - 1);
    x.speed  = speed;
    x.vel    = vel;
    x.env    = 1.f;
    x.rat    = 0;
    if(ratCh > 0.01f && rng.F() < ratCh)
    {
        x.rat    = 1 + (int)(rng.Next() % 3);  // 2..4 hits in the step
        x.ratGap = stepS / (float)(x.rat + 1);
        x.ratT   = x.ratGap;
    }
}

// ------------------------------------------------------------ process -----
inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    const int style = p.row, grit = p.col;
    showStyle       = style;

    const int   pz      = pn::Zone(p.k[0], 4, patZ);
    const float density = p.k[1];
    const int   maxPri  = 1 + (int)(density * 8.99f);
    float       oct     = (p.k[3] - 0.5f) * 5.f; // 1 V/oct around noon
    if(fabsf(oct) < 0.04f)
        oct = 0.f;
    const float speed   = exp2f(oct);
    const float decT    = 0.03f * powf(40.f, p.k[4]); // 30 ms .. 1.2 s
    const bool  full    = p.k[4] > 0.97f;
    const float decC    = full ? 1.f : expf(-1.f / (decT * sr));
    const float g       = p.k[5];
    const float swing   = p.k[6] * 0.5f; // odd 16ths late by up to half a step
    const float swapP   = p.s[0] * p.s[0];
    const float human   = p.s[1];
    const float fillP   = p.s[3];
    const float revCh   = p.s[4];
    const float ratCh   = p.s[5];
    const float vinyl   = p.s[6];
    const bool  frozen  = p.a;

    if(p.action != lastAction)
    {
        lastAction = p.action;
        for(int s = 0; s < kSlots; s++)
            want[s] = true;
        swapFlash = 1.f;
    }

    // grit stage settings per column, scaled by GRIT
    float bits, rate, drive, lpHz, hiss;
    switch(grit)
    {
        case 0: bits = 12.f - 4.f * g; rate = 26000.f - 14000.f * g;
                drive = 1.f + 2.f * g; lpHz = 12000.f - 6000.f * g; hiss = 0.f; break;
        case 1: bits = 16.f - 6.f * g; rate = sr;
                drive = 1.5f + 6.f * g; lpHz = 7000.f - 4500.f * g; hiss = 0.004f + 0.01f * g; break;
        default: bits = 8.f - 4.f * g; rate = 11025.f - 7000.f * g;
                drive = 2.f + 8.f * g; lpHz = 9000.f - 5000.f * g; hiss = 0.f; break;
    }
    const float q     = exp2f(bits - 1.f);
    const float hInc  = fminf(rate / sr, 1.f);
    const float lpC   = pn::OnePole(lpHz, sr);
    const float dNorm = 1.f / tanhf(drive);
    const float stepS = bt::st.period * 0.25f; // a 16th in samples
    const float cF = pn::OnePole(100.f, sr), cS = pn::OnePole(4.f, sr);
    const float cBg = pn::OnePole(3.f, sr);
    const float cLo = pn::OnePole(200.f, sr), cHi = pn::OnePole(3000.f, sr);
    const uint32_t refrN = (uint32_t)(0.07f * sr);
    const uint32_t clsN  = (uint32_t)(0.06f * sr); // sound of the first 60 ms
    static const float kPan[kSlots] = {0.f, -0.1f, 0.35f, -0.35f};
    float pk = 0.f;

    for(size_t i = 0; i < size; i++)
    {
        // ---- listen: record, find hits, sort them by their sound
        const float x = 0.5f * (in[0][i] + in[1][i]);
        ring[w & kMask] = x;
        lpLo += (x - lpLo) * cLo;
        lpHi += (x - lpHi) * cHi;
        const float hi = x - lpHi;
        const float px = x * x, ph = hi * hi, pl = lpLo * lpLo;
        envF += (px - envF) * cF;
        envS += (px - envS) * cS;
        hF += (ph - hF) * cF;
        hS += (ph - hS) * cS;
        if(!classifying)
        {
            bLo += (pl - bLo) * cBg;
            bHi += (ph - bHi) * cBg;
            bAll += (px - bAll) * cBg;
        }
        if(refr > 0)
            refr--;
        if(classifying)
        {
            eLo += pl;
            eHi += ph;
            eAll += px;
            if(++clsT >= clsN)
            {
                classifying = false;
                // energy the hit ADDS over the background, per band,
                // compared with the recent hits of this input
                const float n    = (float)clsN;
                const float dLo  = fmaxf(0.f, eLo - g0[0] * n);
                const float dHi  = fmaxf(0.f, eHi - g0[1] * n);
                const float dAll = fmaxf(1e-12f, eAll - g0[2] * n);
                const float rl = dLo / dAll, rh = dHi / dAll;
                mRl += (rl - mRl) * 0.15f;
                mRh += (rh - mRh) * 0.15f;
                int s;
                if(rl > fmaxf(0.3f, mRl + 0.06f) || rl > 0.92f)
                    s = KICK;
                else if(rh > fmaxf(0.12f, mRh + 0.08f) || rh > 0.6f)
                    s = HAT;
                else
                {
                    s       = nextMid;
                    nextMid = nextMid == SNARE ? PERC : SNARE;
                }
                const bool take = !frozen
                                  && (want[s] || !v[s].cap
                                      || (swapP > 0.f && rng.F() < swapP));
                if(take && nJobs < 4)
                {
                    jobs[nJobs++] = {onsetAt - kPre, s};
                    want[s]       = false;
                }
            }
        }
        if(refr == 0
           && ((envF > 0.0004f && envF > 5.f * envS)
               || (hF > 0.00004f && hF > 5.f * hS)))
        {
            classifying = true;
            clsT        = 0;
            eLo = eHi = eAll = 0.f;
            g0[0]            = bLo;
            g0[1]            = bHi;
            g0[2]            = bAll;
            onsetAt          = w;
            refr             = refrN;
        }

        // ---- sequencer: 16 steps per bar, swing on the odd 16ths
        const float pos16 = bt::Pos(i, 4) * 4.f;
        const int   pair  = (int)(pos16 * 0.5f);
        const float f2    = pos16 - 2.f * (float)pair;
        int         step  = 2 * pair + (f2 >= 1.f + swing ? 1 : 0);
        if(step > 15)
            step = 15;
        if(step != lastStep)
        {
            if(step == 0)
            {
                fillBar = fillP > 0.01f && rng.F() < fillP;
                if(pz == 3)
                    Reroll(density);
            }
            lastStep  = step;
            stepFlash = (step & 3) == 0 ? 1.f : 0.4f;
            const bool fill = fillBar && step >= 12;
            for(int s = 0; s < kSlots; s++)
            {
                const char c = pz == 3 ? gen[s][step] : kPat[style][pz][s][step];
                int        pri = c >= '1' && c <= '9' ? c - '0' : 99;
                if(fill && s == SNARE && pri == 99 && rng.F() < 0.7f)
                    pri = 3; // fill: extra snares at the end of the bar
                if(pri > maxPri)
                    continue;
                float vel = 1.f - 0.07f * (float)(pri - 1);
                if(human > 0.01f)
                    vel *= 1.f - human * 0.4f * rng.F();
                const float rc = fill ? fmaxf(ratCh, 0.5f) : ratCh;
                Hit(s, vel, speed, revCh, rc, stepS);
            }
        }

        // ---- voices
        float o[2] = {0.f, 0.f};
        for(int s = 0; s < kSlots; s++)
        {
            Voice& vc = v[s];
            if(vc.rat > 0)
            {
                vc.ratT -= 1.f;
                if(vc.ratT <= 0.f)
                {
                    vc.rat--;
                    vc.ratT = vc.ratGap;
                    vc.play = true;
                    vc.pos  = vc.dir > 0.f ? 0.f : (float)(kLen - 1);
                    vc.env  = 1.f;
                    vc.vel *= 0.85f;
                }
            }
            if(!vc.play)
                continue;
            const uint32_t ip = (uint32_t)vc.pos;
            const float    fr = vc.pos - (float)ip;
            const float    a  = vc.buf[ip];
            const float    b  = vc.buf[ip + 1 < kLen ? ip + 1 : ip];
            float          y  = (a + (b - a) * fr) * vc.env * vc.vel;
            // fade the last 2 ms of the slice
            const float left = vc.dir > 0.f ? (float)kLen - vc.pos : vc.pos;
            if(left < 96.f)
                y *= left / 96.f;
            vc.env *= decC;
            vc.pos += vc.speed * vc.dir;
            if(vc.pos >= (float)(kLen - 1) || vc.pos < 0.f || vc.env < 0.0005f)
                vc.play = false;
            o[0] += y * (1.f - kPan[s]);
            o[1] += y * (1.f + kPan[s]);
        }

        // ---- grit: sample rate + bits + drive + low-pass (+ vinyl, hiss)
        holdPh += hInc;
        if(holdPh >= 1.f)
        {
            holdPh -= 1.f;
            held[0] = roundf(o[0] * q) / q;
            held[1] = roundf(o[1] * q) / q;
        }
        if(vinyl > 0.01f)
        {
            if(rng.F() < vinyl * vinyl * 0.002f)
                crk = rng.B() * 0.2f * vinyl;
            crk *= 0.3f;
        }
        for(int c = 0; c < 2; c++)
        {
            float y = grit == 2 ? sinf(pn::Clamp(held[c] * drive, -4.71f, 4.71f))
                                : tanhf(held[c] * drive) * dNorm;
            gLp[c] += (y - gLp[c]) * lpC;
            y = gLp[c] + crk + (hiss > 0.f ? rng.B() * hiss : 0.f);
            out[c][i] = pn::Lim(y * 0.8f);
            pk        = fmaxf(pk, fabsf(out[c][i]));
        }
        w++;
    }

    // ---- copy a caught sound into a spare buffer, a bit every block, then
    // swap it into its slot (no big copy inside one audio callback)
    // start the oldest waiting job once its 350 ms are all recorded
    if(!copying && nJobs > 0 && w - jobs[0].start >= kLen)
    {
        copying  = true;
        capStart = jobs[0].start;
        copySlot = jobs[0].slot;
        copyAt   = 0;
        for(int k = 1; k < nJobs; k++)
            jobs[k - 1] = jobs[k];
        nJobs--;
    }
    if(copying)
    {
        const uint32_t n = 1024;
        for(uint32_t k = 0; k < n && copyAt < kLen; k++, copyAt++)
        {
            float y = ring[(capStart + copyAt) & kMask];
            if(copyAt < kPre)
                y *= (float)copyAt / (float)kPre; // fade in the pre-roll
            spare[copyAt] = y;
        }
        if(copyAt >= kLen)
        {
            copying       = false;
            Voice& vc     = v[copySlot];
            float* old    = vc.buf;
            vc.buf        = spare;
            spare         = old;
            vc.cap        = true;
            vc.play       = false;
            swapFlash     = 1.f;
            int caught    = 0;
            for(int s = 0; s < kSlots; s++)
                caught += v[s].cap ? 1 : 0;
            showCaught = caught;
        }
    }
    level = fmaxf(pk, level * 0.95f);
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    static const float kC[3][3] = {
        {1.f, 0.6f, 0.f},  // BOOM BAP gold
        {0.f, 0.6f, 1.f},  // FOUR ON THE FLOOR blue
        {1.f, 0.f, 0.5f},  // BROKEN pink
    };
    const int   s  = showStyle;
    const float b0 = 0.15f + 0.85f * stepFlash;
    l.Set(0, kC[s][0] * b0, kC[s][1] * b0, kC[s][2] * b0);
    stepFlash = stepFlash * 0.75f;
    const float lv = fminf(level * 1.3f, 1.f);
    l.Set(1, lv, lv * 0.5f, 0.f);
    // 2: kit, green = how many slots hold sounds from the input, white flash
    //    when a sound is swapped in
    const float k = (float)showCaught / 4.f;
    const float f = swapFlash;
    l.Set(2, f, 0.1f + 0.9f * k * (1.f - f) + f, f);
    swapFlash = swapFlash * 0.85f;
    l.Set(3, pf, pf * 0.5f, 0.f);
    (void)now;
}

} // namespace bm
