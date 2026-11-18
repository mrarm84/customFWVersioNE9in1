// CloudEngine.h - granular processor over a live 10 s recording.
//
// SUBMODE  row (SW0) = PITCH SET  FIXED | OCTAVES (random +-1 oct) | FIFTHS
//          col (SW1) = ENGINE     CLOUD   (random grains around POSITION)
//                                 STRETCH (grains follow a slow play head:
//                                          time-stretch / freeze-scan)
//                                 SHIMMER (grains an octave up, fed back into
//                                          the recording: rising clouds)
// LAYER 1  K0 POSITION 0..8 s back   K1 SIZE 10 ms..1 s   K2 MIX
//          K3 DENSITY 0.5..100/s (CCW = gate/hold only; STRETCH min 12/s)
//          K4 PITCH (noon 1x, 1 V/oct)  K5 SPRAY 0..2 s  K6 TEXTURE
// LAYER 2  K0 SPREAD (stereo: straight -> random pan -> wide)
//          K1 SCAN (STRETCH: play-head speed -2..+2x, noon = frozen point;
//                   CLOUD/SHIMMER: position drift, noon = follows live)
//          K2 FEEDBACK (grains back into the recording)
//          K3 REVERSE %   K4 PITCH JITTER (+-1 st)
//          K5 ENVELOPE (CCW percussive .. noon bell .. CW reverse swell)
//          K6 VOICES 4..24
// double-tap = FREEZE recording, hold+release = burst of 8 grains,
// gate = spawn a grain
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"

namespace cld
{
using namespace daisy;
using namespace daisysp;

static constexpr uint32_t kRing = 1u << 19, kMask = kRing - 1;
static float DSY_SDRAM_BSS ringL[kRing];
static float DSY_SDRAM_BSS ringR[kRing];
static float* const        ring[2] = {ringL, ringR};

static constexpr int kG = 24;
struct Grain
{
    bool     on = false;
    uint32_t base;
    float    pos, speed, len, t, gl, gr, att;
    int      src; // -1 stereo, -2 mono sum, 0/1 that channel
};
static Grain g[kG];

static float     sr = 48000.f, countdown = 0.f, norm = 1.f, scanBack = 0.f;
static float     fbL = 0.f, fbR = 0.f, drift = 0.f, lastPos = 0.f;
static uint32_t  w = 0;
static bool      frozen = false, lastGate = false;
static uint32_t  lastAction = 0;
static pn::Rng   rng;

static volatile float spawnFlash = 0.f, level = 0.f;
static volatile int   engine = 0, pitchSet = 0;

inline void Clear()
{
    memset(ringL, 0, sizeof(ringL));
    memset(ringR, 0, sizeof(ringR));
    for(int i = 0; i < kG; i++)
        g[i].on = false;
    scanBack = 0.f;
}
inline void Init(float s)
{
    sr = s;
    Clear();
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    lastGate   = p.gate;
    countdown  = 0.f;
    scanBack   = p.k[0] * 8.f * sr;
    lastPos    = p.k[0];
    drift      = 0.f;
}

static const float kOct[4]    = {0.5f, 1.f, 1.f, 2.f};
static const float kFifths[6] = {0.5f, 0.6674f, 1.f, 1.4983f, 2.f, 1.f};

inline void Spawn(const Panel& p)
{
    const int maxV = 4 + (int)(p.s[6] * 20.f);
    int       idx  = -1;
    for(int i = 0; i < maxV && i < kG; i++)
        if(!g[i].on)
        {
            idx = i;
            break;
        }
    if(idx < 0)
        return;
    Grain& gr = g[idx];

    float oct = (p.k[4] - 0.5f) * 5.f;
    if(fabsf(oct) < 0.04f)
        oct = 0.f;
    float sp = exp2f(pn::Clamp(oct, -3.f, 2.f));
    if(p.row == 1)
        sp *= kOct[rng.Next() & 3];
    else if(p.row == 2)
        sp *= kFifths[rng.Next() % 6];
    if(engine == 2)
        sp *= 2.f; // SHIMMER
    sp *= exp2f(rng.B() * p.s[4] / 12.f);
    const float tex = p.k[6];
    const bool  rev = rng.F() < fmaxf(fmaxf(0.f, tex - 0.5f) * 1.6f, p.s[3]);
    if(rev)
        sp = -sp;

    const float len  = 0.01f * powf(100.f, p.k[1]) * sr;
    const float span = len * fabsf(sp);
    float       back = scanBack + p.k[5] * rng.F() * 2.f * sr;
    back = fmaxf(back, span + 256.f);
    back = fminf(back, (float)kRing - span - 8192.f);

    gr.base  = w - (uint32_t)back;
    gr.speed = sp;
    gr.pos   = sp < 0.f ? span : 0.f;
    gr.len   = len;
    gr.t     = 0.f;
    gr.att   = pn::Clamp(p.s[5], 0.02f, 0.98f);

    const float spread = p.s[0];
    if(spread < 0.05f)
    {
        gr.src = -1;
        gr.gl = gr.gr = 1.f;
    }
    else
    {
        const float pan = 0.5f + rng.B() * 0.5f * fminf(spread * 2.f, 1.f);
        gr.src = spread > 0.6f ? (int)(rng.Next() & 1) : -2;
        gr.gl  = sqrtf(1.f - pan) * 1.2f;
        gr.gr  = sqrtf(pan) * 1.2f;
    }
    gr.on      = true;
    spawnFlash = 1.f;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    pitchSet = p.row;
    engine   = p.col;
    frozen   = p.a;
    if(p.gate && !lastGate)
        Spawn(p);
    lastGate = p.gate;
    if(p.action != lastAction)
    {
        lastAction = p.action;
        for(int i = 0; i < 8; i++)
            Spawn(p);
    }

    float rate = 0.f;
    if(p.k[3] > 0.03f)
        rate = 0.5f * powf(200.f, (p.k[3] - 0.03f) / 0.97f);
    if(engine == 1)
        rate = fmaxf(rate, 12.f);
    const float sizeS   = 0.01f * powf(100.f, p.k[1]);
    const float overlap = fmaxf(1.f, rate * sizeS);
    norm += (1.f / sqrtf(overlap) - norm) * 0.05f;
    const float sharp = fminf(p.k[6] * 2.f, 1.f) * 6.f;

    // play head: STRETCH scans the buffer, others drift around POSITION
    {
        const float maxBack = 8.f * sr;
        const float v = engine == 1 ? (p.s[1] - 0.5f) * 4.f // -2..+2 x
                                    : 1.f + (p.s[1] - 0.5f) * 2.f; // 0..2 x
        // distance behind the write head grows when the head is slower
        // (CLOUD/SHIMMER: pure drift around POSITION, also when frozen)
        const float adv = (engine == 1 && frozen) ? -v : 1.f - v;
        if(fabsf(p.k[0] - lastPos) > 0.02f)
        {
            lastPos = p.k[0]; // POSITION moved: jump the head there
            drift   = 0.f;
        }
        drift += adv * (float)size;
        scanBack = p.k[0] * maxBack + drift;
        if(scanBack < 0.f || scanBack > maxBack)
        {
            drift    = 0.f; // ran off the recording: wrap to POSITION
            scanBack = p.k[0] * maxBack;
        }
    }

    const float fb  = fminf((engine == 2 ? 0.45f : 0.f) + p.s[2] * 0.85f, 0.95f);
    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    float       pk  = 0.f;
    for(size_t i = 0; i < size; i++)
    {
        if(!frozen)
        {
            ringL[w & kMask] = pn::Lim(in[0][i] + fbL * fb);
            ringR[w & kMask] = pn::Lim(in[1][i] + fbR * fb);
        }
        if(rate > 0.f)
        {
            countdown -= 1.f;
            if(countdown <= 0.f)
            {
                Spawn(p);
                countdown = sr / rate * (engine == 1 ? 1.f : 0.6f + 0.8f * rng.F());
            }
        }

        float oL = 0.f, oR = 0.f;
        for(int k = 0; k < kG; k++)
        {
            Grain& gr = g[k];
            if(!gr.on)
                continue;
            const float ph = gr.t / gr.len;
            // skewed window: attack portion = att
            float win = ph < gr.att ? ph / gr.att : (1.f - ph) / (1.f - gr.att);
            win       = win * win * (3.f - 2.f * win); // smooth
            win       = fminf(1.f, win * (1.f + sharp));
            if(gr.src == -1)
            {
                oL += pn::ReadAt(ringL, kMask, gr.base, gr.pos) * win;
                oR += pn::ReadAt(ringR, kMask, gr.base, gr.pos) * win;
            }
            else
            {
                float s;
                if(gr.src == -2)
                    s = 0.5f
                        * (pn::ReadAt(ringL, kMask, gr.base, gr.pos)
                           + pn::ReadAt(ringR, kMask, gr.base, gr.pos));
                else
                    s = pn::ReadAt(ring[gr.src], kMask, gr.base, gr.pos);
                s *= win;
                oL += s * gr.gl;
                oR += s * gr.gr;
            }
            gr.pos += gr.speed;
            if(gr.pos < 0.f)
                gr.pos = 0.f;
            gr.t += 1.f;
            if(gr.t >= gr.len)
                gr.on = false;
        }
        if(!frozen)
            w++;
        oL *= norm;
        oR *= norm;
        fbL = oL;
        fbR = oR;
        out[0][i] = pn::Lim(in[0][i] * dry + oL * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + oR * wetG);
        pk        = fmaxf(pk, fabsf(oL));
    }
    level = fmaxf(pk, level * 0.95f);
}

inline void Leds(LedOut& l, uint32_t now, float pf, const Panel& p)
{
    // 0: grain sparkle, colour = engine
    static const float kE[3][3] = {{0.7f, 0.8f, 1.f}, {0.2f, 1.f, 0.5f}, {1.f, 0.9f, 0.3f}};
    const int   e = engine;
    const float b = 0.15f + 0.85f * pf;
    l.Set(0, kE[e][0] * b, kE[e][1] * b, kE[e][2] * b);
    // 1: freeze / level
    if(p.a)
        l.Set(1, 0.8f, 0.9f, 1.f);
    else
    {
        const float lv = fminf(level * 1.5f, 1.f);
        l.Set(1, lv * 0.2f, lv * 0.4f, lv);
    }
    // 2: pitch set
    static const float kC[3][3] = {{0.3f, 0.3f, 0.3f}, {0.f, 1.f, 0.4f}, {1.f, 0.5f, 0.f}};
    const int m = pitchSet;
    l.Set(2, kC[m][0], kC[m][1], kC[m][2]);
    // 3: play position (how far back)
    const float pb = 0.05f + 0.95f * fminf(scanBack / (8.f * sr), 1.f);
    l.Set(3, pb * 0.5f, 0.f, pb);
    (void)now;
}

} // namespace cld
