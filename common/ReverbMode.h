// ReverbMode.h - StarLab reverb as a VersioBendSlice mode.
//
// SUBMODE  row (SW0) = TEXTURE  SPARSE | DENSE | DIFFUSE
//          col (SW1) = SHAPE    HALL    (plain tail)
//                               GATED   (tail opens with the input, cut after HOLD)
//                               REVERSE (tail played backwards in chunks: swells)
// LAYER 1  K0 SIZE/PITCH (noon unity)  K1 DECAY 0.2..60 s  K2 MIX  K3 TONE
//          K4 SHIMMER  K5 DELAY 0..1.5 s  K6 FEEDBACK (echoes)
// LAYER 2  K0 INTERVAL  -1 oct | -4th | +4th | +5th | +1 oct | +1 oct 5th
//          K1 MODULATION (chorus in the tail)
//          K2 GATE HOLD 50 ms..1 s (GATED) / CHUNK 0.1..1.5 s (REVERSE)
//          K3 DIFFUSION (noon = texture default)
//          K4 DUCK (tail ducks under the input)
//          K5 ECHO TONE 1..16 kHz   K6 WOBBLE (slow size/pitch drift)
// double-tap / gate = INFINITE, hold+release = clear the tail
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"
#include "../StarLabCommon/StarLabEngine.h"

namespace rvb
{
using namespace daisy;

static constexpr uint32_t kRev = 1u << 18, kRM = kRev - 1; // > 2 x 1.5 s
static float DSY_SDRAM_BSS rbuf[2][kRev];

static const float kInt[6] = {0.5f, 0.7491535f, 1.3348399f, 1.4983071f, 2.f, 2.9966142f};

static starlab::Params prm;
static float           sr = 48000.f, wobPh = 0.f, gateG = 0.f;
static uint32_t        rw = 0, holdCnt = 0, lastAction = 0;
static bool            lastGate = false, gateFlip = false;
static int             intZ = 4;
static pn::RevReader   rr[2];
static pn::Env         envIn;
volatile bool          clearReq = false; // handled in the main loop

static volatile int shape = 0;

inline void Init(float s)
{
    sr = s;
    starlab::Init(sr);
    memset(rbuf, 0, sizeof(rbuf));
}
inline void Clear()
{
    starlab::Init(sr);
    memset(rbuf, 0, sizeof(rbuf));
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    lastGate   = p.gate;
    gateFlip   = false;
    if(starlab::Frozen() != p.a)
        starlab::ToggleFreeze();
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    shape = p.col;
    if(p.gate && !lastGate)
        gateFlip = !gateFlip;
    lastGate = p.gate;
    if((p.a != gateFlip) != starlab::Frozen())
        starlab::ToggleFreeze();
    if(p.action != lastAction)
    {
        lastAction = p.action;
        clearReq   = true;
    }

    wobPh += 0.17f * (float)size / sr;
    if(wobPh >= 1.f)
        wobPh -= 1.f;
    prm.size     = pn::Clamp(p.k[0] + p.s[6] * 0.12f * sinf(6.2831853f * wobPh), 0.f, 1.f);
    prm.decay    = p.k[1];
    prm.mix      = 1.f; // wet only, mixed below
    prm.tone     = p.k[3];
    prm.shimmer  = p.k[4];
    prm.delay    = starlab::DelayKnobToSeconds(p.k[5]);
    prm.feedback = p.k[6];
    prm.texture  = p.row;
    prm.ratio    = kInt[pn::Zone(p.s[0], 6, intZ)];
    prm.modDepth = p.s[1] * 40.f;
    prm.difScale = p.s[3] * 2.f;
    prm.echoHz   = 1000.f * powf(16.f, p.s[5]);
    starlab::SetParams(prm);

    float wl[64], wr[64];
    const size_t n = size > 64 ? 64 : size;
    starlab::Process(in[0], in[1], wl, wr, n);

    const float hold  = (0.05f + 0.95f * p.s[2]) * sr;
    const float chunk = (0.1f + 1.4f * p.s[2]) * sr;
    const float duck  = p.s[4];
    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    for(size_t i = 0; i < n; i++)
    {
        const float e = envIn.Process(0.5f * (in[0][i] + in[1][i]), 0.2f, 0.0008f);
        float       x[2] = {wl[i], wr[i]};
        if(shape == 1)
        {
            if(e > 0.03f)
                holdCnt = (uint32_t)hold;
            else if(holdCnt > 0)
                holdCnt--;
            const float tg = holdCnt > 0 ? 1.f : 0.f;
            gateG += (tg - gateG) * (tg > gateG ? 0.01f : 0.0015f);
            x[0] *= gateG;
            x[1] *= gateG;
        }
        else if(shape == 2)
        {
            rbuf[0][rw & kRM] = x[0];
            rbuf[1][rw & kRM] = x[1];
            for(int c = 0; c < 2; c++)
            {
                float d1, g1, d2, g2;
                rr[c].Step(chunk, d1, g1, d2, g2);
                x[c] = pn::ReadFrac(rbuf[c], kRM, rw + 1, d1) * g1
                       + pn::ReadFrac(rbuf[c], kRM, rw + 1, d2) * g2;
            }
            rw++;
        }
        if(duck > 0.01f)
        {
            const float dg = 1.f - duck * fminf(e * 5.f, 1.f);
            x[0] *= dg;
            x[1] *= dg;
        }
        out[0][i] = pn::Lim(in[0][i] * dry + x[0] * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + x[1] * wetG);
    }
}

inline void Leds(LedOut& l, uint32_t now, const Panel& p)
{
    const float il = starlab::InPeak(), wl = starlab::WetPeak();
    l.Set(0, il > 0.95f ? 1.f : 0.f, fminf(il * 2.f, 1.f), 0.f);
    static const float kS[3][3] = {{0.5f, 0.5f, 1.f}, {1.f, 0.5f, 0.f}, {1.f, 0.f, 0.5f}};
    const int s = shape;
    const float b = fminf(wl * 1.5f + 0.05f, 1.f);
    l.Set(1, kS[s][0] * b, kS[s][1] * b, kS[s][2] * b);
    if(starlab::Frozen())
        l.Set(2, 0.f, 1.f, 1.f);
    else
        l.Set(2, p.k[4] * 0.4f, 0.f, p.k[4] * 0.6f);
    const float t  = starlab::DelayKnobToSeconds(p.k[5]);
    const bool  bl = t > 0.02f && (now % (uint32_t)(t * 1000.f)) < 30;
    l.Set(3, 0.f, bl ? 1.f : 0.f, bl ? 0.3f : 0.f);
}

} // namespace rvb
