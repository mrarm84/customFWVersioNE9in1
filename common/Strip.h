// Strip.h - LAYER 3 (hold button + turn): the same performance output strip
// in every mode. Neutral at its defaults.
//
//   K0 IN GAIN   x0.25 .. x4 into the engine (noon = x1)
//   K1 FILTER    DJ filter, noon off, CCW low-pass, CW high-pass
//   K2 WIDTH     mono .. normal (noon) .. 2x wide
//   K3 DRIVE     saturation
//   K4 CRUSH     sample-rate + bit reduction
//   K5 GATE      trance gate depth (1/16 of the clock, or 8 Hz free)
//   K6 SPACE     send into a dark stereo cross-feedback echo
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"

namespace strip
{
using namespace daisy;
using namespace daisysp;

static const float kDefault[7] = {0.5f, 0.5f, 0.5f, 0.f, 0.f, 0.f, 0.f};

static constexpr uint32_t kSp = 32768, kSM = kSp - 1;
static float DSY_SDRAM_BSS sp[2][kSp];

static float     sr = 48000.f;
static Svf       flt[2];
static float     holdPh = 0.f, held[2] = {0, 0}, gatePh = 0.f, gateS = 1.f;
static float     spLp[2] = {0, 0};
static uint32_t  w = 0;
static pn::Clock clk;

inline void Init(float s)
{
    sr = s;
    for(int c = 0; c < 2; c++)
    {
        flt[c].Init(sr);
        flt[c].SetRes(0.3f);
    }
    memset(sp, 0, sizeof(sp));
}

inline float InGain(const float* l3) { return powf(4.f, 2.f * l3[0] - 1.f); }

inline void Process(const float* l3, bool gate, float* L, float* R, size_t size)
{
    clk.Tick(gate, size, sr);

    // filter
    const float fd   = l3[1] - 0.5f;
    const bool  fOn  = fabsf(fd) > 0.03f;
    if(fOn)
    {
        const float hz = fd < 0.f ? 20000.f * powf(0.002f, -2.f * fd)
                                  : 20.f * powf(400.f, 2.f * fd);
        flt[0].SetFreq(fminf(hz, sr * 0.3f));
        flt[1].SetFreq(fminf(hz, sr * 0.3f));
    }
    const float width = l3[2] * 2.f;
    const float dr    = l3[3];
    const float dg    = 1.f + dr * dr * 19.f;
    const float dc    = 1.f / (1.f + dr * 1.2f);
    const float cr    = l3[4];
    const float hInc  = 1.f - cr * 0.95f;
    const float q     = exp2f(15.f - cr * 11.f);
    const float gd    = l3[5];
    const float gHz   = clk.Clocked(sr) ? 4.f * sr / (float)clk.period : 8.f;
    const float send  = l3[6];
    const float dL = 0.27f * sr, dR = 0.36f * sr;
    const float spC = pn::OnePole(3500.f, sr);

    float* io[2] = {L, R};
    for(size_t i = 0; i < size; i++)
    {
        float x[2] = {L[i], R[i]};
        for(int c = 0; c < 2; c++)
        {
            if(fOn)
            {
                flt[c].Process(x[c]);
                x[c] = fd < 0.f ? flt[c].Low() : flt[c].High();
            }
            if(dr > 0.01f)
                x[c] = SoftClip(x[c] * dg) * dc;
        }
        if(cr > 0.01f)
        {
            holdPh += hInc;
            if(holdPh >= 1.f)
            {
                holdPh -= 1.f;
                held[0] = x[0];
                held[1] = x[1];
            }
            x[0] = roundf(held[0] * q) / q;
            x[1] = roundf(held[1] * q) / q;
        }
        if(fabsf(width - 1.f) > 0.01f)
        {
            const float m = 0.5f * (x[0] + x[1]);
            const float s = 0.5f * (x[0] - x[1]) * width;
            x[0]          = m + s;
            x[1]          = m - s;
        }
        if(gd > 0.01f)
        {
            gatePh += gHz / sr;
            if(gatePh >= 1.f)
                gatePh -= 1.f;
            const float tgt = gatePh < 0.5f ? 1.f : 0.f;
            gateS += (tgt - gateS) * 0.02f;
            const float g = 1.f - gd * (1.f - gateS);
            x[0] *= g;
            x[1] *= g;
        }
        // space echo (always running so the tail rings out)
        {
            const float eL = pn::ReadFrac(sp[0], kSM, w, dL);
            const float eR = pn::ReadFrac(sp[1], kSM, w, dR);
            spLp[0] += (eL - spLp[0]) * spC;
            spLp[1] += (eR - spLp[1]) * spC;
            sp[0][w & kSM] = pn::LoopClip(x[0] * send + spLp[1] * 0.5f);
            sp[1][w & kSM] = pn::LoopClip(x[1] * send + spLp[0] * 0.5f);
            w++;
            x[0] += spLp[0];
            x[1] += spLp[1];
        }
        io[0][i] = pn::Lim(x[0]);
        io[1][i] = pn::Lim(x[1]);
    }
}

} // namespace strip
