// ResonatorEngine.h - 4 tuned comb resonators per channel.
//
// SUBMODE  row (SW0) = MODEL     STRING | TUBE (hollow, odd) | BELL (dispersive)
//          col (SW1) = STRUCTURE CHORD  | HARMONIC (partial sets) | CLUSTER
// LAYER 1  K0 PITCH (C1..C6, 1 V/oct)  K1 SHAPE (chord / partial set / width)
//          K2 MIX  K3 DECAY 50 ms..20 s  K4 BRIGHT  K5 SPREAD  K6 DRIVE
// LAYER 2  K0 FINE +-1 st      K1 QUANTIZE (CW half = semitones)
//          K2 INPUT level      K3 PLUCK level      K4 DISPERSION
//          K5 DECAY TILT (noon even; CCW highs die first, CW lows)
//          K6 GLIDE 0..1 s
// double-tap = INFINITE, hold+release = STRUM (voices plucked one by one),
// gate = pluck all voices
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"

namespace rez
{
using namespace daisy;
using namespace daisysp;

static constexpr int      kV   = 4;
static constexpr uint32_t kLen = 4096, kM = kLen - 1;
static float DSY_SDRAM_BSS buf[2][kV][kLen];

struct Comb
{
    uint32_t w = 0;
    float    d = 100.f, g = 0.f, exG = 1.f, lp = 0.f, apz = 0.f;
};
static Comb cb[2][kV];

static const int8_t kChord[8][4] = {{0, 0, 0, 0},
                                    {-12, 0, 12, 24},
                                    {0, 7, 12, 19},
                                    {0, 4, 7, 12},
                                    {0, 3, 7, 12},
                                    {0, 5, 7, 12},
                                    {0, 4, 7, 10},
                                    {0, 3, 7, 10}};
static const float kHarm[8][4] = {{1.f, 2.f, 3.f, 4.f},
                                  {1.f, 3.f, 5.f, 7.f},
                                  {1.f, 2.f, 4.f, 8.f},
                                  {1.f, 1.5f, 2.f, 3.f},
                                  {1.f, 2.756f, 5.404f, 8.933f}, // bar
                                  {1.f, 1.25f, 1.5f, 2.f},
                                  {1.f, 2.32f, 4.25f, 6.63f}, // plate
                                  {0.5f, 1.f, 2.f, 4.f}};
static const float kMicro[4] = {0.03f, -0.05f, 0.07f, -0.02f};

static float    sr = 48000.f;
static float    pl[kV], plDec = 0.998f, dcX[2] = {0, 0}, dcY[2] = {0, 0};
static float    dCur[2][kV];
static uint32_t strum[kV];
static bool     lastGate = false;
static uint32_t lastAction = 0;
static int      shapeZ = 0;
static pn::Rng  rng;

static volatile float level = 0.f, pluckShow = 0.f;
static volatile int   model = 0, structure = 0, shapeShown = 0;
static volatile bool  infinite = false;

inline void Clear()
{
    memset(buf, 0, sizeof(buf));
    for(int c = 0; c < 2; c++)
        for(int v = 0; v < kV; v++)
        {
            cb[c][v].lp = cb[c][v].apz = 0.f;
            dCur[c][v]                 = 0.f;
        }
}
inline void Init(float s)
{
    sr    = s;
    plDec = expf(-1.f / (0.006f * sr));
    Clear();
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    lastGate   = p.gate;
    for(int v = 0; v < kV; v++)
        pl[v] = 0.f, strum[v] = 0;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    model     = p.row;
    structure = p.col;
    infinite  = p.a;

    if(p.gate && !lastGate)
    {
        for(int v = 0; v < kV; v++)
            pl[v] = 1.f;
        pluckShow = 1.f;
    }
    lastGate = p.gate;
    if(p.action != lastAction)
    {
        lastAction = p.action;
        for(int v = 0; v < kV; v++)
            strum[v] = 1 + (uint32_t)((float)v * 0.035f * sr);
        pluckShow = 1.f;
    }

    // pitch
    float semis = p.k[0] * 60.f;
    if(p.s[1] > 0.5f)
        semis = roundf(semis);
    semis += (p.s[0] - 0.5f) * 2.f;
    const float root = 32.703f * exp2f(semis / 12.f);

    const int   z      = pn::Zone(p.k[1], 8, shapeZ);
    shapeShown         = z;
    const float t60    = 0.05f * powf(400.f, p.k[3]);
    const float lpC    = pn::OnePole(150.f * powf(120.f, p.k[4]), sr);
    const float spread = p.k[5];
    const float drive  = 0.25f + p.k[6] * p.k[6] * 6.f;
    const float sign   = model == 1 ? -1.f : 1.f;
    const float apC    = fminf((model == 2 ? 0.55f : 0.f) + p.s[4] * 0.4f, 0.9f);
    const float inLvl  = p.s[2] * 2.f;
    const float plLvl  = p.s[3] * 1.2f;
    const float tilt   = (p.s[5] - 0.5f) * 4.f;
    const float glideC = p.s[6] < 0.01f
                             ? 1.f
                             : 1.f - expf(-(float)size / (p.s[6] * sr));

    for(int c = 0; c < 2; c++)
        for(int v = 0; v < kV; v++)
        {
            float ratio;
            switch(structure)
            {
                case 0:
                {
                    float st = kChord[z][v];
                    if(z == 0)
                        st += ((float)v - 1.5f) * 0.07f;
                    ratio = exp2f(st / 12.f);
                    break;
                }
                case 1: ratio = kHarm[z][v]; break;
                default:
                {
                    // cluster: width 0..2 semitones around the root
                    const float w  = p.k[1] * 2.f;
                    const float st = w * ((float)v / 1.5f - 1.f) + kMicro[v];
                    ratio          = exp2f(st / 12.f);
                    break;
                }
            }
            if(c == 1)
                ratio *= exp2f(spread * 0.2f / 12.f);
            float d = sr / (root * ratio);
            if(model == 1)
                d *= 0.5f;
            d = pn::Clamp(d, 2.f, (float)(kLen - 8));
            float& dc = dCur[c][v];
            dc        = dc < 1.f ? d : dc + (d - dc) * glideC;
            Comb& cm  = cb[c][v];
            cm.d      = dc;
            const float tv = t60 * exp2f(tilt * ((float)v / 3.f - 0.5f));
            cm.g   = infinite ? 1.f : powf(0.001f, dc / (tv * sr));
            cm.exG = 0.1f + 0.9f * sqrtf(fmaxf(1.f - cm.g, 0.f));
        }

    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    const float pan = 1.f - spread * 0.6f;
    float       pk  = 0.f;
    for(size_t i = 0; i < size; i++)
    {
        float nz[kV];
        for(int v = 0; v < kV; v++)
        {
            if(strum[v] > 0 && --strum[v] == 0)
                pl[v] = 1.f;
            nz[v] = rng.B() * pl[v] * plLvl;
            pl[v] *= plDec;
        }
        float o[2];
        for(int c = 0; c < 2; c++)
        {
            const float xin = in[c][i];
            dcY[c]          = xin - dcX[c] + 0.995f * dcY[c];
            dcX[c]          = xin;
            const float x0  = infinite ? 0.f : dcY[c] * drive * 0.3f * inLvl;

            float sum = 0.f;
            for(int v = 0; v < kV; v++)
            {
                Comb&       cm = cb[c][v];
                const float y  = pn::ReadFrac(buf[c][v], kM, cm.w, cm.d);
                cm.lp += (y - cm.lp) * lpC;
                float s = cm.lp;
                if(apC > 0.f)
                {
                    const float t = s - apC * cm.apz;
                    s             = cm.apz + apC * t;
                    cm.apz        = t;
                }
                const float x = x0 + nz[v] * 0.8f;
                buf[c][v][cm.w & kM] = pn::LoopClip(x * cm.exG + sign * cm.g * s);
                cm.w++;
                const bool odd = v & 1;
                sum += y * ((odd == (c == 0)) ? pan : 1.f);
            }
            o[c] = sum * 0.35f;
        }
        out[0][i] = pn::Lim(in[0][i] * dry + o[0] * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + o[1] * wetG);
        pk        = fmaxf(pk, fabsf(o[0]) + fabsf(o[1]));
    }
    level = fmaxf(pk * 0.5f, level * 0.95f);
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    static const float kM3[3][3] = {
        {1.f, 0.7f, 0.f}, // STRING gold
        {0.f, 1.f, 0.3f}, // TUBE green
        {0.f, 0.6f, 1.f}, // BELL ice blue
    };
    const int m = model;
    l.Set(0, kM3[m][0] * 0.6f, kM3[m][1] * 0.6f, kM3[m][2] * 0.6f);
    const float lv = fminf(level * 1.5f, 1.f);
    l.Set(1, lv, lv * 0.5f, lv * 0.1f);
    // structure colour, brightness = shape index
    static const float kS[3][3] = {{1.f, 1.f, 1.f}, {1.f, 0.5f, 0.f}, {1.f, 0.f, 0.6f}};
    const int   s = structure;
    const float b = 0.2f + 0.8f * (float)shapeShown / 7.f;
    l.Set(2, kS[s][0] * b, kS[s][1] * b, kS[s][2] * b);
    if(infinite)
        l.Set(3, 0.8f, 0.9f, 1.f);
    else
        l.Set(3, pf, pf, pf);
    (void)now;
}

} // namespace rez
