// FormantEngine.h - morphing vowel filter.
//
// SUBMODE  row (SW0) = MOD SOURCE  LFO | RANDOM (gliding S&H) | ENVELOPE (talk-box)
//          col (SW1) = VOICE       VOWEL (formants on the input)
//                                  CHOIR (two detuned formant sets, wide)
//                                  ROBOT (internal buzz carrier, shaped by the
//                                         input's envelope, sung through the
//                                         formants: vocoder-like)
// LAYER 1  K0 VOWEL U-O-A-E-I  K1 SIZE 0.5x..2x  K2 MIX  K3 RESONANCE
//          K4 RATE 0.05..20 Hz (clock overrides)  K5 DEPTH  K6 DRIVE
// LAYER 2  K0 ROBOT: carrier pitch C1..C5 / VOWEL+CHOIR: BREATH (noise,
//             right of noon)
//          K1 STEREO offset (R vowel ahead, R LFO phase)
//          K2 ENV ATTACK   K3 ENV RELEASE   K4 RANDOM GLIDE
//          K5 LFO SHAPE sine -> square      K6 NASAL (4th formant)
// double-tap = HOLD modulation, hold+release = new random vowel + restart,
// gate = clock
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include "Panel.h"

namespace fmt
{
using namespace daisy;
using namespace daisysp;

// U O A E I (dark -> bright)
static const float kF[5][3] = {{300.f, 870.f, 2240.f},
                               {570.f, 840.f, 2410.f},
                               {730.f, 1090.f, 2440.f},
                               {530.f, 1840.f, 2480.f},
                               {270.f, 2290.f, 3010.f}};
static const float kA[5][3] = {{1.f, 0.35f, 0.1f},
                               {1.f, 0.45f, 0.15f},
                               {1.f, 0.5f, 0.25f},
                               {1.f, 0.6f, 0.3f},
                               {1.f, 0.3f, 0.25f}};

static constexpr int kN = 4; // 3 formants + nasal
static Svf       flt[2][2][kN]; // [channel][set][formant]
static float     amp[2][2][kN];
static float     sr = 48000.f;
static float     lfoPh = 0.f, sh[2] = {0, 0}, shT[2] = {0, 0}, carPh = 0.f;
static uint32_t  lastAction = 0;
static pn::Clock clk;
static pn::Rng   rng;
static pn::Env   envF;

static volatile float showVowel = 0.f, showMod = 0.f, level = 0.f;
static volatile int   modSrc = 0, voice = 0;
static volatile bool  hold = false, tick = false;

inline void Init(float s)
{
    sr = s;
    for(int c = 0; c < 2; c++)
        for(int t = 0; t < 2; t++)
            for(int j = 0; j < kN; j++)
            {
                flt[c][t][j].Init(sr);
                flt[c][t][j].SetRes(0.5f);
            }
}
inline void Clear() {}
inline void Activate(const Panel& p) { lastAction = p.action; }

inline void NewRandom()
{
    shT[0] = rng.B();
    shT[1] = rng.B();
}

static inline void SetSet(int c, int t, float v, float shift, float res, float nasal)
{
    v              = pn::Clamp(v, 0.f, 4.f);
    const int   i0 = (int)v;
    const int   i1 = i0 < 4 ? i0 + 1 : 4;
    const float fr = v - (float)i0;
    for(int j = 0; j < 3; j++)
    {
        const float f = kF[i0][j] + (kF[i1][j] - kF[i0][j]) * fr;
        flt[c][t][j].SetFreq(fminf(f * shift, sr * 0.3f));
        flt[c][t][j].SetRes(res);
        amp[c][t][j] = kA[i0][j] + (kA[i1][j] - kA[i0][j]) * fr;
    }
    flt[c][t][3].SetFreq(fminf(3500.f * shift, sr * 0.3f));
    flt[c][t][3].SetRes(res);
    amp[c][t][3] = nasal * 0.6f;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    modSrc = p.row;
    voice  = p.col;
    hold   = p.a;

    if(clk.Tick(p.gate, size, sr))
    {
        lfoPh = 0.f;
        NewRandom();
        tick = true;
    }
    if(p.action != lastAction)
    {
        lastAction = p.action;
        lfoPh      = 0.f;
        NewRandom();
        sh[0] = shT[0];
        sh[1] = shT[1];
        tick  = true;
    }

    // modulation (block rate)
    float hz = 0.05f * powf(400.f, p.k[4]);
    if(clk.Clocked(sr))
        hz = sr / (float)clk.period;
    const float glide = 0.5f * powf(0.004f, p.s[4]);
    if(!hold)
    {
        lfoPh += hz * (float)size / sr;
        if(lfoPh >= 1.f)
        {
            lfoPh -= floorf(lfoPh);
            if(modSrc == 1)
                NewRandom();
        }
        for(int c = 0; c < 2; c++)
            sh[c] += (shT[c] - sh[c]) * glide;
    }
    const float att = pn::OnePole(1.f / (0.001f + 0.2f * p.s[2] * p.s[2]), sr);
    const float rel = pn::OnePole(1.f / (0.02f + 2.f * p.s[3] * p.s[3]), sr);
    const float env = envF.v;

    const float stereo = p.s[1];
    const float shp    = 1.f + p.s[5] * 8.f;
    const float shN    = 1.f / tanhf(shp);
    float       m[2];
    switch(modSrc)
    {
        case 0:
            m[0] = tanhf(sinf(6.2831853f * lfoPh) * shp) * shN;
            m[1] = tanhf(sinf(6.2831853f * (lfoPh + stereo * 0.25f)) * shp) * shN;
            break;
        case 1:
            m[0] = sh[0];
            m[1] = stereo > 0.05f ? sh[1] : sh[0];
            break;
        default: m[0] = m[1] = fminf(env * 4.f, 1.f) * 2.f - 1.f; break;
    }
    showMod = m[0];

    const float depth = p.k[5] * 2.f;
    const float shift = exp2f((p.k[1] - 0.5f) * 2.f);
    const float res   = 0.2f + 0.75f * p.k[3];
    for(int c = 0; c < 2; c++)
    {
        float v = p.k[0] * 4.f + depth * m[c] + (c == 1 ? stereo * 1.5f : 0.f);
        if(c == 0)
            showVowel = pn::Clamp(v, 0.f, 4.f);
        if(voice == 1)
        {
            SetSet(c, 0, v + (c ? -0.35f : 0.35f), shift * (c ? 0.94f : 1.06f), res, p.s[6]);
            SetSet(c, 1, v + (c ? 0.35f : -0.35f), shift * (c ? 1.12f : 0.89f), res, p.s[6]);
        }
        else
            SetSet(c, 0, v, shift, res, p.s[6]);
    }

    const float drive  = 1.f + p.k[6] * p.k[6] * 15.f;
    const float makeup = 1.6f / sqrtf(drive);
    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    const float carInc = 32.703f * exp2f(p.s[0] * 4.f) / sr;
    const float breath = voice == 2 ? 0.f : fmaxf(0.f, p.s[0] - 0.5f);
    const int   sets   = voice == 1 ? 2 : 1;
    float       pko    = 0.f;
    for(size_t i = 0; i < size; i++)
    {
        const float e = envF.Process(0.5f * (in[0][i] + in[1][i]), att, rel);
        float       car = 0.f;
        if(voice == 2)
        {
            // band-limited-ish saw (polyBLEP)
            carPh += carInc;
            if(carPh >= 1.f)
                carPh -= 1.f;
            car = 2.f * carPh - 1.f;
            if(carPh < carInc)
            {
                const float t = carPh / carInc;
                car -= t + t - t * t - 1.f;
            }
            else if(carPh > 1.f - carInc)
            {
                const float t = (carPh - 1.f) / carInc;
                car -= t * t + t + t + 1.f;
            }
            car *= fminf(e * 6.f, 1.5f);
        }
        const float nz = breath > 0.f ? rng.B() * e * breath * 4.f : 0.f;
        float       o[2];
        for(int c = 0; c < 2; c++)
        {
            const float src = voice == 2 ? car : in[c][i] + nz;
            const float x   = SoftClip(src * drive);
            float       y   = 0.f;
            for(int t = 0; t < sets; t++)
                for(int j = 0; j < kN; j++)
                {
                    flt[c][t][j].Process(x);
                    y += flt[c][t][j].Band() * amp[c][t][j];
                }
            o[c] = y * makeup * (sets == 2 ? 0.7f : 1.f);
        }
        out[0][i] = pn::Lim(in[0][i] * dry + o[0] * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + o[1] * wetG);
        pko       = fmaxf(pko, fabsf(o[0]));
    }
    level = fmaxf(pko, level * 0.95f);
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    // vowel colour U blue, O violet, A red, E yellow, I white
    static const float kC[5][3] = {{0.f, 0.2f, 1.f},
                                   {0.6f, 0.f, 1.f},
                                   {1.f, 0.f, 0.f},
                                   {1.f, 0.8f, 0.f},
                                   {1.f, 1.f, 1.f}};
    const float v  = showVowel;
    const int   i0 = (int)v;
    const int   i1 = i0 < 4 ? i0 + 1 : 4;
    const float fr = v - (float)i0;
    l.Set(0,
          kC[i0][0] + (kC[i1][0] - kC[i0][0]) * fr,
          kC[i0][1] + (kC[i1][1] - kC[i0][1]) * fr,
          kC[i0][2] + (kC[i1][2] - kC[i0][2]) * fr);
    const float lv = fminf(level * 2.f, 1.f);
    static const float kV3[3][3] = {{0.f, 1.f, 0.6f}, {1.f, 0.3f, 0.7f}, {0.6f, 1.f, 0.f}};
    const int vc = voice;
    l.Set(1, kV3[vc][0] * lv, kV3[vc][1] * lv, kV3[vc][2] * lv);
    const float mv = 0.5f + 0.5f * showMod;
    static const float kS[3][3] = {{0.f, 0.f, 1.f}, {1.f, 0.4f, 0.f}, {0.f, 1.f, 0.f}};
    const int s = modSrc;
    l.Set(2, kS[s][0] * mv, kS[s][1] * mv, kS[s][2] * mv);
    if(hold)
        l.Set(3, 0.8f, 0.9f, 1.f);
    else
        l.Set(3, pf, pf * 0.4f, pf);
    (void)now;
}

} // namespace fmt
