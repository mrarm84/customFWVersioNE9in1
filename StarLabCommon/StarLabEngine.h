// StarLabEngine - StarLab-style "experimental reverb" DSP, shared by the
// Versio and Legio firmwares. Include from exactly one .cpp per firmware.
//
// Signal path (dry stays stereo, effect send is L+R mono):
//   in -> soft clip -> ECHO (pre-delay / delay with feedback)
//      -> variable-rate reverb core:
//           INFINITE loop -> input diffusers -> 8-line FDN with in-loop
//           allpass, low/high damping and regenerative shimmer
//      -> wet (+ echoes) -> equal-power mix with dry
//
// SIZE/PITCH runs the whole core at a variable internal sample rate: faster
// = smaller room and everything already in the tail (frozen loop included)
// is transposed up, slower = bigger room and lower pitch, +-2 octaves.
#pragma once

#include <string.h>
#include "daisy_seed.h"
#include "daisysp.h"

namespace starlab
{
using namespace daisy;
using namespace daisysp;

// all 0..1 unless noted, set once per block
struct Params
{
    float size     = 0.5f; // noon = unity (small deadband), up = smaller/higher
    float decay    = 0.5f; // 0.2 s .. 60 s
    float mix      = 0.5f; // dry -> wet, equal power
    float tone     = 0.5f; // <0.5 high damp dark->open, >0.5 low damp
    float shimmer  = 0.f;  // regenerative shimmer amount
    float delay    = 0.f;  // seconds, 0 .. 7.5
    float feedback = 0.f;  // echo repeats; >0 also mixes echoes into wet
    int   texture  = 1;    // 0 Sparse, 1 Dense, 2 Diffuse
    int   interval = 1;    // 0 +1 oct, 1 +5th, 2 -1 oct
    float ratio    = 0.f;   // shimmer ratio override (0 = use interval)
    float modDepth = 12.f;  // line modulation, ticks peak-to-peak
    float difScale = 1.f;   // input diffusion amount scale (0..2)
    float echoHz   = 7000.f; // echo / pre-delay low-pass
};

// KNOB 0..1 -> delay seconds, 0 .. 1.5 s log taper
inline float DelayKnobToSeconds(float k)
{
    return k < 0.01f ? 0.f : 0.001f * powf(1500.f, k);
}

// ------------------------------------------------------------------ buffers
static constexpr int      kLines    = 8;
static constexpr uint32_t kLineMask = 16384 - 1;
static constexpr uint32_t kApMask   = 1024 - 1;
static constexpr int      kDifs     = 6;
static constexpr uint32_t kDifMask  = 4096 - 1;
static constexpr uint32_t kShimMask = 4096 - 1;
static constexpr uint32_t kFrzMask  = 65536 - 1;
static constexpr uint32_t kEchoMask = 524288 - 1; // ~10.9 s @ 48 kHz

float DSY_SDRAM_BSS lineBuf[kLines][kLineMask + 1];
float DSY_SDRAM_BSS apBuf[kLines][kApMask + 1];
float DSY_SDRAM_BSS difBuf[kDifs][kDifMask + 1];
float DSY_SDRAM_BSS shimBuf[kShimMask + 1];
float DSY_SDRAM_BSS frzBuf[kFrzMask + 1];
float DSY_SDRAM_BSS echoBuf[kEchoMask + 1];

// lengths in core ticks (nominal 48 kHz)
static const float kLineLen[kLines]
    = {1433.f, 1601.f, 1867.f, 2053.f, 2251.f, 2399.f, 2687.f, 2917.f};
static const uint32_t kApLen[kLines] = {241, 307, 383, 449, 521, 587, 641, 719};
static const uint32_t kDifLen[kDifs] = {142, 107, 379, 277, 1601, 2333};
static const float    kSign[kLines]
    = {1.f, -1.f, 1.f, -1.f, -1.f, 1.f, -1.f, 1.f};

static constexpr float    kDifG     = 0.7f;
static float              modDepth  = 12.f; // ticks peak-to-peak
static constexpr float    kLfoInc   = 0.37f / 48000.f;
static constexpr float    kShimWin  = 2000.f;
static constexpr uint32_t kFrzLen   = 36000; // 0.75 s at unity rate
static constexpr float    kInGain   = 0.3f;
static constexpr float    kOutGain  = 0.5f;

// texture presets: Sparse / Dense / Diffuse
struct Texture
{
    float lenScale, density, apG, dif[kDifs];
};
static const Texture kTex[3] = {
    {2.2f, 0.f, 0.f, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f}},
    {0.75f, 1.f, 0.3f, {1.f, 1.f, 1.f, 1.f, 0.f, 0.f}},
    {1.3f, 1.f, 0.6f, {1.f, 1.f, 1.f, 1.f, 1.f, 1.f}},
};

// -------------------------------------------------------------- core state
static float sr = 48000.f;

static uint32_t coreT = 0;
static float    g[kLines];
static float    lpS[kLines], hpS[kLines];
static float    lpCoef = 0.9f, hpCoef = 0.003f;
static float    lenScale = 1.f, density = 1.f, apG = 0.3f, difAmt[kDifs];
static float    shimGain = 0.f, shimRatio = 2.f, shimInc = 0.f, shimPh = 0.f;
static float    lfoPh = 0.f;

static bool     frozen = false;
static float    frzAmt = 0.f, frzGain = 1.f, frzPh = 0.f;
static uint32_t frzStart = 0;

// resampler between audio rate and core rate
static float rate = 1.f, rateTarget = 1.f, corePh = 0.f, xPrev = 0.f, aa = 0.f;
static float aaCoef = 1.f;
static float yL = 0.f, yR = 0.f, yPL = 0.f, yPR = 0.f;

// echo
static uint32_t echoW = 0;
static float    echoTime = 0.f, echoTarget = 0.f, echoLp = 0.f, echoFb = 0.f;
static float    echoLvl = 0.f, echoLpCoef = 0.6f;

static float dryG = 1.f, wetG = 0.f;

// for LEDs (written in the audio callback)
static volatile float inPeak = 0.f, wetPeak = 0.f;

// ----------------------------------------------------------------- helpers
// transparent below |1|, soft limit towards |2| above
static inline float LoopClip(float x)
{
    if(x > 1.f)
        return 2.f - 1.f / x;
    if(x < -1.f)
        return -2.f - 1.f / x;
    return x;
}

static inline float Tri(float p)
{
    return 1.f - fabsf(2.f * p - 1.f);
}

// d >= 1, wp = sample about to be written
static inline float
ReadFrac(const float *buf, uint32_t mask, uint32_t wp, float d)
{
    const uint32_t di = (uint32_t)d;
    const float    f  = d - (float)di;
    const float    a  = buf[(wp - di) & mask];
    const float    b  = buf[(wp - di - 1) & mask];
    return a + (b - a) * f;
}

static inline void Hadamard8(float *v)
{
    for(int h = 1; h < kLines; h <<= 1)
        for(int i = 0; i < kLines; i += h << 1)
            for(int j = i; j < i + h; j++)
            {
                const float a = v[j], b = v[j + h];
                v[j]          = a + b;
                v[j + h]      = a - b;
            }
    for(int i = 0; i < kLines; i++)
        v[i] *= 0.35355339f;
}

static inline float OnePoleCoef(float hz)
{
    return 1.f - expf(-TWOPI_F * hz / 48000.f);
}

// -------------------------------------------------------------- core tick
static void CoreTick(float x, float &outL, float &outR)
{
    const uint32_t ct = coreT;

    // INFINITE: loop the last kFrzLen ticks of core input, two crossfaded heads
    if(!frozen && frzAmt < 0.001f)
        frzBuf[ct & kFrzMask] = x;
    frzAmt += ((frozen ? 1.f : 0.f) - frzAmt) * 0.0005f;
    if(frzAmt > 0.001f)
    {
        frzPh += 1.f / kFrzLen;
        if(frzPh >= 1.f)
            frzPh -= 1.f;
        float p2 = frzPh + 0.5f;
        if(p2 >= 1.f)
            p2 -= 1.f;
        const float a = frzBuf[(frzStart + (uint32_t)(frzPh * kFrzLen)) & kFrzMask];
        const float b = frzBuf[(frzStart + (uint32_t)(p2 * kFrzLen)) & kFrzMask];
        x += (a * Tri(frzPh) + b * Tri(p2)) * frzAmt * frzGain;
    }

    // input diffusion (allpass chain, each stage blended in by texture)
    for(int s = 0; s < kDifs; s++)
    {
        float      *b = difBuf[s];
        const float z = b[(ct - kDifLen[s]) & kDifMask];
        const float v = x + kDifG * z;
        b[ct & kDifMask] = v;
        x += (z - kDifG * v - x) * difAmt[s];
    }

    // slow quadrature triangle LFO for line modulation
    lfoPh += kLfoInc;
    if(lfoPh >= 1.f)
        lfoPh -= 1.f;
    float q = lfoPh + 0.25f;
    if(q >= 1.f)
        q -= 1.f;
    const float m0 = (Tri(lfoPh) - 0.5f) * modDepth;
    const float m1 = (Tri(q) - 0.5f) * modDepth;

    // read + damp lines
    float v[kLines], h[kLines];
    float sum = 0.f, sumS = 0.f;
    for(int i = 0; i < kLines; i++)
    {
        const float d = kLineLen[i] * lenScale + 8.f + (i & 1 ? m1 : m0) * kSign[i];
        float       r = ReadFrac(lineBuf[i], kLineMask, ct, d);
        lpS[i] += (r - lpS[i]) * lpCoef;
        hpS[i] += (lpS[i] - hpS[i]) * hpCoef;
        r = lpS[i] - hpS[i];
        v[i] = h[i] = r;
        sum += r;
        sumS += r * kSign[i];
    }
    outL = (v[0] - v[2] + v[4] - v[6]) * kOutGain;
    outR = (v[1] - v[3] + v[5] - v[7]) * kOutGain;

    // regenerative shimmer: pitch-shift the tail and feed it back in
    shimBuf[ct & kShimMask] = sumS * 0.25f;
    float s = 0.f;
    if(shimGain > 0.0005f)
    {
        shimPh += shimInc;
        if(shimPh < 0.f)
            shimPh += 1.f;
        else if(shimPh >= 1.f)
            shimPh -= 1.f;
        float p2 = shimPh + 0.5f;
        if(p2 >= 1.f)
            p2 -= 1.f;
        s = ReadFrac(shimBuf, kShimMask, ct, 2.f + shimPh * kShimWin) * Tri(shimPh)
            + ReadFrac(shimBuf, kShimMask, ct, 2.f + p2 * kShimWin) * Tri(p2);
        s *= shimGain;
    }

    // feedback matrix: Sparse = permuted Householder (light mixing, distinct
    // reflections), Dense/Diffuse = Hadamard (full mixing)
    Hadamard8(h);
    const float hh  = sum * 0.25f;
    const float inj = x * kInGain + s;
    for(int i = 0; i < kLines; i++)
    {
        const float sp = v[(i + 1) & 7] - hh;
        float       f  = (sp + (h[i] - sp) * density) * g[i] + inj * kSign[i];

        float      *ab = apBuf[i];
        const float z  = ab[(ct - kApLen[i]) & kApMask];
        const float w  = f + apG * z;
        ab[ct & kApMask] = w;
        f                = z - apG * w;

        lineBuf[i][ct & kLineMask] = LoopClip(f);
    }
    coreT = ct + 1;
}

// ---------------------------------------------------------------- public
inline void Init(float samplerate)
{
    sr = samplerate;
    // SDRAM is not zeroed at boot
    memset(lineBuf, 0, sizeof(lineBuf));
    memset(apBuf, 0, sizeof(apBuf));
    memset(difBuf, 0, sizeof(difBuf));
    memset(shimBuf, 0, sizeof(shimBuf));
    memset(frzBuf, 0, sizeof(frzBuf));
    memset(echoBuf, 0, sizeof(echoBuf));
    for(int i = 0; i < kLines; i++)
        g[i] = lpS[i] = hpS[i] = 0.f;
    for(int s = 0; s < kDifs; s++)
        difAmt[s] = 0.f;
    echoLpCoef = OnePoleCoef(7000.f);
}

inline bool Frozen()
{
    return frozen;
}

inline void ToggleFreeze()
{
    frozen = !frozen;
    if(frozen && frzAmt < 0.001f)
    {
        frzStart = coreT - kFrzLen;
        frzPh    = 0.f;
    }
}

// call once per block, before Process()
inline void SetParams(const Params &p)
{
    // SIZE/PITCH: +-2 octaves of core rate, deadband at noon
    const float c   = p.size - 0.5f;
    float       oct = 0.f;
    if(c > 0.02f)
        oct = (c - 0.02f) * (2.f / 0.48f);
    else if(c < -0.02f)
        oct = (c + 0.02f) * (2.f / 0.48f);
    rateTarget = powf(2.f, oct);
    aaCoef     = fminf(OnePoleCoef(16000.f * fminf(rate, 1.f)), 1.f);

    // TEXTURE (lengths glide, the rest crossfades)
    const Texture &t = kTex[p.texture];
    lenScale += (t.lenScale - lenScale) * 0.003f;
    density += (t.density - density) * 0.01f;
    apG += (t.apG - apG) * 0.01f;
    for(int s = 0; s < kDifs; s++)
        difAmt[s] += (fminf(t.dif[s] * p.difScale, 1.f) - difAmt[s]) * 0.01f;

    // DECAY (+ a little extra damping when shimmer is up, keeps it stable)
    const float t60      = 0.2f * powf(300.f, p.decay);
    const float shimDamp = 1.f - 0.25f * p.shimmer;
    float       gSum     = 0.f;
    for(int i = 0; i < kLines; i++)
    {
        const float len = kLineLen[i] * lenScale + 8.f + kApLen[i];
        g[i]            = expf(-6.9078f * len / (t60 * 48000.f)) * shimDamp;
        gSum += g[i];
    }
    const float gAvg = gSum / kLines;
    frzGain          = 1.5f * sqrtf(fmaxf(1.f - gAvg * gAvg, 0.01f));

    // TONE
    float hiHz = 18000.f, loHz = 20.f;
    if(p.tone < 0.5f)
        hiHz = 500.f * powf(36.f, p.tone * 2.f);
    else
        loHz = 20.f * powf(40.f, (p.tone - 0.5f) * 2.f);
    lpCoef = OnePoleCoef(hiHz);
    hpCoef = OnePoleCoef(loHz);

    // SHIMMER + INTERVAL
    static const float kRatio[3] = {2.f, 1.4983071f, 0.5f};
    shimGain                     = 0.6f * p.shimmer;
    shimRatio += ((p.ratio > 0.f ? p.ratio : kRatio[p.interval]) - shimRatio) * 0.05f;
    modDepth   = p.modDepth;
    echoLpCoef = OnePoleCoef(p.echoHz);
    shimInc = (1.f - shimRatio) / kShimWin;

    // DELAY / FEEDBACK
    echoTarget = fminf(p.delay, 7.5f);
    echoFb     = p.feedback;
    echoLvl    = fminf(p.feedback * 5.f, 1.f) * 0.8f;

    // MIX
    dryG = cosf(p.mix * HALFPI_F);
    wetG = sinf(p.mix * HALFPI_F);
}

inline void Process(const float *inLs,
                    const float *inRs,
                    float       *outLs,
                    float       *outRs,
                    size_t       size)
{
    float pkIn = 0.f, pkWet = 0.f;
    for(size_t i = 0; i < size; i++)
    {
        const float inL  = inLs[i];
        const float inR  = inRs[i];
        const float send = LoopClip((inL + inR) * 0.5f);
        pkIn             = fmaxf(pkIn, fabsf(send));

        // ECHO / pre-delay
        fonepole(echoTime, echoTarget, 0.0005f);
        float d = echoTime * sr;
        if(d < 1.f)
            d = 1.f;
        const float e = ReadFrac(echoBuf, kEchoMask, echoW, d);
        echoLp += (e - echoLp) * echoLpCoef;
        echoBuf[echoW & kEchoMask] = LoopClip(send + echoLp * echoFb);
        echoW++;

        // variable-rate core
        aa += (e - aa) * aaCoef;
        rate += (rateTarget - rate) * 0.001f;
        const float invRate = 1.f / rate;
        float       acc     = corePh + rate;
        while(acc >= 1.f)
        {
            acc -= 1.f;
            const float f = 1.f - acc * invRate;
            yPL           = yL;
            yPR           = yR;
            CoreTick(xPrev + (aa - xPrev) * f, yL, yR);
        }
        corePh = acc;
        xPrev  = aa;

        const float wl = yPL + (yL - yPL) * acc + echoLp * echoLvl;
        const float wr = yPR + (yR - yPR) * acc + echoLp * echoLvl;
        pkWet          = fmaxf(pkWet, fabsf(wl));

        outLs[i] = inL * dryG + wl * wetG;
        outRs[i] = inR * dryG + wr * wetG;
    }
    inPeak  = fmaxf(pkIn, inPeak * 0.97f);
    wetPeak = fmaxf(pkWet, wetPeak * 0.97f);
}

inline float InPeak()
{
    return inPeak;
}
inline float WetPeak()
{
    return wetPeak;
}

} // namespace starlab
