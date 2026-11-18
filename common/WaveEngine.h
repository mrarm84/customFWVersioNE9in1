// WaveEngine.h - wavetable synth voice. Takes the GRANULAR CLOUD slot in the
// VersioBendSliceWave build (same interface as CloudEngine.h).
//
// SUBMODE  row (SW0) = BANK   ANALOG | VOCAL | DIGITAL      (8 waves each)
//          col (SW1) = VOICE  SINGLE (+ sub osc)
//                             UNISON (4 detuned oscillators, wide)
//                             INPUT PM (audio in phase-modulates the osc)
// LAYER 1  K0 PITCH (C1..C6, 1 V/oct)  K1 WAVE (morph through the 8 waves)
//          K2 MIX (input <-> synth)    K3 DECAY 5 ms..10 s (CCW = drone)
//          K4 WARP (phase distortion)  K5 SUB / DETUNE / PM DEPTH
//          K6 CUTOFF 30 Hz..18 kHz (low-pass)
// LAYER 2  K0 FINE +-1 st   K1 SCAN RATE (CCW off, 0.02..10 Hz)
//          K2 SCAN DEPTH    K3 ATTACK 0.5 ms..2 s   K4 RESONANCE
//          K5 ENV -> CUTOFF K6 GLIDE 0..1 s
// double-tap = DRONE (envelope held open), hold+release = trigger,
// gate = trigger the envelope
//
// Tables: 3 banks x 8 waves x 10 band-limited copies (one per octave of
// harmonics) x 2048 samples, built once at power-up with an FFT (~2 MB SDRAM).
#pragma once
#include "daisy_seed.h"
#include <string.h>
#include "Panel.h"

namespace wav
{
using namespace daisy;

static constexpr int kN = 2048, kNM = kN - 1, kLevels = 10;
static constexpr int kBanks = 3, kWaves = 8;

static float DSY_SDRAM_BSS tbl[kBanks][kWaves][kLevels][kN];
static float DSY_SDRAM_BSS fRe[kN], fIm[kN], sRe[kN], sIm[kN];
static float DSY_SDRAM_BSS twC[kN / 2], twS[kN / 2];

static float sr = 48000.f;

// ------------------------------------------------------- table building ---
static void Fft(float* re, float* im, bool inverse)
{
    for(int i = 1, j = 0; i < kN; i++)
    {
        int bit = kN >> 1;
        for(; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if(i < j)
        {
            float t = re[i];
            re[i]   = re[j];
            re[j]   = t;
            t       = im[i];
            im[i]   = im[j];
            im[j]   = t;
        }
    }
    for(int len = 2; len <= kN; len <<= 1)
    {
        const int half = len >> 1, step = kN / len;
        for(int i = 0; i < kN; i += len)
            for(int k = 0; k < half; k++)
            {
                const float wr = twC[k * step];
                const float wi = inverse ? twS[k * step] : -twS[k * step];
                const int   a = i + k, b = a + half;
                const float xr = re[b] * wr - im[b] * wi;
                const float xi = re[b] * wi + im[b] * wr;
                re[b]          = re[a] - xr;
                im[b]          = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
    }
}

inline float Frac(float x) { return x - floorf(x); }
inline float Saw(float t) { return 2.f * t - 1.f; }

// waves defined in time (t = 0..1, one cycle)
static float TimeWave(int bank, int w, float t)
{
    const float s = sinf(6.2831853f * t);
    if(bank == 0) // ANALOG
        switch(w)
        {
            case 0: return s;
            case 1: return 4.f * fabsf(t - 0.5f) - 1.f; // triangle
            case 2: return Saw(t);
            case 3: return t < 0.5f ? 1.f : -1.f;
            case 4: return t < 0.3f ? 1.f : -1.f;
            case 5: return t < 0.12f ? 1.f : -1.f;
            case 6: return 0.6f * Saw(t) + 0.4f * Saw(Frac(2.f * t)); // 2 saws
            default: return Saw(Frac(2.5f * t));                     // sync saw
        }
    switch(w) // DIGITAL
    {
        case 0: return sinf(2.f * s); // wavefolds
        case 1: return sinf(4.f * s);
        case 2: return sinf(8.f * s);
        case 3: return floorf(s * 2.f + 0.5f) * 0.5f; // 5-level crush
        case 4: return floorf(t * 8.f) / 3.5f - 1.f;  // 8-step saw
        case 5: return s * sinf(6.2831853f * 7.f * t); // ring
        default: return sinf(6.2831853f * Frac(3.3f * t)) * (1.f - t); // sync sine
    }
}

// vowel formants (Hz) for the VOCAL bank, sung on a 110 Hz design pitch
static const float kVowel[kWaves][3] = {
    {325.f, 700.f, 2530.f},  // U
    {450.f, 800.f, 2830.f},  // O
    {800.f, 1150.f, 2900.f}, // A
    {660.f, 1700.f, 2400.f}, // AE
    {400.f, 2000.f, 2600.f}, // E
    {270.f, 2300.f, 3000.f}, // I
    {490.f, 1350.f, 1700.f}, // ER
    {250.f, 1700.f, 2600.f}, // M (nasal)
};

static float VowelAmp(int w, int h)
{
    static const float kG[3] = {1.f, 0.6f, 0.3f};
    static const float kB[3] = {70.f, 90.f, 140.f};
    const float        f     = 110.f * (float)h;
    float              a     = 0.f;
    for(int j = 0; j < 3; j++)
    {
        const float d = (f - kVowel[w][j]) / kB[j];
        a += kG[j] / (1.f + d * d);
    }
    return a / sqrtf((float)h);
}

static void BuildWave(int b, int w)
{
    const bool spectral = b == 1 || (b == 2 && w == 7);
    if(!spectral)
    {
        for(int i = 0; i < kN; i++)
        {
            fRe[i] = TimeWave(b, w, (float)i / (float)kN);
            fIm[i] = 0.f;
        }
        Fft(fRe, fIm, false);
    }
    else
    {
        memset(fRe, 0, sizeof(fRe));
        memset(fIm, 0, sizeof(fIm));
        pn::Rng r;
        r.s = 0x1234567u + (uint32_t)w * 7919u + (uint32_t)b * 104729u;
        for(int h = 1; h < kN / 2; h++)
        {
            float a;
            if(b == 1)
                a = VowelAmp(w, h);
            else // DIGITAL 7: random harmonics
            {
                const float x = r.F();
                a = h <= 48 ? x * x / powf((float)h, 0.6f) : 0.f;
            }
            const float ph = r.F() * 6.2831853f; // spread phases: less peaky
            fRe[h]         = 0.5f * kN * a * sinf(ph);
            fIm[h]         = -0.5f * kN * a * cosf(ph);
            fRe[kN - h]    = fRe[h];
            fIm[kN - h]    = -fIm[h];
        }
    }
    memcpy(sRe, fRe, sizeof(fRe));
    memcpy(sIm, fIm, sizeof(fIm));

    float scale = 1.f;
    for(int L = 0; L < kLevels; L++)
    {
        const int maxH = (kN / 2) >> L; // harmonics 1 .. maxH-1 kept
        for(int k = 0; k < kN; k++)
        {
            const int  h    = k <= kN / 2 ? k : kN - k;
            const bool keep = h >= 1 && h < maxH;
            fRe[k]          = keep ? sRe[k] : 0.f;
            fIm[k]          = keep ? sIm[k] : 0.f;
        }
        Fft(fRe, fIm, true);
        float* t = tbl[b][w][L];
        for(int i = 0; i < kN; i++)
            t[i] = fRe[i] / (float)kN;
        if(L == 0)
        {
            float pk = 0.f;
            for(int i = 0; i < kN; i++)
                pk = fmaxf(pk, fabsf(t[i]));
            scale = pk > 1e-6f ? 1.f / pk : 1.f;
        }
        for(int i = 0; i < kN; i++)
            t[i] *= scale;
    }
}

// ------------------------------------------------------------- voice ------
inline float Read(const float* t, float ph)
{
    const float x = ph * (float)kN;
    const int   i = (int)x;
    const float f = x - (float)i;
    const float a = t[i & kNM], b = t[(i + 1) & kNM];
    return a + (b - a) * f;
}

// phase distortion: first half of the cycle squeezed into 0..w
inline float Warp(float p, float w)
{
    return p < w ? p * (0.5f / w) : 0.5f + (p - w) * (0.5f / (1.f - w));
}

// band-limited copy for a fundamental of f Hz
inline int LevelFor(float f)
{
    const float h = 0.45f * sr / fmaxf(f, 1.f); // harmonics that fit
    if(h >= (float)(kN / 2 - 1))
        return 0;
    int L = (int)ceilf(log2f((float)(kN / 2) / h));
    return L < 0 ? 0 : (L >= kLevels ? kLevels - 1 : L);
}

struct Svf
{
    float ic1 = 0.f, ic2 = 0.f;
    float Lp(float x, float a1, float a2, float a3)
    {
        const float v3 = x - ic2;
        const float v1 = a1 * ic1 + a2 * v3;
        const float v2 = ic2 + a2 * ic1 + a3 * v3;
        ic1            = 2.f * v1 - ic1;
        ic2            = 2.f * v2 - ic2;
        return v2;
    }
};

static float    ph[4], phSub = 0.f, lfoPh = 0.f;
static float    semiCur = -1.f, posCur = 0.f;
static float    env = 0.f;
static bool     attack = false, lastGate = false;
static uint32_t lastAction = 0;
static Svf      flt[2];
static pn::Rng  rng;

static volatile float level = 0.f, spawnFlash = 0.f, posShown = 0.f,
                      envShown = 0.f;
static volatile int  bankShown = 0;
static volatile bool droneShown = false;

inline void Clear()
{
    for(int v = 0; v < 4; v++)
        ph[v] = rng.F();
    phSub = 0.f;
    flt[0] = flt[1] = Svf();
    env             = 0.f;
    attack          = false;
}

inline void Init(float s)
{
    sr = s;
    for(int k = 0; k < kN / 2; k++)
    {
        twC[k] = cosf(6.2831853f * (float)k / (float)kN);
        twS[k] = sinf(6.2831853f * (float)k / (float)kN);
    }
    for(int b = 0; b < kBanks; b++)
        for(int w = 0; w < kWaves; w++)
            BuildWave(b, w);
    Clear();
}

inline void Activate(const Panel& p)
{
    lastAction = p.action;
    lastGate   = p.gate;
    semiCur    = -1.f; // no glide into the first note
    posCur     = p.k[1];
}

inline void Trigger()
{
    attack     = true;
    spawnFlash = 1.f;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    const int bank  = p.row;
    const int voice = p.col;
    bankShown       = bank;

    if(p.gate && !lastGate)
        Trigger();
    lastGate = p.gate;
    if(p.action != lastAction)
    {
        lastAction = p.action;
        Trigger();
    }
    const bool drone = p.a || p.k[3] < 0.03f;
    droneShown       = drone;

    // pitch + glide
    const float semi   = p.k[0] * 60.f + (p.s[0] - 0.5f) * 2.f;
    const float glideC = p.s[6] < 0.01f
                             ? 1.f
                             : 1.f - expf(-(float)size / (p.s[6] * sr));
    semiCur = semiCur < 0.f ? semi : semiCur + (semi - semiCur) * glideC;
    const float f0 = 32.703f * exp2f(semiCur / 12.f);

    // wave position + scan LFO (triangle)
    float lfo = 0.f;
    if(p.s[1] > 0.02f)
    {
        const float rate = 0.02f * powf(500.f, (p.s[1] - 0.02f) / 0.98f);
        lfoPh            = Frac(lfoPh + rate * (float)size / sr);
        lfo              = 4.f * fabsf(lfoPh - 0.5f) - 1.f;
    }
    float pos = p.k[1] + p.s[2] * 0.5f * lfo;
    if(pos < 0.f)
        pos = -pos;
    if(pos > 1.f)
        pos = 2.f - pos;
    pos = pn::Clamp(pos, 0.f, 1.f);
    posCur += (pos - posCur) * 0.3f;
    posShown       = posCur;
    const float wp = posCur * (float)(kWaves - 1);
    int         w0 = (int)wp;
    w0             = w0 > kWaves - 2 ? kWaves - 2 : w0;
    const float wm = wp - (float)w0;

    // warp + band-limit choice
    const float warpW   = 0.5f - 0.45f * p.k[4];
    const float warpMul = 0.5f / warpW;
    const float det     = voice == 1 ? p.k[5] * 0.5f : 0.f; // semitones
    const int   L       = LevelFor(f0 * warpMul * exp2f(det / 12.f));
    const float* t0     = tbl[bank][w0][L];
    const float* t1     = tbl[bank][w0 + 1][L];
    const float* tSub   = tbl[0][0][LevelFor(f0 * 0.5f)];

    const float inc = f0 / sr;
    float       vInc[4];
    static const float kDet[4] = {-1.f, -0.333f, 0.333f, 1.f};
    for(int v = 0; v < 4; v++)
        vInc[v] = inc * exp2f(det * kDet[v] / 12.f);
    const float subLvl = voice == 0 ? p.k[5] : 0.f;
    const float pmAmt  = voice == 2 ? p.k[5] * 2.f : 0.f;

    // envelope
    const float decT = 0.005f * powf(2000.f, pn::Clamp((p.k[3] - 0.03f) / 0.97f, 0.f, 1.f));
    const float dC   = powf(0.001f, 1.f / (decT * sr));
    const float aInc = 1.f / (0.0005f * powf(4000.f, p.s[3]) * sr);

    // filter (coefficients once per block, env at block start)
    const float oct = p.k[6] * 9.2f + p.s[5] * env * 7.f;
    const float fc  = fminf(30.f * exp2f(oct), 0.42f * sr);
    const float g   = tanf(3.14159265f * fc / sr);
    const float kq  = 2.f - 1.94f * p.s[4];
    const float a1  = 1.f / (1.f + g * (g + kq));
    const float a2  = g * a1, a3 = g * a2;

    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    float       pk  = 0.f;
    for(size_t i = 0; i < size; i++)
    {
        if(drone)
            env += (1.f - env) * 0.002f;
        else if(attack)
        {
            env += aInc;
            if(env >= 1.f)
                env = 1.f, attack = false;
        }
        else
            env *= dC;

        float o[2];
        if(voice == 1) // UNISON: L = voices 0 + 2, R = voices 1 + 3
        {
            float v[4];
            for(int k = 0; k < 4; k++)
            {
                ph[k]       = Frac(ph[k] + vInc[k]);
                const float q = Warp(ph[k], warpW);
                v[k]          = Read(t0, q) + (Read(t1, q) - Read(t0, q)) * wm;
            }
            o[0] = (v[0] + v[2]) * 0.6f;
            o[1] = (v[1] + v[3]) * 0.6f;
        }
        else
        {
            ph[0] = Frac(ph[0] + inc);
            for(int c = 0; c < 2; c++)
            {
                const float pp = voice == 2 ? Frac(ph[0] + pmAmt * in[c][i]) : ph[0];
                const float q  = Warp(pp, warpW);
                const float a  = Read(t0, q);
                o[c]           = a + (Read(t1, q) - a) * wm;
            }
            if(subLvl > 0.f)
            {
                phSub = Frac(phSub + inc * 0.5f);
                const float sb = Read(tSub, phSub) * subLvl;
                o[0] += sb;
                o[1] += sb;
            }
        }
        for(int c = 0; c < 2; c++)
            o[c] = flt[c].Lp(o[c] * env * 0.7f, a1, a2, a3);

        out[0][i] = pn::Lim(in[0][i] * dry + o[0] * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + o[1] * wetG);
        pk        = fmaxf(pk, fabsf(o[0]) + fabsf(o[1]));
    }
    level    = fmaxf(pk * 0.5f, level * 0.95f);
    envShown = env;
}

inline void Leds(LedOut& l, uint32_t now, float pf, const Panel& p)
{
    static const float kB[3][3] = {
        {1.f, 0.45f, 0.f}, // ANALOG amber
        {1.f, 0.1f, 0.5f}, // VOCAL pink
        {0.f, 0.8f, 1.f},  // DIGITAL cyan
    };
    const int b = bankShown;
    l.Set(0, kB[b][0] * 0.6f, kB[b][1] * 0.6f, kB[b][2] * 0.6f);
    const float lv = fminf(level * 1.5f, 1.f);
    l.Set(1, lv, lv, lv * 0.6f);
    // wave position: blue -> green -> red
    const float ps = posShown;
    l.Set(2, ps, 0.8f * (1.f - fabsf(2.f * ps - 1.f)), 1.f - ps);
    if(droneShown)
        l.Set(3, 0.f, 0.9f, 0.9f);
    else
    {
        const float e = fmaxf(envShown, pf);
        l.Set(3, e, e, e);
    }
    (void)now;
    (void)p;
}

} // namespace wav
