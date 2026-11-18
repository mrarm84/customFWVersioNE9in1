// ShifterEngine.h - Bode-style frequency shifter.
//
// SUBMODE  row (SW0) = RANGE  FINE +-25 Hz | MID +-500 Hz | WIDE +-4 kHz
//          col (SW1) = TYPE   SHIFT  (single sideband, feedback delay)
//                             SPIRAL (barber-pole: built-in high feedback,
//                                     every echo shifts further, endless)
//                             RING   (ring modulator, sine -> square)
// LAYER 1  K0 SHIFT bipolar, noon = 0   K1 FEEDBACK   K2 MIX
//          K3 LFO RATE 0.03..9 Hz       K4 LFO DEPTH
//          K5 STEREO (SHIFT/SPIRAL: R = L x 1..0..-1, RING: R carrier x1..x2)
//          K6 DELAY 1 ms..1 s (CCW = direct)
// LAYER 2  K0 LFO SHAPE sine -> random steps
//          K1 FEEDBACK TONE (low-pass in the loop)
//          K2 SIDEBAND (SHIFT/SPIRAL: upper -> both -> lower)
//             CARRIER (RING: sine -> square)
//          K3 ENV -> SHIFT (bipolar, noon off)
//          K4 FEEDBACK DRIVE   K5 DELAY SPREAD (R longer)   K6 FINE TUNE
// double-tap = HOLD (infinite feedback), hold+release = flip direction,
// gate rising = flip direction
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"

namespace fsh
{
using namespace daisy;
using namespace daisysp;

// Olli Niemitalo's 90 degree allpass pairs
static const float kA[4] = {0.6923878f, 0.9360654322959f, 0.9882295226860f, 0.9987488452737f};
static const float kB[4] = {0.4021921162426f, 0.8561710882420f, 0.9722909545651f, 0.9952884791278f};

struct AP
{
    float a2 = 0.f, x1 = 0.f, x2 = 0.f, y1 = 0.f, y2 = 0.f;
    float P(float x)
    {
        const float y = a2 * (x + y2) - x2;
        x2            = x1;
        x1            = x;
        y2            = y1;
        y1            = y;
        return y;
    }
};
struct Hilbert
{
    AP    a[4], b[4];
    float bd = 0.f;
    void  Init()
    {
        for(int i = 0; i < 4; i++)
        {
            a[i]    = AP();
            b[i]    = AP();
            a[i].a2 = kA[i] * kA[i];
            b[i].a2 = kB[i] * kB[i];
        }
        bd = 0.f;
    }
    void P(float x, float& I, float& Q)
    {
        float ya = x, yb = x;
        for(int i = 0; i < 4; i++)
        {
            ya = a[i].P(ya);
            yb = b[i].P(yb);
        }
        I  = ya;
        Q  = bd;
        bd = yb;
    }
};

static constexpr uint32_t kDel = 65536, kDM = kDel - 1;
static float DSY_SDRAM_BSS del[2][kDel];

static Hilbert  hil[2];
static float    sr = 48000.f, ph[2] = {0, 0}, lfoPh = 0.f, fbS[2] = {0, 0};
static float    shVal = 0.f, fbLp[2] = {0, 0};
static uint32_t w = 0;
static bool     lastGate = false, flip = false;
static uint32_t lastAction = 0;
static pn::Env  envF;
static pn::Rng  rng;

static volatile float showShift = 0.f, level = 0.f, showLfo = 0.f;
static volatile int   type = 0;
static volatile bool  holdFb = false;

inline void Clear()
{
    memset(del, 0, sizeof(del));
    hil[0].Init();
    hil[1].Init();
    fbS[0] = fbS[1] = fbLp[0] = fbLp[1] = 0.f;
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
    flip       = false;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    static const float kRange[3] = {25.f, 500.f, 4000.f};
    const float        range     = kRange[p.row];
    type                         = p.col;
    holdFb                       = p.a;

    if(p.gate && !lastGate)
        flip = !flip;
    lastGate = p.gate;
    if(p.action != lastAction)
    {
        lastAction = p.action;
        flip       = !flip;
    }

    // envelope of the input (block)
    float e = envF.v;
    for(size_t i = 0; i < size; i++)
        e = envF.Process(0.5f * (in[0][i] + in[1][i]), 0.05f, 0.0005f);

    const float c   = p.k[0] - 0.5f;
    float       mag = fmaxf(0.f, fabsf(c) - 0.015f) / 0.485f;
    float       f   = (c < 0.f ? -1.f : 1.f) * range * mag * mag;

    // LFO: sine -> random steps
    lfoPh += 0.03f * powf(300.f, p.k[3]) * (float)size / sr;
    if(lfoPh >= 1.f)
    {
        lfoPh -= floorf(lfoPh);
        shVal = rng.B();
    }
    const float sn  = sinf(6.2831853f * lfoPh);
    const float lfo = sn + (shVal - sn) * p.s[0];
    showLfo         = lfo;
    f += range * 0.5f * p.k[4] * lfo;
    f += (p.s[3] - 0.5f) * 2.f * range * fminf(e * 4.f, 1.f);
    f += (p.s[6] - 0.5f) * range * 0.1f;
    if(flip)
        f = -f;
    showShift = f / range;

    float fR;
    if(type == 2)
        fR = fabsf(f) * (1.f + p.k[5]);
    else
        fR = f * (1.f - 2.f * p.k[5]);
    const float fL = type == 2 ? fabsf(f) : f;

    float fb = holdFb ? 1.f : p.k[1] * 0.97f;
    float dt = p.k[6] < 0.02f ? 1.f : 0.001f * powf(1000.f, p.k[6]) * sr;
    if(type == 1)
    {
        // SPIRAL: always deep feedback, a usable delay even at CCW
        fb = holdFb ? 1.f : 0.6f + 0.38f * p.k[1];
        dt = (0.02f + 0.38f * p.k[6]) * sr;
    }
    dt              = pn::Clamp(dt, 1.f, (float)(kDel / 2 - 4));
    const float dtR = pn::Clamp(dt * (1.f + 0.5f * p.s[5]), 1.f, (float)(kDel - 4));
    const float fbC = pn::OnePole(500.f * powf(36.f, p.s[1]), sr);
    const float fbD = 1.f + p.s[4] * 6.f;
    const float side = 1.f - 2.f * p.s[2]; // +1 upper, 0 both, -1 lower
    const float sq   = p.s[2];             // RING carrier squareness
    const float inc[2] = {fL / sr, fR / sr};
    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    float       pk  = 0.f;

    for(size_t i = 0; i < size; i++)
    {
        float y[2];
        for(int ch = 0; ch < 2; ch++)
        {
            const float s = (holdFb ? 0.f : in[ch][i]) + fbS[ch];
            const float cs = cosf(6.2831853f * ph[ch]);
            if(type == 2)
            {
                // ring: sine -> square carrier
                const float car = cs + (tanhf(cs * 12.f) - cs) * sq;
                y[ch]           = s * car;
            }
            else
            {
                float I, Q;
                hil[ch].P(s, I, Q);
                const float snp = sinf(6.2831853f * ph[ch]);
                y[ch]           = I * cs - side * Q * snp;
            }
        }
        for(int ch = 0; ch < 2; ch++)
        {
            ph[ch] += inc[ch];
            ph[ch] -= floorf(ph[ch]);
            del[ch][w & kDM] = pn::LoopClip(y[ch]);
        }
        w++;
        for(int ch = 0; ch < 2; ch++)
        {
            const float r = pn::ReadFrac(del[ch], kDM, w, ch ? dtR : dt);
            fbLp[ch] += (r - fbLp[ch]) * fbC;
            fbS[ch] = SoftClip(fbLp[ch] * fbD) / fbD * fb;
        }

        out[0][i] = pn::Lim(in[0][i] * dry + y[0] * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + y[1] * wetG);
        pk        = fmaxf(pk, fabsf(y[0]));
    }
    level = fmaxf(pk, level * 0.95f);
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    const float s = showShift;
    // 0: shift: red = up, blue = down, brightness = amount
    const float a = fminf(fabsf(s) * 1.5f + 0.05f, 1.f);
    if(s >= 0.f)
        l.Set(0, a, 0.f, a * 0.2f);
    else
        l.Set(0, a * 0.2f, 0.f, a);
    const float lv = fminf(level * 1.5f, 1.f);
    l.Set(1, lv * 0.3f, lv, lv);
    const float lf = 0.5f + 0.5f * showLfo;
    l.Set(2, 0.f, lf * 0.6f, lf * 0.2f);
    static const float kM[3][3] = {{1.f, 1.f, 1.f}, {1.f, 0.f, 1.f}, {1.f, 0.6f, 0.f}};
    const int t = type;
    if(holdFb)
        l.Set(3, 0.f, 1.f, 1.f);
    else
        l.Set(3, kM[t][0] * 0.5f, kM[t][1] * 0.5f, kM[t][2] * 0.5f);
    (void)now;
    (void)pf;
}

} // namespace fsh
