// TapeEngine.h - tape mangler. The read head follows the delay time like a
// varispeed tape (moving TIME glides the pitch).
//
// SUBMODE  row (SW0) = HEADS    SINGLE | PING-PONG | MULTI (3 taps)
//          col (SW1) = MACHINE  ECHO
//                               REVERSE (echoes play backwards in chunks)
//                               WARBLE  (short tape: chorus / vibrato /
//                                        flanger, heavy wow & flutter)
// LAYER 1  K0 TIME 20 ms..2 s (WARBLE 3..40 ms; clocked: 1/4..2 x clock)
//          K1 FEEDBACK  K2 MIX  K3 WOW  K4 FLUTTER  K5 DRIVE  K6 TONE
// LAYER 2  K0 AGE (new -> worn -> broken: drift, dropouts, crinkles)
//          K1 HISS  K2 SPREAD (R head offset)  K3 TAP SPACING (multi)
//          K4 LOW CUT 20 Hz..1 kHz  K5 STOP TIME 0.1..3 s
//          K6 SPIN-BACK SPEED
// double-tap = TAPE STOP / START, hold+release = SPIN BACK, gate = clock
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"

namespace tape
{
using namespace daisy;
using namespace daisysp;

static constexpr uint32_t kRing = 1u << 19, kMask = kRing - 1;
static float DSY_SDRAM_BSS ringL[kRing];
static float DSY_SDRAM_BSS ringR[kRing];
static float* const        ring[2] = {ringL, ringR};

static const float kRatio[8] = {0.25f, 0.3333f, 0.5f, 0.6667f, 0.75f, 1.f, 1.5f, 2.f};

static float         sr = 48000.f;
static uint32_t      w  = 0;
static float         d[2] = {24000.f, 24000.f}, T = 24000.f, sm = 1.f;
static float         wowPh = 0.f, fl1 = 0.f, fl2 = 0.f, drift = 0.f, driftT = 0.f;
static float         lpS[2] = {0, 0}, hpS[2] = {0, 0}, dropG = 1.f, crink = 0.f;
static uint32_t      dropT = 0, crinkT = 0, rewindT = 0, lastAction = 0;
static bool          stopped = false;
static pn::RevReader rev[2];
static pn::Clock     clk;
static pn::Rng       rng;
static int           timeZ = 5;

static volatile float level = 0.f, age = 0.f;
static volatile int   machine = 0;

inline void Clear()
{
    memset(ringL, 0, sizeof(ringL));
    memset(ringR, 0, sizeof(ringR));
    lpS[0] = lpS[1] = hpS[0] = hpS[1] = 0.f;
}
inline void Init(float s)
{
    sr = s;
    Clear();
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    stopped    = p.a;
    sm         = stopped ? 0.f : 1.f;
    rewindT    = 0;
    d[0] = d[1] = T;
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    const int heads = p.row;
    machine         = p.col;
    age             = p.s[0];
    const float ag  = age;

    clk.Tick(p.gate, size, sr);
    if(machine == 2)
        T = (0.003f + 0.037f * p.k[0]) * sr;
    else if(clk.Clocked(sr))
        T = (float)clk.period * kRatio[pn::Zone(p.k[0], 8, timeZ)];
    else
        T = 0.02f * powf(100.f, p.k[0]) * sr;
    T = pn::Clamp(T, 32.f, (float)(kRing / 2 - 16384));
    const float TR = T * (1.f + 0.25f * p.s[2]);

    const float fbMax = machine == 2 ? 0.75f : 1.1f;
    const float fb    = p.k[1] * fbMax;
    const float wm    = machine == 2 ? 6.f : 1.f;
    const float wowD  = p.k[3] * p.k[3] * 0.02f * wm;
    const float flD   = p.k[4] * p.k[4] * 0.005f * (machine == 2 ? 4.f : 1.f);
    const float drive = 1.f + p.k[5] * p.k[5] * 7.f;
    const float lpC   = pn::OnePole(800.f * powf(20.f, p.k[6]), sr);
    const float hpC   = pn::OnePole(20.f * powf(50.f, p.s[4]), sr);
    const float hiss  = p.s[1] * p.s[1] * 0.01f;
    const float stopT = 0.1f * powf(30.f, p.s[5]);
    const float spin  = -(1.f + 5.f * p.s[6]);
    const float tapA  = 0.2f + 0.5f * p.s[3];
    const float tapB  = 0.5f * (1.f + tapA);

    stopped = p.a;
    if(p.action != lastAction)
        lastAction = p.action, rewindT = (uint32_t)(0.45f * sr);

    const float dry = pn::EqPowDry(p.k[2]), wetG = pn::EqPowWet(p.k[2]);
    const float wowInc = 0.55f / sr, f1Inc = 6.3f / sr, f2Inc = 11.7f / sr;
    float       pk = 0.f;

    for(size_t i = 0; i < size; i++)
    {
        // ---- transport
        if(stopped)
            sm = fmaxf(0.f, sm - 1.f / (stopT * sr));
        else
            sm = fminf(1.f, sm + 1.f / (0.35f * sr));

        wowPh += wowInc;
        if(wowPh >= 1.f)
            wowPh -= 1.f;
        fl1 += f1Inc;
        if(fl1 >= 1.f)
            fl1 -= 1.f;
        fl2 += f2Inc;
        if(fl2 >= 1.f)
            fl2 -= 1.f;
        if(ag > 0.05f && (rng.Next() & 4095) == 0)
            driftT = rng.B();
        drift += (driftT - drift) * 0.00005f;

        float mod = wowD * (sinf(6.2831853f * wowPh) + drift * ag * 2.f)
                    + flD * (0.6f * sinf(6.2831853f * fl1)
                             + 0.4f * sinf(6.2831853f * fl2));
        if(ag > 0.6f)
        {
            if(crinkT > 0)
                crinkT--;
            else
            {
                crink *= 0.995f;
                if(rng.F() < (ag - 0.6f) * 4.f / sr)
                {
                    crinkT = (uint32_t)((0.02f + 0.06f * rng.F()) * sr);
                    crink  = rng.B() * 0.3f;
                }
            }
            mod += crink;
        }

        const float tgt[2] = {T, TR};
        for(int c = 0; c < 2; c++)
        {
            float rate;
            if(rewindT > 0)
                rate = spin;
            else
            {
                const float corr
                    = pn::Clamp((d[c] - tgt[c]) / (0.25f * sr), -1.5f, 1.5f);
                rate = sm * (1.f + mod + corr);
            }
            d[c] += 1.f - rate;
            d[c] = pn::Clamp(d[c], 2.f, (float)(kRing - 8192));
        }
        if(rewindT > 0)
            rewindT--;

        // ---- playback
        float wet[2], src[2];
        for(int c = 0; c < 2; c++)
        {
            const float* r = ring[c];
            if(machine == 1)
            {
                float d1, g1, d2, g2;
                rev[c].Step(c ? TR : T, d1, g1, d2, g2);
                const float off = d[c] - tgt[c]; // stops / spins still apply
                src[c] = pn::ReadFrac(r, kMask, w, fmaxf(d1 + off, 1.f)) * g1
                         + pn::ReadFrac(r, kMask, w, fmaxf(d2 + off, 1.f)) * g2;
            }
            else
                src[c] = pn::ReadFrac(r, kMask, w, d[c]);
            if(heads == 2)
                wet[c] = 0.6f
                         * (src[c]
                            + 0.6f * pn::ReadFrac(r, kMask, w, fmaxf(d[c] * tapB, 1.f))
                            + 0.4f * pn::ReadFrac(r, kMask, w, fmaxf(d[c] * tapA, 1.f)));
            else
                wet[c] = src[c];
        }

        // ---- tape age
        if(ag > 0.05f)
        {
            float tg = 1.f;
            if(dropT > 0)
            {
                dropT--;
                tg = 0.05f;
            }
            else if(rng.F() < ag * ag * 2.f / sr)
                dropT = (uint32_t)((0.01f + 0.08f * rng.F()) * sr);
            dropG += (tg - dropG) * 0.01f;
            wet[0] *= dropG;
            wet[1] *= dropG;
        }
        if(hiss > 0.f)
        {
            wet[0] += rng.B() * hiss;
            wet[1] += rng.B() * hiss;
        }
        const float stopFade = fminf(1.f, sm * 4.f + (rewindT > 0 ? 1.f : 0.f));
        wet[0] *= stopFade;
        wet[1] *= stopFade;

        // ---- feedback: tone -> low cut -> saturation
        float fbo[2];
        for(int c = 0; c < 2; c++)
        {
            lpS[c] += (src[c] * fb - lpS[c]) * lpC;
            hpS[c] += (lpS[c] - hpS[c]) * hpC;
            fbo[c] = SoftClip((lpS[c] - hpS[c]) * drive) / drive;
        }

        // ---- record
        if(heads == 1)
        {
            ringL[w & kMask] = pn::Lim((in[0][i] + in[1][i]) * 0.5f + fbo[1]);
            ringR[w & kMask] = pn::Lim(fbo[0]);
        }
        else
        {
            ringL[w & kMask] = pn::Lim(in[0][i] + fbo[0]);
            ringR[w & kMask] = pn::Lim(in[1][i] + fbo[1]);
        }
        w++;

        out[0][i] = pn::Lim(in[0][i] * dry + wet[0] * wetG);
        out[1][i] = pn::Lim(in[1][i] * dry + wet[1] * wetG);
        pk        = fmaxf(pk, fabsf(wet[0]));
    }
    level = fmaxf(pk, level * 0.95f);
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    // 0: delay time blink, colour = machine
    static const float kM3[3][3] = {{1.f, 0.55f, 0.f}, {1.f, 0.f, 0.4f}, {0.3f, 1.f, 0.6f}};
    const int   m  = machine;
    const float ds = d[0] / sr;
    const bool  bl = ds > 0.02f && (now % (uint32_t)(ds * 1000.f + 1.f)) < 35;
    const float b0 = bl ? 1.f : 0.1f;
    l.Set(0, kM3[m][0] * b0, kM3[m][1] * b0, kM3[m][2] * b0);
    // 1: transport / level
    if(rewindT > 0)
        l.Set(1, 1.f, 0.f, 1.f);
    else if(stopped)
        l.Set(1, 1.f * (1.f - sm) + 0.2f, 0.f, 0.f);
    else
    {
        const float lv = fminf(level * 1.5f, 1.f);
        l.Set(1, lv, lv * 0.35f, 0.f);
    }
    // 2: wow wobble
    const float wb = 0.5f + 0.5f * sinf(6.2831853f * wowPh);
    l.Set(2, 0.6f * wb, 0.25f * wb, 0.05f);
    // 3: tape age green -> amber -> red
    const float a = age;
    const float b = 0.4f + 0.6f * pf;
    l.Set(3, fminf(a * 2.f, 1.f) * b, fminf((1.f - a) * 2.f, 1.f) * 0.8f * b, 0.f);
}

} // namespace tape
