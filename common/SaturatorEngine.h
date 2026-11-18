// SaturatorEngine.h - beat saturator: drive that pumps with the beat, tape
// transport (wow, flutter, stop, brake), vinyl crackle and pops.
//
// Signal: send -> tape compressor -> tape (varispeed read head) -> shaper
//         (drive x beat pump) -> tone tilt -> beat gain pump -> + crackle,
//         pops, hiss, hum (these follow the program level, so a silent send
//         stays silent)
//
// SUBMODE  row (SW0) = SHAPER  TAPE (tanh) | TUBE (asymmetric) | FOLD (fuzz)
//          col (SW1) = PUMP    DUCK  (sidechain dip on every division)
//                              PUNCH (drive burst on every division)
//                              SWELL (quiet on the beat, loud off the beat)
// LAYER 1  K0 DRIVE   K1 PUMP depth   K2 SEND   K3 POPS (crackle + pops)
//          K4 WOW/FLUTTER   K5 TONE (CCW dark, CW bright)   K6 NOISE
//                   (hiss, hum in the top part)
// LAYER 2  K0 PUMP DIVISION 1/4, 1/2, 1, 2 beats, 1 bar
//          K1 BIAS (asymmetry, noon none)   K2 LPF
//          K3 POP SYNC (CCW random pops, CW pops on the 1/16 grid)
//          K4 FLUTTER RATE 2..16 Hz   K5 TAPE COMPRESSION
//          K6 OUTPUT -6..+6 dB (noon 0)
// double-tap = TAPE STOP / START, hold+release = BRAKE (stops over one
// beat, then spins back up)
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include "Panel.h"
#include "BeatTracker.h"

namespace sat
{
using namespace daisy;
using namespace daisysp;

static constexpr uint32_t kRing = 1u << 17, kMask = kRing - 1; // 2.7 s
static float DSY_SDRAM_BSS ringL[kRing];
static float DSY_SDRAM_BSS ringR[kRing];
static float* const        ring[2] = {ringL, ringR};

static const float kDiv[5] = {0.25f, 0.5f, 1.f, 2.f, 4.f};

static float    sr = 48000.f, d0 = 288.f;
static uint32_t w = 0, brakeT = 0, lastAction = 0;
static float    d[2] = {288.f, 288.f}, sm = 1.f;
static float    wowPh = 0.f, fl1 = 0.f, fl2 = 0.f, humPh = 0.f;
static float    env = 0.f, toneLp[2] = {0, 0}, dcX[2] = {0, 0}, dcY[2] = {0, 0};
static float    popV[2] = {0, 0}, bigV = 0.f, lastG16 = -1.f;
static bool     stopped = false, wasHold = false;
static int      divZ = 2;
static pn::Rng  rng;

static volatile float level = 0.f, pumpShow = 1.f, popShow = 0.f;
static volatile int   character = 0;
static volatile bool  showStop = false;

inline void Clear()
{
    memset(ringL, 0, sizeof(ringL));
    memset(ringR, 0, sizeof(ringR));
    d[0] = d[1] = d0;
    toneLp[0] = toneLp[1] = dcX[0] = dcX[1] = dcY[0] = dcY[1] = 0.f;
    popV[0] = popV[1] = bigV = 0.f;
    env                      = 0.f;
}
inline void Init(float s)
{
    sr = s;
    d0 = 0.006f * sr;
    Clear();
}
inline void Activate(const Panel& p)
{
    lastAction = p.action;
    stopped    = p.a;
    sm         = stopped ? 0.f : 1.f;
    brakeT     = 0;
    d[0] = d[1] = d0;
}

inline float Shape(float u, int ch)
{
    switch(ch)
    {
        case 0: return tanhf(u);
        case 1: return u >= 0.f ? 1.f - expf(-u) : (expf(1.6f * u) - 1.f) / 1.6f;
        default: return sinf(pn::Clamp(u, -4.71f, 4.71f));
    }
}

inline void Process(const Panel&              p,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    const int ch   = p.row;
    const int pump = p.col;
    character      = ch;

    const float drive = 1.f + p.k[0] * p.k[0] * 24.f;
    const float pd    = p.k[1];
    const float pops  = p.k[3];
    const float wow   = p.k[4];
    const float tone  = p.k[5];
    const float noise = p.k[6];
    const float div   = kDiv[pn::Zone(p.s[0], 5, divZ)];
    const float bias  = (p.s[1] - 0.5f) * 0.8f;
    const float popSy = p.s[3];
    const float flHz  = 2.f + 14.f * p.s[4];
    const float comp  = p.s[5];
    const float outG  = exp2f((p.s[6] - 0.5f) * 2.f);

    stopped = p.a;
    if(p.action != lastAction)
    {
        lastAction = p.action;
        brakeT     = (uint32_t)bt::st.period; // one beat
    }

    const float norm  = ch == 2 ? 1.f : 1.f / Shape(drive, ch);
    const float biasC = Shape(bias, ch);
    const float wowD  = wow * wow * 0.012f;
    const float flD   = wow * wow * 0.003f;
    const float wowI = 0.6f / sr, f1I = flHz / sr, f2I = flHz * 1.73f / sr;
    const float stopStep = 1.f / fmaxf(0.8f * bt::st.period, 0.05f * sr);
    const float aAtt = pn::OnePole(100.f, sr), aRel = pn::OnePole(6.f, sr);
    const float tC   = pn::OnePole(900.f, sr);
    const float tilt = (tone - 0.5f) * 2.f;
    const float gLo  = 1.f - 0.6f * fmaxf(tilt, 0.f);
    const float gHi  = tilt < 0.f ? 1.f + tilt : 1.f + 1.2f * tilt;
    const float hiss = noise * noise * 0.02f;
    const float hum  = noise > 0.6f ? (noise - 0.6f) * 0.04f : 0.f;
    const float crP  = pops * pops * 0.0015f;
    const float bigR = pops * (1.f - popSy) * 1.5f / sr;
    float       pk = 0.f, pumpMin = 1.f;

    for(size_t i = 0; i < size; i++)
    {
        // ---- beat pump
        const float bp = bt::Pos(i, 16);
        float       ph = bp / div;
        ph -= floorf(ph);
        float g = 1.f, dm = 1.f;
        switch(pump)
        {
            case 0:
            {
                const float r = fminf(ph * 3.f, 1.f);
                g             = 1.f - pd * 0.9f * (1.f - r) * (1.f - r);
                break;
            }
            case 1: dm = 1.f + pd * 5.f * expf(-ph * 12.f); break;
            default: g = 1.f - pd * 0.9f * (0.5f + 0.5f * cosf(6.2831853f * ph)); break;
        }
        pumpMin = fminf(pumpMin, pump == 1 ? 1.f / dm : g);

        // ---- pops on the 1/16 grid, or random
        const float g16 = floorf(bp * 4.f);
        if(g16 != lastG16)
        {
            if(lastG16 >= 0.f && rng.F() < pops * popSy * 0.5f)
                bigV = (0.3f + 0.5f * rng.F()) * pops, popShow = 1.f;
            lastG16 = g16;
        }
        if(rng.F() < bigR)
            bigV = (0.3f + 0.5f * rng.F()) * pops, popShow = 1.f;

        // ---- transport
        const bool hold = stopped || brakeT > 0;
        if(brakeT > 0)
            brakeT--;
        // restart after a long stop: skip most of the backlog while the
        // output is still faded out (no long 4x chirp through old audio)
        if(!hold && wasHold)
            for(int c = 0; c < 2; c++)
                if(d[c] > d0 + 0.3f * sr)
                    d[c] = d0 + 0.05f * sr;
        wasHold = hold;
        if(hold)
            sm = fmaxf(0.f, sm - stopStep);
        else
            sm = fminf(1.f, sm + 1.f / (0.08f * sr));

        wowPh += wowI;
        if(wowPh >= 1.f)
            wowPh -= 1.f;
        fl1 += f1I;
        if(fl1 >= 1.f)
            fl1 -= 1.f;
        fl2 += f2I;
        if(fl2 >= 1.f)
            fl2 -= 1.f;
        const float mod = wowD * sinf(6.2831853f * wowPh)
                          + flD * (0.6f * sinf(6.2831853f * fl1)
                                   + 0.4f * sinf(6.2831853f * fl2));

        // ---- tape compressor + record
        const float x0  = in[0][i], x1 = in[1][i];
        const float lvl = fmaxf(fabsf(x0), fabsf(x1));
        env += (lvl - env) * (lvl > env ? aAtt : aRel);
        const float gc = 1.f / (1.f + comp * 6.f * env);
        ringL[w & kMask] = x0 * gc;
        ringR[w & kMask] = x1 * gc;

        const float ng    = fminf(1.f, env * 12.f);
        const float stopF = fminf(1.f, sm * 3.f);
        humPh += 50.f / sr;
        if(humPh >= 1.f)
            humPh -= 1.f;
        const float humS = hum
                           * (sinf(6.2831853f * humPh)
                              + 0.4f * sinf(6.2831853f * 3.f * humPh));
        const float big = bigV * rng.B();
        bigV *= 0.992f;

        for(int c = 0; c < 2; c++)
        {
            // read head: speed = transport x wow; catches up after a stop
            float rate = sm * (1.f + mod);
            if(!hold)
            {
                // far behind (after a stop / brake): catch up fast (spin-up
                // chirp); inside the wow range: a slow pull that leaves
                // the wow alone
                const float dev = d[c] - d0;
                rate += fabsf(dev) > 0.01f * sr
                            ? pn::Clamp(dev / (0.1f * sr), -0.5f, 3.f)
                            : dev / (2.f * sr);
            }
            d[c] += 1.f - rate;
            d[c] = pn::Clamp(d[c], 2.f, (float)(kRing - 8));
            const float y = pn::ReadFrac(ring[c], kMask, w, d[c]);

            // shaper
            float s = (Shape(y * drive * dm + bias, ch) - biasC) * norm;
            dcY[c]  = s - dcX[c] + 0.995f * dcY[c];
            dcX[c]  = s;
            s       = dcY[c];

            // tone tilt
            toneLp[c] += (s - toneLp[c]) * tC;
            s = toneLp[c] * gLo + (s - toneLp[c]) * gHi;

            // crackle + pops + hiss + hum, following the program
            if(rng.F() < crP)
                popV[c] += rng.B() * 0.25f * pops;
            const float crk = popV[c];
            popV[c] *= 0.35f;
            const float nz = (crk + big + rng.B() * hiss + humS) * ng;

            const float o = pn::Lim((s * g * stopF + nz) * outG);
            out[c][i]     = o;
            pk            = fmaxf(pk, fabsf(o));
        }
        w++;
    }
    level    = fmaxf(pk, level * 0.95f);
    pumpShow = pumpMin;
    showStop = stopped || brakeT > 0;
}

inline void Leds(LedOut& l, uint32_t now, float pf)
{
    static const float kC[3][3] = {
        {1.f, 0.5f, 0.f}, // TAPE amber
        {1.f, 0.1f, 0.f}, // TUBE red-orange
        {0.9f, 0.f, 1.f}, // FOLD purple
    };
    const int   c  = character;
    const float b0 = 0.15f + 0.85f * pumpShow; // dips with the pump
    l.Set(0, kC[c][0] * b0, kC[c][1] * b0, kC[c][2] * b0);
    const float lv = fminf(level * 1.2f, 1.f);
    l.Set(1, lv, lv * 0.6f, 0.f);
    const float pp = popShow;
    l.Set(2, pp, pp, pp * 0.8f);
    popShow = popShow * 0.85f;
    if(showStop)
        l.Set(3, (now / 120) & 1 ? 1.f : 0.2f, 0.f, 0.f);
    else
        l.Set(3, pf, pf * 0.4f, 0.f);
}

} // namespace sat
