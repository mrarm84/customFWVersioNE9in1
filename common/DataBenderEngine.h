// DataBenderEngine.h - shared DSP for the Versio and Legio Data Bender ports.
// Header-only; include it from exactly one .cpp per firmware.
//
// The front panel layer fills a dbe::Controls struct once per audio block
// and calls dbe::Process(). Everything else (buffer, slicing, bend, break,
// corrupt, clock, freeze, re-capture) lives here.
//
// DC coupling: there is NO DC blocker in the dry or buffer path, so CV fed
// into the audio inputs gets buffered/bent/broken like audio. Only the DJ
// high-pass and Vinyl corruptions remove DC.
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <cmath>

namespace dbe
{
using namespace daisy;
using namespace daisysp;

// ---------------------------------------------------------------- public ---
enum EngineMode
{
    MACRO_TRAVERSE,
    MACRO_SILENCE,
    MICRO
};
enum CorruptType
{
    C_DECIMATE,
    C_DROPOUT,
    C_DESTROY,
    C_DJFILTER,
    C_VINYL,
    C_LAST
};

struct Controls
{
    float      time        = 0.5f; // 0..1
    float      repeats     = 0.f;  // 0..1
    float      mix         = 0.5f; // 0..1
    float      bend        = 0.5f; // 0..1 (Micro: noon = 1x)
    float      brk         = 0.f;  // 0..1
    float      corrupt     = 0.f;  // 0..1
    float      extraOct    = 0.f;  // added to Micro speed (1 V/oct CV)
    int        corruptType = C_DECIMATE;
    EngineMode mode        = MACRO_TRAVERSE;
    bool       unique      = false; // per-channel random decisions
    bool       hardWindow  = false;
    bool       gate        = false; // clock input level
    bool       freeze      = false;
    bool       reverse     = false;
    uint32_t   recapture   = 0; // increment to request a re-capture

    // secondary parameters (defaults = original behaviour)
    float windowMs      = 10.f;  // glitch window length
    float reverseChance = 0.f;   // per-slice probability of a reversed slice
    float width         = 1.f;   // wet stereo width 0 (mono) .. 2 (wide)
    float breakShape    = 0.5f;  // traverse range / silence gate length
    float corruptColor  = 0.5f;  // per-type second corrupt parameter
    float regen         = 0.f;   // wet fed back into the buffer
    float shuffle       = 0.f;   // chance a repeat picks a random slice
    float randomRepeats = 0.f;   // chance the repeat count is re-rolled
};

// state readable by the UI (LEDs)
struct Status
{
    volatile bool  frozen      = false;
    volatile bool  reverse     = false;
    volatile bool  capturing   = false;
    volatile bool  clocked     = false;
    volatile float periodFlash = 0.f;
    volatile float sliceFlash  = 0.f;
    volatile float outLevel    = 0.f;
};
static Status status;

// --------------------------------------------------------------- buffer ----
static constexpr uint32_t kRingBits = 19;
static constexpr uint32_t kRing     = 1u << kRingBits; // 524288 ~ 10.9 s
static constexpr uint32_t kMask     = kRing - 1;
static constexpr uint32_t kSafety   = 4096;
static float DSY_SDRAM_BSS ringL[kRing];
static float DSY_SDRAM_BSS ringR[kRing];
static float* const        ring[2] = {ringL, ringR};

static float sr = 48000.f;

// ------------------------------------------------------------------ rng ----
static uint32_t rngState = 0x9E3779B9u;
inline uint32_t Rnd()
{
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}
inline float Frand() { return (Rnd() >> 8) * (1.f / 16777216.f); }

// ----------------------------------------------------------------- state ---
struct Voice
{
    uint32_t base  = 0;    // ring index of region start
    float    len   = 1.f;  // region length (samples)
    float    phase = 0.f;  // read position inside region
    float    speed = 1.f;  // macro bend speed
    float    slope = 0.f;  // macro bend speed ramp (tape stop/start)
    bool     live  = true; // straight passthrough of the write head
    bool     mute   = false;
    uint32_t muteAt = 0;   // mute starts this many samples into the slice
    float    sign   = 1.f; // -1 = randomly reversed slice
    float    gain   = 1.f;
};

struct Ratio
{
    uint8_t num, den;
};
static const Ratio kMacroRatios[8]
    = {{1, 4}, {1, 3}, {1, 2}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {8, 1}};
static const Ratio kMicroRatios[8]
    = {{1, 16}, {1, 12}, {1, 8}, {1, 6}, {1, 4}, {1, 3}, {1, 2}, {1, 1}};
static const uint8_t kMacroRepeats[8]  = {1, 2, 3, 4, 6, 8, 12, 16};
static const uint8_t kMicroRepeats[10] = {1, 2, 3, 4, 6, 8, 12, 16, 24, 32};

struct CorruptParams
{
    float holdRate  = 1.f;
    float quant     = 32768.f;
    float dropProb  = 0.f;
    float dropAmt   = 0.f;
    float drive     = 1.f;
    float destroyMx = 0.f;
    float djD       = 0.f;
    float vinylAmt  = 0.f;
    float wowDepth  = 0.f;
    float dropLen   = 1.f;
    float foldMix   = 1.f;
    float djRes     = 0.35f;
    float popMul    = 1.f;
};

static Voice voice[2];

static EngineMode mode       = MACRO_TRAVERSE;
static bool       uniqueSt   = false;
static bool       hardWindow = false;

static uint32_t writePos    = 0;
static uint32_t S           = 0;     // buffer (period) start
static uint32_t T           = 24000; // latched buffer length
static uint32_t N           = 1;     // latched repeats
static uint32_t sliceLen    = 24000;
static uint32_t sliceIdx    = 0;
static uint32_t t           = 0; // samples since period start
static uint32_t tInSlice    = 0;
static uint32_t curSliceLen = 24000; // length of current (maybe partial) slice

static bool     frozen       = false;
static uint32_t frozenS      = 0;
static uint32_t frozenLen    = 0;
static bool     reverse      = false;
static bool     capturing    = false;
static uint32_t captureEnd   = 0;
static uint32_t recaptureAck = 0;

static float    targetT     = 24000.f;
static uint32_t targetN     = 1;
static float    bendAmt     = 0.f;
static float    microSpeed  = 1.f;
static float    breakAmt    = 0.f;
static int      corruptType = C_DECIMATE;
static float    mixPrev = 0.f, mixNow = 0.f;

static uint32_t sampleClock = 0;
static uint32_t lastEdge    = 0;
static uint32_t clockPeriod = 0;
static bool     haveEdge    = false;
static bool     lastGate    = false;
static uint32_t edgesSince  = 0;
static bool     clocked     = false;
static bool     resyncReq   = false;
static int      clockIdx    = 3;

static float windowMs      = 10.f;
static float revChance     = 0.f;
static float width         = 1.f;
static float breakShape    = 0.5f;
static float regen         = 0.f;
static float shuffle       = 0.f;
static float randomRepeats = 0.f;
static float fbWet[2]      = {0.f, 0.f};

static CorruptParams cp;

// --------------------------------------------------------------- corrupt ---
struct Dropper
{
    uint32_t remain = 0;
    float    g      = 1.f;
    float    Process()
    {
        float target = 1.f;
        if(remain > 0)
        {
            remain--;
            target = 0.f;
        }
        else if(Frand() < cp.dropProb)
        {
            remain = (uint32_t)((0.004f + Frand() * (0.02f + 0.12f * cp.dropAmt))
                                * cp.dropLen * sr);
        }
        g += (target - g) * 0.08f;
        return g;
    }
};

// Vinyl Sim pop loudness at full Corrupt (0.12 ~ -18 dBFS peak, was 1.0)
static constexpr float kPopLevel = 0.12f;

struct Corruptor
{
    Svf                    dj, vlp, vhp;
    DelayLine<float, 2048> wow;
    float                  holdPh = 0.f, held = 0.f;
    float                  crackle = 0.f, crackleLp = 0.f;

    void Init()
    {
        dj.Init(sr);
        vlp.Init(sr);
        vhp.Init(sr);
        dj.SetRes(0.35f);
        vlp.SetRes(0.1f);
        vhp.SetRes(0.1f);
        wow.Init();
    }

    void SetBlock()
    {
        float d = cp.djD;
        if(d < 0.f)
            dj.SetFreq(20000.f * powf(0.002f, -d)); // 20 kHz -> 40 Hz
        else
            dj.SetFreq(20.f * powf(400.f, d)); // 20 Hz -> 8 kHz
        dj.SetRes(cp.djRes);
        vlp.SetFreq(15000.f - cp.vinylAmt * 11500.f);
        vhp.SetFreq(25.f + cp.vinylAmt * 280.f);
    }

    static inline float Fold(float x)
    {
        x = x * 0.25f + 0.25f;
        x -= floorf(x);
        return 1.f - 4.f * fabsf(x - 0.5f);
    }

    float Process(float x, int type, float dropGain, float wowLfo)
    {
        dj.Process(x); // always run so switching types doesn't click
        switch(type)
        {
            case C_DECIMATE:
            {
                holdPh += cp.holdRate;
                if(holdPh >= 1.f)
                {
                    holdPh -= 1.f;
                    held = x;
                }
                return roundf(held * cp.quant) / cp.quant;
            }
            case C_DROPOUT: return x * dropGain;
            case C_DESTROY:
            {
                const float d  = x * cp.drive;
                const float cl = SoftClip(d);
                float y = SoftClip((cl + (Fold(d) - cl) * cp.foldMix) * 0.9f);
                return x + (y - x) * cp.destroyMx;
            }
            case C_DJFILTER:
            {
                if(fabsf(cp.djD) < 0.03f)
                    return x;
                return cp.djD < 0.f ? dj.Low() : dj.High();
            }
            case C_VINYL:
            default:
            {
                const float a = cp.vinylAmt;
                wow.Write(x);
                wow.SetDelay((0.005f + cp.wowDepth * wowLfo) * sr);
                float y = wow.Read();
                vlp.Process(y);
                y = vlp.Low();
                vhp.Process(y);
                y = vhp.High();
                // pops: mostly small ticks, the odd bigger one, never loud
                if(Frand() < a * a * 0.0025f * cp.popMul)
                {
                    const float r = Frand();
                    crackle = (Frand() < 0.5f ? -1.f : 1.f) * a * kPopLevel
                              * (0.15f + 0.85f * r * r * r);
                }
                crackleLp += 0.35f * (crackle - crackleLp); // soften the click
                crackle *= 0.6f;
                y += crackleLp + (Frand() * 2.f - 1.f) * a * 0.006f;
                y = SoftClip(y * (1.f + a)) / (1.f + 0.4f * a);
                float m = a * 8.f;
                m       = m > 1.f ? 1.f : m;
                return x + (y - x) * m;
            }
        }
    }
};

static Corruptor corr[2];
static Dropper   drop[2];
static float     wowPhase = 0.f;

// --------------------------------------------------------------- helpers ---
inline float ReadRing(const float* buf, uint32_t base, float pos)
{
    uint32_t ip = (uint32_t)pos;
    float    f  = pos - (float)ip;
    float    a  = buf[(base + ip) & kMask];
    float    b  = buf[(base + ip + 1) & kMask];
    return a + (b - a) * f;
}

inline float WrapPhase(float p, float len)
{
    while(p >= len)
        p -= len;
    while(p < 0.f)
        p += len;
    return p;
}

inline float Limit(float x)
{
    const float ax = fabsf(x);
    if(ax < 0.9f)
        return x;
    float y = 0.9f + 0.1f * tanhf((ax - 0.9f) * 10.f);
    return x < 0.f ? -y : y;
}

// decide what a voice does for the slice that is starting now
inline void DecideVoice(Voice& v)
{
    const float L     = (float)sliceLen;
    const bool  micro = (mode == MICRO);
    float       speed = 1.f;
    float       slope = 0.f;
    bool        mute  = false;
    uint32_t    back  = 0; // samples before S (history traverse)
    uint32_t    fwd   = 0; // samples after S (frozen traverse)
    bool        bent  = false;

    if(!micro)
    {
        // BREAK
        if(breakAmt > 0.02f && Frand() < breakAmt)
        {
            if(mode == MACRO_TRAVERSE)
            {
                if(frozen)
                {
                    uint32_t cnt = frozenLen / sliceLen;
                    fwd          = (Rnd() % (cnt > 0 ? cnt : 1)) * sliceLen;
                }
                else
                {
                    uint32_t maxK = 1 + (uint32_t)(breakShape * 14.f);
                    back          = (1 + Rnd() % maxK) * sliceLen;
                }
            }
            else
                mute = Frand() < 0.85f;
        }
        // BEND (automated tape manipulation)
        if(bendAmt > 0.02f && Frand() < bendAmt)
        {
            bent      = true;
            int types = 2 + (int)(bendAmt * 4.99f);
            if(types > 6)
                types = 6;
            switch(Rnd() % types)
            {
                case 0: speed = -1.f; break;     // reverse
                case 1: speed = 0.5f; break;     // half speed
                case 2: slope = -1.f / L; break; // tape stop
                case 3: speed = 2.f; break;      // double speed
                case 4: speed = -0.5f; break;    // reverse slow
                default:
                    speed = 0.f; // tape start
                    slope = 1.f / L;
                    break;
            }
        }
    }
    else
    {
        // Micro BREAK: CCW half traverse (scan), CW half silence
        if(breakAmt < 0.5f)
        {
            float u = breakAmt * 2.f;
            if(u > 0.02f)
            {
                if(frozen)
                    fwd = (uint32_t)(u
                                     * (float)(frozenLen > sliceLen
                                                   ? frozenLen - sliceLen
                                                   : 0));
                else
                {
                    float maxBack = 16.f * (float)T
                                    * fmaxf(0.05f, breakShape * 2.f);
                    float lim     = (float)(kRing - T - kSafety);
                    if(maxBack > lim)
                        maxBack = lim;
                    back = (uint32_t)(u * maxBack);
                }
            }
        }
        else if(Frand() < (breakAmt - 0.5f) * 1.8f)
            mute = true;
        bent = fabsf(microSpeed - 1.f) > 0.002f;
    }

    // random per-slice reverse
    const bool revSlice = revChance > 0.001f && Frand() < revChance;
    if(revSlice)
        bent = true;

    const bool clean = !bent && !reverse && slope == 0.f;

    // silence gating: above half, the start of a muted slice still plays
    uint32_t muteAt = 0;
    if(mute && breakShape > 0.5f)
        muteAt = (uint32_t)((breakShape - 0.5f) * 1.5f * L);

    v.live  = false;
    v.len   = L;
    v.speed = speed;
    v.slope = slope;
    v.mute   = mute;
    v.muteAt = muteAt;
    v.sign   = revSlice ? -1.f : 1.f;
    v.phase  = 0.f;

    if(shuffle > 0.001f && Frand() < shuffle && back == 0 && fwd == 0 && (sliceIdx > 0 || !clean || frozen))
    {
        // BROKEN: every repeat grabs a random slice
        if(frozen)
        {
            uint32_t cnt = frozenLen / sliceLen;
            v.base       = S + (Rnd() % (cnt > 0 ? cnt : 1)) * sliceLen;
        }
        else
            v.base = S - T + (Rnd() % N) * sliceLen;
    }
    else if(frozen)
        v.base = S + fwd;
    else if(back == 0)
    {
        if(sliceIdx == 0)
        {
            if(clean)
            {
                v.live = true;
                v.base = S;
            }
            else
                v.base = S - sliceLen; // previous material, fully recorded
        }
        else
            v.base = S; // repeat of the first slice
    }
    else
    {
        if(sliceIdx == 0 && back < sliceLen)
            back = sliceLen;
        uint32_t lim = kRing - T - kSafety;
        if(back > lim)
            back = (lim / sliceLen) * sliceLen;
        v.base = S - back;
    }
}

inline void StartSlice()
{
    uint32_t remaining = T - t;
    curSliceLen        = remaining < sliceLen ? remaining : sliceLen;
    tInSlice           = 0;
    DecideVoice(voice[0]);
    if(uniqueSt)
        DecideVoice(voice[1]);
    else
    {
        float g       = voice[1].gain;
        voice[1]      = voice[0];
        voice[1].gain = g;
    }
    status.sliceFlash = 1.f;
}

inline void StartPeriod()
{
    float tt = targetT;
    if(frozen && tt > (float)frozenLen)
        tt = (float)frozenLen;
    T = (uint32_t)tt;
    N = targetN;
    if(N > 1 && randomRepeats > 0.001f && Frand() < randomRepeats)
        N = 1 + Rnd() % N;
    if(T < 64)
        T = 64;
    sliceLen = T / N;
    if(sliceLen < 32)
        sliceLen = 32;
    t        = 0;
    sliceIdx = 0;
    S        = frozen ? frozenS : writePos;
    StartSlice();
    status.periodFlash = 1.f;
}

inline void OnClockEdge()
{
    if(haveEdge)
    {
        uint32_t per = sampleClock - lastEdge;
        if(per > (uint32_t)(0.01f * sr) && per < (uint32_t)(10.f * sr))
            clockPeriod = per;
    }
    haveEdge = true;
    lastEdge = sampleClock;

    const Ratio& r
        = (mode == MICRO) ? kMicroRatios[clockIdx] : kMacroRatios[clockIdx];
    edgesSince++;
    if(edgesSince >= r.num)
    {
        edgesSince = 0;
        resyncReq  = true;
    }
}

// ------------------------------------------------------ block controls -----
inline void ApplyControls(const Controls& c)
{
    mode             = c.mode;
    uniqueSt         = c.unique;
    hardWindow       = c.hardWindow;
    windowMs         = c.windowMs;
    revChance        = c.reverseChance;
    width            = c.width;
    breakShape       = c.breakShape;
    regen            = c.regen;
    shuffle          = c.shuffle;
    randomRepeats    = c.randomRepeats;
    const bool micro = (mode == MICRO);

    // clock in
    if(c.gate && !lastGate)
        OnClockEdge();
    lastGate = c.gate;
    clocked  = haveEdge && clockPeriod > 0
              && (sampleClock - lastEdge) < (uint32_t)(clockPeriod * 2.5f)
              && (sampleClock - lastEdge) < (uint32_t)(10.f * sr);

    // TIME
    if(clocked)
    {
        float pos = c.time * 7.999f; // ratio index with hysteresis
        if(fabsf(pos - ((float)clockIdx + 0.5f)) > 0.65f)
            clockIdx = (int)pos;
        const Ratio& r = micro ? kMicroRatios[clockIdx] : kMacroRatios[clockIdx];
        targetT        = (float)clockPeriod * r.num / r.den;
    }
    else
    {
        float tmin = micro ? 0.005f : 0.05f;
        float tmax = micro ? 0.4f : 4.f;
        targetT    = tmin * powf(tmax / tmin, c.time) * sr;
    }
    const float maxT = 5.f * sr;
    if(targetT > maxT)
        targetT = maxT;

    // REPEATS
    targetN = micro ? kMicroRepeats[(int)(c.repeats * 9.999f)]
                    : kMacroRepeats[(int)(c.repeats * 7.999f)];

    // MIX
    mixPrev = mixNow;
    mixNow  = c.mix;

    // BEND
    bendAmt = c.bend;
    {
        float oct = (c.bend - 0.5f) * 5.f; // 1 V/oct over 0..5 V
        if(fabsf(oct) < 0.03f)
            oct = 0.f;
        oct += c.extraOct;
        if(fabsf(oct) < 0.003f)
            oct = 0.f;
        microSpeed = exp2f(fclamp(oct, -4.f, 4.f));
    }

    // BREAK
    breakAmt = c.brk;

    // CORRUPT
    corruptType  = c.corruptType;
    const float a = c.corrupt;
    const float col = c.corruptColor;
    // Decimate: colour < .5 = more rate reduction, > .5 = more bit crush
    const float aRate = a * fminf(1.f, 2.f * (1.f - col));
    const float aBits = a * fminf(1.f, 2.f * col);
    cp.holdRate       = 1.f - aRate * aRate * 0.985f;
    cp.quant          = exp2f(15.f - aBits * 12.f);
    cp.dropProb   = a < 0.01f ? 0.f : (a * a * 30.f) / sr;
    cp.dropAmt    = a;
    cp.drive      = (1.f + a * a * 30.f)
                * (col > 0.5f ? 1.f + (col - 0.5f) * 4.f : 1.f);
    cp.foldMix    = fminf(1.f, col * 2.f);     // clip -> fold
    cp.dropLen    = exp2f((col - 0.5f) * 4.f); // short .. long gaps
    cp.djRes      = fclamp(0.35f + (col - 0.5f) * 1.2f, 0.f, 0.95f);
    cp.popMul     = 2.f * col;                 // wow <-> crackle balance
    cp.destroyMx  = a * 4.f > 1.f ? 1.f : a * 4.f;
    cp.djD        = (a - 0.5f) * 2.f;
    cp.vinylAmt   = a;
    cp.wowDepth   = a * 0.0025f * 2.f * (1.f - col);
    corr[0].SetBlock();
    corr[1].SetBlock();

    reverse = c.reverse;

    // RE-CAPTURE / resync
    if(c.recapture != recaptureAck)
    {
        recaptureAck = c.recapture;
        edgesSince   = 0;
        if(c.freeze)
        {
            // record one fresh buffer, then freeze on it
            frozen     = false;
            capturing  = true;
            captureEnd = writePos + (uint32_t)targetT;
        }
        else
            capturing = false;
        StartPeriod();
    }

    // FREEZE
    if(capturing)
    {
        if(!c.freeze)
            capturing = false;
        else if((int32_t)(writePos - captureEnd) >= 0)
        {
            capturing = false;
            frozenLen = (uint32_t)targetT;
            frozenS   = writePos - frozenLen;
            frozen    = true;
            StartPeriod();
        }
    }
    else if(c.freeze != frozen)
    {
        if(c.freeze)
        {
            frozenLen = T;
            frozenS   = writePos - T;
            frozen    = true;
            StartPeriod(); // loop the last buffer right away
        }
        else
            frozen = false; // next buffer start goes live again
    }

    if(resyncReq)
    {
        resyncReq = false;
        StartPeriod();
    }

    status.frozen    = frozen;
    status.reverse   = reverse;
    status.capturing = capturing;
    status.clocked   = clocked;
}

// ---------------------------------------------------------------- public ---
inline void Init(float samplerate, uint32_t seed)
{
    sr = samplerate;
    for(uint32_t i = 0; i < kRing; i++)
    {
        ringL[i] = 0.f;
        ringR[i] = 0.f;
    }
    corr[0].Init();
    corr[1].Init();
    rngState ^= seed * 2654435761u + 1u;
    if(rngState == 0)
        rngState = 0x9E3779B9u;
    StartPeriod();
}

inline void SetInitialMix(float m) { mixNow = mixPrev = m; }

inline void Process(const Controls&            c,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    ApplyControls(c);

    const bool  micro   = (mode == MICRO);
    const bool  twoTap  = micro && !hardWindow;
    const float revSign = reverse ? -1.f : 1.f;
    const float wowInc  = 2.f * PI_F * 0.55f / sr;
    float       peak    = 0.f;

    for(size_t i = 0; i < size; i++)
    {
        const float inL = in[0][i];
        const float inR = in[1][i];

        if(!frozen)
        {
            if(regen > 0.001f)
            {
                ringL[writePos & kMask] = Limit(inL + regen * fbWet[0]);
                ringR[writePos & kMask] = Limit(inR + regen * fbWet[1]);
            }
            else
            {
                ringL[writePos & kMask] = inL;
                ringR[writePos & kMask] = inR;
            }
        }

        // glitch window for this sample
        float wN = (float)curSliceLen * 0.25f;
        const float wBase = windowMs * 0.001f * sr;
        float       W     = hardWindow ? fmaxf(8.f, wBase * 0.05f) : wBase;
        if(W > wN)
            W = wN;
        if(W < 1.f)
            W = 1.f;
        float win;
        {
            float a = (float)tInSlice / W;
            float b = (float)(curSliceLen - tInSlice) / W;
            win     = a < b ? a : b;
            if(win > 1.f)
                win = 1.f;
        }

        float wet[2];
        for(int ch = 0; ch < 2; ch++)
        {
            Voice& v = voice[ch];
            float  s;
            if(v.live)
                s = ring[ch][(v.base + tInSlice) & kMask];
            else
            {
                float sp
                    = (micro ? microSpeed : v.speed) * revSign * v.sign;
                if(twoTap)
                {
                    float p2 = WrapPhase(v.phase + v.len * 0.5f, v.len);
                    float x  = v.phase / v.len;
                    float g1 = 1.f - fabsf(2.f * x - 1.f);
                    s        = ReadRing(ring[ch], v.base, v.phase) * g1
                        + ReadRing(ring[ch], v.base, p2) * (1.f - g1);
                }
                else
                    s = ReadRing(ring[ch], v.base, v.phase);
                v.phase = WrapPhase(v.phase + sp, v.len);
                v.speed += v.slope;
            }
            const float gt = (v.mute && tInSlice >= v.muteAt) ? 0.f : 1.f;
            v.gain += (gt - v.gain) * 0.01f;
            const bool needWin = !(v.live && N == 1);
            wet[ch]            = s * v.gain * (needWin ? win : 1.f);
        }

        // corrupt (wet path)
        wowPhase += wowInc;
        if(wowPhase > 2.f * PI_F)
            wowPhase -= 2.f * PI_F;
        const float wowLfo = sinf(wowPhase) + 0.3f * sinf(3.1f * wowPhase + 1.f);
        float       dg0 = 1.f, dg1 = 1.f;
        if(corruptType == C_DROPOUT)
        {
            dg0 = drop[0].Process();
            dg1 = uniqueSt ? drop[1].Process() : dg0;
        }
        wet[0] = corr[0].Process(wet[0], corruptType, dg0, wowLfo);
        wet[1] = corr[1].Process(wet[1], corruptType, dg1, wowLfo);

        // stereo width (mid/side on the wet signal)
        if(fabsf(width - 1.f) > 0.01f)
        {
            const float mid  = 0.5f * (wet[0] + wet[1]);
            const float side = 0.5f * (wet[0] - wet[1]) * width;
            wet[0]           = mid + side;
            wet[1]           = mid - side;
        }

        // regen source: only buffer playback, never the live passthrough
        fbWet[0] = voice[0].live ? 0.f : wet[0];
        fbWet[1] = voice[1].live ? 0.f : wet[1];

        // equal-power mix, interpolated across the block
        const float m  = mixPrev + (mixNow - mixPrev) * ((float)i / (float)size);
        const float dG = cosf(m * HALFPI_F);
        const float wG = sinf(m * HALFPI_F);
        const float oL = Limit(inL * dG + wet[0] * wG);
        const float oR = Limit(inR * dG + wet[1] * wG);
        out[0][i]      = oL;
        out[1][i]      = oR;
        peak           = fmaxf(peak, fmaxf(fabsf(oL), fabsf(oR)));

        // advance time
        if(!frozen)
            writePos++;
        sampleClock++;
        t++;
        tInSlice++;
        if(t >= T)
            StartPeriod();
        else if(tInSlice >= curSliceLen)
        {
            sliceIdx++;
            StartSlice();
        }
    }
    status.outLevel = peak;
}

inline float DjAmount() { return fabsf(cp.djD); }

// LED colour per corrupt type
inline void CorruptColor(int type, float& r, float& g, float& b)
{
    switch(type)
    {
        case C_DECIMATE: r = 1.f, g = 0.8f, b = 0.f; break; // yellow
        case C_DROPOUT: r = 1.f, g = 0.3f, b = 0.f; break;  // orange
        case C_DESTROY: r = 1.f, g = 0.f, b = 0.f; break;   // red
        case C_DJFILTER: r = 0.5f, g = 0.f, b = 1.f; break; // purple
        default: r = 0.6f, g = 0.6f, b = 0.5f; break;       // warm white
    }
}

} // namespace dbe
