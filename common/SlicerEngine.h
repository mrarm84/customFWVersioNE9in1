// SlicerEngine.h - clock-locked beat slicer, second mode of VersioBendSlice.
// Header-only; include it from exactly one .cpp per firmware.
//
// The input is recorded continuously into a stereo ring. Time is cut into
// STEPS (one step = a division of the incoming clock, or an internal time).
// A LOOP is N steps. Every step plays one SLICE of the bar:
//   - straight step  -> the slice being recorded right now (zero latency,
//                       so with everything at zero the wet path is the input)
//   - shuffled step  -> another slice of the bar (the freshest recorded copy)
//   - plus per step:  reverse, ratchet (2..8 retriggers), octave jumps,
//                     gate length, global pitch.
// Which steps deviate comes from a per-step "gene" (stored random numbers),
// compared against the SHUFFLE / STUTTER knobs, so turning a knob up adds
// deviations progressively and repeatably:
//   LOCKED  genes never change -> the same rearranged beat every loop
//   DRIFT   a few genes re-roll at every loop start -> slowly evolving beat
//   CHAOS   genes re-roll every step -> pure random slicing
// Re-capture (hold) re-rolls all genes = a new pattern.
#pragma once
#include "daisy_seed.h"
#include "daisysp.h"
#include <cmath>

namespace slc
{
using namespace daisy;
using namespace daisysp;

// ---------------------------------------------------------------- public ---
enum Character
{
    CH_LOCKED,
    CH_DRIFT,
    CH_CHAOS
};

struct Controls
{
    float    rate       = 0.5f; // step length / clock division, CW = finer
    float    steps      = 0.5f; // slices per loop
    float    mix        = 0.5f; // dry -> wet, equal power
    float    shuffle    = 0.f;  // probability a step plays another slice
    float    stutter    = 0.f;  // probability + density of ratchets
    float    pitch      = 0.5f; // noon = 1x, 1 V/oct over the 0..5 V CV
    float    gate       = 1.f;  // slice length: CCW staccato .. CW legato
    int      character  = CH_LOCKED;
    int      play       = 0;     // 0 CUT, 1 TAPE, 2 GRAIN
    float    fade       = 0.564f; // slice fades 0.5 .. 20 ms (default 4 ms)
    float    stereo     = 0.f;   // chance R slices on its own
    float    revChance  = 0.f;   // extra chance of reversed steps (1 = all)
    float    swing      = 0.f;   // odd steps late
    float    texture    = 0.f;   // CUT start offset / TAPE ramp / GRAIN size
    float    ratPitch   = 0.5f;  // ratchet pitch step, noon = none
    float    drop       = 0.f;   // chance a step is silent
    bool     clockIn    = false; // gate input level
    bool     freeze     = false;
    bool     reverse    = false;
    uint32_t recapture  = 0; // increment = re-capture / resync / new pattern
};

enum StepKind : uint8_t
{
    K_SHUF = 1,
    K_REV  = 2,
    K_RAT  = 4,
    K_OCT  = 8,
};

struct Status
{
    volatile bool    frozen    = false;
    volatile bool    reverse   = false;
    volatile bool    capturing = false;
    volatile bool    clocked   = false;
    volatile float   stepFlash = 0.f;
    volatile float   loopFlash = 0.f;
    volatile float   outLevel  = 0.f;
    volatile uint8_t stepKind  = 0;
};
static Status status;

// --------------------------------------------------------------- buffer ----
static constexpr uint32_t kRingBits = 19;
static constexpr uint32_t kRing     = 1u << kRingBits; // 524288 ~ 10.9 s
static constexpr uint32_t kMask     = kRing - 1;
static constexpr uint32_t kSafety   = 4096;
static constexpr uint32_t kMaxLoop  = kRing / 2 - kSafety; // ~5.3 s
static constexpr uint32_t kMaxSteps = 32;
static float DSY_SDRAM_BSS ringL[kRing];
static float DSY_SDRAM_BSS ringR[kRing];
static float* const        ring[2] = {ringL, ringR};

static float sr = 48000.f;

// ------------------------------------------------------------------ rng ----
static uint32_t rngState = 0x2545F491u;
inline uint32_t Rnd()
{
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}
inline float Frand() { return (Rnd() >> 8) * (1.f / 16777216.f); }

// ------------------------------------------------------------ tables -------
struct Ratio
{
    uint8_t num, den; // step = clock period * num / den
};
static const Ratio kRatios[8]
    = {{2, 1}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 6}, {1, 8}, {1, 16}};
static const uint8_t kSteps[8] = {2, 3, 4, 6, 8, 12, 16, 32};
static const uint8_t kRats[5]  = {2, 3, 4, 6, 8};

// ------------------------------------------------------------- genes -------
struct Gene
{
    float slice, dev, rev, rat, ratN, oct, rv, un, dp;
};
static Gene genes[2][kMaxSteps];

inline void RollGene(Gene& g)
{
    g.slice = Frand();
    g.dev   = Frand();
    g.rev   = Frand();
    g.rat   = Frand();
    g.ratN  = Frand();
    g.oct   = Frand();
    g.rv    = Frand();
    g.un    = Frand();
    g.dp    = Frand();
}
inline void RollAll()
{
    for(int c = 0; c < 2; c++)
        for(uint32_t s = 0; s < kMaxSteps; s++)
            RollGene(genes[c][s]);
}

// ------------------------------------------------------------- voices ------
struct Voice
{
    uint32_t base  = 0;
    float    len   = 1.f; // wrap length of the read region
    float    phase = 0.f;
    float    speed = 1.f;
    float    slope = 0.f; // speed change per sample (tape stop / start)
};

struct Chan
{
    Voice    cur, old;
    float    oldAmp = 0.f;
    uint32_t oldT   = 0;
    bool     oldOn  = false;
    float    env    = 0.f;
    uint32_t tSub   = 0; // samples since last (re)trigger

    // plan for the current step
    uint32_t srcBase = 0;
    float    srcLen  = 1.f;
    float    speed   = 1.f;
    bool     rev     = false;
    int      rat     = 1;
    int      hit     = 0;
    uint32_t ratLen  = 1;
    uint8_t  kind    = 0;
    bool     mute    = false;
    float    slope   = 0.f; // TAPE ramp (per sample)
    float    startOff = 0.f; // CUT start offset into the slice
};
static Chan chan[2];

// ------------------------------------------------------------- state -------
static uint32_t writePos   = 0;
static uint32_t G          = 0; // global step counter (clock aligned)
static uint32_t k          = 0; // step inside the loop
static uint32_t L          = 12000; // latched step length
static uint32_t nSteps     = 8;     // latched steps per loop
static uint32_t loopLen    = 96000;
static uint32_t curBar     = 0; // ring index where this loop started
static uint32_t stepStart  = 0; // ring index where this step started
static uint32_t tInStep    = 0;
static uint32_t curStepLen = 12000;

static float    targetL     = 12000.f;
static uint32_t targetSteps = 8;

static bool     frozen       = false;
static uint32_t frozenBase   = 0;
static uint32_t frozenLen    = 0;
static bool     capturing    = false;
static bool     capArmed     = false;
static bool     reverse      = false;
static uint32_t recaptureAck = 0;

static int   character  = CH_LOCKED;
static int   playMode   = 0;
static float fadeMs     = 4.f;
static float stereoAmt  = 0.f;
static float revAmt     = 0.f;
static float swingAmt   = 0.f;
static float texAmt     = 0.f;
static float ratSemi    = 0.f;
static float dropAmt    = 0.f;
static float shuffleAmt = 0.f;
static float stutterAmt = 0.f;
static float speedKnob  = 1.f;
static float gateFrac   = 1.f;
static float mixPrev = 0.f, mixNow = 0.f;

// clock
static uint32_t sampleClock = 0;
static uint32_t lastEdge    = 0;
static uint32_t clockPeriod = 0;
static bool     haveEdge    = false;
static bool     lastGate    = false;
static bool     clocked     = false;
static uint32_t edgeIdx     = 0;
static int      rateIdx     = 1;
static bool     syncReq     = false;
static uint32_t syncG       = 0;

// --------------------------------------------------------------- helpers ---
inline float ReadRing(const float* buf, uint32_t base, float pos)
{
    uint32_t ip = (uint32_t)pos;
    float    f  = pos - (float)ip;
    float    a  = buf[(base + ip) & kMask];
    float    b  = buf[(base + ip + 1) & kMask];
    return a + (b - a) * f;
}

inline float Limit(float x)
{
    const float ax = fabsf(x);
    if(ax < 0.9f)
        return x;
    float y = 0.9f + 0.1f * tanhf((ax - 0.9f) * 10.f);
    return x < 0.f ? -y : y;
}

inline float ReadVoice(Voice& v, int c)
{
    const float s = ReadRing(ring[c], v.base, v.phase);
    v.phase += v.speed;
    while(v.phase >= v.len)
        v.phase -= v.len;
    while(v.phase < 0.f)
        v.phase += v.len;
    return s;
}

// (re)start the channel's voice at the planned source, crossfading the old one
inline void Trigger(Chan& ch)
{
    ch.old    = ch.cur;
    ch.oldAmp = ch.env;
    ch.oldT   = 0;
    ch.oldOn  = ch.env > 0.0001f;

    Voice& v = ch.cur;
    v.base   = ch.srcBase;
    v.len    = ch.srcLen;
    // ratchet pitch: every retrigger steps by ratSemi semitones
    const float hm = ch.hit > 0 && ratSemi != 0.f
                         ? exp2f(ratSemi * (float)ch.hit / 12.f)
                         : 1.f;
    v.speed = ch.speed * hm;
    v.slope = ch.slope * hm;
    if(ch.slope > 0.f || ch.slope < 0.f)
    {
        // tape start begins from standstill
        if((ch.slope > 0.f) == (ch.speed > 0.f))
            v.speed = 0.f;
    }
    float st = ch.startOff + (ch.rev ? (float)ch.ratLen - 1.f : 0.f);
    if(st >= v.len)
        st = v.len - 1.f;
    v.phase = st < 0.f ? 0.f : st;
    ch.tSub = 0;
}

// work out what one channel plays during step k
inline void Plan(Chan& ch, Gene& g)
{
    if(character == CH_CHAOS)
        RollGene(g);

    const float sh = shuffleAmt < 0.02f ? 0.f : shuffleAmt;
    const float st = stutterAmt < 0.02f ? 0.f : stutterAmt;

    uint32_t src  = k;
    uint8_t  kind = 0;
    if(nSteps > 1 && g.dev < sh)
    {
        src = (k + 1 + (uint32_t)(g.slice * (float)(nSteps - 1))) % nSteps;
        kind |= K_SHUF;
    }
    bool rev = g.rev < sh * (character == CH_CHAOS ? 0.5f : 0.3f)
               || g.rv < revAmt;
    if(rev)
        kind |= K_REV;

    int rat = 1;
    if(g.rat < st)
    {
        int n = 1 + (int)(st * 4.99f); // more retrigger choices as it rises
        if(n > 5)
            n = 5;
        int idx = (int)(g.ratN * (float)n);
        if(idx > 4)
            idx = 4;
        rat = kRats[idx];
        kind |= K_RAT;
    }

    float sp = speedKnob;
    if(character == CH_CHAOS && g.oct < sh * 0.25f)
    {
        sp *= g.oct < sh * 0.125f ? 2.f : 0.5f;
        kind |= K_OCT;
    }
    if(sp > 4.f)
        sp = 4.f;
    if(reverse)
        rev = !rev;

    ch.rev    = rev;
    ch.speed  = rev ? -sp : sp;
    ch.rat    = rat;
    ch.hit    = 0;
    ch.kind   = kind;
    ch.mute   = g.dp < dropAmt;
    ch.ratLen = curStepLen / (uint32_t)rat;
    if(ch.ratLen < 32)
        ch.ratLen = 32;

    // TAPE: deviated steps and the last step of every loop ramp the speed
    // (tape stop or tape start)
    ch.slope = 0.f;
    if(playMode == 1 && (kind != 0 || k + 1 == nSteps))
    {
        const float rl = (float)curStepLen * (0.25f + 0.75f * (1.f - texAmt));
        const bool  stop = k + 1 == nSteps || g.ratN > 0.5f;
        // stop: speed -> 0 ; start: 0 -> speed (Trigger zeroes the start)
        ch.slope = (stop ? -ch.speed : ch.speed) / rl;
    }

    // where the slice lives in the ring
    ch.srcLen   = (float)L;
    ch.startOff = 0.f;
    const bool deviated = src != k || rev || sp > 1.0001f || ch.slope != 0.f;
    if(frozen)
    {
        const uint32_t fl = frozenLen > 0 ? frozenLen : 1;
        ch.srcBase        = frozenBase + (src * L) % fl;
        if(ch.srcLen > (float)fl)
            ch.srcLen = (float)fl;
    }
    else if(!deviated)
    {
        // the slice that is being recorded right now (zero latency);
        // never wraps, reads at or behind the write head
        ch.srcBase = stepStart;
        ch.srcLen  = (float)(kRing / 4);
    }
    else if(src < k)
        ch.srcBase = curBar + src * L; // already recorded in this loop
    else
        ch.srcBase = curBar - loopLen + src * L; // previous loop

    if(playMode == 2)
    {
        // GRAIN: every step loops a short grain of its slice
        ch.srcLen = (0.005f + 0.115f * texAmt * texAmt) * sr;
        if(ch.srcLen > (float)L)
            ch.srcLen = (float)L;
    }
    else if(playMode == 0 && texAmt > 0.01f && (deviated || frozen))
        ch.startOff = texAmt * 0.5f * (float)L; // CUT: start later in the slice
}

inline void BeginStep(uint32_t newK)
{
    if(newK == 0)
    {
        // loop boundary
        if(capturing)
        {
            if(capArmed)
            {
                frozenBase = curBar; // the loop that was just recorded
                frozenLen  = loopLen;
                frozen     = true;
                capturing  = false;
            }
            else
                capArmed = true;
        }

        float tl = targetL;
        if(tl > (float)kMaxLoop)
            tl = (float)kMaxLoop;
        if(tl < 256.f)
            tl = 256.f;
        L            = (uint32_t)tl;
        uint32_t lim = kMaxLoop / L;
        nSteps       = targetSteps < lim ? targetSteps : lim;
        if(nSteps < 1)
            nSteps = 1;
        loopLen = L * nSteps;
        if(!frozen)
            curBar = writePos;

        if(character == CH_DRIFT)
            for(int c = 0; c < 2; c++)
                for(uint32_t s = 0; s < kMaxSteps; s++)
                    if(Frand() < 0.125f)
                        RollGene(genes[c][s]);
        status.loopFlash = 1.f;
    }

    k         = newK;
    tInStep   = 0;
    stepStart = writePos;
    // swing: on-beat steps longer, off-beat steps shorter
    curStepLen = (uint32_t)((float)L * (1.f + swingAmt * ((G & 1) ? -1.f : 1.f)));
    if(curStepLen < 64)
        curStepLen = 64;

    Plan(chan[0], genes[0][k]);
    if(genes[1][k].un < stereoAmt)
        Plan(chan[1], genes[1][k]);
    else
    {
        Chan& a    = chan[0];
        Chan& b    = chan[1];
        b.srcBase  = a.srcBase;
        b.srcLen   = a.srcLen;
        b.speed    = a.speed;
        b.rev      = a.rev;
        b.rat      = a.rat;
        b.hit      = 0;
        b.ratLen   = a.ratLen;
        b.kind     = a.kind;
        b.mute     = a.mute;
        b.slope    = a.slope;
        b.startOff = a.startOff;
    }
    Trigger(chan[0]);
    Trigger(chan[1]);

    status.stepFlash = 1.f;
    status.stepKind  = chan[0].kind | chan[1].kind;
}

inline void OnClockEdge()
{
    const bool wasClocked = clocked;
    if(haveEdge)
    {
        uint32_t per = sampleClock - lastEdge;
        if(per > (uint32_t)(0.01f * sr) && per < (uint32_t)(10.f * sr))
            clockPeriod = per;
    }
    haveEdge = true;
    lastEdge = sampleClock;

    if(!wasClocked)
    {
        // clock (re)appeared: this edge is the downbeat
        edgeIdx = 0;
        syncG   = 0;
        syncReq = true;
        return;
    }
    edgeIdx++;
    const Ratio& r = kRatios[rateIdx];
    if(edgeIdx % r.num == 0)
    {
        syncG   = (edgeIdx / r.num) * r.den;
        syncReq = true;
    }
}

inline void ApplyControls(const Controls& c)
{
    character  = c.character;
    playMode   = c.play;
    fadeMs     = 0.5f * powf(40.f, c.fade);
    stereoAmt  = c.stereo;
    revAmt     = c.revChance;
    swingAmt   = c.swing * 0.33f;
    texAmt     = c.texture;
    ratSemi    = (c.ratPitch - 0.5f) * 8.f; // +-4 semitones per hit
    if(fabsf(ratSemi) < 0.15f)
        ratSemi = 0.f;
    dropAmt    = c.drop;
    shuffleAmt = c.shuffle;
    stutterAmt = c.stutter;
    reverse    = c.reverse;
    gateFrac   = c.gate > 0.98f ? 1.f : 0.04f + 0.96f * c.gate;

    mixPrev = mixNow;
    mixNow  = c.mix;

    {
        float oct = (c.pitch - 0.5f) * 5.f; // 1 V/oct over 0..5 V
        if(fabsf(oct) < 0.04f)
            oct = 0.f;
        speedKnob = exp2f(fclamp(oct, -3.f, 2.f));
    }

    // clock in
    if(c.clockIn && !lastGate)
        OnClockEdge();
    lastGate = c.clockIn;
    clocked  = haveEdge && clockPeriod > 0
              && (sampleClock - lastEdge) < (uint32_t)(clockPeriod * 2.5f)
              && (sampleClock - lastEdge) < (uint32_t)(10.f * sr);

    // RATE: clock division, or internal step time (CW = shorter)
    if(clocked)
    {
        float pos = c.rate * 7.999f;
        if(fabsf(pos - ((float)rateIdx + 0.5f)) > 0.65f)
            rateIdx = (int)pos;
        const Ratio& r = kRatios[rateIdx];
        targetL        = (float)clockPeriod * r.num / r.den;
    }
    else
    {
        const float tmax = 0.75f, tmin = 0.025f;
        targetL          = tmax * powf(tmin / tmax, c.rate) * sr;
    }
    targetSteps = kSteps[(int)(c.steps * 7.999f)];

    // RE-CAPTURE / resync / new pattern
    if(c.recapture != recaptureAck)
    {
        recaptureAck = c.recapture;
        RollAll();
        edgeIdx = 0;
        syncReq = false;
        if(c.freeze)
        {
            frozen    = false; // record one fresh loop, then freeze on it
            capturing = true;
            capArmed  = false;
        }
        else
            capturing = false;
        G = 0;
        BeginStep(0);
    }

    // FREEZE
    if(capturing)
    {
        if(!c.freeze)
            capturing = false;
    }
    else if(c.freeze != frozen)
    {
        if(c.freeze)
        {
            frozenBase = curBar - loopLen; // last complete loop, grid aligned
            frozenLen  = loopLen;
            frozen     = true;
        }
        else
        {
            frozen = false;
            curBar = writePos - k * L; // keep the grid going from here
            stepStart = writePos;
        }
    }

    // clock sync of the step grid
    if(syncReq)
    {
        syncReq        = false;
        const uint32_t kT = syncG % nSteps;
        G                 = syncG;
        if(tInStep > curStepLen / 4 || k != kT)
            BeginStep(kT);
    }

    status.frozen    = frozen;
    status.reverse   = reverse;
    status.capturing = capturing;
    status.clocked   = clocked;
}

inline float ChanSample(Chan& ch, int c, float W)
{
    // ratchet retrigger
    if(ch.tSub >= ch.ratLen && ch.hit + 1 < ch.rat)
    {
        ch.hit++;
        Trigger(ch);
    }

    float w = W;
    if(w > (float)ch.ratLen * 0.25f)
        w = (float)ch.ratLen * 0.25f;
    if(w < 1.f)
        w = 1.f;

    // fade-in mirrors the old voice's fade-out: the sum is exactly 1, so
    // straight (live) steps crossfade into identical audio = transparent
    float env = (float)ch.tSub / w;
    if(env > 1.f)
        env = 1.f;
    if(gateFrac < 1.f)
    {
        const float gl = (float)ch.ratLen * gateFrac;
        float       go = (gl - (float)ch.tSub) / w;
        go             = go < 0.f ? 0.f : (go > 1.f ? 1.f : go);
        env *= go;
    }
    if(ch.mute)
        env = 0.f;

    // TAPE ramps: speed slides towards / away from standstill
    Voice& v = ch.cur;
    if(v.slope != 0.f)
    {
        const float ns = v.speed + v.slope;
        if((ns > 0.f) != (v.speed > 0.f) && v.speed != 0.f)
        {
            v.speed = 0.f; // stopped
            v.slope = 0.f;
        }
        else
            v.speed = ns;
        if(fabsf(v.speed) > fabsf(ch.speed) && fabsf(ch.speed) > 0.f)
        {
            v.speed = ch.speed; // started: reached play speed
            v.slope = 0.f;
        }
    }

    float s = ReadVoice(ch.cur, c) * env;
    if(ch.oldOn)
    {
        const float a = ch.oldAmp * (1.f - (float)ch.oldT / w);
        if(a <= 0.f)
            ch.oldOn = false;
        else
            s += ReadVoice(ch.old, c) * a;
        ch.oldT++;
    }
    ch.env = env;
    ch.tSub++;
    return s;
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
    rngState ^= seed * 2246822519u + 7u;
    if(rngState == 0)
        rngState = 0x2545F491u;
    RollAll();
    G = 0;
    BeginStep(0);
}

inline void SetInitialMix(float m) { mixNow = mixPrev = m; }

// called when this engine becomes the active one
inline void Activate()
{
    frozen    = false;
    capturing = false;
    syncReq   = false;
    edgeIdx   = 0;
    chan[0].env = chan[1].env = 0.f;
    G = 0;
    BeginStep(0);
}

// keep recording while the other engine is active, so the slicer never
// starts on stale audio
inline void Idle(AudioHandle::InputBuffer in, size_t size)
{
    for(size_t i = 0; i < size; i++)
    {
        ringL[writePos & kMask] = in[0][i];
        ringR[writePos & kMask] = in[1][i];
        writePos++;
        sampleClock++;
    }
}

inline void Process(const Controls&            c,
                    AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    size)
{
    ApplyControls(c);

    const float W    = fadeMs * 0.001f * sr;
    float       peak = 0.f;

    for(size_t i = 0; i < size; i++)
    {
        const float inL = in[0][i];
        const float inR = in[1][i];
        if(!frozen)
        {
            ringL[writePos & kMask] = inL;
            ringR[writePos & kMask] = inR;
        }

        const float wL = ChanSample(chan[0], 0, W);
        const float wR = ChanSample(chan[1], 1, W);

        const float m  = mixPrev + (mixNow - mixPrev) * ((float)i / (float)size);
        const float dG = cosf(m * HALFPI_F);
        const float wG = sinf(m * HALFPI_F);
        const float oL = Limit(inL * dG + wL * wG);
        const float oR = Limit(inR * dG + wR * wG);
        out[0][i]      = oL;
        out[1][i]      = oR;
        peak           = fmaxf(peak, fmaxf(fabsf(oL), fabsf(oR)));

        if(!frozen)
            writePos++;
        sampleClock++;
        tInStep++;
        if(tInStep >= curStepLen)
        {
            G++;
            uint32_t nk = k + 1;
            if(nk >= nSteps)
                nk = 0;
            BeginStep(nk);
        }
    }
    status.outLevel = peak;
}

} // namespace slc
