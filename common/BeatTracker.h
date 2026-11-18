// BeatTracker.h - "beat recognition" for every mode of VersioBeatSend.
//
// Tempo source, in order of priority:
//   GATE   a clock on the gate jack (1 pulse = 1 beat): phase hard-synced
//          to every edge.
//   AUDIO  beat tracking of the audio input:
//            onset strength = rise of log(1 + 100 * rms) in 4 bands
//                     (<150 Hz, 150-800, 800-4k, >4k), each band minus its
//                     own running mean; one frame per 128 samples (375/s)
//            tempo  = leaky (5 s) autocorrelation of the onset strength,
//                     60..200 BPM, weighted towards 120 BPM
//            phase  = the offset that best lines a beat comb up with the
//                     last 8 beats of onsets (kick band counted twice), then
//                     a gentle PLL. Once locked only the spot near the
//                     running beat is searched, so 8th notes / an off-beat
//                     bass line can't pull it; if a much better spot shows
//                     up elsewhere for 0.35 s (tempo jump) it jumps there.
//            lock   = smoothed tempo confidence AND comb sharpness above a
//                     threshold for 1 s (noise and pads stay unlocked)
//            bar    = the beat with clearly stronger kicks becomes beat 1
//   FREE   nothing locked: the beat keeps running at the last tempo
//          (120 BPM at power-up), but no clock goes to the engines, so they
//          run on their own knob times.
//
// Tuned on synthetic test signals (straight / swung drums 95-174 BPM, drums
// under a loud pad, drums + bass + pad, -34 dB input, noise, pads, a tempo
// jump) with the Python model in VersioBeatSend/tools:
//   python -I beat_tracker_test.py .
//
// Process() runs in the audio callback (filters, onset frames, the
// autocorrelation update). Update() does the heavy tempo/phase search and
// runs in the main loop, so it never loads the audio callback.
//
// The reused engines (bender, tape, slicer, resonator) get st.pulse as their
// gate input. The repeater and saturator read the beat position (Pos()).
#pragma once
#include "daisy_seed.h"
#include <cmath>
#include <string.h>
#include "Panel.h"

namespace bt
{
using namespace daisy;

enum Source : uint8_t
{
    SRC_FREE,
    SRC_AUDIO,
    SRC_GATE
};

// snapshot at the start of the current audio block
struct State
{
    float    period = 24000.f;       // samples per beat
    float    phase  = 0.f;           // 0..1 inside the beat
    float    inc    = 1.f / 24000.f; // phase per sample in this block
    uint32_t beat   = 0;             // beat counter, beat % 4 = place in bar
    uint8_t  source = SRC_FREE;
    bool     pulse  = false; // clock for the reused engines
};
static State st;

// beat position at sample i of this block, 0..mod beats
inline float Pos(size_t i, uint32_t mod)
{
    float p = (float)(st.beat % mod) + st.phase + st.inc * (float)i;
    if(p >= (float)mod)
        p -= (float)mod;
    return p;
}

// ------------------------------------------------------------ settings -----
static constexpr uint32_t kHop   = 128;
static constexpr uint32_t kHist  = 4096, kHM = kHist - 1;
static constexpr int      kLmin  = 112; // frames: 200 BPM
static constexpr int      kLmax  = 375; // frames: 60 BPM
static constexpr uint32_t kEvery = 32;  // search every 85 ms
static constexpr int      kNJ    = 8;   // beats in the phase comb
static constexpr float    kGamma = 100.f;
static constexpr float    kLockC = 0.007f, kLockK = 0.74f; // lock above
static constexpr float    kUnlC  = 0.004f, kUnlK = 0.65f;  // unlock below
static constexpr int      kGoodN = 12, kBadN = 24;         // ~1 s / ~2 s
static constexpr float    kPll   = 0.15f;
static constexpr float    kJumpR = 1.5f;
static constexpr float    kBarR  = 1.6f;

// ---------------------------------------------------------------- state ----
static float DSY_SDRAM_BSS hist[kHist];  // onset strength per frame
static float DSY_SDRAM_BSS histL[kHist]; // kick band onset per frame
static float               acf[kLmax + 2];
static float               wTab[kLmax + 2];
static float               comb[kLmax + 2];

static float    sr = 48000.f, fr = 375.f;
static float    f0 = 0.f, f1 = 0.f, f2 = 0.f, f3 = 0.f;
static float    c150 = 0.f, c800 = 0.f, c4k = 0.f;
static float    acc[4] = {0.f, 0.f, 0.f, 0.f}, prevE[4] = {0.f, 0.f, 0.f, 0.f};
static float    bandMean[4] = {0.f, 0.f, 0.f, 0.f};
static float    acf0 = 0.f, accDecay = 0.999f, lowOn = 0.f;
static uint32_t hopN = 0, silent = 0;
static volatile uint32_t fn = 0;

// audio thread <-> main loop
static volatile bool     evalReq = false;
static volatile uint32_t evalFn  = 0;
static volatile float    evalPhase = 0.f;
static volatile float    periodF = 187.5f; // beat period in frames
static volatile float    pllMul  = 1.f;
static volatile bool     snapReq = false;
static volatile float    snapPhase = 0.f;
static volatile uint32_t snapFn  = 0;
static volatile bool     audioLock = false, emit = false;
// free running (nothing locked): this tempo if > 0 (drum mode TEMPO),
// otherwise the last locked tempo (120 BPM at power-up)
static volatile float    freeBpm = 0.f;
static float             runPer  = 24000.f;

// main loop only
static float candF = 0.f, cS = 0.f, kS = 0.f;
static int   candN = 0, goodN = 0, badN = 0, jumpN = 0;

// audio thread only
static float     phase = 0.f;
static uint32_t  beat  = 0;
static float     barAcc[4] = {0.f, 0.f, 0.f, 0.f};
static int       barN = 0, barBest = 0;
static pn::Clock clk;
static bool      gateWas = false;

// for the LEDs
static volatile float flash     = 0.f;
static volatile bool  flashDown = false;

inline float Bpm() { return 60.f * sr / st.period; }

inline void Init(float s)
{
    sr = s;
    fr = sr / (float)kHop;
    memset(hist, 0, sizeof(hist));
    memset(histL, 0, sizeof(histL));
    memset(acf, 0, sizeof(acf));
    const float l120 = fr * 0.5f; // 120 BPM
    for(int L = 0; L <= kLmax + 1; L++)
    {
        const float x = L > 0 ? log2f((float)L / l120) / 0.8f : 0.f;
        wTab[L]       = expf(-0.5f * x * x);
    }
    accDecay  = expf(-1.f / (fr * 5.f)); // ~5 s memory
    c150      = pn::OnePole(150.f, sr);
    c800      = pn::OnePole(800.f, sr);
    c4k       = pn::OnePole(4000.f, sr);
    periodF   = fr * 0.5f;
    runPer    = 0.5f * sr;
    st.period = periodF * (float)kHop;
    st.inc    = 1.f / st.period;
}

// ------------------------------------------------------- audio thread ------
inline void Frame()
{
    float tot = 0.f, o = 0.f;
    for(int b = 0; b < 4; b++)
    {
        tot += acc[b];
        const float e = log1pf(kGamma * sqrtf(acc[b] / (float)kHop));
        acc[b]        = 0.f;
        const float d = fmaxf(0.f, e - prevE[b]);
        prevE[b]      = e;
        if(b == 0)
            lowOn = d;
        bandMean[b] += (d - bandMean[b]) * 0.01f;
        o += fmaxf(0.f, d - bandMean[b]);
    }
    silent = (tot / (float)kHop < 1e-6f) ? silent + 1 : 0;

    const uint32_t f = fn;
    hist[f & kHM]    = o;
    histL[f & kHM]   = lowOn;
    for(int L = kLmin - 1; L <= kLmax + 1; L++)
        acf[L] = acf[L] * accDecay + o * hist[(f - (uint32_t)L) & kHM];
    acf0 = acf0 * accDecay + o * o;

    // kick band onsets near a beat, per place in the bar
    if(phase < 0.08f || phase > 0.96f)
    {
        const uint32_t b = phase > 0.5f ? beat + 1 : beat;
        barAcc[b & 3] += lowOn;
    }
    for(int b = 0; b < 4; b++)
        barAcc[b] *= 0.9995f;

    fn = f + 1;
    if(fn % kEvery == 0 && !evalReq)
    {
        evalFn    = fn;
        evalPhase = phase;
        evalReq   = true;
    }
}

inline void OnBeat()
{
    beat++;
    if(audioLock && (beat & 3) == 0)
    {
        // beat 1 moves only if another beat has clearly stronger kicks two
        // bars in a row (kicks on 1 and 3 alike: it stays put)
        int best = 0;
        for(int b = 1; b < 4; b++)
            if(barAcc[b] > barAcc[best])
                best = b;
        if(best != 0 && barAcc[best] > kBarR * barAcc[0])
        {
            barN    = best == barBest ? barN + 1 : 1;
            barBest = best;
        }
        else
            barN = 0;
        if(barN >= 2)
        {
            float r[4];
            for(int b = 0; b < 4; b++)
                r[(b - best) & 3] = barAcc[b];
            memcpy(barAcc, r, sizeof(r));
            beat += (uint32_t)(4 - best);
            barN = 0;
        }
        else
            emit = true; // the engines' clock starts on a downbeat
    }
    flash     = 1.f;
    flashDown = (beat & 3) == 0;
}

// once per audio block, before the engines (raw input, before the send)
inline void Process(const float* L, const float* R, size_t n, bool gate)
{
    for(size_t i = 0; i < n; i++)
    {
        const float x = 0.5f * (L[i] + R[i]);
        f0 += c150 * (x - f0);
        f1 += c150 * (f0 - f1);
        f2 += c800 * (x - f2);
        f3 += c4k * (x - f3);
        const float b0 = f1, b1 = f2 - f1, b2 = f3 - f2, b3 = x - f3;
        acc[0] += b0 * b0;
        acc[1] += b1 * b1;
        acc[2] += b2 * b2;
        acc[3] += b3 * b3;
        if(++hopN >= kHop)
        {
            hopN = 0;
            Frame();
        }
    }

    // phase correction from the main loop's search
    if(snapReq)
    {
        const float P = periodF;
        float       p = snapPhase + (float)(fn - snapFn) / P;
        phase         = p - floorf(p);
        snapReq       = false;
    }

    const bool edge   = clk.Tick(gate, (uint32_t)n, sr);
    const bool gateOn = clk.Clocked(sr);
    if(gateOn && edge)
    {
        beat      = gateWas ? beat + 1 : 0; // clock appeared: beat 1
        phase     = 0.f;
        flash     = 1.f;
        flashDown = (beat & 3) == 0;
    }
    gateWas = gateOn;

    if(audioLock)
        runPer = periodF * (float)kHop;
    const float per = gateOn      ? (float)clk.period
                      : audioLock ? periodF * (float)kHop
                      : freeBpm > 0.f ? 60.f * sr / freeBpm
                                      : runPer;
    st.period       = per;
    st.inc          = (audioLock && !gateOn ? pllMul : 1.f) / per;
    st.phase        = phase;
    st.beat         = beat;
    st.source       = gateOn ? SRC_GATE : (audioLock ? SRC_AUDIO : SRC_FREE);
    if(gateOn)
        st.pulse = gate;
    else
        st.pulse = audioLock && emit && phase < 0.25f;

    phase += st.inc * (float)n;
    if(phase >= 1.f)
    {
        if(gateOn)
            phase = 0.9999f; // wait for the next edge
        else
        {
            phase -= 1.f;
            OnBeat();
        }
    }
}

// --------------------------------------------------------- main loop -------
inline float A(int L) { return acf[L] + 0.5f * (acf[L - 1] + acf[L + 1]); }
inline float Score(int L) { return A(L) * wTab[L]; }

inline void SetPhase(float est, uint32_t atFn)
{
    snapPhase = est;
    snapFn    = atFn;
    pllMul    = 1.f;
    snapReq   = true;
}

// call from the main loop, as often as you like
inline void Update()
{
    if(!evalReq)
        return;
    const uint32_t f  = evalFn;
    const float    ph = evalPhase;

    // ---- tempo
    float best = -1.f, sum = 0.f;
    int   bestL = kLmin;
    for(int L = kLmin; L <= kLmax; L++)
    {
        const float s = Score(L);
        sum += s;
        if(s > best)
            best = s, bestL = L;
    }
    const float mean = sum / (float)(kLmax - kLmin + 1);
    const float conf = (best - mean) / (2.f * acf0 + 1e-9f);
    float       cand = (float)bestL;
    if(bestL > kLmin && bestL < kLmax)
    {
        const float y0 = Score(bestL - 1), y2 = Score(bestL + 1);
        const float dn = y0 - 2.f * best + y2;
        if(dn < 0.f)
            cand += pn::Clamp(0.5f * (y0 - y2) / dn, -0.5f, 0.5f);
    }
    const bool locked = audioLock;
    float      P      = periodF;
    if(!locked)
        P = cand;
    else if(fabsf(cand - P) < 0.04f * P)
        P += 0.25f * (cand - P), candN = 0;
    else
    {
        // a different tempo has to win 6 times in a row
        if(fabsf(cand - candF) < 0.04f * cand)
            candN++;
        else
            candF = cand, candN = 1;
        if(candN >= 6)
            P = cand, candN = 0;
    }
    periodF = P;

    // ---- phase comb over the last kNJ beats (kick band counted twice)
    const int Pi = (int)(P + 0.5f);
    float     cMean = 0.f;
    for(int k = 0; k < Pi; k++)
    {
        float s = 0.f, w = 1.f;
        for(int j = 0; j < kNJ; j++)
        {
            const uint32_t back = (uint32_t)k + (uint32_t)((float)j * P + 0.5f);
            const uint32_t ix   = (f - 1u - back) & kHM;
            s += w * (hist[ix] + histL[ix]);
            w *= 0.85f;
        }
        comb[k] = s;
        cMean += s;
    }
    cMean /= (float)Pi;
    int bg = 0;
    for(int k = 1; k < Pi; k++)
        if(comb[k] > comb[bg])
            bg = k;
    int bk = bg;
    if(locked)
    {
        // follow the beat we have: only look near the predicted spot
        const float kp  = ph * P;
        const float win = 0.15f * P;
        float       bl  = -1.f;
        for(int k = 0; k < Pi; k++)
        {
            float d = fabsf((float)k - kp);
            if(d > 0.5f * P)
                d = P - d;
            if(d <= win && comb[k] > bl)
                bl = comb[k], bk = k;
        }
    }
    const float bs = comb[bk];
    // sharpness of the peak we follow (drops when the phase is wrong)
    const float cb = bs > 1e-9f ? (bs - cMean) / bs : 0.f;

    // ---- lock
    cS += (conf - cS) / 12.f;
    kS += (cb - kS) / 12.f;
    const bool sig = silent < (uint32_t)(fr * 3.f);
    if(sig && cS > kLockC && kS > kLockK)
        goodN++, badN = 0;
    else if(!sig || cS < kUnlC || kS < kUnlK)
        badN++, goodN = 0;
    if(!locked && goodN >= kGoodN && f > (uint32_t)(fr * 2.f))
        audioLock = true;
    if(locked && badN >= kBadN)
        audioLock = false, emit = false;

    evalReq = false;
    if(bs <= 1e-9f)
        return;

    // ---- phase
    if(locked)
    {
        // the beat moved (tempo jump): a much better spot elsewhere for a
        // while -> jump there
        jumpN = comb[bg] > kJumpR * bs ? jumpN + 1 : 0;
        if(jumpN >= 4)
        {
            jumpN = 0;
            badN  = 0;
            kS    = fmaxf(kS, kLockK);
            float est = ((float)bg + 0.5f) / P;
            SetPhase(est - floorf(est), f);
            return;
        }
    }
    float est = ((float)bk + 0.5f) / P; // beat fraction since that onset
    est -= floorf(est);
    float e = ph - est;
    if(e >= 0.5f)
        e -= 1.f;
    if(e < -0.5f)
        e += 1.f;
    if(!audioLock)
    {
        // not sending a clock yet: snap, but only while a beat is building
        // up (random input must not shake the free-running beat)
        if(goodN > 0)
            SetPhase(est, f);
    }
    else
        pllMul = pn::Clamp(1.f - kPll * e * P / (float)kEvery, 0.8f, 1.2f);
}

} // namespace bt
