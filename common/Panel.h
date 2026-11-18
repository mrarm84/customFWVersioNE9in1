// Panel.h - what the front panel hands to the simple engines, plus small
// shared DSP helpers. Knobs are 0..1 (knob + CV), switches 0/1/2 (left/mid/
// right), a/b are the button toggles (tap / double-tap), action counts
// "hold 0.6..4 s + release" gestures.
#pragma once
#include <cstdint>
#include <cmath>

// k = layer 1 knobs, s = layer 2 knobs (layer 3 is the global output strip)
// row = Switch 0 position, col = Switch 1 position -> submode row * 3 + col
struct Panel
{
    float    k[7]   = {0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f};
    float    s[7]   = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    int      row    = 0;
    int      col    = 0;
    bool     gate   = false;
    bool     a      = false; // double-tap toggle
    uint32_t action = 0;     // hold 0.6..4 s + release, counter
};

struct LedOut
{
    float c[4][3];
    void  Set(int i, float r, float g, float b)
    {
        c[i][0] = r;
        c[i][1] = g;
        c[i][2] = b;
    }
};

namespace pn
{
inline float Clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

inline float OnePole(float hz, float sr)
{
    const float c = 1.f - expf(-6.2831853f * hz / sr);
    return c > 1.f ? 1.f : c;
}

// transparent below 0.9, soft ceiling at 1
inline float Lim(float x)
{
    const float ax = fabsf(x);
    if(ax < 0.9f)
        return x;
    const float y = 0.9f + 0.1f * tanhf((ax - 0.9f) * 10.f);
    return x < 0.f ? -y : y;
}

// transparent below |1|, soft limit towards |2| (for feedback loops)
inline float LoopClip(float x)
{
    if(x > 1.f)
        return 2.f - 1.f / x;
    if(x < -1.f)
        return -2.f - 1.f / x;
    return x;
}

// read d samples behind write position wp (d >= 1)
inline float ReadFrac(const float* buf, uint32_t mask, uint32_t wp, float d)
{
    const uint32_t di = (uint32_t)d;
    const float    f  = d - (float)di;
    const float    a  = buf[(wp - di) & mask];
    const float    b  = buf[(wp - di - 1) & mask];
    return a + (b - a) * f;
}

// read at base + pos (forward interpolation)
inline float ReadAt(const float* buf, uint32_t mask, uint32_t base, float pos)
{
    const uint32_t ip = (uint32_t)pos;
    const float    f  = pos - (float)ip;
    const float    a  = buf[(base + ip) & mask];
    const float    b  = buf[(base + ip + 1) & mask];
    return a + (b - a) * f;
}

struct Rng
{
    uint32_t s = 0x6C8E9CF5u;
    uint32_t Next()
    {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    }
    float F() { return (Next() >> 8) * (1.f / 16777216.f); } // 0..1
    float B() { return F() * 2.f - 1.f; }                     // -1..1
};

// gate clock: period measurement, "clocked" detection
struct Clock
{
    uint32_t n = 0, last = 0, period = 0;
    bool     have = false, lastGate = false;
    bool     Tick(bool gate, uint32_t samples, float sr) // true on rising edge
    {
        bool edge = gate && !lastGate;
        lastGate  = gate;
        if(edge)
        {
            const uint32_t per = n - last;
            if(have && per > (uint32_t)(0.01f * sr) && per < (uint32_t)(10.f * sr))
                period = per;
            have = true;
            last = n;
        }
        n += samples;
        return edge;
    }
    bool Clocked(float sr) const
    {
        return have && period > 0 && (n - last) < (uint32_t)(period * 2.5f)
               && (n - last) < (uint32_t)(10.f * sr);
    }
};

inline float EqPowDry(float m) { return cosf(m * 1.5707963f); }
inline float EqPowWet(float m) { return sinf(m * 1.5707963f); }

inline float Tri(float x) { return 1.f - fabsf(2.f * x - 1.f); }

// reverse playback of a delay line in chunks of T samples: two heads half a
// chunk apart, triangle windows (sum = 1). Read delays go up to 2T + 1.
struct RevReader
{
    float p = 0.f;
    void  Step(float T, float& d1, float& g1, float& d2, float& g2)
    {
        p += 1.f;
        if(p >= T)
            p -= T;
        float p2 = p + T * 0.5f;
        if(p2 >= T)
            p2 -= T;
        d1 = 2.f * p + 1.f;
        d2 = 2.f * p2 + 1.f;
        g1 = Tri(p / T);
        g2 = Tri(p2 / T);
    }
};

// one-pole envelope follower with attack/release coefficients
struct Env
{
    float v = 0.f;
    float Process(float x, float att, float rel)
    {
        const float ax = fabsf(x);
        v += (ax - v) * (ax > v ? att : rel);
        return v;
    }
};

// quantized zone from a 0..1 knob with hysteresis
inline int Zone(float k, int n, int& cur)
{
    const float pos = k * (float)n;
    if(fabsf(pos - ((float)cur + 0.5f)) > 0.6f)
    {
        int z = (int)pos;
        cur   = z >= n ? n - 1 : (z < 0 ? 0 : z);
    }
    return cur;
}

} // namespace pn
