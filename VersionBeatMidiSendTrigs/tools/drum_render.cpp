// drum_render.cpp - renders WAV files from the REAL drum engine
// (common/TrigDrumsEngine.h + BeatTracker.h), compiled for the PC with the
// stand-in headers in tools/host. Same code as the firmware, driven the
// same way: 8-sample blocks, the switches, double-taps, triggers, DUB.
//
//   build:  g++ -O2 -std=gnu++17 -I host -I ../../common drum_render.cpp -o drum_render.exe
//   run:    drum_render.exe ../renders
#include "TrigDrumsEngine.h"
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

static const float SR = 48000.f;
static const int   B  = 8; // firmware block size

struct Hit
{
    int   at, v;
    float vel;
};

struct Job
{
    int  kit      = 1;     // 0..7 (switch position kit + 1), PUNCH default
    bool pattern  = false; // SEQ (left / left), plays kit `kit`
    int  patIndex = 0;     // 0..6
    int  audio    = 0;     // AUDIO IN: 0 THRU, 1 SKIN, 2 CHOP
    std::vector<Hit> hits;
    std::function<bool(int)>       dub;   // sample -> button held
    std::function<void(int)>       tweak; // sample -> change tdr::prm
};

static void Reset()
{
    tdr::Init(SR);
    tdr::lastStep = -1;
    tdr::pat      = 0;
    tdr::kitBase  = 0;
    tdr::kitPos   = -1;
    tdr::kitHeld  = 0;
    tdr::audMode  = 0;
    tdr::dub      = 0.f;
    tdr::dubReq   = false;
    for(int v = 0; v < tdr::kV; v++)
        tdr::hitPan[v] = 0.f;
    bt::Init(SR);
    bt::phase = 0.f;
    bt::beat  = 0;
}

static std::vector<float> Run(const Job& j, int n)
{
    Reset();
    Panel p;
    p.a = false;
    auto block = [&](float* l, float* r, const float* il, const float* ir) {
        const float* in[2]  = {il, ir};
        float*       out[2] = {l, r};
        bt::Process(il, ir, B, false);
        tdr::Process(p, in, out, B);
    };
    static float zero[B] = {0}, scratch[2][B];
    // switches: the kit's position, or SEQ (left / left) with that kit as
    // the one used last; engine state set directly
    const int pos = j.pattern ? 0 : j.kit + 1;
    p.row         = pos / 3;
    p.col         = pos % 3;
    tdr::Activate(p);
    block(scratch[0], scratch[1], zero, zero);
    Reset(); // clean voices / beat
    tdr::kitBase = j.kit;
    tdr::kitPos  = pos;
    tdr::kitHeld = (uint32_t)SR;
    tdr::audMode = j.audio;
    tdr::lastA   = p.a;
    tdr::pat     = j.pattern ? j.patIndex : 0;

    std::vector<float> out(2 * (size_t)n);
    size_t             hi = 0;
    std::vector<Hit>   hits = j.hits;
    for(int i = 0; i < n; i += B)
    {
        if(j.tweak)
            j.tweak(i);
        tdr::dubReq = j.dub ? j.dub(i) : false;
        while(hi < hits.size() && hits[hi].at <= i)
        {
            tdr::req[hits[hi].v]    = hits[hi].vel;
            tdr::reqExt[hits[hi].v] = false;
            hi++;
        }
        float l[B], r[B];
        block(l, r, zero, zero);
        for(int k = 0; k < B && i + k < n; k++)
        {
            out[2 * (i + k)]     = l[k];
            out[2 * (i + k) + 1] = r[k];
        }
    }
    return out;
}

static void Wav(const std::string& path, const std::vector<float>& x)
{
    FILE* f = fopen(path.c_str(), "wb");
    if(!f)
    {
        printf("cannot write %s\n", path.c_str());
        return;
    }
    const uint32_t frames = (uint32_t)(x.size() / 2), data = frames * 4;
    auto w32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
    auto w16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
    fwrite("RIFF", 1, 4, f), w32(36 + data), fwrite("WAVEfmt ", 1, 8, f);
    w32(16), w16(1), w16(2), w32(48000), w32(48000 * 4), w16(4), w16(16);
    fwrite("data", 1, 4, f), w32(data);
    for(float v : x)
    {
        v       = v > 1.f ? 1.f : (v < -1.f ? -1.f : v);
        int16_t s = (int16_t)(v * 32767.f);
        fwrite(&s, 2, 1, f);
    }
    fclose(f);
}

static std::string out_dir = "renders";
static void Save(const char* name, const std::vector<float>& x)
{
    float pk = 0.f;
    size_t over = 0;
    for(float v : x)
    {
        pk = fmaxf(pk, fabsf(v));
        over += fabsf(v) > 0.9f;
    }
    Wav(out_dir + "/" + name + ".wav", x);
    printf("%-26s peak %.2f  over 0.9: %.3f %%\n", name, pk, 100.0 * over / x.size());
}

int main(int argc, char** argv)
{
    if(argc > 1)
        out_dir = argv[1];
    const int bar = (int)(SR * 2.f); // 120 BPM

    // every voice alone, soft then accented, on each kit
    const char* kits[tdr::kKits] = {"1_analog", "2_punch",  "3_minimal",
                                    "4_rhythmbox", "5_digital", "6_crunch",
                                    "7_deep", "8_industrial"};
    for(int k = 0; k < tdr::kKits; k++)
    {
        Job j;
        j.kit = k;
        int t = 0;
        for(int v = 0; v < 8; v++)
        {
            j.hits.push_back({t, v, 0.5f});
            j.hits.push_back({t + (int)(0.6f * SR), v, 1.f});
            t += (int)(1.6f * SR);
        }
        Save((std::string("voices_") + kits[k]).c_str(), Run(j, t));
    }
    // one groove (TECHNO pattern) on every kit, for comparing them
    for(int k = 0; k < tdr::kKits; k++)
    {
        Job j;
        j.kit      = k;
        j.pattern  = true;
        j.patIndex = 1;
        Save((std::string("groove_kit") + kits[k]).c_str(), Run(j, 2 * bar));
    }
    // the patterns (PUNCH kit)
    for(int pt = 0; pt < tdr::kPats; pt++)
    {
        Job j;
        j.pattern  = true;
        j.patIndex = pt;
        char nm[32];
        snprintf(nm, sizeof nm, "pattern%d", pt + 1);
        Save(nm, Run(j, 4 * bar));
    }
    // DUB TECHNO with a dub throw (button held through bar 5 of 8)
    {
        Job j;
        j.pattern  = true;
        j.patIndex = 6;
        j.dub      = [=](int i) { return i >= 4 * bar && i < 5 * bar; };
        Save("pattern7_dub_throw", Run(j, 8 * bar));
    }
    // every chord, stab and resonator
    for(int v : {tdr::SA, tdr::SB})
    {
        Job j;
        const int seg = (int)(1.5f * SR);
        for(int z = 0; z < 8; z++)
            j.hits.push_back({z * seg, v, 0.9f});
        j.tweak = [=](int i) { tdr::prm[v][tdr::P_A] = ((float)(i / seg) + 0.5f) / 8.f; };
        Save(v == tdr::SA ? "chords_stab" : "chords_reso", Run(j, 8 * seg));
    }
    // FILTER sweep (CCW low-pass ... noon ... CW high-pass) on a stab loop
    {
        Job j;
        const int n = 16 * (int)SR;
        for(int t = 0; t < n; t += (int)(SR * 0.5f))
            j.hits.push_back({t, tdr::SA, 0.9f});
        j.tweak = [=](int i) { tdr::prm[tdr::SA][tdr::P_FILT] = (float)i / n; };
        Save("fx_filter_sweep_stab", Run(j, n));
    }
    // DRIVE sweep (clean -> saturation -> distortion) on kick + snare
    {
        Job j;
        const int n = 16 * (int)SR;
        for(int t = 0; t < n; t += (int)(SR * 0.5f))
        {
            j.hits.push_back({t, tdr::BD, 0.9f});
            if((t / (int)(SR * 0.5f)) % 2)
                j.hits.push_back({t, tdr::SD, 0.9f});
        }
        j.tweak = [=](int i) {
            tdr::prm[tdr::BD][tdr::P_DRIVE] = (float)i / n;
            tdr::prm[tdr::SD][tdr::P_DRIVE] = (float)i / n;
        };
        Save("fx_drive_sweep_bd_sd", Run(j, n));
    }
    // SEND sweep (dry -> reverb -> reverb + echo) on clap
    {
        Job j;
        const int n = 16 * (int)SR;
        for(int t = 0; t < n; t += (int)SR)
            j.hits.push_back({t, tdr::CP, 0.9f});
        j.tweak = [=](int i) { tdr::prm[tdr::CP][tdr::P_SEND] = (float)i / n; };
        Save("fx_send_sweep_clap", Run(j, n));
    }
    return 0;
}
