// PC stand-in for libDaisy's daisy_seed.h: only what the drum engine and
// the beat tracker use, so the REAL engine code compiles on a computer.
#pragma once
#include <cstddef>
#include <cstdint>

#define DSY_SDRAM_BSS // plain memory on a PC

namespace daisy
{
struct AudioHandle
{
    typedef const float* const* InputBuffer;
    typedef float**             OutputBuffer;
};
} // namespace daisy
