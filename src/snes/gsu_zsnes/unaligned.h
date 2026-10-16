/* Wide reads and writes at an arbitrary byte offset. The assembly did these
   with a plain mov, so the ported C inherited casts that are undefined when
   the address is not aligned and fault outright on a strict-alignment target.
   memcpy says the same thing legally. The host compiler folds it back into the
   single instruction the assembly used; the device build (-mno-unaligned-
   access) cannot and calls memcpy, so a hot path whose address is known to be
   even uses the ld16a / st16a forms below, which compile to ldrh / strh. */
#ifndef UNALIGNED_H
#define UNALIGNED_H

#include <stdint.h>
#include <string.h>

static inline uint32_t ld32u(void const* const p)
{
    uint32_t v;

    memcpy(&v, p, sizeof v);
    return v;
}

static inline void st32u(void* const p, uint32_t const v)
{
    memcpy(p, &v, sizeof v);
}

static inline uint16_t ld16u(void const* const p)
{
    uint16_t v;

    memcpy(&v, p, sizeof v);
    return v;
}

static inline void st16u(void* const p, uint16_t const v)
{
    memcpy(p, &v, sizeof v);
}

/* The same for an address known to be 2-byte aligned. */
static inline uint16_t ld16a(void const* const p)
{
    uint16_t v;

    memcpy(&v, __builtin_assume_aligned(p, 2), sizeof v);
    return v;
}

static inline void st16a(void* const p, uint16_t const v)
{
    memcpy(__builtin_assume_aligned(p, 2), &v, sizeof v);
}

#endif
