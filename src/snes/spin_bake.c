/* See spin_bake.h. This half runs once, at ROM load: find the wait loop in the
 * cartridge image and install it. Nothing here is on any hot path. */
#include "spin_bake.h"
#include "cart.h"
#include "dma.h"

#ifndef SNES_BAKE_NO_INSTALL
#define SNES_BAKE_NO_INSTALL 0
#endif

SpinBake g_bake;



/* The interpreter's charge for each opcode, in the units run_dots consumes:
 * `cycles * 6 + memOps * 2`, as run_one_opcode() computes it from what the
 * ENGINE returns. Both numbers were read out of the Thumb-2 rig — the engine
 * the device runs — and not out of the C-interpreter rig, whose own formula is
 * `(cycles - memOps) * 6` and would have baked 0 and 6 here. Two numbers that
 * look like cycle counts and are not the device's are exactly the kind of thing
 * that ships as a silent timing corruption, so: rig_snes.c prints SPINPAT, and
 * the gate is a bit-identical STATEHASH against the same run with no bake.
 *
 * They are valid only for an 8-bit accumulator and a page-aligned direct page;
 * spin_bake_step() refuses anything else rather than guessing. */
#ifndef SNES_BAKE_CHARGE_LOAD
#define SNES_BAKE_CHARGE_LOAD    24
#endif
#ifndef SNES_BAKE_CHARGE_BRANCH
#define SNES_BAKE_CHARGE_BRANCH  22
#endif
#define BAKE_MF            1
#define BAKE_DPL           0
#define BAKE_CHARGE_LOAD   SNES_BAKE_CHARGE_LOAD
#define BAKE_CHARGE_BRANCH SNES_BAKE_CHARGE_BRANCH

/* LDA dp / BEQ back-to-itself. Three of the four bytes are fixed, so the
 * expected number of accidental matches in a 4 MB cartridge is about a
 * quarter of one — and an accidental match costs nothing anyway: it installs a
 * pc that is never executed, and spin_bake_step() is a compare that stays
 * false. `laps` is what says whether the installed one was the real one. */
static bool sig_at(const uint8_t *rom, uint32_t off) {
  return rom[off] == 0xa5 && rom[off + 2] == 0xf0 && rom[off + 3] == 0xfc;
}

/* Where the CPU executes a given ROM offset. Only the canonical window is
 * mapped; the $80 mirror is handled by bank_alt at match time. */
static bool map_offset(uint8_t type, uint32_t off, uint8_t *bank, uint16_t *addr) {
  if (type == 1) {                       /* LoROM: 32 K per bank at $8000 */
    uint32_t b = off >> 15;
    if (b > 0x3f) return false;
    *bank = (uint8_t)b;
    *addr = (uint16_t)(0x8000u | (off & 0x7fffu));
    return true;
  }
  if (type == 2) {                       /* HiROM: 64 K per bank from $c0 */
    uint32_t b = 0xc0u + (off >> 16);
    if (b > 0xff) return false;
    *bank = (uint8_t)b;
    *addr = (uint16_t)(off & 0xffffu);
    return true;
  }
  return false;
}

/* $7e/$7f (and their $fe/$ff partners) are WRAM, not a ROM mirror: accepting
 * bank ^ 0x80 there would let a pc in RAM match a loop found in ROM. */
static uint8_t mirror_bank(uint8_t bank) {
  uint8_t alt = (uint8_t)(bank ^ 0x80);
  if ((bank & 0x7f) >= 0x7e) return bank;
  return alt;
}

/* The consume arithmetic below is a copy of run_dots' -- deliberately, and it
 * is what the bit-identical STATEHASH/AUDIOHASH gate exists to police. A span
 * replayed here must step hPos and apuDotsAccum in exactly the chunks the
 * interpreter would have, or the APU catch-up sequence diverges and the hash
 * says so on the first frame. */
int spin_bake_run_span(Snes *s, Cpu *cpu, int dots) {
  /* This is run_dots' loop with one substitution: where it would dispatch the
   * interpreter, the two opcodes of the wait loop are executed here instead.
   * Everything else -- the DMA bail, the chunked consume, the order of the
   * hPos/apuDotsAccum updates -- is copied exactly, because the gate on this
   * whole feature is a bit-identical STATEHASH and AUDIOHASH.
   *
   * The first version returned as soon as cpuCyclesLeft was non-zero, which is
   * the state a span USUALLY starts in: spans end mid-opcode. It replayed 33
   * laps a frame where the per-opcode version replayed 3,643, and the gate
   * correctly parked it. Entering mid-opcode is the common case, not an edge. */
  for (;;) {
    if (dots <= 0) return dots;
    if (s->dma->dmaBusy || s->dma->hdmaTimer > 0) {
      /* Run the burst here rather than handing the span back. Returning meant
       * run_dots had to re-test the pc when the burst ended, and that test --
       * on every DMA cycle, reaching into g_bake -- cost 1.1% of Super
       * Metroid, a cartridge with no match in it at all. Handling DMA here
       * deletes the test instead of making it cheaper, and A Link to the Past's
       * rain (HDMA every scanline) keeps its replay across the burst. Copied
       * from run_dots line for line; the hash gate is what proves it. */
      dma_cycle(s->dma);
      s->apuDotsAccum += 2;
      s->hPos += 2; dots -= 2;
      continue;
    }

    if (s->cpuCyclesLeft == 0) {
      if (cpu->nmiWanted || cpu->irqWanted || cpu->waiting || cpu->stopped) return dots;
      if (s->hIrqEnabled) return dots;
      if (s->vIrqEnabled && s->vPos == s->vTimer) return dots;
      if (cpu->k != g_bake.bank && cpu->k != g_bake.bank_alt) return dots;
      if (cpu->mf != g_bake.mf || (cpu->dp & 0xff) != g_bake.dpl) return dots;

      if (cpu->pc == g_bake.pc_load) {
        /* LDA dp: direct page is bank 0, so this is WRAM only below $2000. */
        const uint16_t ea = (uint16_t)(cpu->dp + g_bake.dp_off);
        const uint16_t hi = g_bake.mf ? ea : (uint16_t)(ea + 1);
        if (ea >= 0x2000 || hi >= 0x2000 || hi < ea) return dots;
        const uint16_t val = g_bake.mf ? s->ram[ea]
                                       : (uint16_t)(s->ram[ea] | (s->ram[hi] << 8));
        if (g_bake.mf) cpu->a = (uint16_t)((cpu->a & 0xff00) | val);
        else           cpu->a = val;
        cpu->z = (val == 0);
        cpu->n = g_bake.mf ? ((val & 0x80) != 0) : ((val & 0x8000) != 0);
        if (val != 0) return dots;   /* loop is leaving -- let the interpreter leave it */
        /* Whole laps in one step. Inside a span nothing can end the loop: the
         * polled byte is WRAM that only the CPU, an NMI/IRQ or DMA could
         * change, and every one of those was ruled out above or bails at an
         * opcode boundary that this span does not reach (vPos is constant
         * inside a span, events run outside it). So every remaining lap is
         * this one again: the same a/z/n, the same two charges, consumed in
         * full because each is an even number >= 2. Advance by k laps, keep
         * the last one (and any partial lap) for the exact path below.
         * apuDotsAccum is a plain sum until snes_catchupApu, so one add of
         * k*L equals k adds of L. */
        {
          const int L = g_bake.charge_load + g_bake.charge_branch;
          if (L > 0 && !((g_bake.charge_load | g_bake.charge_branch) & 1) &&
              g_bake.charge_load >= 2 && g_bake.charge_branch >= 2 && dots >= 2 * L) {
            const int k = dots / L - 1;
            s->hPos = (uint16_t)(s->hPos + k * L);
            s->apuDotsAccum += (uint32_t)(k * L);
            dots -= k * L;
            g_bake.laps += (uint32_t)k;
          }
        }
        cpu->pc = g_bake.pc_branch;
        s->cpuCyclesLeft += g_bake.charge_load;
      } else if (cpu->pc == g_bake.pc_branch) {
        if (!cpu->z) return dots;    /* BEQ not taken: the loop has ended */
        cpu->pc = g_bake.pc_load;
        s->cpuCyclesLeft += g_bake.charge_branch;
        g_bake.laps++;
      } else {
        return dots;                 /* not in the loop */
      }
    }

    int step;
    if (s->cpuCyclesLeft >= 2) {
      step = s->cpuCyclesLeft;
      if (step > dots) step = dots;
      s->cpuCyclesLeft -= (uint8_t)step;
    } else {
      step = 2;
      s->cpuCyclesLeft -= 2;
    }
    s->apuDotsAccum += step;
    s->hPos += step; dots -= step;
  }
}

void spin_bake_reset(void) {
  g_bake = (SpinBake){0};
  g_bake.pc_load = 0xffff;
  g_bake.pc_branch = 0xffff;
}

/* Observation window and the rate that has to clear it.
 *
 * The threshold is arithmetic, not taste. The guard costs a fixed ~2.4% of a
 * frame's instructions and a replayed lap returns ~142 of them, so break-even
 * on the reference scene (4.96 M insn/frame) is 4.96e6 * 0.024 / 142 = 838
 * laps a frame. Measured either side of it, on hardware: A Link to the Past's
 * play scene replays 3,976 a frame and gains 15.8% DRAWN frames; Super Mario
 * Kart replays 0 and pays the guard for nothing.
 *
 * The window is short and the park is long because the two errors are not
 * symmetric: staying armed on a cart that does not spin costs a few percent
 * for a second, while parking a cart that does spin gives up a sixth of the
 * frames it draws. Retry rather than decide once -- a cart's spin rate is a
 * property of the SCENE, not the cartridge. The rig's cold-boot window put
 * Zelda at 730 laps/frame and its play scene is 3,976, so a single verdict
 * taken at a title screen would have been wrong by 5.4x. */
#define BAKE_WIN_FRAMES   180u    /* ~3 s */
#define BAKE_PARK_FRAMES  900u    /* ~15 s, first backoff step */
#define BAKE_PARK_MAX     57600u  /* ~16 min; the retry never stops, it thins */
#define BAKE_MIN_LAPS     838u    /* per frame; see above */

/* Disarming moves the pc out of reach instead of switching code paths.
 *
 * The obvious gate -- two specialisations of run_dots/run_frame_events chosen
 * by a flag, so the disarmed one folds the guard away -- was BUILT AND
 * MEASURED, and it is much worse than the thing it was protecting against.
 * Merely instantiating both clones changes what gcc does with the frame loop:
 * on hardware, a build that installs NOTHING drew 14.16 fps against the
 * baseline's 16.44, a **13.9% loss for code that never runs**. (The rig said
 * +2.2% instructions for the same build; the device said worse.) One code path
 * is not a style preference here, it is the measurement.
 *
 * So there is exactly one run_dots, always carrying the guard, and disarming
 * is done by setting pc_load to a value the compare can never see: the guard
 * stays, its cost stays (~2.4% of a frame), and the replay stops. That is the
 * honest trade -- a non-spinning cart pays the compare, not a different
 * program. */
void spin_bake_frame_tick(void) {
  SpinBake *b = &g_bake;
  if (!b->on) return;

  if (!b->armed) {
    /* Parked: nothing is replayed and nothing is counted, so the only way to
     * re-measure is to put the pc back and look again. A cart's spin rate is a
     * property of the SCENE -- the rig's cold-boot window put Zelda at 730
     * laps/frame and its play scene is 3,976 -- so this must retry, not decide
     * once. */
    if (b->park_countdown && --b->park_countdown == 0) {
      b->armed = true;
      b->pc_load = b->pc_load_real;
      b->win_frames = 0;
      b->win_laps = b->laps;
    }
    return;
  }

  if (++b->win_frames < BAKE_WIN_FRAMES) return;
  uint32_t replayed = b->laps - b->win_laps;
  b->win_frames = 0;
  b->win_laps = b->laps;
  if (replayed < BAKE_WIN_FRAMES * BAKE_MIN_LAPS) {
    b->armed = false;
    b->pc_load = 0xffff;          /* the loop's pc is 2 bytes below its branch;
                                     0xffff can be neither, by construction */
    /* Back off, doubling. A cart that does not spin should stop being asked:
     * Mario Kart still lost 2.2% of its drawn frames at a flat 900-frame park,
     * which is 17% of the time armed and paying the entry test for a loop it
     * never runs. Doubling puts it under 1% within a few windows while still
     * retrying for ever -- the retry is the point, because spin rate belongs to
     * the scene and a title screen is not a level. */
    if (b->park_frames < BAKE_PARK_MAX)
      b->park_frames = b->park_frames ? (b->park_frames << 1) : BAKE_PARK_FRAMES;
    else
      b->park_frames = BAKE_PARK_MAX;
    b->park_countdown = b->park_frames;
  } else {
    b->park_frames = 0;           /* it is earning: forget the backoff */
  }
}

bool spin_bake_scan(Snes *snes) {
  spin_bake_reset();
#if SNES_BAKE_NO_INSTALL
  /* DIAGNOSTIC: compile the mechanism in, install nothing. Prices the
   * per-opcode guard on its own, so a ROM's number can be split into what the
   * test costs and what the skip returns. */
  (void)snes;
  return false;
#endif

  Cart *cart = snes->cart;
  if (!cart || !cart->rom || cart->romSize < 4) return false;
  const uint8_t *rom = cart->rom;

  for (uint32_t off = 0; off + 4 <= cart->romSize; off++) {
    if (!sig_at(rom, off)) continue;

    uint8_t bank;
    uint16_t addr;
    if (!map_offset(cart->type, off, &bank, &addr)) continue;
    if (addr > 0xfffb) continue;         /* the BEQ's operand must not wrap */

    /* Verify the mapping through the cart's own reader, not through the
     * offset arithmetic that produced it.
     *
     * map_offset() knows LoROM and HiROM. The library has carts this project
     * loads whose mapping it does not know, and on one of those a wrong pc is
     * not a missed optimisation -- it is a pc where OTHER code lives, and this
     * would execute LDA dp / BEQ in place of whatever is really there. Asking
     * cart_read() closes that by construction: if the CPU fetching this address
     * does not see the four bytes, nothing is installed. */
    if (cart_read(cart, bank, addr)              != 0xa5 ||
        cart_read(cart, bank, (uint16_t)(addr+1)) != rom[off + 1] ||
        cart_read(cart, bank, (uint16_t)(addr+2)) != 0xf0 ||
        cart_read(cart, bank, (uint16_t)(addr+3)) != 0xfc)
      continue;

    g_bake.sites++;
    if (g_bake.on) continue;             /* first match wins; count the rest */

    g_bake.pc_load       = addr;
    g_bake.pc_branch     = (uint16_t)(addr + 2);
    g_bake.bank          = bank;
    g_bake.bank_alt      = mirror_bank(bank);
    g_bake.dp_off        = rom[off + 1];
    g_bake.mf            = BAKE_MF;
    g_bake.dpl           = BAKE_DPL;
    g_bake.charge_load   = BAKE_CHARGE_LOAD;
    g_bake.charge_branch = BAKE_CHARGE_BRANCH;
    g_bake.on            = true;
    g_bake.pc_load_real  = addr;
    g_bake.armed         = true;   /* the first window decides whether it stays */
  }
  return g_bake.on;
}
