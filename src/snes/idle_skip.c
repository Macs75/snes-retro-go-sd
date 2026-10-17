/* See idle_skip.h. This half runs once a frame, at the NMI request: decode the
 * loop around the interrupted pc and arm it if it is provably read-only. The
 * span runner that replays it lives next to run_dots in main_snes.c, because
 * it has to copy run_dots' arithmetic exactly. */
#include "idle_skip.h"
#include "cart.h"

#ifdef TARGET_GNW
/* allocated by idle_alloc() before anything reads them */
IdleSkip *g_idle_sites;
IdleSkip *g_idlep;
#else
static IdleSkip s_sites_static[IDLE_SITES];
IdleSkip *g_idle_sites = s_sites_static;
IdleSkip *g_idlep = &s_sites_static[0];
#endif
IdleStats g_idle_stats;
uint8_t g_idle_budget, g_idle_gsu_budget;
static uint32_t s_frame;

#ifdef TARGET_GNW
#include <assert.h>
#include <stdio.h>
#include "gw_malloc.h"
#endif
void idle_alloc(void) {
#ifdef TARGET_GNW
  /* DTCM: the span runner reads the active site on every span. AHB, then the
   * RAM_EMU pool, only if DTCM is full. */
  const size_t n = sizeof(IdleSkip) * IDLE_SITES;
  void *p = dtc_malloc(n);
  if (!p || p == (void *)-1) { printf("idle: sites in AHB (DTCM full)\n"); p = ahb_calloc(1, n); }
  if (!p) p = ram_calloc(1, n);
  assert(p);
  g_idle_sites = (IdleSkip *)p; g_idlep = &g_idle_sites[0];
#endif
}

#include <stddef.h>
_Static_assert(offsetof(Cpu, stopped) + 1 - offsetof(Cpu, a) == IDLE_IMG_LEN,
               "Cpu's a..stopped are no longer contiguous: IdleState's image is wrong");
/* the span runner compares the image as 7 halfwords (a..db), 3 words and a
 * byte (the 13 flag bytes from c): both halves must stay aligned that way */
_Static_assert(offsetof(Cpu, a) % 2 == 0 && offsetof(Cpu, c) == offsetof(Cpu, a) + 14 &&
               offsetof(Cpu, c) % 4 == 0, "Cpu layout: idle_img_eq's aligned compare is wrong");
_Static_assert(offsetof(IdleState, img) == 2, "IdleState: img+14 must be word-aligned");

/* rejected pcs, so a busy frame's landing spot is not decoded every frame */
static uint32_t s_reject[4];
static uint8_t  s_reject_i;

static void site_clear(IdleSkip *g) {
  *g = (IdleSkip){0};
  g->head = 0xffff;
  g->len = 0;
}

void idle_reset(void) {
  for (int i = 0; i < IDLE_SITES; i++) site_clear(&g_idle_sites[i]);
  g_idlep = &g_idle_sites[0];
  g_idle_stats = (IdleStats){0};
  g_idle_budget = g_idle_gsu_budget = 0;
  s_frame = 0;
  for (int i = 0; i < 4; i++) s_reject[i] = 0xffffffffu;
  s_reject_i = 0;
}

/* $7e/$7f are WRAM, not a ROM mirror */
static uint8_t mirror_bank(uint8_t bank) {
  if ((bank & 0x7f) >= 0x7e) return bank;
  return (uint8_t)(bank ^ 0x80);
}

enum { MEM_NO = 0, MEM_WRAM, MEM_ROM, MEM_RDNMI, MEM_GSUSFR };

/* What a read at bank:off reaches. WRAM gives its offset into Snes.ram. ROM is
 * accepted only for the two plain mappings, where it is constant. $4210 is
 * RDNMI: in this core it reads 0x02 | NMI-flag << 7 (the open bus is never
 * updated after reset) and clearing the flag is its only side effect, so while
 * the flag is clear it is a constant read -- and the flag is only set at the
 * vblank event, a span boundary. Anything else -- other I/O registers, SRAM, a
 * coprocessor window -- is refused. */
static int classify(const Snes *s, uint8_t bank, uint16_t off, uint32_t *ram_off) {
  uint8_t b = bank & 0x7f;
  if (bank == 0x7e || bank == 0x7f) { *ram_off = ((uint32_t)(bank & 1) << 16) | off; return MEM_WRAM; }
  if (b < 0x40 && off < 0x2000)      { *ram_off = off; return MEM_WRAM; }
  if (b < 0x40 && off == 0x4210)     return MEM_RDNMI;
  if (s->cart && s->cart->gsu) {            /* Super FX: its own map (cart_gsu_read) */
    if (b < 0x40 && (off == 0x3030 || off == 0x3031)) return MEM_GSUSFR;
    if (b < 0x40 && off >= 0x8000) return MEM_ROM;
    if (b >= 0x40 && b < 0x60) return MEM_ROM;
    return MEM_NO;                          /* Game Pak RAM, other GSU registers */
  }
  uint8_t type = s->cart ? s->cart->type : 0;
  if (type == 1) {                          /* LoROM */
    if (b < 0x7e && off >= 0x8000) return MEM_ROM;
  } else if (type == 2) {                   /* HiROM */
    if (b >= 0x40 && b < 0x7e) return MEM_ROM;
    if (b < 0x40 && off >= 0x8000) return MEM_ROM;
  }
  return MEM_NO;
}

/* The whitelist. Every entry writes nothing to memory and leaves the stack,
 * the program bank, DB, DP and the E/I flags alone. SEP/REP change M/X: the
 * decoder follows them, and the lap must come back with the widths it started
 * with. Indexed reads are allowed when the loop never changes that index
 * register, so the proof fixes their address. */
enum { K_NO = 0, K_IMP, K_IMM, K_IMM8, K_DP, K_ABS, K_LONG, K_BRC, K_BRA, K_JMP };
enum { W_8 = 0, W_M, W_X };
enum { IX_NONE = 0, IX_X, IX_Y };
typedef struct { uint8_t kind, width, ix, modx, mody; } OpInfo;

static OpInfo op_info(uint8_t op) {
  switch (op) {
    /* WAI: with a masked IRQ pending it returns at once, and otherwise the
     * span runner stops at it (a real wait is the scheduler's WAI skip) */
    case 0xcb:
    case 0xea: case 0x18: case 0x38: case 0xb8:
    case 0x8a: case 0x98: case 0x7b: case 0x3b: case 0xeb:
    case 0x1a: case 0x3a: case 0x0a: case 0x4a: case 0x2a: case 0x6a:
      return (OpInfo){K_IMP, W_8, IX_NONE, 0, 0};
    /* implied, writing X or Y */
    case 0xaa: case 0xba: case 0xbb: case 0xe8: case 0xca:   return (OpInfo){K_IMP, W_8, IX_NONE, 1, 0};
    case 0xa8: case 0x9b: case 0xc8: case 0x88:              return (OpInfo){K_IMP, W_8, IX_NONE, 0, 1};
    /* SEP / REP */
    case 0xe2: case 0xc2:                                    return (OpInfo){K_IMM8, W_8, IX_NONE, 0, 0};
    /* immediate */
    case 0xa9: case 0xc9: case 0x89: case 0x29: case 0x09: case 0x49: return (OpInfo){K_IMM, W_M, IX_NONE, 0, 0};
    case 0xe0: case 0xc0:                                    return (OpInfo){K_IMM, W_X, IX_NONE, 0, 0};
    case 0xa2:                                               return (OpInfo){K_IMM, W_X, IX_NONE, 1, 0};
    case 0xa0:                                               return (OpInfo){K_IMM, W_X, IX_NONE, 0, 1};
    /* direct page */
    case 0xa5: case 0xc5: case 0x24: case 0x25: case 0x05: case 0x45: return (OpInfo){K_DP, W_M, IX_NONE, 0, 0};
    case 0xe4: case 0xc4:                                    return (OpInfo){K_DP, W_X, IX_NONE, 0, 0};
    case 0xa6:                                               return (OpInfo){K_DP, W_X, IX_NONE, 1, 0};
    case 0xa4:                                               return (OpInfo){K_DP, W_X, IX_NONE, 0, 1};
    case 0xb5: case 0xd5: case 0x34: case 0x35: case 0x15: case 0x55: return (OpInfo){K_DP, W_M, IX_X, 0, 0};
    case 0xb4:                                               return (OpInfo){K_DP, W_X, IX_X, 0, 1};
    case 0xb6:                                               return (OpInfo){K_DP, W_X, IX_Y, 1, 0};
    /* absolute (DB:addr) */
    case 0xad: case 0xcd: case 0x2c: case 0x2d: case 0x0d: case 0x4d: return (OpInfo){K_ABS, W_M, IX_NONE, 0, 0};
    case 0xec: case 0xcc:                                    return (OpInfo){K_ABS, W_X, IX_NONE, 0, 0};
    case 0xae:                                               return (OpInfo){K_ABS, W_X, IX_NONE, 1, 0};
    case 0xac:                                               return (OpInfo){K_ABS, W_X, IX_NONE, 0, 1};
    case 0xbd: case 0xdd: case 0x3c: case 0x3d: case 0x1d: case 0x5d: return (OpInfo){K_ABS, W_M, IX_X, 0, 0};
    case 0xb9: case 0xd9: case 0x39: case 0x19: case 0x59:   return (OpInfo){K_ABS, W_M, IX_Y, 0, 0};
    case 0xbc:                                               return (OpInfo){K_ABS, W_X, IX_X, 0, 1};
    case 0xbe:                                               return (OpInfo){K_ABS, W_X, IX_Y, 1, 0};
    /* long */
    case 0xaf: case 0xcf: case 0x2f: case 0x0f: case 0x4f:   return (OpInfo){K_LONG, W_M, IX_NONE, 0, 0};
    case 0xbf: case 0xdf: case 0x3f: case 0x1f: case 0x5f:   return (OpInfo){K_LONG, W_M, IX_X, 0, 0};
    /* branches */
    case 0x10: case 0x30: case 0x50: case 0x70:
    case 0x90: case 0xb0: case 0xd0: case 0xf0:              return (OpInfo){K_BRC, W_8, IX_NONE, 0, 0};
    case 0x80:                                               return (OpInfo){K_BRA, W_8, IX_NONE, 0, 0};
    case 0x4c:                                               return (OpInfo){K_JMP, W_8, IX_NONE, 0, 0};
  }
  return (OpInfo){K_NO, W_8, IX_NONE, 0, 0};
}

/* Code bytes: from the cartridge, or from WRAM for a loop that runs there. */
static bool s_code_ram;
static uint8_t code_at(Snes *s, uint8_t bank, uint16_t pc) {
  if (s_code_ram) {
    uint32_t off = (bank == 0x7e || bank == 0x7f) ? ((uint32_t)(bank & 1) << 16) | pc : pc;
    return s->ram[off & 0x1ffff];
  }
  return cart_read(s->cart, bank, pc);
}

/* Try `head` as the loop head for an interrupted `pc`. On success fill `out`. */
static bool try_head(Snes *s, uint8_t bank, uint16_t head, uint16_t pc, IdleSkip *out) {
  Cpu *cpu = s->cpu;
  const bool m8_0 = cpu->mf || cpu->e, x8_0 = cpu->xf || cpu->e;
  bool m8 = m8_0, x8 = x8_0;
  uint16_t bound[IDLE_MAX_LEN + 1];      /* instruction starts, for branch targets */
  uint16_t fwd[IDLE_MAX_LEN];            /* forward branch targets to check */
  int nb = 0, nf = 0;
  bool saw_pc = false, modx = false, mody = false, sep_rep = false;
  out->nmem = 0; out->nidx = 0; out->rdnmi = false; out->uses_db = false; out->uses_dp = false;
  out->gsu_poll = false;
  out->gsu_and = 0;
  uint16_t gsu_lda_next = 0xffff;                  /* the instruction after a status LDA */
  uint32_t ro;
  const int ccls = classify(s, bank, head, &ro);
  if (ccls != MEM_ROM && ccls != MEM_WRAM) return false;
  s_code_ram = ccls == MEM_WRAM;
  uint16_t q = head;
  for (;;) {
    if ((uint16_t)(q - head) >= IDLE_MAX_LEN) return false;
    if (classify(s, bank, q, &ro) != ccls) return false;       /* code stays in one kind of memory */
    if (q == pc) saw_pc = true;
    bound[nb++] = q;
    uint8_t op = code_at(s, bank, q);
    OpInfo in = op_info(op);
    int w = in.width == W_M ? (m8 ? 1 : 2) : in.width == W_X ? (x8 ? 1 : 2) : 1;
    int len;
    switch (in.kind) {
      case K_IMP:  len = 1; break;
      case K_IMM:  len = 1 + w; break;
      case K_IMM8: len = 2; break;
      case K_DP:   len = 2; break;
      case K_ABS:  len = 3; break;
      case K_LONG: len = 4; break;
      case K_BRC: case K_BRA: len = 2; break;
      case K_JMP:  len = 3; break;
      default: return false;
    }
    modx |= in.modx; mody |= in.mody;
    if (q == gsu_lda_next && op == 0x29) {            /* AND #imm right after the status LDA */
      uint16_t imm = code_at(s, bank, (uint16_t)(q + 1));
      if (w == 2) imm |= (uint16_t)(code_at(s, bank, (uint16_t)(q + 2)) << 8);
      if (w == out->gsu_w)
        out->gsu_and = out->gsu_w == 1 && out->gsu_base == 0x3031 ? (uint16_t)(imm << 8) : imm;
    }
    if (in.kind == K_IMM8) {                                   /* SEP/REP */
      if (cpu->e) return false;
      uint8_t v = code_at(s, bank, (uint16_t)(q + 1));
      if (v & ~0x30) return false;                             /* only M and X */
      if (op == 0xe2) { if (v & 0x20) m8 = true;  if (v & 0x10) x8 = true; }
      else            { if (v & 0x20) m8 = false; if (v & 0x10) x8 = false; }
      sep_rep = true;
    }
    /* the memory operand */
    if (in.kind == K_DP || in.kind == K_ABS || in.kind == K_LONG) {
      uint8_t  mb;
      uint16_t ma;
      if (in.kind == K_DP) {
        if (cpu->e && (cpu->dp & 0xff)) return false;          /* page-wrapping dp */
        mb = 0; ma = (uint16_t)(cpu->dp + code_at(s, bank, (uint16_t)(q + 1)));
        out->uses_dp = true;
      } else {
        ma = (uint16_t)(code_at(s, bank, (uint16_t)(q + 1)) | code_at(s, bank, (uint16_t)(q + 2)) << 8);
        if (in.kind == K_LONG) mb = code_at(s, bank, (uint16_t)(q + 3));
        else { mb = cpu->db; out->uses_db = true; }
      }
      if (in.ix != IX_NONE) {
        /* resolved by the proof with the lap's X/Y; recorded relative */
        if (out->nidx == IDLE_MAX_IDX) return false;
        out->idx[out->nidx].bank = mb;
        out->idx[out->nidx].base = in.kind == K_DP ? code_at(s, bank, (uint16_t)(q + 1)) : ma;
        out->idx[out->nidx].dpmode = in.kind == K_DP;
        out->idx[out->nidx].reg = in.ix;
        out->idx[out->nidx].w = (uint8_t)w;
        out->nidx++;
      } else {
        for (int k = 0; k < w; k++) {
          uint32_t off;
          int c = classify(s, mb, (uint16_t)(ma + k), &off);
          if ((uint16_t)(ma + k) < ma) return false;             /* wraps the bank */
          if (c == MEM_NO) return false;
          if (c == MEM_RDNMI) {
            if (w != 1) return false;                            /* a 16-bit read takes $4211 too */
            out->rdnmi = true;
          }
          if (c == MEM_GSUSFR) {
            if (k == 0) {                                        /* one GSU status read a loop */
              if (out->gsu_poll) return false;
              out->gsu_poll = true;
              out->gsu_pc = q;
              out->gsu_base = ma;
              out->gsu_w = (uint8_t)w;
              out->gsu_bit = op == 0x2c || op == 0x24 || op == 0x3c || op == 0x34;
              if (op == 0xad || op == 0xaf) gsu_lda_next = (uint16_t)(q + len);   /* LDA abs / long */
            }
          }
          if (c == MEM_WRAM) {
            int j;
            for (j = 0; j < out->nmem; j++) if (out->mem_off[j] == off) break;
            if (j == out->nmem) {
              if (out->nmem == IDLE_MAX_MEM) return false;
              out->mem_off[out->nmem++] = off;
            }
          }
        }
      }
    }
    /* control flow */
    if (in.kind == K_BRC || in.kind == K_BRA || in.kind == K_JMP) {
      uint16_t tgt = in.kind == K_JMP
        ? (uint16_t)(code_at(s, bank, (uint16_t)(q + 1)) | code_at(s, bank, (uint16_t)(q + 2)) << 8)
        : (uint16_t)(q + 2 + (int8_t)code_at(s, bank, (uint16_t)(q + 1)));
      if (tgt == head) {
        /* the closing branch: the loop is head .. q+len-1 */
        if (!saw_pc) return false;
        uint16_t end = (uint16_t)(q + len - 1);
        if (end < head) return false;
        if (m8 != m8_0 || x8 != x8_0) return false;              /* widths must come back */
        for (int i = 0; i < nf; i++) {
          if (fwd[i] > end) continue;                            /* leaves the loop */
          int j;
          for (j = 0; j < nb; j++) if (bound[j] == fwd[i]) break;
          if (j == nb) return false;                             /* into an operand */
        }
        for (int i = 0; i < out->nidx; i++)                      /* index registers held */
          if ((out->idx[i].reg == IX_X && modx) || (out->idx[i].reg == IX_Y && mody)) return false;
        (void)sep_rep;
        out->head = head;
        out->len = (uint16_t)(end - head);
        out->bank = bank;
        out->bank_alt = s_code_ram ? bank : mirror_bank(bank);
        out->e = cpu->e; out->mf = cpu->mf; out->xf = cpu->xf;
        out->db = cpu->db; out->dp = cpu->dp;
        out->code_ram = s_code_ram;
        out->ncode = (uint8_t)(out->len + 1);
        for (int i = 0; i < out->ncode; i++) out->code[i] = code_at(s, bank, (uint16_t)(head + i));
        return true;
      }
      if (in.kind != K_BRC) return false;                        /* jumps elsewhere */
      if (tgt <= q) return false;                                /* inner backward branch */
      fwd[nf++] = tgt;
    }
    q = (uint16_t)(q + len);
  }
}

bool idle_prepare_reads(Snes *s, IdleSkip *g) {
  Cpu *cpu = s->cpu;
  int n = 0;
  for (int i = 0; i < g->nmem; i++) g->pmem_off[n++] = g->mem_off[i];
  for (int i = 0; i < g->nidx; i++) {
    uint16_t ix = g->idx[i].reg == IX_X ? cpu->x : cpu->y;
    if (cpu->xf || cpu->e) ix &= 0xff;
    uint8_t mb = g->idx[i].bank;
    uint32_t ea;
    if (g->idx[i].dpmode) {
      if (cpu->e && (cpu->dp & 0xff)) return false;
      ea = (uint16_t)(cpu->dp + g->idx[i].base + ix);            /* bank 0, wraps in it */
      mb = 0;
    } else {
      ea = (uint32_t)g->idx[i].base + ix;                        /* may carry into the next bank */
      mb = (uint8_t)(mb + (ea >> 16));
      ea &= 0xffff;
    }
    for (int k = 0; k < g->idx[i].w; k++) {
      uint32_t off;
      uint16_t a = (uint16_t)(ea + k);
      if (a < ea) return false;
      int c = classify(s, mb, a, &off);
      if (c == MEM_NO || c == MEM_RDNMI || c == MEM_GSUSFR) return false;
      if (c == MEM_WRAM) {
        if (n == (int)(sizeof g->pmem_off / sizeof g->pmem_off[0])) return false;
        g->pmem_off[n++] = off;
      }
    }
  }
  g->npmem = (uint8_t)n;
  return true;
}

static bool site_has(const IdleSkip *g, uint8_t bank, uint16_t pc) {
  return (bank == g->bank || bank == g->bank_alt) && (uint16_t)(pc - g->head) <= g->len;
}

#if SNES_IDLE_ACCUM
/* Accumulator templates. An accumulator wait loop spins x += y in WRAM until
 * the NMI; the generic decoder rejects it on the STA. Each template is matched
 * by bank + head + the exact loop bytes (the guard), so it can only ever fire
 * on the one ROM that contains that code -- a different game, revision or hack
 * will not have these bytes at this address. acc_dest/acc_add are the WRAM byte
 * offsets the loop reads/writes, w the width in bytes. The offsets assume
 * DP == dp: most loops hold an in-loop TCD that forces DP = 0, but a loop that
 * does not (its DP is ambient) is only safe while DP is what the offsets were
 * taken at, so the skip is gated on cpu->dp == dp and re-checked every frame. */
typedef struct {
  uint8_t  bank;
  uint16_t head;
  uint8_t  len;             /* bytes head..last byte of the BRA              */
  uint16_t acc_dest, acc_add;
  uint8_t  acc_w;
  uint16_t dp;              /* DP the acc_dest/acc_add offsets were taken at  */
  uint32_t pre_off;         /* WRAM precondition (0/pre_w==0 = none): see IdleSkip */
  uint16_t pre_val;
  uint8_t  pre_w;
  uint8_t  nguard;
  uint8_t  guard[16];
} AccumTemplate;

static const AccumTemplate s_accum_tpl[] = {
  /* Castlevania - Vampire's Kiss (E) / Dracula X. 80:80c4:
   *   A9 00 00 LDA #$0000 / 5B TCD / A5 46 LDA $46 / 65 42 ADC $42 /
   *   85 46 STA $46 / 80 F4 BRA $80C4.  16-bit M (REP #$20 before), DP=0. */
  { 0x80, 0x80c4, 12, 0x46, 0x42, 2, 0, 0, 0, 0, 12,
    { 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x46, 0x65, 0x42, 0x85, 0x46, 0x80, 0xf4 } },

  /* Teenage Mutant Ninja Turtles IV: Turtles in Time. 00:8154, a 16-byte loop
   * whose body calls a per-lap subroutine before the accumulator:
   *   22 .. 9d 00 JSL $009dXX / A9 00 00 LDA #$0000 / 5B TCD / A5 94 LDA $94 /
   *   65 3A ADC $3A / 85 94 STA $94 / 80 F0 BRA $8154.  DP=0 after the TCD.
   * The JSL (009d16 USA/EU, 009d0b JP) is LDA #$1e00 / TCD / LDX $2c /
   * BEQ ->RTL: it writes nothing and reads no I/O when [$001e2c]==0, which is
   * its state throughout the vsync wait (verified over a full gameplay replay:
   * 100% of laps took the BEQ). The precondition gates the native skip on that,
   * so a lap is only ever skipped while it really is the pure accumulator.
   * Three regions share the loop and the accumulator. USA (crc 5940bd99) and
   * EUR (crc b0a3441e) are byte-identical here (JSL $009d16), so one guard
   * covers both; JP (crc bdba1b55) differs only in the JSL target ($009d0b,
   * guard[1]=0x0b) and gets its own guard-verified entry. */
  { 0x00, 0x8154, 16, 0x94, 0x3a, 2, 0, 0x001e2c, 0, 2, 16,   /* USA 5940bd99 + EUR b0a3441e */
    { 0x22, 0x16, 0x9d, 0x00, 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x94, 0x65, 0x3a, 0x85, 0x94, 0x80, 0xf0 } },
  { 0x00, 0x8154, 16, 0x94, 0x3a, 2, 0, 0x001e2c, 0, 2, 16,   /* JPN bdba1b55 */
    { 0x22, 0x0b, 0x9d, 0x00, 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x94, 0x65, 0x3a, 0x85, 0x94, 0x80, 0xf0 } },

  /* The Adventures of Batman & Robin (USA, crc b3ef81f5). 80:80ed, an 8-byte
   * bare accumulator with no setup in the loop:
   *   A5 88 LDA $88 / 65 3A ADC $3A / 85 88 STA $88 / 80 F8 BRA $80ED.
   * 16-bit M (traced mf=0), binary, native. Unlike Dracula/TMNT there is no
   * in-loop TCD, so DP is ambient 0 here, not forced: the dp=0 gate keeps the
   * skip to the context the $88/$3a offsets were taken at. No precondition. */
  { 0x80, 0x80ed, 8, 0x88, 0x3a, 2, 0, 0, 0, 0, 8,
    { 0xa5, 0x88, 0x65, 0x3a, 0x85, 0x88, 0x80, 0xf8 } },

  /* International Superstar Soccer Deluxe (USA crc 0a20e602, EUR crc cba724ba --
   * byte-identical here). 80:80d4, the Dracula-shape accumulator with an in-loop
   * TCD that forces DP=0:
   *   A9 00 00 LDA #$0000 / 5B TCD / A5 78 LDA $78 / 65 3A ADC $3A /
   *   85 78 STA $78 / 80 F4 BRA $80D4.  16-bit M, native, DP=0.
   * The guard runs 4 bytes past the BRA (C2 30 48 DA) because plain ISS (non-
   * Deluxe) has the same 12 loop bytes at a different head ($80ef); head plus
   * these bytes pin this to ISS Deluxe. */
  { 0x80, 0x80d4, 12, 0x78, 0x3a, 2, 0, 0, 0, 0, 16,
    { 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x78, 0x65, 0x3a, 0x85, 0x78, 0x80, 0xf4, 0xc2, 0x30, 0x48, 0xda } },

  /* International Superstar Soccer (USA crc 49627238, EUR crc 6cd568a9 --
   * byte-identical here). 80:80ef, the same Dracula-shape accumulator as ISS
   * Deluxe but at a different head:
   *   A9 00 00 LDA #$0000 / 5B TCD / A5 78 LDA $78 / 65 3A ADC $3A /
   *   85 78 STA $78 / 80 F4 BRA $80EF.  DP=0 from the in-loop TCD.
   * The 16-byte guard is identical to ISS Deluxe's (same loop + trailing bytes);
   * the head ($80ef vs Deluxe's $80d4) is what tells the two games apart. */
  { 0x80, 0x80ef, 12, 0x78, 0x3a, 2, 0, 0, 0, 0, 16,
    { 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x78, 0x65, 0x3a, 0x85, 0x78, 0x80, 0xf4, 0xc2, 0x30, 0x48, 0xda } },

  /* The Legend of the Mystical Ninja (USA, crc 82479d6a). 00:80b4, the same
   * JSL+accumulator shape as TMNT IV:
   *   22 27 9d 00 JSL $009d27 / A9 00 00 LDA #$0000 / 5B TCD / A5 94 LDA $94 /
   *   65 3A ADC $3A / 85 94 STA $94 / 80 F0 BRA $80B4.  DP=0 after the TCD.
   * The per-lap JSL ($009d27) is the Konami routine LDY #$1e00 / PHY / PLD /
   * ... / LDX $2c / BEQ ->RTL: with DP=$1e00 it reads [$001e2c] and early-RTLs
   * (writing nothing) when it is zero, which is its state through the vsync
   * wait. Same precondition as TMNT -- the skip is gated on [$001e2c]==0 so a
   * lap is only ever taken while the JSL really is a no-op. EUR (crc 797724c8)
   * JSLs a different target ($009d3e) whose code is byte-identical to $009d27,
   * so it gets its own guard entry below (guard[1]=0x3e), same precondition. */
  { 0x00, 0x80b4, 16, 0x94, 0x3a, 2, 0, 0x001e2c, 0, 2, 16,
    { 0x22, 0x27, 0x9d, 0x00, 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x94, 0x65, 0x3a, 0x85, 0x94, 0x80, 0xf0 } },
  { 0x00, 0x80b4, 16, 0x94, 0x3a, 2, 0, 0x001e2c, 0, 2, 16,   /* EUR 797724c8 (JSL $009d3e) */
    { 0x22, 0x3e, 0x9d, 0x00, 0xa9, 0x00, 0x00, 0x5b, 0xa5, 0x94, 0x65, 0x3a, 0x85, 0x94, 0x80, 0xf0 } },
};

bool idle_accum_match(Snes *s, uint8_t bank, uint16_t pc, IdleSkip *out) {
  for (unsigned t = 0; t < sizeof s_accum_tpl / sizeof s_accum_tpl[0]; t++) {
    const AccumTemplate *tp = &s_accum_tpl[t];
    if (bank != tp->bank && bank != mirror_bank(tp->bank)) continue;
    if ((uint16_t)(pc - tp->head) >= tp->len) continue;          /* pc inside the loop */
    /* guard: the loop's bytes must be exactly this code (read from ROM) */
    uint32_t ro;
    if (classify(s, tp->bank, tp->head, &ro) != MEM_ROM) continue;
    bool ok = true;
    for (int i = 0; i < tp->nguard; i++)
      if (cart_read(s->cart, tp->bank, (uint16_t)(tp->head + i)) != tp->guard[i]) { ok = false; break; }
    if (!ok) continue;
    /* the environment the native laps assume: 16-bit A, not emulation mode,
     * binary (not decimal) add. Checked again before each skip. */
    Cpu *cpu = s->cpu;
    if (cpu->e || cpu->mf || cpu->d) continue;
    if (cpu->dp != tp->dp) continue;                             /* offsets assume this DP */
    site_clear(out);
    out->kind    = IDLE_KIND_ACCUM;
    out->head    = tp->head;
    out->len     = tp->len;
    out->bank    = tp->bank;
    out->bank_alt = mirror_bank(tp->bank);
    out->e = cpu->e; out->mf = cpu->mf; out->xf = cpu->xf;
    out->uses_db = false; out->uses_dp = true; out->dp = tp->dp;  /* re-check DP each frame */
    out->acc_dest = tp->acc_dest;
    out->acc_add  = tp->acc_add;
    out->acc_w    = tp->acc_w;
    out->acc_pre_off = tp->pre_off;
    out->acc_pre_val = tp->pre_val;
    out->acc_pre_w   = tp->pre_w;
    out->snap_ok  = false;
    return true;
  }
  return false;
}
#endif

static void idle_sample(Snes *s);
void idle_nmi_sample(Snes *s) {
  g_idle_budget = 2;
  g_idle_gsu_budget = 2;
  s_frame++;
  idle_sample(s);
}

/* A loop can be entered in more than one environment (Yoshi's Island polls
 * the GSU from the same WRAM loop with DB $01 and with DB $03 every frame):
 * each is a site of its own, so "already active" means the environment too. */
void idle_gsu_poll_sample(Snes *s) {
  const Cpu *cpu = s->cpu;
  if (site_has(&g_idle, cpu->k, cpu->pc) && idle_env_ok(cpu, &g_idle)) return;   /* already the active site */
  idle_sample(s);
}

void idle_irq_sample(Snes *s) {
  const Cpu *cpu = s->cpu;
  if (site_has(&g_idle, cpu->k, cpu->pc) && idle_env_ok(cpu, &g_idle)) return;
  idle_sample(s);
}

void idle_line_select(Snes *s) {
  const Cpu *cpu = s->cpu;
  if (site_has(&g_idle, cpu->k, cpu->pc) && idle_env_ok(cpu, &g_idle)) return;
  for (int i = 0; i < IDLE_SITES; i++) {
    IdleSkip *g = &g_idle_sites[i];
    if (g->len == 0 && g->head == 0xffff) continue;
    if (site_has(g, cpu->k, cpu->pc) && idle_env_ok(cpu, g) && idle_code_ok(s, g)) { g_idlep = g; return; }
  }
}

#ifdef HOST_BUILD
/* IDLE_TRACE=n: log the first n site decisions (why a site is re-decoded) */
#include <stdlib.h>
static long idle_trace_n = -2;
#define IDLE_TRACE(...) do { if (idle_trace_n == -2) { const char *e_ = getenv("IDLE_TRACE"); idle_trace_n = e_ ? atol(e_) : 0; } \
                             if (idle_trace_n > 0) { idle_trace_n--; printf(__VA_ARGS__); } } while (0)
#else
#define IDLE_TRACE(...) do { } while (0)
#endif

static void idle_sample(Snes *s) {
  Cpu *cpu = s->cpu;
  if (cpu->stopped) return;
  const uint8_t bank = cpu->k;
  const uint16_t pc = cpu->pc;
  /* the pc is in a known site: make it the active one. A site of the same
   * loop decoded in another environment (M/X/E, DB, DP) stays: this one gets
   * a slot of its own. Only a site whose code changed is decoded again in
   * place. */
  int slot = -1;
  for (int i = 0; i < IDLE_SITES; i++) {
    IdleSkip *g = &g_idle_sites[i];
    if (g->len == 0 && g->head == 0xffff) continue;
    if (!site_has(g, bank, pc)) continue;
    if (idle_env_ok(cpu, g) && idle_code_ok(s, g)) { g->used = s_frame; g_idlep = g; return; }
    if (idle_code_ok(s, g)) continue;   /* another environment of this loop */
    IDLE_TRACE("IDLE: frame %lu %02x:%04x in slot %d (%02x:%04x+%u) but %s: cpu e%d m%d x%d db%02x dp%04x, site e%d m%d x%d db%02x(%d) dp%04x(%d)\n",
               (unsigned long)s_frame, bank, pc, i, g->bank, g->head, g->len,
               !idle_env_ok(cpu, g) ? "env changed" : "code changed", cpu->e, cpu->mf, cpu->xf, cpu->db, cpu->dp,
               g->e, g->mf, g->xf, g->db, g->uses_db, g->dp, g->uses_dp);
    slot = i;
    break;
  }
  const uint32_t pc24 = ((uint32_t)bank << 16) | pc;
  if (slot < 0) for (int i = 0; i < 4; i++) if (s_reject[i] == pc24) return;
  IdleSkip cand;
  for (int back = 0; back < IDLE_MAX_LEN; back++) {
    uint16_t head = (uint16_t)(pc - back);
    if (head > pc) break;                                        /* wrapped below $0000 */
    site_clear(&cand);
    if (try_head(s, bank, head, pc, &cand)) {
      if (slot < 0) {                                            /* an empty slot, else the oldest */
        slot = 0;
        for (int i = 0; i < IDLE_SITES; i++) {
          if (g_idle_sites[i].len == 0 && g_idle_sites[i].head == 0xffff) { slot = i; break; }
          if (g_idle_sites[i].used < g_idle_sites[slot].used) slot = i;
        }
      }
      cand.used = s_frame;
      IDLE_TRACE("IDLE: frame %lu rearm %02x:%04x+%u gsu_poll=%d code_ram=%d in slot %d env e%d m%d x%d db%02x(%d) dp%04x(%d)\n",
                 (unsigned long)s_frame, bank, cand.head, cand.len, cand.gsu_poll, cand.code_ram, slot,
                 cand.e, cand.mf, cand.xf, cand.db, cand.uses_db, cand.dp, cand.uses_dp);
      g_idle_sites[slot] = cand;
      g_idlep = &g_idle_sites[slot];
      g_idle_stats.rearms++;
      return;
    }
  }
#if SNES_IDLE_ACCUM
  /* generic decode failed: try the accumulator templates (guard-verified) */
  if (idle_accum_match(s, bank, pc, &cand)) {
    if (slot < 0) {
      slot = 0;
      for (int i = 0; i < IDLE_SITES; i++) {
        if (g_idle_sites[i].len == 0 && g_idle_sites[i].head == 0xffff) { slot = i; break; }
        if (g_idle_sites[i].used < g_idle_sites[slot].used) slot = i;
      }
    }
    cand.used = s_frame;
    g_idle_sites[slot] = cand;
    g_idlep = &g_idle_sites[slot];
    g_idle_stats.rearms++;
    return;
  }
#endif
  IDLE_TRACE("IDLE: frame %lu reject %02x:%04x (no idle loop decoded there)\n", (unsigned long)s_frame, bank, pc);
  g_idle_stats.rejects++;
  s_reject[s_reject_i++ & 3] = pc24;
}
