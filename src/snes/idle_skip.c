/* See idle_skip.h. This half runs once a frame, at the NMI request: decode the
 * loop around the interrupted pc and arm it if it is provably read-only. The
 * span runner that replays it lives next to run_dots in main_snes.c, because
 * it has to copy run_dots' arithmetic exactly. */
#include "idle_skip.h"
#include "cart.h"

static IdleSkip s_idle_static;
IdleSkip *g_idlep = &s_idle_static;

#ifdef TARGET_GNW
#include "gw_malloc.h"
#endif
void idle_alloc(void) {
#ifdef TARGET_GNW
  void *p = dtc_malloc(sizeof(IdleSkip));
  if (p && p != (void *)-1) g_idlep = (IdleSkip *)p;
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

void idle_reset(void) {
  g_idle = (IdleSkip){0};
  g_idle.head = 0xffff;
  g_idle.len = 0;
  for (int i = 0; i < 4; i++) s_reject[i] = 0xffffffffu;
  s_reject_i = 0;
}

/* $7e/$7f are WRAM, not a ROM mirror */
static uint8_t mirror_bank(uint8_t bank) {
  if ((bank & 0x7f) >= 0x7e) return bank;
  return (uint8_t)(bank ^ 0x80);
}

enum { MEM_NO = 0, MEM_WRAM, MEM_ROM };

/* What a read at bank:off reaches. WRAM gives its offset into Snes.ram. ROM is
 * accepted only for the two plain mappings, where it is constant; anything else
 * -- I/O registers, SRAM, a coprocessor window -- is refused. */
static int classify(const Snes *s, uint8_t bank, uint16_t off, uint32_t *ram_off) {
  uint8_t b = bank & 0x7f;
  if (bank == 0x7e || bank == 0x7f) { *ram_off = ((uint32_t)(bank & 1) << 16) | off; return MEM_WRAM; }
  if (b < 0x40 && off < 0x2000)      { *ram_off = off; return MEM_WRAM; }
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
 * the program bank, DB, DP and the M/X/E/I flags alone. */
enum { K_NO = 0, K_IMP, K_IMM, K_DP, K_ABS, K_LONG, K_BRC, K_BRA, K_JMP };
enum { W_8 = 0, W_M, W_X };
typedef struct { uint8_t kind, width; } OpInfo;

static OpInfo op_info(uint8_t op) {
  switch (op) {
    /* implied: nop, carry/overflow flags, transfers, register inc/dec, A shifts */
    /* WAI: with a masked IRQ pending it returns at once, and otherwise the
     * span runner stops at it (a real wait is the scheduler's WAI skip) */
    case 0xcb:
    case 0xea: case 0x18: case 0x38: case 0xb8:
    case 0xaa: case 0xa8: case 0x8a: case 0x98: case 0x9b: case 0xbb: case 0xba:
    case 0x7b: case 0x3b: case 0xeb:
    case 0xe8: case 0xc8: case 0xca: case 0x88: case 0x1a: case 0x3a:
    case 0x0a: case 0x4a: case 0x2a: case 0x6a:
      return (OpInfo){K_IMP, W_8};
    /* immediate */
    case 0xa9: case 0xc9: case 0x89: case 0x29: case 0x09: case 0x49: return (OpInfo){K_IMM, W_M};
    case 0xa2: case 0xa0: case 0xe0: case 0xc0:                       return (OpInfo){K_IMM, W_X};
    /* direct page */
    case 0xa5: case 0xc5: case 0x24: case 0x25: case 0x05: case 0x45: return (OpInfo){K_DP, W_M};
    case 0xa6: case 0xa4: case 0xe4: case 0xc4:                       return (OpInfo){K_DP, W_X};
    /* absolute (DB:addr) */
    case 0xad: case 0xcd: case 0x2c: case 0x2d: case 0x0d: case 0x4d: return (OpInfo){K_ABS, W_M};
    case 0xae: case 0xac: case 0xec: case 0xcc:                       return (OpInfo){K_ABS, W_X};
    /* long */
    case 0xaf: case 0xcf: case 0x2f: case 0x0f: case 0x4f:            return (OpInfo){K_LONG, W_M};
    /* branches */
    case 0x10: case 0x30: case 0x50: case 0x70:
    case 0x90: case 0xb0: case 0xd0: case 0xf0:                       return (OpInfo){K_BRC, W_8};
    case 0x80:                                                        return (OpInfo){K_BRA, W_8};
    case 0x4c:                                                        return (OpInfo){K_JMP, W_8};
  }
  return (OpInfo){K_NO, W_8};
}

static uint8_t code_at(Snes *s, uint8_t bank, uint16_t pc) { return cart_read(s->cart, bank, pc); }

/* Try `head` as the loop head for an interrupted `pc`. On success fill `out`. */
static bool try_head(Snes *s, uint8_t bank, uint16_t head, uint16_t pc, IdleSkip *out) {
  Cpu *cpu = s->cpu;
  const bool m8 = cpu->mf || cpu->e, x8 = cpu->xf || cpu->e;
  uint16_t bound[IDLE_MAX_LEN + 1];      /* instruction starts, for branch targets */
  uint16_t fwd[IDLE_MAX_LEN];            /* forward branch targets to check */
  int nb = 0, nf = 0;
  bool saw_pc = false;
  out->nmem = 0;
  uint16_t q = head;
  for (;;) {
    if ((uint16_t)(q - head) >= IDLE_MAX_LEN) return false;
    uint32_t ro;
    if (classify(s, bank, q, &ro) != MEM_ROM) return false;   /* code must be ROM */
    if (q == pc) saw_pc = true;
    bound[nb++] = q;
    uint8_t op = code_at(s, bank, q);
    OpInfo in = op_info(op);
    int w = in.width == W_M ? (m8 ? 1 : 2) : in.width == W_X ? (x8 ? 1 : 2) : 1;
    int len;
    switch (in.kind) {
      case K_IMP:  len = 1; break;
      case K_IMM:  len = 1 + w; break;
      case K_DP:   len = 2; break;
      case K_ABS:  len = 3; break;
      case K_LONG: len = 4; break;
      case K_BRC: case K_BRA: len = 2; break;
      case K_JMP:  len = 3; break;
      default: return false;
    }
    /* the memory operand */
    if (in.kind == K_DP || in.kind == K_ABS || in.kind == K_LONG) {
      uint8_t  mb;
      uint16_t ma;
      if (in.kind == K_DP) {
        if (cpu->e && (cpu->dp & 0xff)) return false;          /* page-wrapping dp */
        mb = 0; ma = (uint16_t)(cpu->dp + code_at(s, bank, (uint16_t)(q + 1)));
      } else {
        ma = (uint16_t)(code_at(s, bank, (uint16_t)(q + 1)) | code_at(s, bank, (uint16_t)(q + 2)) << 8);
        mb = in.kind == K_LONG ? code_at(s, bank, (uint16_t)(q + 3)) : cpu->db;
      }
      for (int k = 0; k < w; k++) {
        uint32_t off;
        int c = classify(s, mb, (uint16_t)(ma + k), &off);
        if ((uint16_t)(ma + k) < ma) return false;               /* wraps the bank */
        if (c == MEM_NO) return false;
        if (c == MEM_WRAM) {
          int j;
          for (j = 0; j < out->nmem; j++) if (out->mem_off[j] == off) break;
          if (j == out->nmem) {
            if (out->nmem == IDLE_MAX_MEM) return false;
            out->mem_off[out->nmem] = off;
            out->mem_w[out->nmem] = 1;
            out->nmem++;
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
        for (int i = 0; i < nf; i++) {
          if (fwd[i] > end) continue;                            /* leaves the loop */
          int j;
          for (j = 0; j < nb; j++) if (bound[j] == fwd[i]) break;
          if (j == nb) return false;                             /* into an operand */
        }
        out->head = head;
        out->len = (uint16_t)(end - head);
        out->bank = bank;
        out->bank_alt = mirror_bank(bank);
        out->e = cpu->e; out->mf = cpu->mf; out->xf = cpu->xf;
        out->db = cpu->db; out->dp = cpu->dp;
        return true;
      }
      if (in.kind != K_BRC) return false;                        /* jumps elsewhere */
      if (tgt <= q) return false;                                /* inner backward branch */
      fwd[nf++] = tgt;
    }
    q = (uint16_t)(q + len);
  }
}

void idle_nmi_sample(Snes *s) {
  Cpu *cpu = s->cpu;
  g_idle.proof_budget = 2;
  if (cpu->stopped) return;
  const uint8_t bank = cpu->k;
  const uint16_t pc = cpu->pc;
  /* still in the armed loop: nothing to do */
  if ((bank == g_idle.bank || bank == g_idle.bank_alt) && (uint16_t)(pc - g_idle.head) <= g_idle.len)
    return;
  const uint32_t pc24 = ((uint32_t)bank << 16) | pc;
  for (int i = 0; i < 4; i++) if (s_reject[i] == pc24) return;
  IdleSkip cand;
  for (int back = 0; back < IDLE_MAX_LEN; back++) {
    uint16_t head = (uint16_t)(pc - back);
    if (head > pc) break;                                        /* wrapped below $0000 */
    cand = g_idle;
    if (try_head(s, bank, head, pc, &cand)) {
      uint32_t laps = g_idle.laps, ff = g_idle.ff_dots, wai = g_idle.wai_dots;
      uint32_t rearms = g_idle.rearms + 1, rejects = g_idle.rejects;
      g_idle = cand;
      g_idle.snap_ok = false;
      g_idle.laps = laps; g_idle.ff_dots = ff; g_idle.wai_dots = wai;
      g_idle.rearms = rearms; g_idle.rejects = rejects;
      return;
    }
  }
  g_idle.rejects++;
  s_reject[s_reject_i++ & 3] = pc24;
}
