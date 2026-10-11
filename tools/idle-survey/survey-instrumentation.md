# Survey instrumentation (measurement build only)

The survey needs two host binaries. **Neither of these probes is in the shipping
core** — they are measurement-only edits, kept out of the tree so the device
pays nothing. Apply them in a throwaway work-tree to build `host_survey`, and
build `host_base` from the clean tree.

```
host_base    unmodified host build with the idle skip OFF  (the reference)
host_survey  host build with the idle skip ON + the three probes below
```

Build both with the normal host target (`make host`, producing `./snes_host`);
rename the outputs to `host_base` / `host_survey` and put them where `run_one.sh`
looks (its `BIN` dir). For `host_base`, build with `SNES_IDLE_SKIP=0` (or the
pre-idle-skip commit) so it is a true unmodified reference.

The core already has `SNES_PC_HISTOGRAM` in `src/snes/cpu.c` (a raw per-opcode
address counter into `g_pchist`); the survey does **not** use it — it uses the
cost-weighted, span-runner histogram below instead, which attributes cycles the
way the device feels them.

## Probe 1 — cost-weighted PC histogram (`src/main_snes.c`, in the idle span runner)

At the point where a span's work is attributed (there are two, the fast-forward
and the final-lap paths), bump a 16M-entry histogram keyed by the span's head PC,
weighted by an approximate device cost (`cycles*6 + memops*2`):

```c
{ extern uint32_t *g_pch_n, *g_pch_c, g_pch_on;
  if (g_pch_on) { g_pch_n[pch_pc_]++; g_pch_c[pch_pc_] += cycles * 6 + s->cpuMemOps * 2; } }
```

`pch_pc_` is the 24-bit `(bank<<16)|pc` of the span being charged, masked to
`0xffffff`.

## Probe 2 — window + dump (`src/main_snes.c`, once per emulated frame)

Allocate on first use, turn the histogram on at frame `PCH_LO`, and dump
`pch.txt` at frame `PCH_HI` (`addr  count  cost`, one line per touched address):

```c
uint32_t *g_pch_n, *g_pch_c, g_pch_on;   /* file scope */

/* at the end of each frame: */
{ extern uint32_t *g_pch_n, *g_pch_c, g_pch_on; static long fr, lo=-1, hi;
  if (lo < 0) { const char *a = getenv("PCH_LO"), *b = getenv("PCH_HI");
    lo = a ? atol(a) : 1L<<30; hi = b ? atol(b) : 1L<<30;
    if (a) { g_pch_n = calloc(1<<24,4); g_pch_c = calloc(1<<24,4); } }
  fr++;
  if (fr == lo && g_pch_n) g_pch_on = 1;
  if (fr == hi && g_pch_n) { g_pch_on = 0;
    FILE *f = fopen("pch.txt","w");
    for (uint32_t q = 0; q < (1u<<24); q++)
      if (g_pch_n[q]) fprintf(f, "%06x %u %u\n", q, g_pch_n[q], g_pch_c[q]);
    fclose(f); } }
```

## Probe 3 — NMI-landing log (`src/snes/cpu.c`, in `cpu_doInterrupt`)

At the NMI (`!irq`), log the PC the CPU was executing when the NMI fired — this
is almost always inside the wait loop — to `nmi.txt` (`frame  bank:pc  waiting`):

```c
if (!irq) { extern FILE *g_nmi_f; extern long g_nmi_frame, g_nmi_lo;
  if (g_nmi_frame >= g_nmi_lo) {
    if (!g_nmi_f) g_nmi_f = fopen("nmi.txt","w");
    fprintf(g_nmi_f, "%ld %06x %d\n",
            g_nmi_frame, ((unsigned)cpu->k << 16) | cpu->pc, (int)cpu->waiting); } }
```

Define `FILE *g_nmi_f; long g_nmi_frame, g_nmi_lo;` at file scope, set
`g_nmi_lo` from `PCH_LO` at start-up, and `g_nmi_frame++` once per emulated
frame (next to Probe 2).

## Outputs a run leaves in `gNNN/`

| file | from | contents |
|---|---|---|
| `base.txt` / `base_err.txt` | host_base | the reference run + its `REC/hash` lines |
| `sv.txt` | host_survey | the idle-build run + its `REC/hash` lines (compared to base for correctness) |
| `sv_err.txt` | host_survey, `IDLE_STATS=1` | `IDLE f=… site=… laps=… ff=… wai=… rejects=…` every 600 frames |
| `pch.txt` | Probe 1+2 | cost-weighted PC histogram over `PCH_LO..PCH_HI` |
| `nmi.txt` | Probe 3 | where the NMI landed each frame |

`analyze.py` reads all of these.
