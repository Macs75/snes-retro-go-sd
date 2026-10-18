# Super FX: one-switch GSU dispatch (tried, reverted)

Status: implemented, verified exact, measured on the device, REVERTED. The patch
is not in the tree; this records what it was and why it lost.

## What it was

The ZSNES GSU core dispatches each opcode through four 1024-entry tables of
handler pointers (FxTabled at the top level, FxTable / FxTableb / FxTablec for an
opcode nested in a prefix), with the interpreter state (PC, opcode/ALT word,
source/destination, done flag, budget) in globals, and prefixes (TO/FROM/WITH/
ALTn, branch delay slots) dispatching their opcode recursively.

The rewrite:
- every handler took a context pointer `FxCtx *S` (state as fields of a local
  struct) and was `always_inline`;
- MainLoop became one `switch` over ALT << 8 | opcode, generated from the ZSNES
  tables (the four tables differ in only 48 slots: STOP, the default PLOT, and
  the 46 branch / TO / ALTn / FROM slots of the b and c tables), each handler
  inlined once (case lists grouped by handler);
- prefixes set a "nest" flag and pushed their tail (restore src/dst, refresh R14,
  apply R15, clear ALT) on a stack unwound after the nested opcode. Branches push
  nothing (delay slots can chain branches, so the depth is unbounded).

Exact: 3.0M-instruction GSU traces of Yoshi's Island and Star Fox byte-identical
to the table version; the seven FX REC hash runs identical; the Yoshi device
recording matched the device at all 160 checkpoints.

## Why it lost (device PERF, Yoshi recording, log 3 -> log 4)

| scene | GSU % of budget, tables -> switch |
|---|---|
| heavy intro | 36.6 -> 40.5 (+11%) |
| heaviest intro | 60.6 -> 67.6 (+11%) |
| title screen | 102.4 -> 103.4 |
| gameplay | 23.5 -> 22.6 (-4%) |

MainLoop became one 46 KB function; GCC could not keep the context in
registers across it (1,128 of its 16,200 ARM instructions are stack accesses,
most to one spill slot), so the spills replaced the global accesses while the
switch added a jump-table lookup, the context checks and the tail stack per
opcode. Host callgrind was no guide (the host-only trace/dump hooks are inline
in the loop there).

The overlay did shrink (92.5 KB -> 75.7 KB of pool: the four 4 KB tables went).

## What the other cores in this tree do instead

The 65816 Thumb-2 engine: hand-written, in ITCM, computed jump inside one
routine, state through one pinned base register (r4). The arcade 68000 (Musashi)
and NEC (MAME 0.151) cores: C, one function-pointer call per opcode like the
GSU, but all CPU state in ONE struct (one base address); their big win was the
NEC per-game "block code" (tools/arcade/nec_hle_build.py in gw-arcade-core).
That is the next step for the GSU (per-game solutions are allowed for Super FX).
