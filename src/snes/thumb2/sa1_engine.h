/* The offsets the SA-1's engine (sa1_engine.S) reads; sa1.c asserts them. */
#ifndef SA1_ENGINE_H
#define SA1_ENGINE_H

/* struct Sa1: the read map at 0, I-RAM at SA1E_IRAM, the engine's I-RAM
 * write mark a word before it (relative to I-RAM) */
#define SA1E_IRAM       0x2008
#define SA1E_WDIRTY     (-8)

/* Sa1Run */
#define RUN_DEBT        0
#define RUN_LAPCYC      4
#define RUN_LAPINS      8
#define RUN_EXTEV       16
#define RUN_LAPEV       20
#define RUN_CANDPC      24
#define RUN_CANDK       26
#define RUN_DIRTY       27
#define RUN_OPS         52
#define RUN_AS          56

#endif
