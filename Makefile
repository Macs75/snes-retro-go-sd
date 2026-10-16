# SNES (LakeSnes) — standalone Retro-Go SD dynamic core.
#
#   make / make docker
#   make host             — Linux/macOS SDL binary (same src/main_snes.c)
#   make host HOST_SDL=3  — same with SDL3
#
# Output: snes.bin → /cores/snes.bin ; ROMs under /roms/snes/ (.sfc .smc .fig .swc)
#
# Memory (match jshsakura overlay, adapted to this firmware ABI):
#   ITCM     — Thumb-2 65816 + bus accessors + app_main_snes (same as jsh)
#   DTCM     — small structs + Apu/ARAM via dtc_* (jsh: structs on DTCM heap,
#              Apu on AHB; AHB here is ~56 KiB so Apu cannot follow)
#   AHB      — ~56 KiB newlib heap only
#   RAM_EMU  — core image + WRAM/VRAM/SRAM/FB/Ppu (ram_* leftover)

PROJECT_KIND ?= core

CORE_NAME  := snes
CORE_ENTRY := app_main_snes

OPT ?= -O3

SNES := src/snes

SNES_THUMB2_CPU ?= 1
SNES_THUMB2_SPC ?= 1
# Match jshsakura shipping defaults (Makefile.common).
SNES_SPIN_SKIP ?= 0
# idle-loop skip found at each NMI (src/snes/idle_skip.h); it covers the bake's
# LDA dp / BEQ shape, so the bake is off when it is on
SNES_IDLE_SKIP ?= 1
ifeq ($(SNES_IDLE_SKIP),1)
SNES_SPIN_BAKE ?= 0
else
SNES_SPIN_BAKE ?= 1
endif
SNES_DSP_MONO ?= 0
SNES_SPC_IDLE_SKIP ?= 1
# silent DSP voices leave the per-sample loop, caught up when they wake (src/snes/dsp.c)
SNES_DSP_IDLE_SKIP_VOICE ?= 1
SNES_BUS_IN_ITCM ?= 1
# PERF=1: per-component cycle profiler, PROF/... log lines every 120 frames (src/snes_perf.h)
PERF ?= 0
PERF_PPU ?= 0
PERF_AUDIO ?= 0
PERF_CPU ?= 0

CORE_C_SOURCES := \
$(SNES)/apu.c \
$(SNES)/cart.c \
$(SNES)/cpu.c \
$(SNES)/dma.c \
$(SNES)/dsp.c \
$(SNES)/dsp1_hle.c \
$(SNES)/cx4_hle.c \
$(SNES)/sdd1.c \
$(SNES)/spc7110.c \
$(SNES)/gsu.c \
$(SNES)/gsu_zsnes/gsu_core.c \
$(SNES)/input.c \
$(SNES)/ppu.c \
$(SNES)/snes.c \
$(SNES)/snes_other.c \
$(SNES)/spc.c \
$(SNES)/spin_skip.c \
$(SNES)/spin_bake.c \
$(SNES)/idle_skip.c \
$(SNES)/rc_dispatch.c \
$(SNES)/tracing.c \
src/snes_audio_stretch.c \
src/main_snes.c

CORE_ASM_SOURCES :=
ifeq ($(SNES_THUMB2_CPU),1)
CORE_ASM_SOURCES += $(SNES)/thumb2/snes_thumb2.S
CORE_C_SOURCES += $(SNES)/thumb2/cpu_thumb2_offsets_check.c
endif
ifeq ($(SNES_THUMB2_SPC),1)
CORE_ASM_SOURCES += $(SNES)/thumb2/spc_thumb2.S
CORE_C_SOURCES += $(SNES)/thumb2/spc_thumb2_offsets_check.c
endif

CORE_C_INCLUDES := \
-Isrc \
-I$(SNES)

CORE_C_DEFS := \
-DPROJECT_KIND_CORE=1 \
-DCOVERFLOW=1 \
-DCHEAT_CODES=0 \
-DTARGET_GNW \
-DPPU_RGB565 \
-DGNW_SNES_CORE \
-DSNES_PPU_DIRECT_MATH \
-DSNES_DIRECT_VIDEO \
-DSNES_PRESENT_DMA2D \
-DSNES_SPC_IDLE_SKIP=$(SNES_SPC_IDLE_SKIP) \
-DSNES_DSP_IDLE_SKIP_VOICE=$(SNES_DSP_IDLE_SKIP_VOICE) \
-DSNES_PPU_OPAQUE_TILE=1 \
-DSNES_PPU_BLEND_LUT=1 \
-DSNES_PPU_VIRGIN_Z=1 \
-DSNES_SKIP_SPRITE_EVAL_ON_SKIP=1 \
-DSNES_STRETCH_FOLLOW=1

ifeq ($(SNES_BUS_IN_ITCM),1)
CORE_C_DEFS += -DSNES_BUS_IN_ITCM
endif
# PROFLITE=1: keep only the cheap per-frame tracking (frame skips + audio step)
# and print a two-line PROFLITE report; no per-line/per-opcode DWT probes. For
# a low-overhead "how does it fare" check. Implies SNES_PERF (the cheap
# counters live there); leave PERF_PPU/PERF_CPU off.
ifeq ($(PROFLITE),1)
CORE_C_DEFS += -DSNES_PERF -DSNES_PROFLITE
endif
ifeq ($(PERF),1)
CORE_C_DEFS += -DSNES_PERF
# PERF_PPU=1 adds the PPU split (probes on every drawn line, a few K a frame)
ifeq ($(PERF_PPU),1)
CORE_C_DEFS += -DSNES_PERF_PPU
endif
# PERF_CPU=1 splits the 65816 figure (engine / spin bake / events / rest) and
# counts opcodes, C fallbacks and slow bus accesses. Diagnostic only: two timer
# reads per opcode, and the probes push the ITCM engine 0x400 further.
ifeq ($(PERF_CPU),1)
CORE_C_DEFS += -DSNES_PERF_CPU -DSNES_OP_CENSUS=1
endif
# PERF_AUDIO=1 adds the stretcher / audio-rate diagnostics
ifeq ($(PERF_AUDIO),1)
CORE_C_DEFS += -DSNES_PERF_AUDIO
endif
endif
ifeq ($(SNES_SPIN_SKIP),1)
CORE_C_DEFS += -DSNES_SPIN_SKIP
endif
ifeq ($(SNES_SPIN_BAKE),1)
CORE_C_DEFS += -DSNES_SPIN_BAKE
endif
ifeq ($(SNES_IDLE_SKIP),1)
CORE_C_DEFS += -DSNES_IDLE_SKIP
endif
ifeq ($(SNES_DSP_MONO),1)
CORE_C_DEFS += -DSNES_DSP_MONO
endif
ifeq ($(SNES_THUMB2_CPU),1)
CORE_C_DEFS += -DSNES_THUMB2_CPU
endif
ifeq ($(SNES_THUMB2_SPC),1)
CORE_C_DEFS += -DSPC_THUMB2_SPC
endif

CORE_LDSCRIPT := snes_core.ld
CORE_LDLIBS := -lm
ifeq ($(PERF),1)
ifeq ($(PERF_CPU),1)
CORE_LDLIBS += -Wl,--defsym=__SNES_ITCM_ENGINE_AT=0x2204
endif
endif

GNW_CORE_SDK ?= sdk
BUILD_DIR ?= build/$(PROJECT_KIND)

include $(GNW_CORE_SDK)/Makefile

# Quiet noisy upstream warnings.
$(BUILD_DIR)/apu.o \
$(BUILD_DIR)/cart.o \
$(BUILD_DIR)/cpu.o \
$(BUILD_DIR)/dma.o \
$(BUILD_DIR)/dsp.o \
$(BUILD_DIR)/dsp1_hle.o \
$(BUILD_DIR)/cx4_hle.o \
$(BUILD_DIR)/sdd1.o \
$(BUILD_DIR)/input.o \
$(BUILD_DIR)/ppu.o \
$(BUILD_DIR)/snes.o \
$(BUILD_DIR)/snes_other.o \
$(BUILD_DIR)/spc.o \
$(BUILD_DIR)/spin_skip.o \
$(BUILD_DIR)/spin_bake.o \
$(BUILD_DIR)/rc_dispatch.o \
$(BUILD_DIR)/tracing.o \
$(BUILD_DIR)/snes_audio_stretch.o \
$(BUILD_DIR)/main_snes.o: CFLAGS += \
	-Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function \
	-Wno-unused-parameter -Wno-sign-compare -Wno-strict-aliasing \
	-Wno-implicit-fallthrough -Wno-parentheses -Wno-maybe-uninitialized \
	-Wno-type-limits

PACKED_BIN := snes.bin
PACK_CORE  := $(GNW_CORE_SDK)/tools/pack_core.py
PAD_LOGO    := src/assets/pad.bmp
HEADER_LOGO := src/assets/header.bmp

#######################################
# Packed header version
#######################################
# gnw_core_meta_t only stores major.minor.patch (0..255).
# CORE_VERSION is the full git describe string passed to the packer; it
# extracts the leading vX.Y.Z (NOTAG / missing tags → 0.0.0).
# Override: make CORE_VERSION=v1.2.3
CORE_VERSION ?= $(shell git describe --tags --dirty 2>/dev/null || echo NOTAG)

.PHONY: pack
pack: $(TARGET_BIN)
	$(V)$(ECHO) [ PACK CORE ] $(PACKED_BIN) version=$(CORE_VERSION)
	$(V)python3 $(PACK_CORE) \
		--elf $(TARGET_ELF) --bin $(TARGET_BIN) \
		--system-name "SNES" --dirname snes \
		--extensions "sfc smc fig swc" \
		--core-name "lakesnes" \
		--version "$(CORE_VERSION)" \
		--pad-logo $(PAD_LOGO) \
		--header-logo $(HEADER_LOGO) \
		--logo-invert \
		--out $(PACKED_BIN)
	$(V)python3 tools/append_gsu_overlay.py $(TARGET_ELF) $(PACKED_BIN) \
		--nm $(subst objcopy,nm,$(CP)) --objcopy $(CP)

all: pack

clean::
	$(V)rm -f $(PACKED_BIN)

#######################################
# Docker
#######################################
.PHONY: docker docker_pull docker_shell print-PROJECT_KIND print-PACKED_BIN print-CORE_NAME print-DOCKER_IMAGE \
	print-TARGET_ELF print-TARGET_MAP print-CORE_VERSION

print-PROJECT_KIND:
	@echo $(PROJECT_KIND)
print-PACKED_BIN:
	@echo $(PACKED_BIN)
print-CORE_NAME:
	@echo $(CORE_NAME)
print-DOCKER_IMAGE:
	@echo $(DOCKER_IMAGE)
print-TARGET_ELF:
	@echo $(TARGET_ELF)
print-TARGET_MAP:
	@echo $(BUILD_DIR)/$(CORE_NAME)_core.map
print-CORE_VERSION:
	@echo $(CORE_VERSION)

RELEASE_VERSION ?= v1.5
DOCKER_REPOSITORY ?= sylverb/retro-go-sd-builder
DOCKER_IMAGE ?= $(DOCKER_REPOSITORY):$(RELEASE_VERSION)

DOCKER_TTY_FLAG := $(shell if [ -t 0 ]; then echo -it; else echo; fi)
DOCKER_USER := $(shell id -u):$(shell id -g)
DOCKER_RUN := docker run --rm $(DOCKER_TTY_FLAG) \
	--user $(DOCKER_USER) \
	-v "$(CURDIR):/opt/workdir" \
	-w /opt/workdir \
	$(DOCKER_IMAGE)

docker:
	$(V)$(ECHO) "[ DOCKER ]" $(DOCKER_IMAGE)
	$(V)$(DOCKER_RUN) make --no-print-directory -j$$(nproc)

docker_pull:
	$(V)$(ECHO) "[ PULL ]" $(DOCKER_IMAGE)
	$(V)docker pull $(DOCKER_IMAGE)

docker_shell:
	$(DOCKER_RUN) bash

#######################################
# Host SDL (Linux / macOS)
#######################################
include host/Makefile.host
include host/qemu/Makefile.qemu
