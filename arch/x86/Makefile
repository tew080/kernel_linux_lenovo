# SPDX-License-Identifier: GPL-2.0
# Unified Makefile for i386 and x86_64

# select defconfig based on actual architecture
ifeq ($(ARCH),x86)
  ifeq ($(shell uname -m | sed -e 's/i.86/i386/'),i386)
        KBUILD_DEFCONFIG := i386_defconfig
  else
        KBUILD_DEFCONFIG := x86_64_defconfig
  endif
else
        KBUILD_DEFCONFIG := $(ARCH)_defconfig
endif

ifdef CONFIG_CC_IS_GCC
RETPOLINE_CFLAGS	:= $(call cc-option,-mindirect-branch=thunk-extern -mindirect-branch-register)
RETPOLINE_VDSO_CFLAGS	:= $(call cc-option,-mindirect-branch=thunk-inline -mindirect-branch-register)
endif
ifdef CONFIG_CC_IS_CLANG
RETPOLINE_CFLAGS	:= -mretpoline-external-thunk
RETPOLINE_VDSO_CFLAGS	:= -mretpoline
endif
RETPOLINE_CFLAGS	+= $(call cc-option,-mindirect-branch-cs-prefix)

ifdef CONFIG_MITIGATION_RETHUNK
RETHUNK_CFLAGS		:= -mfunction-return=thunk-extern
RETHUNK_RUSTFLAGS	:= -Zfunction-return=thunk-extern
RETPOLINE_CFLAGS	+= $(RETHUNK_CFLAGS)
RETPOLINE_RUSTFLAGS	+= $(RETHUNK_RUSTFLAGS)
endif

export RETHUNK_CFLAGS
export RETHUNK_RUSTFLAGS
export RETPOLINE_CFLAGS
export RETPOLINE_RUSTFLAGS
export RETPOLINE_VDSO_CFLAGS

# For gcc stack alignment is specified with -mpreferred-stack-boundary,
# clang has the option -mstack-alignment for that purpose.
ifneq ($(call cc-option, -mpreferred-stack-boundary=4),)
      cc_stack_align4 := -mpreferred-stack-boundary=2
      cc_stack_align8 := -mpreferred-stack-boundary=3
else ifneq ($(call cc-option, -mstack-alignment=16),)
      cc_stack_align4 := -mstack-alignment=4
      cc_stack_align8 := -mstack-alignment=8
endif

# How to compile the 16-bit code.  Note we always compile for -march=i386;
# that way we can complain to the user if the CPU is insufficient.
REALMODE_CFLAGS	:= -std=gnu11 -m16 -g -Os -DDISABLE_BRANCH_PROFILING -D__DISABLE_EXPORTS \
		   -Wall -Wstrict-prototypes -march=i386 -mregparm=3 \
		   -fno-strict-aliasing -fomit-frame-pointer -fno-pic \
		   -mno-mmx -mno-sse $(call cc-option,-fcf-protection=none)

REALMODE_CFLAGS += -ffreestanding
REALMODE_CFLAGS += -fno-stack-protector
REALMODE_CFLAGS += -Wno-address-of-packed-member
REALMODE_CFLAGS += $(cc_stack_align4)
REALMODE_CFLAGS += $(CLANG_FLAGS)
ifdef CONFIG_CC_IS_CLANG
REALMODE_CFLAGS += -Wno-gnu
endif
export REALMODE_CFLAGS

# BITS is used as extension for files which are available in a 32 bit
# and a 64 bit version to simplify shared Makefiles.
# e.g.: obj-y += foo_$(BITS).o
export BITS

ifdef CONFIG_CC_OPTIMIZE_FOR_PERFORMANCE
KBUILD_CFLAGS += -O3 $(call cc-option,-fivopts)
KBUILD_RUSTFLAGS += -Copt-level=3
endif

#
# Prevent GCC from generating any FP code by mistake.
#
# This must happen before we try the -mpreferred-stack-boundary, see:
#
#    https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53383
#
KBUILD_CFLAGS += -mno-sse -mno-mmx -mno-sse2 -mno-3dnow -mno-avx -mno-avx2 -fno-tree-vectorize
KBUILD_RUSTFLAGS += --target=$(objtree)/scripts/target.json
KBUILD_RUSTFLAGS += -Ctarget-feature=-sse,-sse2,-sse3,-ssse3,-sse4.1,-sse4.2,-avx,-avx2

#
# CFLAGS for compiling floating point code inside the kernel.
#
CC_FLAGS_FPU := -msse -msse2
ifdef CONFIG_CC_IS_GCC
# Stack alignment mismatch, proceed with caution.
# GCC < 7.1 cannot compile code using `double` and -mpreferred-stack-boundary=3
# (8B stack alignment).
# See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53383
#
# The "-msse" in the first argument is there so that the
# -mpreferred-stack-boundary=3 build error:
#
#  -mpreferred-stack-boundary=3 is not between 4 and 12
#
# can be triggered. Otherwise gcc doesn't complain.
CC_FLAGS_FPU += -mhard-float
CC_FLAGS_FPU += $(call cc-option,-msse -mpreferred-stack-boundary=3,-mpreferred-stack-boundary=4)
endif

ifeq ($(CONFIG_X86_KERNEL_IBT),y)
#
# Kernel IBT has S_CET.NOTRACK_EN=0, as such the compilers must not generate
# NOTRACK prefixes. Current generation compilers unconditionally employ NOTRACK
# for jump-tables, as such, disable jump-tables for now.
#
# (jump-tables are implicitly disabled by RETPOLINE)
#
#   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=104816
#
KBUILD_CFLAGS += $(call cc-option,-fcf-protection=branch -fno-jump-tables)
KBUILD_RUSTFLAGS += -Zcf-protection=branch -Zno-jump-tables
else
KBUILD_CFLAGS += $(call cc-option,-fcf-protection=none)
endif

ifeq ($(CONFIG_X86_32),y)
        BITS := 32
        UTS_MACHINE := i386
        CHECKFLAGS += -D__i386__

        KBUILD_AFLAGS += -m32
        KBUILD_CFLAGS += -m32

        KBUILD_CFLAGS += -msoft-float -mregparm=3 -freg-struct-return

        # Never want PIC in a 32-bit kernel, prevent breakage with GCC built
        # with nonstandard options
        KBUILD_CFLAGS += -fno-pic

        # Align the stack to the register width instead of using the default
        # alignment of 16 bytes. This reduces stack usage and the number of
        # alignment instructions.
        KBUILD_CFLAGS += $(cc_stack_align4)

        # CPU-specific tuning. Anything which can be shared with UML should go here.
        include $(srctree)/arch/x86/Makefile_32.cpu
        KBUILD_CFLAGS += $(cflags-y)

        # temporary until string.h is fixed
        KBUILD_CFLAGS += -ffreestanding

    ifeq ($(CONFIG_STACKPROTECTOR),y)
        ifeq ($(CONFIG_SMP),y)
            KBUILD_CFLAGS += -mstack-protector-guard-reg=fs \
                             -mstack-protector-guard-symbol=__ref_stack_chk_guard
        else
            KBUILD_CFLAGS += -mstack-protector-guard=global
        endif
    endif
else
        BITS := 64
        UTS_MACHINE := x86_64
        CHECKFLAGS += -D__x86_64__

        KBUILD_AFLAGS += -m64
        KBUILD_CFLAGS += -m64

        # Align jump targets to 1 byte, not the default 16 bytes:
        KBUILD_CFLAGS += $(call cc-option,-falign-jumps=1)

        # Pack loops tightly as well:
        KBUILD_CFLAGS += $(call cc-option,-falign-loops=1)

        # Don't autogenerate traditional x87 instructions
        KBUILD_CFLAGS += -mno-80387
        KBUILD_CFLAGS += $(call cc-option,-mno-fp-ret-in-387)

        # By default gcc and clang use a stack alignment of 16 bytes for x86.
        # However the standard kernel entry on x86-64 leaves the stack on an
        # 8-byte boundary. If the compiler isn't informed about the actual
        # alignment it will generate extra alignment instructions for the
        # default alignment which keep the stack *mis*aligned.
        # Furthermore an alignment to the register width reduces stack usage
        # and the number of alignment instructions.
        KBUILD_CFLAGS += $(cc_stack_align8)

	# Use -mskip-rax-setup if supported.
	KBUILD_CFLAGS += $(call cc-option,-mskip-rax-setup)

ifdef CONFIG_X86_NATIVE_CPU
          KBUILD_CFLAGS += -march=native
          KBUILD_RUSTFLAGS += -Ctarget-cpu=native
endif

ifdef CONFIG_MNATIVE_INTEL
          KBUILD_CFLAGS += -march=native
          KBUILD_RUSTFLAGS += -Ctarget-cpu=native
endif

ifdef CONFIG_MNATIVE_AMD
          KBUILD_CFLAGS += -march=native
          KBUILD_RUSTFLAGS += -Ctarget-cpu=native
endif

ifdef CONFIG_MK8
          KBUILD_CFLAGS += -march=k8
          KBUILD_RUSTFLAGS += -Ctarget-cpu=k8
endif

ifdef CONFIG_MK8SSE3
          KBUILD_CFLAGS += -march=k8-sse3
          KBUILD_RUSTFLAGS += -Ctarget-cpu=k8-sse3
endif

ifdef CONFIG_MK10
          KBUILD_CFLAGS += -march=amdfam10
          KBUILD_RUSTFLAGS += -Ctarget-cpu=amdfam10
endif

ifdef CONFIG_MBARCELONA
          KBUILD_CFLAGS += -march=barcelona
          KBUILD_RUSTFLAGS += -Ctarget-cpu=barcelona
endif

ifdef CONFIG_MBOBCAT
          KBUILD_CFLAGS += -march=btver1
          KBUILD_RUSTFLAGS += -Ctarget-cpu=btver1
endif

ifdef CONFIG_MJAGUAR
          KBUILD_CFLAGS += -march=btver2
          KBUILD_RUSTFLAGS += -Ctarget-cpu=btver2
endif

ifdef CONFIG_MBULLDOZER
          KBUILD_CFLAGS += -march=bdver1
          KBUILD_RUSTFLAGS += -Ctarget-cpu=bdver1
endif

ifdef CONFIG_MPILEDRIVER
          KBUILD_CFLAGS += -march=bdver2 -mno-tbm
          KBUILD_RUSTFLAGS += -Ctarget-cpu=bdver2 -mno-tbm
endif

ifdef CONFIG_MSTEAMROLLER
          KBUILD_CFLAGS += -march=bdver3 -mno-tbm
          KBUILD_RUSTFLAGS += -Ctarget-cpu=bdver3 -mno-tbm
endif

ifdef CONFIG_MEXCAVATOR
          KBUILD_CFLAGS += -march=bdver4 -mno-tbm
          KBUILD_RUSTFLAGS += -Ctarget-cpu=bdver4 -mno-tbm
endif

ifdef CONFIG_MZEN
          KBUILD_CFLAGS += -march=znver1
          KBUILD_RUSTFLAGS += -Ctarget-cpu=znver1
endif

ifdef CONFIG_MZEN2
          KBUILD_CFLAGS += -march=znver2
          KBUILD_RUSTFLAGS += -Ctarget-cpu=znver2
endif

ifdef CONFIG_MZEN3
          KBUILD_CFLAGS += -march=znver3
          KBUILD_RUSTFLAGS += -Ctarget-cpu=znver3
endif

ifdef CONFIG_MZEN4
          KBUILD_CFLAGS += -march=znver4
          KBUILD_RUSTFLAGS += -Ctarget-cpu=znver4
endif

ifdef CONFIG_MZEN5
          KBUILD_CFLAGS += -march=znver5
          KBUILD_RUSTFLAGS += -Ctarget-cpu=znver5
endif

ifdef CONFIG_MPSC
          KBUILD_CFLAGS += -march=nocona
          KBUILD_RUSTFLAGS += -Ctarget-cpu=nocona
endif

ifdef CONFIG_MCORE2
          KBUILD_CFLAGS += -march=core2
          KBUILD_RUSTFLAGS += -Ctarget-cpu=core2
endif

ifdef CONFIG_MNEHALEM
          KBUILD_CFLAGS += -march=nehalem
          KBUILD_RUSTFLAGS += -Ctarget-cpu=nehalem
endif

ifdef CONFIG_MWESTMERE
          KBUILD_CFLAGS += -march=westmere
          KBUILD_RUSTFLAGS += -Ctarget-cpu=westmere
endif

ifdef CONFIG_MSILVERMONT
          KBUILD_CFLAGS += -march=silvermont
          KBUILD_RUSTFLAGS += -Ctarget-cpu=silvermont
endif

ifdef CONFIG_MGOLDMONT
          KBUILD_CFLAGS += -march=goldmont
          KBUILD_RUSTFLAGS += -Ctarget-cpu=goldmont
endif

ifdef CONFIG_MGOLDMONTPLUS
          KBUILD_CFLAGS += -march=goldmont-plus
          KBUILD_RUSTFLAGS += -Ctarget-cpu=goldmont-plus
endif

ifdef CONFIG_MSANDYBRIDGE
          KBUILD_CFLAGS += -march=sandybridge
          KBUILD_RUSTFLAGS += -Ctarget-cpu=sandybridge
endif

ifdef CONFIG_MIVYBRIDGE
          KBUILD_CFLAGS += -march=ivybridge
          KBUILD_RUSTFLAGS += -Ctarget-cpu=ivybridge
endif

ifdef CONFIG_MHASWELL
          KBUILD_CFLAGS += -march=haswell
          KBUILD_RUSTFLAGS += -Ctarget-cpu=haswell
endif

ifdef CONFIG_MBROADWELL
          KBUILD_CFLAGS += -march=broadwell
          KBUILD_RUSTFLAGS += -Ctarget-cpu=broadwell
endif

ifdef CONFIG_MSKYLAKE
          KBUILD_CFLAGS += -march=skylake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=skylake
endif

ifdef CONFIG_MSKYLAKEX
          KBUILD_CFLAGS += -march=skylake-avx512
          KBUILD_RUSTFLAGS += -Ctarget-cpu=skylake-avx512
endif

ifdef CONFIG_MCANNONLAKE
          KBUILD_CFLAGS += -march=connonlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=connonlike
endif

ifdef CONFIG_MICELAKE_CLIENT
          KBUILD_CFLAGS += -march=icelake-client
          KBUILD_RUSTFLAGS += -Ctarget-cpu=icelake-client
endif

ifdef CONFIG_MICELAKE_SERVER
          KBUILD_CFLAGS += -march=icelake-server
          KBUILD_RUSTFLAGS += -Ctarget-cpu=icelake-server
endif

ifdef CONFIG_MCOOPERLAKE
          KBUILD_CFLAGS += -march=cooperlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=coolerlake
endif

ifdef CONFIG_MCASCADELAKE
          KBUILD_CFLAGS += -march=cascadelake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=cascadelake
endif

ifdef CONFIG_MTIGERLAKE
          KBUILD_CFLAGS += -march=tigerlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=tigerlake
endif

ifdef CONFIG_MSAPPHIRERAPIDS
          KBUILD_CFLAGS += -march=sapphirerapids
          KBUILD_RUSTFLAGS += -Ctarget-cpu=sapphirerapids
endif

ifdef CONFIG_MROCKETLAKE
          KBUILD_CFLAGS += -march=rocketlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=rocketlake
endif

ifdef CONFIG_MALDERLAKE
          KBUILD_CFLAGS += -march=alderlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=alderlake
endif

ifdef CONFIG_MRAPTORLAKE
          KBUILD_CFLAGS += -march=raptorlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=raptorlake
endif

ifdef CONFIG_MMETEORLAKE
          KBUILD_CFLAGS += -march=meteorlake
          KBUILD_RUSTFLAGS += -Ctarget-cpu=meteorlake
endif

ifdef CONFIG_MEMERALDRAPIDS
          KBUILD_CFLAGS += -march=emeraldrapids
          KBUILD_RUSTFLAGS += -Ctarget-cpu=emeraldrapids
endif

ifdef CONFIG_X86_64_NATIVE
        KBUILD_CFLAGS += -march=native
        KBUILD_RUSTFLAGS += -Ctarget-cpu=native
else
        KBUILD_CFLAGS += -march=x86-64 -mtune=generic
        KBUILD_RUSTFLAGS += -Ctarget-cpu=x86-64 -Ztune-cpu=generic
endif

        KBUILD_CFLAGS += -mno-red-zone
        KBUILD_CFLAGS += -mcmodel=kernel
        KBUILD_RUSTFLAGS += -Cno-redzone=y
        KBUILD_RUSTFLAGS += -Ccode-model=kernel
endif

#
# If the function graph tracer is used with mcount instead of fentry,
# '-maccumulate-outgoing-args' is needed to prevent a GCC bug
# (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=42109)
#
ifdef CONFIG_FUNCTION_GRAPH_TRACER
  ifndef CONFIG_HAVE_FENTRY
	ACCUMULATE_OUTGOING_ARGS := 1
  endif
endif

ifeq ($(ACCUMULATE_OUTGOING_ARGS), 1)
	# This compiler flag is not supported by Clang:
	KBUILD_CFLAGS += $(call cc-option,-maccumulate-outgoing-args,)
endif

# Workaround for a gcc prelease that unfortunately was shipped in a suse release
KBUILD_CFLAGS += -Wno-sign-compare
#
KBUILD_CFLAGS += -fno-asynchronous-unwind-tables

# Avoid indirect branches in kernel to deal with Spectre
ifdef CONFIG_MITIGATION_RETPOLINE
  KBUILD_CFLAGS += $(RETPOLINE_CFLAGS)
  KBUILD_RUSTFLAGS += $(RETPOLINE_RUSTFLAGS)
  # Additionally, avoid generating expensive indirect jumps which
  # are subject to retpolines for small number of switch cases.
  # LLVM turns off jump table generation by default when under
  # retpoline builds, however, gcc does not for x86. This has
  # only been fixed starting from gcc stable version 8.4.0 and
  # onwards, but not for older ones. See gcc bug #86952.
  ifndef CONFIG_CC_IS_CLANG
    KBUILD_CFLAGS += -fno-jump-tables
  endif
endif

ifdef CONFIG_MITIGATION_SLS
  KBUILD_CFLAGS += -mharden-sls=all
endif

ifdef CONFIG_CALL_PADDING
PADDING_CFLAGS := -fpatchable-function-entry=$(CONFIG_FUNCTION_PADDING_BYTES),$(CONFIG_FUNCTION_PADDING_BYTES)
KBUILD_CFLAGS += $(PADDING_CFLAGS)
export PADDING_CFLAGS

PADDING_RUSTFLAGS := -Zpatchable-function-entry=$(CONFIG_FUNCTION_PADDING_BYTES),$(CONFIG_FUNCTION_PADDING_BYTES)
KBUILD_RUSTFLAGS += $(PADDING_RUSTFLAGS)
export PADDING_RUSTFLAGS
endif

KBUILD_LDFLAGS += -m elf_$(UTS_MACHINE)

ifdef CONFIG_X86_NEED_RELOCS
LDFLAGS_vmlinux := --emit-relocs --discard-none
else
LDFLAGS_vmlinux :=
endif

#
# The 64-bit kernel must be aligned to 2MB.  Pass -z max-page-size=0x200000 to
# the linker to force 2MB page size regardless of the default page size used
# by the linker.
#
ifdef CONFIG_X86_64
LDFLAGS_vmlinux += -z max-page-size=0x200000
endif


archscripts: scripts_basic
	$(Q)$(MAKE) $(build)=arch/x86/tools relocs

###
# Syscall table generation

archheaders:
	$(Q)$(MAKE) $(build)=arch/x86/entry/syscalls all

###
# Kernel objects

libs-y  += arch/x86/lib/

# drivers-y are linked after core-y
drivers-$(CONFIG_MATH_EMULATION) += arch/x86/math-emu/
drivers-$(CONFIG_PCI)            += arch/x86/pci/

# suspend and hibernation support
drivers-$(CONFIG_PM) += arch/x86/power/

drivers-$(CONFIG_VIDEO) += arch/x86/video/

####
# boot loader support. Several targets are kept for legacy purposes

boot := arch/x86/boot

BOOT_TARGETS = bzdisk fdimage fdimage144 fdimage288 hdimage isoimage

PHONY += bzImage $(BOOT_TARGETS)

# Default kernel to build
all: bzImage

# KBUILD_IMAGE specify target image being built
KBUILD_IMAGE := $(boot)/bzImage

bzImage: vmlinux
ifeq ($(CONFIG_X86_DECODER_SELFTEST),y)
	$(Q)$(MAKE) $(build)=arch/x86/tools posttest
endif
	$(Q)$(MAKE) $(build)=$(boot) $(KBUILD_IMAGE)
	$(Q)mkdir -p $(objtree)/arch/$(UTS_MACHINE)/boot
	$(Q)ln -fsn ../../x86/boot/bzImage $(objtree)/arch/$(UTS_MACHINE)/boot/$@

$(BOOT_TARGETS): vmlinux
	$(Q)$(MAKE) $(build)=$(boot) $@

PHONY += install
install:
	$(call cmd,install)

vdso-install-$(CONFIG_X86_64)		+= arch/x86/entry/vdso/vdso64.so.dbg
vdso-install-$(CONFIG_X86_X32_ABI)	+= arch/x86/entry/vdso/vdsox32.so.dbg
vdso-install-$(CONFIG_COMPAT_32)	+= arch/x86/entry/vdso/vdso32.so.dbg

archprepare: checkbin
checkbin:
ifdef CONFIG_MITIGATION_RETPOLINE
ifeq ($(RETPOLINE_CFLAGS),)
	@echo "You are building kernel with non-retpoline compiler." >&2
	@echo "Please update your compiler." >&2
	@false
endif
endif

ifdef CONFIG_UNWINDER_ORC
orc_hash_h := arch/$(SRCARCH)/include/generated/asm/orc_hash.h
orc_hash_sh := $(srctree)/scripts/orc_hash.sh
targets += $(orc_hash_h)
quiet_cmd_orc_hash = GEN     $@
      cmd_orc_hash = mkdir -p $(dir $@); \
		     $(CONFIG_SHELL) $(orc_hash_sh) < $< > $@
$(orc_hash_h): $(srctree)/arch/x86/include/asm/orc_types.h $(orc_hash_sh) FORCE
	$(call if_changed,orc_hash)
archprepare: $(orc_hash_h)
endif

archclean:
	$(Q)rm -rf $(objtree)/arch/i386
	$(Q)rm -rf $(objtree)/arch/x86_64

define archhelp
  echo  '* bzImage		- Compressed kernel image (arch/x86/boot/bzImage)'
  echo  '  install		- Install kernel using (your) ~/bin/$(INSTALLKERNEL) or'
  echo  '			  (distribution) /sbin/$(INSTALLKERNEL) or install to '
  echo  '			  $$(INSTALL_PATH) and run lilo'
  echo  ''
  echo  '  fdimage		- Create 1.4MB boot floppy image (arch/x86/boot/fdimage)'
  echo  '  fdimage144		- Create 1.4MB boot floppy image (arch/x86/boot/fdimage)'
  echo  '  fdimage288		- Create 2.8MB boot floppy image (arch/x86/boot/fdimage)'
  echo  '  hdimage		- Create a BIOS/EFI hard disk image (arch/x86/boot/hdimage)'
  echo  '  isoimage		- Create a boot CD-ROM image (arch/x86/boot/image.iso)'
  echo  '			  bzdisk/fdimage*/hdimage/isoimage also accept:'
  echo  '			  FDARGS="..."  arguments for the booted kernel'
  echo  '			  FDINITRD=file initrd for the booted kernel'

endef
