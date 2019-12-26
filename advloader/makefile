KBUILD_SRC = $(PWD)
TOOLCHAIN = /home/qing/Desktop/Work/TI/RTOS/ccs/tools/compiler/gcc-arm-none-eabi-7-2018-q2-update
CROSS_COMPILE = $(TOOLCHAIN)/bin/arm-none-eabi-

IMG_LOAD_TYPE=MMCSD  ##MCSPI for spi images byteswap.
IMG_LOAD_ADDR =0x402F0400

CFLAGS_INTERNAL = -c -mcpu=cortex-a8 -mtune=cortex-a8 -march=armv7-a -mfpu=neon -mfloat-abi=hard -mlong-calls -fdata-sections -funsigned-char -ffunction-sections -Wall -Dgcc -Darmv7a -D SUPPORT_UNALIGNED -DAM335X_FAMILY_BUILD 
LNKFLAGS_INTERNAL = -mfloat-abi=hard -e Entry -u Entry -u __aeabi_uidiv -u __aeabi_idiv -Wl,--gc-sections --specs=nano.specs -Wl,--start-group -lstdc++ -lgcc -lm -lc -lrdimon -Wl,--end-group -O2
################################################################
ifeq ($(KBUILD_SRC),)
	srctree := .
else
	ifeq ($(KBUILD_SRC)/,$(dir $(CURDIR)))
		srctree := ..
	else
		srctree := $(KBUILD_SRC)
	endif
endif

objtree		:= .
src		:= $(srctree)
obj		:= $(objtree)

export srctree objtree 

KBUILD_BUILTIN := 1

export KBUILD_BUILTIN

# That's our default target when none is given on the command line
PHONY := _all
_all:

# We need some generic definitions (do not try to remake the file).
$(srctree)/script/include.mk: ;
include $(srctree)/script/include.mk

include $(srctree)/soc/Makefile
# Make variables (CC, etc...)

AS		= $(CROSS_COMPILE)as
# Always use GNU ld
ifneq ($(shell $(CROSS_COMPILE)ld.bfd -v 2> /dev/null),)
LD		= $(CROSS_COMPILE)ld.bfd
else
LD		= $(CROSS_COMPILE)ld
endif
CC		= $(CROSS_COMPILE)gcc
CPP		= $(CC) -E
AR		= $(CROSS_COMPILE)ar
NM		= $(CROSS_COMPILE)nm
LDR		= $(CROSS_COMPILE)ldr
STRIP		= $(CROSS_COMPILE)strip
OBJCOPY		= $(CROSS_COMPILE)objcopy
OBJDUMP		= $(CROSS_COMPILE)objdump
AWK		= awk
PYTHON		= python

export CROSS_COMPILE AS LD CC CPP AR NM LDR STRIP OBJCOPY OBJDUMP
export MAKE AWK PYTHON

KBUILDINCLUDE  := -Iinclude \
			$(if $(KBUILD_SRC), -I$(srctree)/include)

c_flags		= $(CFLAGS_INTERNAL) $(KBUILDINCLUDE)
a_flags		= $(CFLAGS_INTERNAL) $(KBUILDINCLUDE)
LDFLAGS     = $(LNKFLAGS_INTERNAL) 
## empty ld_flags since it just package a relocalbe .o
ld_flags	= 

export c_flags a_flags ld_flags
########################################
libs-y	:= board/
libs-y	+= soc/
libs-y	+= driver/
libs-y	+= common/
libs-y	+= utils/

libs-y  := $(sort $(libs-y))

build-dirs := $(patsubst %/,%,$(filter %/, $(libs-y))) 
build-alldirs :=$(sort $(build-dirs) $(patsubst %/, %, $(filter %/, $(libs-))))

libs-y  := $(patsubst %/, %/built-in.o, $(libs-y))


ifndef LDSCRIPT
	LDSCRIPT := $(srctree)/soc/am335x/gcc/am335x_boot.lds
endif

ALL-y = MLO 

PHONY += all host-tools
_all: all

all:		$(ALL-y);


MLO: host-tools advboot.bin
	@echo 'generate the MLO...'
	@$(srctree)/tools/tiimage $(IMG_LOAD_ADDR) $(IMG_LOAD_TYPE) advboot.bin $@
	@echo '[Done!]'

cmd_link_mlo ?= $(CC) $(LDFLAGS) -o $@  -Wl,-T $(LDSCRIPT) $(init-y) \
      $(libs-y) $(PLATFORM_LIBS) -Wl,-Map=$@.map

advboot.bin: advboot
	@echo 'generate the $@...'
	$(Q)$(OBJCOPY) -O binary  advboot $@

advboot: $(init-y) $(libs-y) 
	$(Q)echo 'Linking target $@'
	$(call cmd_link_mlo)

$(sort $(init-y) $(libs-y)): $(build-dirs) ;
	
PHONY += $(build-dirs) 
$(build-dirs):
	$(Q)$(MAKE) $(build)=$@

#########################################
### host tools.
host-tools:
	$(Q)$(MAKE) -C $(srctree)/tools

host-tools_clean:
	$(Q)$(MAKE) -C $(srctree)/tools clean
#########################################
## clean
clean := -f $(srctree)/script/clean.mk obj

clean-dirs	:= $(foreach f,$(build-alldirs),$(if $(wildcard $(srctree)/$f/Makefile),$f))
clean-dirs  := $(addprefix _clean_, $(clean-dirs))

PHONY += $(clean-dirs) clean host-tools_clean
clean: $(clean-dirs) host-tools_clean
	@rm -rf  MLO
	@rm -rf advboot advboot.map  advboot.bin

$(clean-dirs):
	$(Q)$(MAKE) $(clean)=$(patsubst _clean_%,%,$@)

# Declare the contents of the .PHONY variable as phony.  We keep that
# information in a variable so we can use it in if_changed and friends.
.PHONY: $(PHONY)
