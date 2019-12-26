############## User Defined ############################
DEPOT =/home/qing/Desktop/Work/TI/AM335x/TIRTOS
cfgfile = main.cfg
srcs = main.c  app.c
EXEC_DIR = .
EXEC_PROG = demo
IMG_LOAD_TYPE = NONE
IMG_LOAD_ADDR = 0x80000000
#libs = 
PDK_INSTALL_DIR  =$(DEPOT)/pdk_am335x_1_0_16
XDC_INSTALL_DIR  =$(DEPOT)/xdctools_3_55_02_22_core
BIOS_INSTALL_DIR =$(DEPOT)/bios_6_76_02_02

gnu.targets.arm.A8F =/home/qing/Desktop/Work/TI/RTOS/ccs/tools/compiler/gcc-arm-none-eabi-7-2018-q2-update
## =================== No need to modify ===================##
## Replace all *.c in the $(src) content to *.o
## and add prefix before this name.
objs = $(addprefix bin/$(PROFILE)/obj/,$(patsubst %.c,%.oa8fg,$(srcs)))
CONFIG =  bin/$(PROFILE)/configuro
PKGPATH := $(BIOS_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(XDC_INSTALL_DIR)/packages
PKGPATH := $(PKGPATH)+$(PDK_INSTALL_DIR)/packages
#############################################################
SOCDEF =	SOC_AM335x 
SOC    =	AM335x
LDLIBS = -L$(BIOS_INSTALL_DIR)/packages/gnu/targets/arm/libs/install-native/arm-none-eabi/lib/hard -lstdc++ -lgcc -lm -lgcc -lc -lrdimon -lnosys

ifeq ($(MODULE_LIST),)
  MODULE_LIST= gpio i2c uart spi
endif

ifeq ($(SOC_TYPE),) 
  SOC_TYPE= am335x
endif	

ifeq ($(CORE_NAME),)
  CORE_NAME=a8
endif

MODULE_PATH=$(PDK_INSTALL_DIR)/packages/ti/drv/$(DRV_MOD)/lib/$(SOC_TYPE)/$(CORE_NAME)/release
libs = $(foreach DRV_MOD,$(MODULE_LIST),$(wildcard $(MODULE_PATH)/*.aa8fg))
### For board library
libs += $(wildcard $(PDK_INSTALL_DIR)/packages/ti/board/lib/evmAM335x/$(CORE_NAME)/release/*.aa8fg)	
### For csl library
libs += $(wildcard $(PDK_INSTALL_DIR)/packages/ti/csl/lib/$(SOC_TYPE)/$(CORE_NAME)/release/*.aa8fg)
### For osal library
libs += $(wildcard $(PDK_INSTALL_DIR)/packages/ti/osal/lib/tirtos/$(SOC_TYPE)/$(CORE_NAME)/release/*.aa8fg)
### for utils library
libs += $(wildcard $(PDK_INSTALL_DIR)/packages/ti/utils/profiling/lib/$(CORE_NAME)/release/*.aa8fg)
################## tools  #################################### 
CP      = cp
ECHO    = echo
MKDIR   = mkdir -p
RM      = rm -f
RMDIR   = rm -rf
################# toolchain macros ###########################
TOOLCHAIN_PATH =  $(gnu.targets.arm.A8F)

CCPROFILE_debug = -g -ggdb -D_DEBUG_=1
CCPROFILE_release = -O2
LDPROFILE_debug = -g -ggdb
LDPROFILE_release =

CC = $(TOOLCHAIN_PATH)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep
LD = $(TOOLCHAIN_PATH)/bin/arm-none-eabi-gcc
OBJCOPY	= $(TOOLCHAIN_PATH)/bin/arm-none-eabi-objcopy

CPPFLAGS = 
CFLAGS = -std=c99 -Wall -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections $(CCPROFILE_$(PROFILE)) @$(CONFIG)/compiler.opt -I. -D$(SOCDEF) -D$(SOC) -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_target_name__=A8F

LDFLAGS = $(LDPROFILE_$(PROFILE)) -Werror -mfloat-abi=hard -nostartfiles -Wl,-static -Wl,--gc-sections -Wl,-Map=$(@D)/obj/$(@F).map -Wl,-T,$(CONFIG)/linker.cmd --specs=nano.specs

.PRECIOUS: %/compiler.opt %/linker.cmd

PHONY = all host-tools debug release App
all: debug release

debug:
	$(MAKE) PROFILE=debug  App

release:
	$(MAKE) PROFILE=release  App

App: bin/$(PROFILE)/$(EXEC_PROG)_ti.bin
	@mv bin/$(PROFILE)/$(EXEC_PROG)_ti.bin  bin/$(PROFILE)/app

bin/$(PROFILE)/$(EXEC_PROG)_ti.bin: host-tools	bin/$(PROFILE)/$(EXEC_PROG).bin
	@echo 'generate the $@...'
	@./tools/tiimage $(IMG_LOAD_ADDR) $(IMG_LOAD_TYPE) bin/$(PROFILE)/$(EXEC_PROG).bin $@
	@echo '[Done!]'

bin/$(PROFILE)/$(EXEC_PROG).bin: bin/$(PROFILE)/$(EXEC_PROG)
	@$(ECHO) "#"
	@$(ECHO) "# Generate $@ ..."
	@$(OBJCOPY) -O binary bin/$(PROFILE)/$(EXEC_PROG)  $@

bin/$(PROFILE)/$(EXEC_PROG): $(objs) $(libs) $(CONFIG)/linker.cmd
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(LD) $(LDFLAGS) -o $@ $(objs) \
		-Wl,--start-group $(libs) $(LDLIBS) -Wl,--end-group

bin/$(PROFILE)/obj/%.oa8fg:	 %.c $(CONFIG)/compiler.opt
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) -p bin/$(PROFILE)/obj
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $<

%/linker.cmd %/compiler.opt: $(CONFIG)/.config ;
$(CONFIG)/.config: $(cfgfile) ./Common/config.bld
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	$(XDC_INSTALL_DIR)/xs --xdcpath="$(subst +,;,$(PKGPATH))" \
            xdc.tools.configuro -o $(CONFIG) \
            -t gnu.targets.arm.A8F \
            -c $(TOOLCHAIN_PATH) \
            -p ti.platforms.evmAM3359 \
            -b ./Common/config.bld -r $(PROFILE) \
			--cfgArgs "{ configBld: \"Common/config.bld\" }" \
            $(cfgfile)
	@$(ECHO) "" > $@


#########################################
### host tools.
host-tools:
	@$(MAKE) -C ./tools

host-tools_clean:
	@$(MAKE) -C ./tools clean

PHONY += install help clean host-tools_clean
install:
	@$(ECHO) "#"
	@$(ECHO) "# Making $@ ..."
	@$(MKDIR) $(EXEC_DIR)/debug
	$(CP) bin/debug/$(EXEC_PROG) $(EXEC_DIR)/debug
	@$(MKDIR) $(EXEC_DIR)/release
	$(CP) bin/release/$(EXEC_PROG) $(EXEC_DIR)/release

help:
	@$(ECHO) "make                   # build executable"
	@$(ECHO) "make clean             # clean everything"

clean:	host-tools_clean
	$(RMDIR) bin

.PHONY: $(PHONY)
