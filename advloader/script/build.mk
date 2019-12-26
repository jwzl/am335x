prefix := .
src := $(patsubst $(prefix)/%,%,$(obj))

PHONY := __build
__build:

obj-y :=
lib-y := 
subdir-y := 
extra-y  := 

include script/include.mk
include $(srctree)/config.mk

# include the makefile
kbuild_dir := $(if $(filter /%,$(src)),$(src),$(srctree)/$(src))
kbuild_file := $(if $(wildcard $(kbuild_dir)/Kbuild),$(kbuild_dir)/Kbuild,$(kbuild_dir)/Makefile)
include $(kbuild_file) 

ifndef obj
$(warning kbuild: build.mk is included improperly)
endif

ifneq ($(strip $(lib-y)),)
lib-target := $(obj)/lib.a
endif

ifneq ($(strip $(obj-y) $(lib-target)),)
builtin-target := $(obj)/built-in.o
endif

####################################
__subdir-y := $(patsubst %/,%,$(filter %/, $(obj-y)))
subdir-y	+= $(__subdir-y)
obj-y	   := $(patsubst %/,%/built-in.o, $(obj-y))	
# $(subdir-obj-y) is the list of objects in $(obj-y) which uses dir/ to
# tell kbuild to descend
subdir-obj-y := $(filter %/built-in.o, $(obj-y))
lib-y 		:= $(filter-out $(obj-y), $(sort $(lib-y)))

# Add subdir path
obj-y		 	 := $(addprefix $(obj)/,$(obj-y))
lib-y 			 := $(addprefix $(obj)/,$(lib-y))	
subdir-y		 := $(addprefix $(obj)/,$(subdir-y))
subdir-obj-y 	 := $(addprefix $(obj)/,$(subdir-obj-y))
extra-y			 := $(addprefix $(obj)/,$(extra-y))


__build: $(if $(KBUILD_BUILTIN),$(builtin-target) $(lib-target) $(extra-y)) \
		$(subdir-y)
	@:


#
# Rule to compile a set of .o files into one .o file
#
ifdef builtin-target
# If the list of objects to link is empty, just create an empty built-in.o
cmd_link_o_target = $(if $(strip $(obj-y)),\
				@set -e;\
				$(LD) $(ld_flags) -r -o $@ $(filter $(obj-y), $^),\
				rm -f $@; $(AR) rcs$(KBUILD_ARFLAGS) $@)


$(builtin-target): $(obj-y)
	$(call echo-cmd, cmd_link_o_target)
	$(call cmd_link_o_target)

endif

#
# Rule to compile a set of .o files into one .a file
#
ifdef lib-target
cmd_link_l_target = rm -rf $@; $(AR) rcs$(KBUILD_ARFLAGS) $@ $(lib-y)
$(lib-target): $(lib-y)
	$(call cmd_link_l_target)	

endif

cmd_cc_o_c = $(CC) $(c_flags) -c -o $@ $<
define rule_cc_o_c
	$(call echo-cmd, cmd_cc_o_c) $(cmd_cc_o_c)
endef

# generate the .o file
$(obj)/%.o: $(src)/%.c 
	$(call rule_cc_o_c)

cmd_as_o_S	= $(CC) $(a_flags) -c -o $@  $<
$(obj)/%.o: $(src)/%.S
	$(call cmd_as_o_S)

# To build objects in subdirs, we need to desend into the directories
$(sort $(subdir-obj-y)): $(subdir-y) ;

PHONY += $(subdir-y)
$(subdir-y):
	$(Q)$(MAKE) $(build)=$@

.PHONY: $(PHONY)
