src := $(obj)

PHONY := __clean
__clean:

# Shorthand for $(Q)$(MAKE) scripts/Makefile.clean obj=dir
# Usage:
# $(Q)$(MAKE) $(clean)=dir
clean := -f $(srctree)/script/clean.mk obj

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

__subdir-y := $(patsubst %/,%,$(filter %/, $(obj-y)))
subdir-y	+= $(__subdir-y)

obj-y		 	 := $(addprefix $(obj)/,$(obj-y))
extra-y			 := $(addprefix $(obj)/,$(extra-y))
subdir-y	:= $(addprefix $(obj)/,$(subdir-y))
subdir-y	:= $(foreach f, $(subdir-y), \
				$(if $(wildcard $(srctree)/$f/Makefile),$f))

# build a list of files to remove, usually relative to the current
# directory
__clean-files	:= $(extra-y) $(filter %.o, $(sort $(obj-y))) $(obj)/built-in.o

__clean: $(subdir-y)
	@rm -f $(__clean-files)

PHONY += $(subdir-y)
$(subdir-y):
	$(Q)$(MAKE) $(clean)=$@


.PHONY: $(PHONY)
