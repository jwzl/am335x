Q := @
# echo command.
echo-cmd = $(if $($(1)),\
		echo ' $($(1))';)
###
# Shorthand for $(Q)$(MAKE) -f scripts/Makefile.build obj=
# Usage:
# $(Q)$(MAKE) $(build)=dir
build := -f $(srctree)/script/build.mk obj
