ALL-y = tiimage

CC		:= cc
LD		:= ld
AR		:= ar
CFLAGS  := -Wall -O2 
LDFLAGS :=



# That's our default target when none is given on the command line
PHONY := all
all:		$(ALL-y);


 
tiimage: tiimage.c
	${CC} ${CFLAGS} ${LDFLAGS} -o $@ $^		

	



PHONY += clean
clean :
	@-rm -f ${ALL-y} *.o 

.PHONY: $(PHONY)
