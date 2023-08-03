## Variables prefixed with "DOWNWARD_" are used to configure the
## build. They can be changed on the command line or set as
## environment variables.

## By default, build in 32-bit mode. Use DOWNWARD_BITWIDTH=64
## to build in 64-bit mode and DOWNWARD_BITWIDTH=native to use
## the native bitness of the OS.
DOWNWARD_BITWIDTH ?= 32

ifeq ($(DOWNWARD_BITWIDTH), 32)
    BITWIDTHOPT = -m32
else ifeq ($(DOWNWARD_BITWIDTH), 64)
    BITWIDTHOPT = -m64
else ifneq ($(DOWNWARD_BITWIDTH), native)
    $(error Bad value for DOWNWARD_BITWIDTH)
endif

CXXFLAGS = -I.
CXXFLAGS += -g
#CFLAGS += -pg
CXXFLAGS += -Wall
CXXFLAGS += --pedantic
CXXFLAGS += -O3
CXXFLAGS += $(BITWIDTHOPT)
#CFLAGS += -DBLISS_DEBUG
CXXFLAGS += -fPIC


SRCS = defs.cc graph.cc partition.cc orbit.cc uintseqhash.cc heap.cc
SRCS += timer.cc utils.cc bliss_C.cc

OBJS = $(addsuffix .o, $(basename $(SRCS)))

GMPOBJS = $(addsuffix g,  $(OBJS))

LIB =
#LIB += /usr/lib/ccmalloc.o -ldl

#CXX = g++
RANLIB = ranlib
AR = ar
BLISSLIB = libbliss.a

gmp:	LIB += -lgmp
gmp:	CFLAGS += -DBLISS_USE_GMP

normal:	bliss
gmp:	bliss_gmp


all:: lib bliss

%.o %.og:	%.cc
	$(CXX) $(CXXFLAGS) -c -o $@ $<

lib: $(OBJS)
	rm -f $(BLISSLIB)
	$(AR) cr $(BLISSLIB) $(OBJS)
	$(RANLIB) $(BLISSLIB)

lib_gmp: $(GMPOBJS)
	rm -f $(BLISSLIB)
	$(AR) cr $(BLISSLIB) $(GMPOBJS)
	$(RANLIB) $(BLISSLIB)

bliss: bliss.o lib $(OBJS)
	$(CXX) $(CXXFLAGS) -o bliss bliss.o $(OBJS) $(LIB)

bliss_gmp: bliss.og lib_gmp $(GMPOBJS)
	$(CXX) $(CXXFLAGS) -o bliss bliss.og $(GMPOBJS) $(LIB)


clean:
	rm -f bliss $(BLISSLIB) $(OBJS) bliss.o $(GMPOBJS) bliss.og

# DO NOT DELETE
