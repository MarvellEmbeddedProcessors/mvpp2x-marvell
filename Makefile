M:=$(CURDIR)

ifndef KDIR
$(error KDIR is not set. must declare kernel directory. example: export KDIR=~/linux-4.0.4/kernel/)
endif

obj-m := mvpp2x.o
mvpp2x-objs := mvpp2_ethtool.o mvpp2_hw.o mvpp2_main.o

ccflags-y := -I$(M)
ccflags-y += -I${KDIR}/include
ccflags-y += -DDEBUG

all:
	make -C ${KDIR} M=$(M) modules

clean:
	make -C ${KDIR} SUBDIRS=$(M) clean
