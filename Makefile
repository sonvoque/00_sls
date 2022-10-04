
PROJECT_SOURCEFILES += util.c aes_lib.c

CONTIKI_PROJECT = udp-echo-server

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

MODULES +=  core/net/mac core/net core/net/mac/sicslowmac core/net/mac/contikimac core/net/llsec/noncoresec


MAEK_WITH_SENSOR = 0
ifeq ($(MAEK_WITH_SENSOR),1)
CFLAGS += -DCC2538DK_HAS_SENSOR=1
CONTIKI_TARGET_SOURCEFILES += tsl256x.c bmpx8x.c si7021.c
MODULES += platform/cc2538dk/dev
endif



ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+= -DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif


all: $(CONTIKI_PROJECT)

CONTIKI = ../../..
CONTIKI_WITH_IPV6 = 1
#CFLAGS += -DUIP_CONF_ND6_SEND_NA=1
include $(CONTIKI)/Makefile.include