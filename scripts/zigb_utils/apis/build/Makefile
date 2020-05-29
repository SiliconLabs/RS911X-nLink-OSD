# Makefile for WiSeConnect Application
TOP_DIR := $(PWD)
RM=/bin/rm -f
PROGNAME=rsi_wsc_zigb_app
INCLUDES=  -I./ -I/work/colibri-2.6-kernel/include/
LINKS= -lpthread

REF_APPS_SRC =../ref_apps/src
CORE_SRC=../core/src
WLAN_CORE_SRC=../../wlan/core/src
HAL_SRC=../../hal/src
INTF_SRC=../../intf/spi/src
CFLAGS= -g -Os  $(INCLUDES)
CFLAGS+= -I../core/include/
CFLAGS+= -I../../wlan/core/include/
CFLAGS+= -I../ref_apps/include/
CFLAGS+= -I../../wlan/ref_apps/include/
CFLAGS+= -I../../ref_apps/include/
CFLAGS+= -I../../hal/include/
CFLAGS+= -I../../intf/spi/include/
CFLAGS+= -DZB_ENABLE
CFLAGS+= -DZB_API_TEST


OBJS = main.o \
		 $(CORE_SRC)/rsi_zigb_frame_desc.o \
	   $(CORE_SRC)/rsi_zigb_api.o \
	   $(CORE_SRC)/rsi_zigb_delay.o \
	   $(CORE_SRC)/rsi_zigb_utility.o \
	   $(CORE_SRC)/rsi_zigb_app_frame_process.o \
		 $(CORE_SRC)/rsi_zigb_build_frame_descriptor.o \
	   $(CORE_SRC)/rsi_zigb_execute_cmd.o \
		 $(REF_APPS_SRC)/rsi_zigb_app_cb_handler.o \
		 $(REF_APPS_SRC)/rsi_zigb_api_test.o \
		 $(REF_APPS_SRC)/rsi_zigb_config.o \
		 $(REF_APPS_SRC)/zigb_main.o \
 		 $(WLAN_CORE_SRC)/rsi_lib_util.o \
		 $(HAL_SRC)/rsi_hal_mcu_interrupt.o \
		 $(HAL_SRC)/rsi_hal_mcu_ioports.o \
		 $(HAL_SRC)/rsi_hal_mcu_spi.o \
		 $(HAL_SRC)/rsi_hal_mcu_timers.o \
		 $(INTF_SRC)/rsi_spi_frame_rd_wr.o \
		 $(INTF_SRC)/rsi_spi_functs.o \
		 $(INTF_SRC)/rsi_spi_iface_init.o \
		 $(INTF_SRC)/rsi_spi_mem_rd_wr.o \
		 $(INTF_SRC)/rsi_spi_reg_rd_wr.o 

.c.o:
	$(RM) $@
	$(CC) $(CFLAGS)  -o $*.o  -c $*.c

all: $(OBJS)
	$(CC) $(CFLAGS) $(LINKS) -o $(PROGNAME) $(OBJS) 

clean:
	$(RM)  $(OBJS) $(PROGNAME) core *~
