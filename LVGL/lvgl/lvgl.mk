LVGL_PATH ?= ${shell pwd}/lvgl

ASM_SOURCES += $(shell find $(LVGL_PATH)/src -type f -name '*.S')
C_SOURCES += $(shell find $(LVGL_PATH)/src -type f -name '*.c')
CXXEXT := .cpp
CXXSRCS += $(shell find $(LVGL_PATH)/src -type f -name '*${CXXEXT}')

AFLAGS += "-I$(LVGL_PATH)"
CFLAGS += "-I$(LVGL_PATH)"
CXXFLAGS += "-I$(LVGL_PATH)"
