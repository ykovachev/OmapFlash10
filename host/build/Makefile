################################################################################
# Makfile - OMAPFlash EMCC Driver
################################################################################
OS_TYPE :=

GCC_DIR := "C:\cygwin\bin"

# for now just assume Linux or Windows. You can overwrite if OS_TYPE is passed to makefile
ifeq ($(OS_TYPE),)
	ifeq (,$(findstring Linux,$(shell uname)))
		OS_TYPE := WINDOWS
	else
		OS_TYPE := LINUX
	endif
endif

RM := rm -rf

VPATH += ../../common/src/:../.

C_SRCS += \
board_configuration.c \
bootimg.c \
engine.c \
fastboot.c \
omap_protocol.c \
output.c \
peripheralboot.c \
protocol.c \
adler32.c  \
crc32.c \
memory_partitions.c \


OBJS += \
board_configuration.o \
bootimg.o \
engine.o \
fastboot.o \
omap_protocol.o \
output.o \
peripheralboot.o \
protocol.o \
adler32.o  \
crc32.o \
memory_partitions.o \

C_DEPS += \
board_configuration.d \
bootimg.d \
engine.d \
fastboot.d \
omap_protocol.d \
output.d \
peripheralboot.d \
protocol.d \
adler32.d  \
crc32.d \
memory_partitions.d \


CFLAGS :=
IFLAGS := -I"../../common/inc" 
DFLAGS := -DNO_ZIP
LFLAGS := 

TARGET_NAME := omapflash_linux

ifeq ($(OS_TYPE), LINUX)
CC := gcc
CPP := g++
else
CC := gcc-linux-4.1
CPP := g++-linux-4.1
endif


#### LINUX ####
#ifeq ($(OS_TYPE), LINUX)

LIBS += -lpthread 
#CC := gcc
#CPP := g++

C_SRCS += \
omap_linux.c \
usb_linux.c \
util_linux.c 

OBJS += \
omap_linux.o \
usb_linux.o \
util_linux.o 

C_DEPS += \
omap_linux.d \
usb_linux.d \
util_linux.d 

#endif

#### WINDOWS ####
#ifeq ($(OS_TYPE), WINDOWS)

#LIBS := -ladbwinapi 
#CC := $(GCC_DIR)\gcc-linux
#CPP := $(GCC_DIR)\g++-linux

#C_SRCS += \
#../omap_windows.cpp \
#../usb_windows.c \
#../util_windows.c 

#OBJS += \
#./omap_windows.o \
#./usb_windows.o \
#./util_windows.o 

#C_DEPS += \
#./omap_windows.d \
#./usb_windows.d \
#./util_windows.d 

#endif

# Source file compilation
%.o: %.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	$(CC) $(DFLAGS) $(IFLAGS) -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<" 
	@echo 'Finished building: $<'
	@echo ' '

# All Target
all: $(TARGET_NAME)

# Tool invocations
$(TARGET_NAME): $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C Linker'
	$(CC) $(LFLAGS) -o $(TARGET_NAME)  $(OBJS) $(LIBS)
 	@echo 'Finished building target: $@'
	@echo ' '

	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-@echo 'make Clean'
	-$(RM) $(OBJS) $(TARGET_NAME)
	-@echo 'Finished Clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
