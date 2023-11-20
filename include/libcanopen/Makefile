# Builds Arduino library.
#
# FIXME: Build directories are created every time you run make.
# FIXME: Library rebuild every time.
# FIXME: ar must not update the library but rebuild it.
# FIXME: Have to specify ARCH even for make clean-all.
#
# TODO: Add autodependency generation.
# TODO: Automatic source search.
# TODO: Refactor variables.

LIB_NAME = canopen

LIB_SRC := $(wildcard 					   \
				src/*.c                    \
				src/*.cpp)

LIB_INC = -Iinclude					\

PDO_NUM = 2       # possible to disable some PDOs totally

ifeq ($(ARCH),nano)
MCU = atmega328p
MACH = avr
else ifeq ($(ARCH),mini)
MCU = atmega328p
MACH = avr
else ifeq ($(ARCH),mega)
MCU = atmega1280
MACH = avr
else ifeq ($(strip $(ARCH)),)
$(error Please specify for which Arduino board you build the library, for example ARCH=nano)
else
$(error Unknown Arduino board: $(ARCH))
endif

BUILD_DIR = build
ARCH_BUILD_DIR = $(BUILD_DIR)/$(ARCH)
SRC_DIR = src

ifeq ($(MACH),avr)
LIB_INC += -Iinclude/avr \
		   -I../arduino/include		\
		   -I../arduino/include/SPI

LIB_SRC += $(wildcard 					\
				src/arch/$(MACH)/*.c    \
				src/arch/$(MACH)/*.cpp)
endif
OBJS := $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(patsubst $(SRC_DIR)/%,$(ARCH_BUILD_DIR)/%,$(LIB_SRC))))

TARGET = $(ARCH_BUILD_DIR)/lib$(LIB_NAME).a
# Tools.
AR = avr-ar
ARFLAGS = rcs

CC = avr-gcc
CXX = $(CC)

CFLAGS = -c -mmcu=$(MCU) -DF_CPU=16000000L -Os $(LIB_INC) -DPDO_NUM=$(PDO_NUM)
CXXFLAGS = $(CFLAGS)

all: $(TARGET)

new-project: $(DESTDIR)/.git $(DESTDIR)/lib/canopen
	@printf "\n\n==============================================\n"
	@printf "Read README.developer\n"
	@printf "To start work do:\ncd $(DESTDIR)\nmake\n"

$(DESTDIR)/.git:
	@if [ -z "$(DESTDIR)" ]; then printf 'run:\n\tmake new-project ARCH=<arch> DESTDIR=/dir/to/new-project\n'; exit 1; fi
	mkdir -p $(DESTDIR)
	cp skel/* skel/.??* $(DESTDIR)
	sed -i -e 's/^ARCH.*/ARCH = $(ARCH)/' $(DESTDIR)/Makefile
	cd $(DESTDIR) && 				 \
	git init . && 					 \
	git add * .??* && 				 \
	git commit -m init

$(DESTDIR)/lib/canopen:
	cd $(DESTDIR) && mkdir -p lib && \

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJS)

$(ARCH_BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	+[ -d $(dir $@) ] || mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $< -o $@

$(ARCH_BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	+[ -d $(dir $@) ] || mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $< -o $@

clean:
	-rm -rf $(ARCH_BUILD_DIR) 2>/dev/null

clean-all:
	-rm -rf $(BUILD_DIR)

.PHONY: clean clean-all


