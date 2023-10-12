.SILENT:

GCC=/Users/baden/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc
LD=/Users/baden/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-ld
OBJCOPY=/Users/baden/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objcopy

# GCC=/Users/baden/.local/opt/arm-gnu-toolchain-12.2.mpacbti-rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-gcc
# LD=/Users/baden/.local/opt/arm-gnu-toolchain-12.2.mpacbti-rel1-darwin-arm64-arm-none-eabi/bin/arm-none-eabi-ld

# GCC=/Users/baden/.local/opt/zephyr-sdk-0.15.2-rc1/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc
# LD=/Users/baden/.local/opt/zephyr-sdk-0.15.2-rc1/arm-zephyr-eabi/bin/arm-zephyr-eabi-ld

INCLUDES_DIRS=-I./NUC100BSP/Library/Device/Nuvoton/NUC100Series/Include -I./NUC100BSP/Library/CMSIS/Include -I./NUC100BSP/Library/StdDriver/inc

# CFLAGS += -W -Wall --std=gnu11 -Os
# CFLAGS += -mcpu=cortex-m0 -march=armv6-m

CFLAGS += -Wall -Werror -g -O0 
CFLAGS += -std=c99 -ffreestanding -ffunction-sections -fdata-sections
CFLAGS += -mcpu=cortex-m0 -mfloat-abi=soft -march=armv6-m   -mthumb

LINKER_SCRIPT = ./NUC100BSP/Library/Device/Nuvoton/NUC100Series/Source/GCC/gcc_arm.ld

all: build/main.hex

clean:
	rm -f build/*.o build/*.elf

build/system_NUC100Series.o: ./NUC100BSP/Library/Device/Nuvoton/NUC100Series/Source/system_NUC100Series.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/startup_NUC100Series.o: ./NUC100BSP/Library/Device/Nuvoton/NUC100Series/Source/GCC/startup_NUC100Series.S
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/gpio.o: ./NUC100BSP/Library/StdDriver/src/gpio.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/uart.o: ./NUC100BSP/Library/StdDriver/src/uart.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/sys.o: ./NUC100BSP/Library/StdDriver/src/sys.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/clk.o: ./NUC100BSP/Library/StdDriver/src/clk.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/retarget.o: ./NUC100BSP/Library/StdDriver/src/retarget.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

build/main.o: main.c
	mkdir -p build
	${GCC} $< -c -o $@ ${INCLUDES_DIRS} ${CFLAGS}

OBJS = build/main.o build/gpio.o build/uart.o build/sys.o build/clk.o build/system_NUC100Series.o build/startup_NUC100Series.o

build/main.elf: ${OBJS}
	mkdir -p build
	# ${LD} -T ${LINKER_SCRIPT} -o main.elf ${OBJS} ${CFLAGS} 
	# arm-none-eabi-gcc -Wall -Werror -g -O0 -std=c99 -ffreestanding -ffunction-sections -fdata-sections  -mcpu=cortex-m0 -mfloat-abi=soft -march=armv6-m   -mthumb  -Wall -Iinclude/headers   -o build/main.o -c src/main.c
	${GCC} -Wall -Werror -g -O0 -std=c99 -ffreestanding -ffunction-sections -fdata-sections \
	-mcpu=cortex-m0 -mfloat-abi=soft -march=armv6-m  \
	-mthumb  -Wall -Iinclude/headers   -Wl,-Map=build/firmware.map,--gc-sections \
	-T ${LINKER_SCRIPT} --specs=nano.specs ${OBJS} -o $@

build/main.hex: build/main.elf
	${OBJCOPY} -O ihex build/main.elf build/main.hex


flash: build/main.hex
	pyocd flash -t nuc140ve3cn build/main.hex
