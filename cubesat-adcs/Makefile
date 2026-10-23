# make test        host unit + closed-loop tests (g++)
# make firmware    build the Cortex-M3 image (runs on the custom RTOS)
# make run         run the firmware in QEMU
# make qemu-test   run in QEMU and check the mission results
# make wasm        build the WebAssembly module used by the live demo
# make demo        build docs/index.html (live demo with the .wasm embedded)

CORE = core/adcs.cpp sim/plant.cpp sim/world.cpp sim/mission.cpp
Q    = targets/qemu

ARMCXX   = arm-none-eabi-g++
ARMFLAGS = -mcpu=cortex-m3 -mthumb -std=c++17 -Os -g -Wall -Wextra -I$(Q) \
           -ffreestanding -fno-exceptions -fno-rtti -fno-threadsafe-statics \
           -ffunction-sections -fdata-sections
ARMLD    = -mcpu=cortex-m3 -mthumb -T $(Q)/linker.ld -nostdlib -nostartfiles -Wl,--gc-sections -lgcc
FW_SRC   = $(Q)/startup.cpp $(Q)/libc_min.cpp $(Q)/kernel/kernel.cpp $(Q)/drivers/uart.cpp $(Q)/main.cpp $(CORE)
QEMU     = qemu-system-arm -M lm3s6965evb -nographic -semihosting-config enable=on,target=native

all: test firmware wasm

test:
	@mkdir -p build
	g++ -std=c++17 -O2 -Wall -Wextra $(CORE) tests/test_adcs.cpp -o build/test_adcs
	./build/test_adcs

firmware: build/adcs.elf
build/adcs.elf: $(FW_SRC) $(wildcard core/*.h sim/*.h $(Q)/kernel/*.h)
	@mkdir -p build
	$(ARMCXX) $(ARMFLAGS) $(FW_SRC) $(ARMLD) -o $@
	arm-none-eabi-size $@

run: build/adcs.elf
	$(QEMU) -kernel $<

qemu-test: build/adcs.elf
	./tests/run_qemu_test.sh

wasm: build/adcs.wasm
build/adcs.wasm: $(CORE) targets/wasm/wasm_api.cpp
	@mkdir -p build
	clang++ --target=wasm32 -std=c++17 -O2 -nostdlib -fno-exceptions -fno-rtti -ffreestanding \
	  -Wl,--no-entry -Wl,--export-dynamic -Wl,--allow-undefined $(CORE) targets/wasm/wasm_api.cpp -o $@

demo: build/adcs.wasm
	python3 web/build_demo.py build/adcs.wasm web/index.template.html docs/index.html

clean:
	rm -rf build

.PHONY: all test firmware run qemu-test wasm demo clean
