# Builds the custom BME280 chip to WebAssembly for the Wokwi simulator.
#
# Needs clang with the wasi sysroot; the simplest way is the same
# container the Wokwi examples use (and what ci.yml runs):
#
#   docker run --rm -v "$PWD":/src -w /src wokwi/builder-clang-wasm \
#       make -C chip
#
# `make -C chip host-test` needs only cc: it links the chip logic against
# stub Wokwi APIs and replays the driver's I2C traffic against it.

SOURCES = bme280.chip.c
TARGET  = dist/bme280.chip.wasm

.PHONY: all clean host-test

all: $(TARGET) dist/bme280.chip.json

clean:
	rm -rf dist build

dist:
	mkdir -p dist

$(TARGET): dist $(SOURCES) wokwi-api.h
	clang --target=wasm32-unknown-wasi --sysroot /opt/wasi-libc \
	    -nostartfiles -Wl,--import-memory -Wl,--export-table \
	    -Wl,--no-entry -Werror -o $(TARGET) $(SOURCES)

dist/bme280.chip.json: dist bme280.chip.json
	cp bme280.chip.json dist/

# ------------------------------------------------------------- host test
build:
	mkdir -p build

host-test: build test_chip.c $(SOURCES) wokwi-api.h
	cc -O2 -Wall -Wextra -Werror -std=c11 -Wno-unknown-attributes \
	    -Wno-unused-function -o build/test_chip test_chip.c -lm
	./build/test_chip
