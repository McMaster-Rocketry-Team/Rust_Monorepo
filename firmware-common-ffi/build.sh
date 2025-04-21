#! /bin/sh

if [ "$(basename "$PWD")" != "firmware-common-ffi" ]; then
    echo "Error: Script must be run from the 'firmware-common-ffi' directory."
    exit 1
fi

rm -rf out

# generate header file
cbindgen --config cbindgen.toml --crate firmware-common-ffi --output out/firmware_common_ffi.h

# ========== Compile for riscv32imc-esp-espidf (ESP32-C3) ==========

# rustup target add riscv32imc-unknown-none-elf
# build the library
# ESP32-C3 uses riscv32imc-esp-espidf target
# All the targets for ESP32 are here: https://doc.rust-lang.org/rustc/platform-support/esp-idf.html
CARGO_PROFILE_RELEASE_PANIC=abort cargo build --release -Z build-std=core,panic_abort -Z build-std-features=panic_immediate_abort --target=riscv32imc-esp-espidf
mkdir -p ./out/riscv32imc-esp-espidf/
cp ../target/riscv32imc-esp-espidf/release/libfirmware_common_ffi.a ./out/riscv32imc-esp-espidf/libfirmware_common_ffi.a

# remove the .riscv.attributes section
# Rust uses LLVM to build, which outputs an .a file with instruction set "rv32i2p1_m2p0_c2p0_zmmul1p0"
# This instruction set is not supported by platformio's GCC toolchain (but is supported by the ESP32-C3 hardware)
# We need to remove the .riscv.attributes section so it does not trigger an error when linking
llvm-objcopy --remove-section=.riscv.attributes ./out/riscv32imc-esp-espidf/libfirmware_common_ffi.a

# ========== Compile for thumbv7em-none-eabihf (most STM32) ==========

CARGO_PROFILE_RELEASE_PANIC=abort cargo build --release -Z build-std=core,panic_abort -Z build-std-features=panic_immediate_abort --target=thumbv7em-none-eabihf
mkdir -p ./out/thumbv7em-none-eabihf/
cp ../target/thumbv7em-none-eabihf/release/libfirmware_common_ffi.a ./out/thumbv7em-none-eabihf/libfirmware_common_ffi.a

# Compress the out folder into release.tar.gz
tar -czf release.tar.gz -C out .
mv release.tar.gz ./out/

echo "Build complete. Output files are in the 'out' directory."