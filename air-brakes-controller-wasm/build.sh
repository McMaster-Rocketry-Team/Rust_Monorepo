#! /bin/sh

if [ "$(basename "$PWD")" != "air-brakes-controller-wasm" ]; then
    echo "Error: Script must be run from the 'air-brakes-controller-wasm' directory."
    exit 1
fi

rm -rf ./out
mkdir -p out

cargo build --release --target=wasm32-unknown-unknown
cp ../target/wasm32-unknown-unknown/release/air_brakes_controller_wasm.wasm ./out/air_brakes_controller.wasm

echo Compiled wasm at ./out/air_brakes_controller.wasm