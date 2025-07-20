#! /bin/sh

rm -rf out
mkdir out
mkdir out/lib

java-pack build
cp ../target/debug/libair_brakes_controller_core.so ./out/lib/air-brakes-controller-core.so
cp ./target/java_bindgen/target/air-brakes-controller-core-0.1.0-jar-with-dependencies.jar ./out/