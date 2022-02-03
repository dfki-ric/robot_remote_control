#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y build-essential git cmake pkg-config libboost-test-dev libboost-system-dev libprotobuf-dev protobuf-compiler libzmqpp-dev libudt-dev zlib1g-dev libreadline-dev
mkdir build
cd build
cmake $@ ..
