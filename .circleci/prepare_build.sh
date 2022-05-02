#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y build-essential git cmake pkg-config libboost-test-dev libboost-system-dev libboost-filesystem-dev libprotobuf-dev protobuf-compiler libudt-dev libreadline-dev
mkdir build
cd build
cmake $@ ..
