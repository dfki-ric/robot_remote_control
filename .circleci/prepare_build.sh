#!/bin/bash
apt-get update
apt-get install -y build-essential git cmake pkg-config libboost-test-dev libboost-system-dev libprotobuf-dev protobuf-compiler libzmqpp-dev
./install_source_dependencies.sh ./external
. env.sh 
mkdir build
cd build
cmake $@ ..
