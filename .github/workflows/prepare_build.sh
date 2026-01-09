#!/bin/bash
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
sudo apt-get install -y build-essential git cmake pkg-config libboost-test-dev libboost-system-dev libboost-filesystem-dev libprotobuf-dev protobuf-compiler libzmq3-dev libudt-dev zlib1g-dev libreadline-dev libncurses5-dev libwebsocketpp-dev
mkdir build
cd build
cmake $@ ..
