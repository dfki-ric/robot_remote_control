#!/bin/bash
set -e # abort with error if any command returns with something other than zero

############
# Install script for compiling proto3 from source
#
# In case your OS does not provide protobuf3, install the build dependencies of protobuf:
# $> sudo apt-get install autoconf automake libtool curl make g++ unzip
############

function build_protobuf {
  wget https://github.com/protocolbuffers/protobuf/archive/v3.1.0.tar.gz
  tar xzf v3.1.0.tar.gz
  cd protobuf-3.1.0
  ./autogen.sh
  ./configure --prefix=$1
  make
  make install
  cd ../
}

INSTALL_PATH=""

if [ -z $1 ]; then
    echo "[INFO] You have not specified an install path. Packages are installed to /usr/local."
    echo -n "       Press Ctrl+C to stop"
    sleep 1 && echo -n "."
    sleep 1 && echo -n "."
    sleep 1 && echo -n "."
else
    INSTALL_PATH=$1
fi

build_protobuf "$INSTALL_PATH"
