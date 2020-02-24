#!/bin/bash
set -e #abort with error if any command returns with something other than zero

############
# Install script for envire dependencies
# Run "$> sudo ./install_dependencies.sh" to install system wide
# Run "$> ./install_dependencies.sh <path_to_prefix>" to install to a local prefix
############

function build {
  git clone --branch $2 $1 $3
  mkdir $3/build
  cd $3/build
  cmake .. $4
  make -j install
  cd ../../
}

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



PREFIX=""
ABS_PREFIX=""

if [ "$#" -eq 1 ]; then

  ABS_PREFIX=`readlink -f $1`
  PREFIX="-DCMAKE_INSTALL_PREFIX=$ABS_PREFIX"

  echo "" > env.sh #create empty env.sh
  echo "export CMAKE_PREFIX_PATH=$ABS_PREFIX" >> env.sh
  echo "export PKG_CONFIG_PATH=$ABS_PREFIX/lib/pkgconfig:$ABS_PREFIX/share/pkgconfig:$ABS_PREFIX/lib64/pkgconfig:$PKG_CONFIG_PATH" >> env.sh
  echo "export LD_LIBRARY_PATH=$ABS_PREFIX/lib:$ABS_PREFIX/lib64:$LD_LIBRARY_PATH" >> env.sh
  echo "export PATH=$ABS_PREFIX/bin:$PATH" >> env.sh

  source env.sh

fi

#build https://github.com/rock-core/base-cmake.git master base-cmake "$PREFIX"

# In case your OS does not provide Protobuf 3, install the build dependencies of protobuf:
# $> sudo apt-get install autoconf automake libtool curl make g++ unzip
# and uncomment the next line
if [[ $(lsb_release -rs) == "16.04" ]]; then
    build_protobuf "$ABS_PREFIX"
fi


echo "please also install following OS dependencies:"
echo "protobuf and zmqpp for development"
echo "in ubuntu : $> apt install libprotobuf-dev protobuf-compiler libzmqpp-dev"


