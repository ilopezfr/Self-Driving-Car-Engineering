#!/bin/bash
export WORKDIR="/root/CarND-MPC-Project/"

CURDIR=$PWD
port=${1:-4567}

docker run --privileged --rm -it \
  -v $CURDIR:$WORKDIR \
  -p $port:$port \
  -w $WORKDIR \
  mpc:latest bash
