#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

pushd ../PX4Firmware
git checkout ce602658
popd

pushd ../PX4NuttX
git checkout f6e0eff9
popd

pushd ArduCopter
make configure
make clean
make px4-cleandep
make px4-v2
popd

exit 0
