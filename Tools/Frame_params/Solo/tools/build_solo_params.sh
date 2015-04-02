#!/bin/bash

set -e
set -x

tools/mavparmstrip.py --strip tools/paramstriplist base/defaults.param Solo.param
tools/mavparmmerge.py Solo.param base/solo.param Solo.param
tools/mavparmmerge.py Solo.param configs/tmprops.param Solo.param
tools/mavparmmerge.py Solo.param configs/rccal.param Solo.param
