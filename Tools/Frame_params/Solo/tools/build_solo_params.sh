#!/bin/bash

set -e
set -x
#setup common.param:
tools/mavparmstrip.py --strip tools/paramstriplist base/defaults.param common.param
tools/mavparmmerge.py common.param base/solo.param common.param

tools/mavparmmerge.py common.param configs/minus.param Solo-minus.param
tools/mavparmmerge.py common.param configs/tmprops.param Solo-TMprops.param
tools/mavparmmerge.py common.param configs/apcprops.param Solo-APCprops.param
tools/mavparmmerge.py Solo-TMprops.param configs/hcescs.param Solo-TMprops-HCESCs.param
