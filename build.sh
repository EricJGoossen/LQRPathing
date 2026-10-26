#!/bin/sh
# Build the demo and the pure pursuit comparison. EIGEN_INCLUDE: a directory holding eigen3/
# (default /usr/include; libeigen3-dev on Debian/Ubuntu).
set -e
I=${EIGEN_INCLUDE:-/usr/include}
SRC="src/path.cpp src/path_generator.cpp src/state_generators/position_state_generator.cpp
     src/state_generators/velocity_state_generator.cpp src/state_controllers/LQR.cpp
     src/auto_nav_controllers/LQR_autonav_controller.cpp"
g++ -std=c++20 -O2 -Wall -Wextra -I src -I "$I" -o LQRPathing src/main.cpp $SRC
# ARUW's pure pursuit, vendored with host stand-ins (see third_party/aruw-mcb/README.md).
V=third_party/aruw-mcb
ARUW="-I $V/shim -I $V/src -iquote $V/shim/tap/architecture -iquote $V/shim/tap/algorithms"
ARUW_SRC="$V/src/aruwsrc/algorithms/auto_nav_path.cpp
          $V/src/aruwsrc/control/chassis/chassis_auto_nav_controller.cpp $V/src/tap/algorithms/ramp.cpp"
g++ -std=c++20 -O2 -Wall -Wextra -I src -I "$I" $ARUW -o compare_pure_pursuit tools/compare_pure_pursuit.cpp \
    $SRC $ARUW_SRC
