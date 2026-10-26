#!/bin/sh
# Build the demo and the pure pursuit comparison. EIGEN_INCLUDE: a directory holding eigen3/
# (default /usr/include; libeigen3-dev on Debian/Ubuntu).
set -e
I=${EIGEN_INCLUDE:-/usr/include}
SRC="src/path.cpp src/path_generator.cpp src/state_generators/position_state_generator.cpp
     src/state_generators/velocity_state_generator.cpp src/state_controllers/LQR.cpp
     src/auto_nav_controllers/LQR_autonav_controller.cpp"
g++ -std=c++20 -O2 -Wall -Wextra -I src -I "$I" -o LQRPathing src/main.cpp $SRC
g++ -std=c++20 -O2 -Wall -Wextra -I src -I "$I" -o compare_pure_pursuit tools/compare_pure_pursuit.cpp $SRC
