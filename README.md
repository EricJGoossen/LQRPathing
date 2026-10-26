# LQRPathing

Path tracking for ARUW's chassis with an explicit model of what the robot can do, as an
alternative to pure pursuit. The plan (from the design proposal):

1. **Path** (`state_generators/`): a cubic spline through the waypoints, reparametrized by arc
   length; at each sample the speed is capped by the top speed and by the curvature,
   `v <= sqrt(a_lateral / k)`, then a forward pass limits acceleration and a backward pass limits
   braking, `v_{n+1} = sqrt(v_n^2 + 2 a d)`. `Path::TimeParametrize` turns that into a time for
   every sample.
2. **Control** (`state_controllers/LQR`): LQR on the world-frame model `x_dot = A x + B u`,
   `x = [x, y, theta, x_dot, y_dot, theta_dot]`, `u = [F_x, F_y, tau]` (A and B constant in the world
   frame), with K from the continuous algebraic Riccati equation; `u = -K (x - x_ref)`.
3. **Following** (`auto_nav_controllers/LQRAutonavController`): advances the reference along the
   path's time parametrization and rotates the force into the robot frame.

Real-time retargeting by the driver (step 3 of the proposal) is not built.

```sh
EIGEN_INCLUDE=/usr/include ./build.sh      # a directory holding eigen3/
./LQRPathing                               # demo: a path's speed profile and the LQR gain
./compare_pure_pursuit 0.4 0.5 3.2 0.9 0.8 1.9 3.4 2.9 0.6 3.9   # waypoints in meters
```

`tools/compare_pure_pursuit.cpp` runs this LQR and a reimplementation of the team's pure pursuit
(aruw-mcb's `ChassisAutoNavController`) on the same simulated holonomic chassis and path, and
prints both trajectories plus time to finish and distance from the path. The chassis's mass,
inertia and traction limit there are picked, not measured.
