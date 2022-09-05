# Stanley Lateral Controller

This is the design document for the Stanley Lateral Controller implemented in the `stanley` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

Stanley Lateral Controller is a geometric control algorithm that calculates steering angles to follow a trajectory.

- Unlike Pure Pursuit and MPC which calculates lateral commands from the rear axle position, Stanley controller uses position of front axle.
- Stanley controller has more tunable parameters than Pure Pursuit controller.
- Although it has more parameters than pure pursuit it is still easier to tune than MPC.

## Design

<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The process of stanley controller is described below.

1. Get driving direction.
2. If vehicle is driving forward a virtual path with the length of *wheelbase + 1.0* is appended to the trajectory. 
   * Reason for this step is Autoware planning modules assume that controllers take the rear axle as the reference point. 
   * In order to make the vehicle stop smoothly at the end of the trajectory this virtual path is appended to the trajectory. 
   * While driving backwards this step is passed because Stanley controller takes the rear axle as the reference point in reverse gear.
3. Path smoothing is applied to the trajectory. For path smoothing Move Avarage filter is used.
4. Get front axle Pose by offsetting the rear axle Pose by the lenght of the *wheelbase*.
5. Set the Pose for the calculations according to the driving direction.
6. Get the the nearest point of trajectory to be tracked.
7. Calculate the curvature. The points used in curvature calculations are:
    * The point to be tracked.
    * `curvature_calc_dist` meters ahead of the first point.
    * `curvature_calc_dist` meters ahead of the second point.
8. Set stanley gains according to the curvature and direction.
9. Calculate heading error. Heading error, $\psi(t)$, is the yaw angle (heading) of
the vehicle with respect to the closest trajectory segment. Please see the original paper for more details.
10. Calculate crosstrack yaw error. Crosstrack yaw error is obtained by the equation below $$\arctan{\frac{k*e(t)}{k_{soft}+v(t)}} $$
Where $e(t)$ is the distance between front axle (rear axle in reverse) and the target point, $k_{soft}$ is the gain for permitting control to be arbitrarily soft at low speeds and $k$ is the crosstrack gain. Please see the original paper for more details.
11. Negative feedback on yaw rate for active damping is calculate using the equation below $$k_{d,yaw} * (r_{meas} - r_{traj})$$ where $k_{d,yaw}$ is a tuned
gain, $r_{traj}$ is the yaw rate for the trajectory, and $r_{meas}$ is the
measured yaw rate. Please see the original paper for more details.
12. Steer damping for time delay and overshoots in the controller commands is calculated using the equation below $$k_{d,steer}* (\delta_{meas}(i) − \delta_{meas}(i + 1))$$ where where $\delta_{meas}$ is the discrete time
measurement of the steering angle, and $i$ is the index of the
measurement one control period earlier.  The value of $k_{d,steer}$ is tuned
to be large enough to damp the steering wheel response, but
small enough to have minimal effect on performance. Please see the original paper for more details.
13. Calculate the final steering angle with the equation below $$\delta(t)= \psi(t)+ \arctan{\frac{k*e(t)}{k_{soft} * v(t)}} + k_{d,yaw} * (r_{meas}-r_{traj}) + k_{d,steer} * (\delta_{meas}(i) - \delta_{meas}(i+1))$$ Please see the original paper for more details.
    * Negative of the steering angle is used in reverse gear.



## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
`StanleyLateralController` class inherits the methods of the `LateralControllerBase` defined in the `trajectory_follower` package.

## Parameter description

These parameters are defined in `param/lateral_controller_defaults.yaml` and `vehicle_info.param.yaml`.

| Name                       | Type   | Description                                                    | Default value |
| :------------------------- | :----- | :------------------------------------------------------------- | :------------ |
| k_straight                 | double | stanley cross track gain for straights                         |               |
| k_turn                     | double | stanley cross track gain for turns                             |               |
| k_soft                     | double | stanley gain for low speed                                     |               |
| k_d_yaw                    | double | stanley gain for negative yaw rate feedback                    |               |
| k_d_steer                  | double | stanley gain for steer damping                                 |               |
| reverse_k                  | double | stanley cross track gain for reverse gear                      |               |
| reverse_k_soft             | double | stanley gain for low speed in reverse gear                     |               |
| reverse_k_d_yaw            | double | stanley gain for negative yaw rate feedback for reverse gear   |               |
| curvature_threshold        | double | curvature threshold for turns                                  |               |
| curvature_calc_dist        | double | point-to-point distance used in curvature calculation          |               |
| convergence_threshold      | double | convergence threshold for stanley controller[m]                |               |
| max_steer_angle            | double | maximum steering angle[rad]                                    |               |
| enable_path_smoothing      | bool   | path smoothing flag.                                           | true          |
| path_filter_moving_ave_num | int    | number of data points moving average filter for path smoothing | 35            |





## References / External links

<!-- Optional -->
- [Original paper](http://ai.stanford.edu/~gabeh/papers/hoffmann_stanley_control07.pdf)