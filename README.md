# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

This repository utilizes Model Predictive Control (MPC) to allow a vehicle to drive around a test lap. A path is provided which the vehicle has to track. Our MPC then determines the appropriate actuations so that the vehicle's driven path matches with the provided path. This is implemented towards Udacity's Self-Driving Car's Term 2 Simulator.

[//]: # (Image References)

[image1]: ./images/LatAdjusted.gif "Latency adjusted"
[image2]: ./images/LatNotAdjusted.gif "Latency not adjusted"

## Demonstration

Below is a short clip of the vehicle being controlled by our MPC model

![alt text][image1]


## Model Description

### Variables

The variables that are involved in this repo can be grouped into 
1. states (simulator environment inputs)
2. actuators (output to simulator to control vehicle)
3. waypoints (given path for MPC to follow)
4. errors (deviations from desired position)

#### States

The states present in the simulator are used as inputs for the MPC at each time step. They are,

1. x position (global co-ordinates)
2. y position (global co-ordinates)
3. psi (vehicle orientation)
4. velocity
5. steering angle
6. throttle

#### Actuators

Of the states mentioned, (5) and (6) are also the actuators for which our MPC model seeks to optimize for. They are then fed back to the simulator.

#### Waypoints

Waypoints are co-ordinates on the global frame that our vehicle tries to follow,

1. Waypoints X (sets of 6 values)
2. Waypoints Y (sets of 6 values)

#### Errors

We define the error terms below because they are useful as metrics on MPC's performance. The lower these errors go, the better our vehicle would track the given path. They are,

1. Cross-Track-Error (CTE) - Shortest distance from vehicle to given path
2. Psi error (epsi) - Deviation of vehicle orientation with respect to orientation inferred from given path

### Motion Model

Our MPC has an internal representation of the vehicle kinematics. These corresponds to how we expect the vehicle to behave given its current state and control actions. Their relationships are as follows,

```
x(t)    = x(t-1)    + v(t-1) * cos ( psi(t-1) ) * dt
y(t)    = y(t-1)    + v(t-1) * sin ( psi(t-1) ) * dt
psi(t)  = psi(t-1)  + v(t-1) *  ( -steer(t-1) )/Lf * dt
v(t)    = v(t-1)    + a(t-1) * dt

CTE(t)  = CTE(t-1)  + (f(t-1)   - y(t-1)) + v(t-1) * sin ( epsi(t-1) ) * dt
epsi(t) = epsi(t-1) + (psi(t-1) - g(t-1)) + v(t-1) * ( -steer(t-1) ) / Lf * dt
```

where

```
Lf   = Distance between front of vehicle and centre of gravity
f(t) = estimated y from x(t) (in this repo, it's a 3rd order polynomial function)
g(t) = estimated psi from x(t), f(t)

```

## MPC Algorithm

The idea of an MPC is that by modelling the environment (states) and vehicle behavior (kinematics), we could optimize our system of equations such that our desired metric is obtained. The general algorithm is provided below

1. Model environment
2. Model internal state of robot/vehicle and its temporal process
3. Determine bounds on variables
4. Determine objective functions and their target value
5. Perform optimization such that objective functions' targets are achieved
6. Report/execute on actuator variables

In the context of Udacity's simulator, we have the following (refer to github /src for complete implementation)

### Optimization Objectives

1. Cost function

```
  AD<double> KinematicObjFunctionSet::CalculateCost(const ADvector &Vars) const{

    AD<double> Cost = 0;

    for (size_t i = 0; i < N; ++i){
      // minimize deviation from desired state
      Cost += 1000 * CppAD::pow(Vars[CTE_Idx+i ] - target_CTE, 2);
      Cost += 1000 * CppAD::pow(Vars[epsi_Idx+i] - target_epsi, 2);
      Cost += CppAD::pow(Vars[v_Idx+i]    - target_v, 2);

      if (i < N-1){
        // minimize use of actuators
        Cost += CppAD::pow(Vars[steer_Idx+i], 2);
        Cost += CppAD::pow(Vars[a_Idx+i]    , 2);
      }

      if (i < N-2){
        // minimize large changes in actuators
        Cost += CppAD::pow(Vars[steer_Idx + i + 1] - Vars[steer_Idx + i], 2);
        Cost += CppAD::pow(Vars[a_Idx     + i + 1] - Vars[a_Idx     + i], 2);
      }
    }

    return Cost;
  }
```

2. Motion update

```
  fg[1 + x_Idx    + i + 1] = x1    - (x0        + v0 * cos(psi0)*dt);
  fg[1 + y_Idx    + i + 1] = y1    - (y0        + v0 * sin(psi0)*dt);
  fg[1 + psi_Idx  + i + 1] = psi1  - (psi0      + v0 * (-steer0) / LF*dt);
  fg[1 + v_Idx    + i + 1] = v1    - (v0        + a0 * dt);
  fg[1 + CTE_Idx  + i + 1] = CTE1  - ((f0 - y0) + (v0 * sin(epsi0)*dt));
  fg[1 + epsi_Idx + i + 1] = epsi1 - ((psi0 - g0) + v0 * (-steer0) / LF*dt);
```


## Time step

The time step variables used in the MPC are
1. N, number of time steps forward that our model predicts
2. dt, the duration of each time step

The higher the dt, the more accurate our model would be, however the downside is that it would be more computationally demanding. Given the trade-offs involved, our choice of N, and dt is guided by the minimum number of N, and dt that allows our MPC to smoothly tracks the given path. The following table lists some of the combinations we attempted,

| N   | dt    |  Comments              |
|:---:|:-----:|:----------------------:|
| 20  | 0.05  |  MPC passes test track |
| 10  | 0.10  |  MPC passes test track |
|  5  | 0.20  |  MPC fails test track  |

Our final choice is N of 10, and dt of 0.10.

## Preprocessing

Before passing the input state to our MPC model, we transformed the x, y and other inputs from global co-ordinates to relative vehicle co-ordinates. Specifically, we performed the following two transformations,

1. relative distance
2. rotate to vehicle co-ordinates

```
  void TransformWayPoint(const DataPackage &DP_in,
           VectorXd &ptsx_transf, VectorXd &ptsy_transf){

    for (size_t i = 0; i < DP_in.WayPointX.size(); ++i){

      // relative distance
      double shift_x = DP_in.WayPointX[i] - DP_in.Input[PX];
      double shift_y = DP_in.WayPointY[i] - DP_in.Input[PY];
      double psi = DP_in.Input[PSI];

      // transform to vehicle co-ordinates
      ptsx_transf[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
      ptsy_transf[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
    }
  }
```

While not strictly necessary, this transformation step accorded us the convenience in having a better polynomial fit, and simpler initialization (since x, y, and psi are 0 after the transformation).

## Polynomial Fitting of WayPoints

The given waypoints as sets of 6 co-ordinates are discrete. For the ability of evaluating the y co-ordinate of the waypoint on an arbitrary duration, we used a 3rd order polynomial fit.

## Latency Handling

In realistic situations, the time which actuators are executed may lag the time which the inputs are obtained. This could introduce errors in our MPC model. The Udacity simulator triggers a 100 miliseconds thread sleep between input and actuator outputs to artificially create a latency.

To account for the latency, instead of passing the input to the MPC directly, we evolve the inputs such that they approximate a future state before passing them to the MPC.

In the following code snippet, we evolve the input state by the latency duration (see `else` case).

```
  VectorXd GetState(const DataPackage &DP_in,
      const VectorXd &coef,
      double Latency_in){

    assert(Latency_in >= 0);

    VectorXd State = VectorXd::Constant(nState, 0.0);

    if (Latency_in == 0){

      double CTE = coef[0]; // approximating shortest dist by using y instead of |(x,y) - (x',y')|
      double epsi = -atan(coef[1]); // psi - arctan(f'(x)), psi being 0 after transform

      State << 0, 0, 0, DP_in.Input[VEL], CTE, epsi;

    } else {

      double steer = DP_in.Input[STR];
      double v     = DP_in.Input[VEL];

      double x   = v * cos(steer) * Latency_in;
      double y   = v * sin(steer) * Latency_in;
      double psi = v * (-steer) / LF * Latency_in;
      v         += DP_in.Input[ACC] * Latency_in;

      double CTE  = polyeval(coef, x);
      double epsi = -atan(     coef[1]
                         + 2 * coef[2] * x
                         + 3 * coef[3] * x * x);

      State << x, y, psi, v, CTE, epsi;
    }

    return State;
  }
```

Demonstration of with/without latency

Without latency adjustment

![alt text][image2]


With latency adjustment

![alt text][image1]

## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
