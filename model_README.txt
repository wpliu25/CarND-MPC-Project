MPC
=====
Model/State
------------
The same '6' values taught in the lesson was used as state: {x,y,psi,v,cte,epsi}
x: location in x
y: location in y
psi: orientation
v: velocity
cte: cross track error
epsi: orientation error

Actuator
--------
'N-1' actuators values are returned by MPC:Solve() to model the MPC simulator points in the Green line. Other values returned are steering and throttle.


Cost Constraints
----------------
Costs that reference state are as follows:
1). Penalize CTE error and orientation heavily and less for desired velocity at scale 2000
// scaled penalty for errors (MPC.cpp lines 61-64)
fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);
fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2);
fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);

2). Minimize the use of actuators (MPC.cpp lines 70-71). Though relatively small, compared to error
costs, penalize for start/stop from braking at scale 5.
fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
fg[0] += 5*CppAD::pow(vars[a_start + t], 2);

3). Minimize the value gap between sequential actuations for smoothness. Relative to penality
for CTD and orientation error steering is penalized at a scale of 200, a medium cost. while acceleration is also a small relative cost.
// (MPC.cpp lines 76-77)
fg[0] += 200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);

Latency
-------
To account for 100ms delay (main.cpp line 196) I model for the delay (dt=0.1) in the callback from the simulator:

// Account for delay/latency (main.cpp lines 105-111)
double dt = 0.1;
double Lf = 2.67;
double px_latency = (px + v * CppAD::cos(psi) * dt);
double py_latency = (py + v * CppAD::sin(psi) * dt);
double psi_latency = (psi + v/Lf * mpc.delta_ / Lf * dt);
double v_latency = (v + mpc.a_ * dt);

This requires using the steering angle and acceleration from the previous iteration, which is saved by MCP::Solve(). After applying this solution the car is able to approach a goal velocity of 100MPH without leaving the track for a few laps.

Update
-------
The outline of the update in main.cpp follows the structure provided in the quiz solutions. However, a significant different is the coordinate system. This project required the coversion of points provided in the 'global' coordinate system to the car's local coordinate system by shifting reference angle to 90 degrees. After this conversion x, y, psi state variables was set to '0', which greatly simplifies the solution. 

For the polynomial fit I used '3' which matched the order of magnitude in cost.

Length (N) and Duration (dt)
============================
I initially started with these values from the quiz solution:
N = 25
dt = 0.05
but eventually chose:
N = 10
dt = 0.1

From empirical observations of running the program, 'N' was shortened from 25, to 12, and finally to 10. At 'N=25' the simulation was much too slow with the optimizer having too many control input values to optimize. This resulted in a behavior that the car kept having to brake. 'N=10' was chosen since the # of brake occurred mostly around curves and the number seemed reasonable.

From empirical observations of running the program, 'dt' was lengthened from 0.05, to 0.1. The duration effected the response of the vehicle and at '0.05' was too short, resulting in highly frequent actuaion. The behavior is a swerving car that seems to pass through the centerline too much.

At 'N=10' and 'dt=0.1' the prediction horizon 'T=1' seconds which seems inline with the lessons recommendation of a few seconds at most.

Known issues
============
The car does eventually leave the track. The latency model can definitely be improved along with better tuning of the prediction horizon.

