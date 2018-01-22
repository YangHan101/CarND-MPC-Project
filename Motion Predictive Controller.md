## Motion Predictive Controller

### Description of the Model.

The model employed mainly followed the instruciton video. The state of the vehicle was described by a vector of 6 elements: x coordinate, y coordinate, azimuth angle of the car, speed of the vehicle, cross track error and azimuth error. Two actuators were added to simulate the control of the vehicle. Acceleration with positive and negative values represents the gas paddle and the brake. Steering wheel angle is also included to as an actuator to provide angle control.

The update equation is as following:

```c++
x1 = (x0 + v0 * cos(psi0) * dt); 
y1 = (y0 + v0 * sin(psi0) * dt);
v1 = (v0 + a0 * dt);
psi1 = (psi0 - v0 / Lf * delta0 * dt); 
cte1 = (f0 - y0 + v0 * sin(epsi0) * dt);
epsi1 = (psi0 - psides0 - v0 / Lf * delta0 * dt);
```



### Selection of N and dt.

In order to compensate for the latency dt is chosen to be 0.1s.

A larger N allows estimations to more into the future. However it takes more time to compute. So I chose to make N = 10. It seems to work pretty well.

### Description of waypoint preprocessing

Before using polyfit to fit the way points. I converted the way points into the car coordinate system. By doing so, the initial x, y and psi input into the controller becomes zero. 

Besides, the calculation of CTE is easier in this coordinate system. We can simply do ```polyeval(coeffs, 0)```. In other coordinate system this measurement is along the incorrect direction.

### Description of the solution to latency

Since dt = 0.1, we can simply use the actuator value from one time step ahead when possible. 
