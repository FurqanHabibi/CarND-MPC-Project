## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

### 1. The Model
I will describe the state, actuators and update equations used in my model.

#### States
These are the states used in my model:
- x

  The x position of the car in the car's coordinate frame. As this is described in the car's own coordinate frame, the initial value will always be 0.

- y

  The y position of the car in the car's coordinate frame. As this is described in the car's own coordinate frame, the initial value will always be 0.

- psi

  The heading of the car in radians in the car's coordinate frame. As this is described in the car's own coordinate frame, the initial value will also be 0.

- v

  The velocity of the car. Since velocity is a scalar, it's value is not effected by the coordinate frame used. Hence this will be the velocity value reveived from the simulator.

- cte

  The current cross-track error of the car in regard to the ideal waypoint of the road received from the simulator. In order to simplify the calculation, I decided to formulate this as the y value of the fitted polynomial of the waypoint in the x position of the car. Thus the calculation will just be the evaluation of the polynomial expression at x.

- epsi

  The current heading error of the car, meaning the difference between the current heading and the desired heading of the car according to the waypoints' fitted polynomial. The desired heading can be calculated by taking the arctangent of the derivative of the polynomial at x.

#### Actuators
These are the actuators used in my model:
- a

  The acceleration to be executed using the car's throttle. The value will range from 1 to -1, with negative value meaning brake or backward acceleration.

- delta

  The difference of heading to be executed by applying the corresponding steering angle. The value will range from 1 to -1, with 1 meaning a 25 degree steering angle.

#### Update equations
Below are the update equations used in my model. They are almost the exact same equations introduced in the lesson.
```
- x1 = (x0 + v0 * cos(psi0) * dt)

- y1 = (y0 + v0 * sin(psi0) * dt)

- psi1 = (psi0 + (v0 / Lf) * delta0 * dt)

- v1 = (v0 + a0 * dt)

- cte1 = ((f0 - y0) + v0 * sin(epsi0) * dt)
  f0 = polyeval(coeffs, x0)

- epsi1 = ((psi0 - psides0) + (v0 / Lf) * delta0 * dt)
  psides0 = atan(polyeval(polyderivative(coeffs), x0))
```

### 2. Timestep Length and Elapsed Duration (N & dt)
I chose the N value of **5** and the dt value of **0.01**. In the following paragraphs I will explain my reasoning in chosing those values.

The N value determines how many step in the horizon the MPC will calculate and optimize. This has a direct connection with how far ahead the horizon points will be calculated. However, the waypoints that are received from the simulator, that will become the reference in the calculation by way of fitted polynomial, are only 6 points. This means the fitted polynomial are only reliably accurate until the 6th waypoint, with the polynomial thereafter can always diverge erratically. This situation means the position of the 6th waypoint received from the simulator is the furthest point that can be used for horizon calculation. In conclusion, one has to choose the N value so as to not make the horizon points exceeds the 6th point of the waypoints.

The dt value is the time between one prediction and the next. The bigger this value is, the further ahead the horizon points will be. Thus similar as the N value above, one has to choose the dt value so as to not make the horizon points exceeds the last point of the waypoints.

Another important point that influences the choosing of N and dt is the actuation delay of the car. To accomodate actuation delay, I incorporate the delay in the horizon point calculation, which means the time between one prediction and the next will not only be dt but also the actuation delay. This then will exemplify the effect of furthering the horizon points ahead.

With the reasons that I mentioned above, and by also taking into consideration the reference speed of 60, I then found that N value of 5 and dt value of 0.01 with an actuation delay of 0.1 to be the best value as to not make the last horizon point exceeds the last waypoints' point. Before coming up with this value I tried bigger N and bigger dt which resulted in horizon points exceeding the last waypoint and giving back poor point predictions in regard to the actual shape of the road.

### 3. Polynomial Fitting and MPC Preprocessing
As mentioned in the lesson, polynomial with order of 3 can adequately model the common shape of car roads in the world. So I decided to fit the waypoints to polynomial with oreder of 3. I also implemented a derivative function which actually can handle polynomial of any order.

Before I fit the waypoints to a polynomial, I first transform the points from the global coordinate frame to the car coordinate frame. This is because I decided to do all the MPC processing in car coordinate, which can simplify quite a lot of calculations.

I don't really implement any other preprocessing besides the coordinate transformation.

### 4. Model Predictive Control with Latency
To accomodate the 0.1 s latency of the car actuation, I modify the update equations quite a bit. Mainly, I changed the previous state of the update equations to incorporate the delay and reflects it in the state value. So I applied the below calculation to find the value of the previous state right before the actuations start having effect, which is after the delay passed. I then use this value instead of the original previous state value in the update equations.
```
pred_x0 = x0 + v0 * cos(psi0) * act_delay;
pred_y0 = y0 + v0 * sin(psi0) * act_delay;
pred_psi0 = psi0 + (v0 / Lf) * delta0 * act_delay;
pred_v0 = v0 + a0 * act_delay;
pred_delta0 = delta0;
pred_a0 = a0;
pred_f0 = polyeval(coeffs, pred_x0);
pred_psides0 = atan(polyeval(polyderivative(coeffs), pred_x0));
pred_cte0 = pred_f0 - pred_y0;
pred_epsi0 = pred_psi0 - pred_psides0;
```
By doing this the MPC can correctly predict the future horizon even with a delay in actuation.
