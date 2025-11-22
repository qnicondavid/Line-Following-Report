# Line Following Algorithm Report
---
## Robot Configuration
The robot is built around an **Arduino-compatible microcontroller** and a **four-wheel mecanum drive**.  
Each wheel is powered by a **DC motor** driven by an **H-bridge**, so the software can set both the **direction** and the **PWM duty cycle** of every wheel independently.

In this project, the mecanum wheels are used in a **simple differential-drive fashion**:
- The two left wheels always receive the same command.
- The two right wheels always receive the same command.

Sideways motion is not used for line following, which keeps the control problem **one-dimensional (left–right).**

---

## Sensor Setup

At the front of the robot, two **analog IR reflectance sensors** are mounted close to the floor:

- One sensor is positioned slightly to the **left** of the line.  
- The other sensor is positioned slightly to the **right** of the line.  

These sensors measure the amount of **infrared light reflected** from the ground.  
In practice, the **black electrical tape** produces a **higher analog reading** than the surrounding floor.  

- When a sensor is directly above the tape, its output value **increases**.  
- When it is above the brighter background, the output value is **lower**.  

The **difference between the left and right sensor readings** provides the **primary feedback signal** for the controller, enabling the robot to stay centered on the line.

In the code, the two analog inputs are defined as:
```cpp
const int PIN_IR_LEFT_ANALOG = A1;
const int PIN_IR_RIGHT_ANALOG = A3;
```
and read every control cycle:
```cpp
int leftSensor  = analogRead(PIN_IR_LEFT_ANALOG);
int rightSensor = analogRead(PIN_IR_RIGHT_ANALOG);
```
---
## Task Description

The task of the line-following algorithm is to keep the robot centered on a strip of **black electrical tape** placed on a lighter background.  

Intuitively, the goal is:
- Both front IR sensors see the tape in a similar way when the robot is correctly centered.  
- If the robot drifts to one side, one sensor moves further away from the tape and the other moves closer.  

We denote the left and right sensor readings at discrete time step *k* as $L(t)$ and $R(t)$.

A simple scalar error is then defined as:

$$
e(t) = L(t) - R(t)
$$

When the robot is perfectly centered, the two sensors see the same surface and the error satisfies  $e(k) \approx 0$.  

If the robot drifts to the **right**, the left sensor moves onto the tape while the right sensor sees more of the bright floor, making $e(k)$ **positive**.  
If the robot drifts to the **left**, the right sensor moves onto the tape while the left sensor sees more of the bright floor, making $e(k)$ **negative**.  

The controller must drive this error back to zero by adjusting the speed of the left and right wheels.

---
## PID Control & Advantages
To keep the robot on the line we use a PID controller. 

PID stands for Proportional–Integral–Derivative. It is a very common feedback control algorithm in robotics and industry because it is easy to implement, runs efficiently on small microcontrollers, and can be tuned experimentally without requiring an exact mathematical model of the robot.

In continuous time, for an error signal $e(t)$ and a control output $\omega(t)$, the PID law is:

$$
\omega(t) = K_P\ e(t)
      + K_I \int e(t)\ dt
      + K_D \frac{d e(t)}{dt}
$$

Where:  
- $K_P$ is the proportional gain  
- $K_I$ is the integral gain  
- $K_D$ is the derivative gain  

Each term has a clear role:

**Proportional term** reacts to the current error. A larger deviation from the line produces a larger correction, which gives the main steering action.  

**Integral term** accumulates past error. If the robot is always slightly to one side of the line (for example, because the sensors are not perfectly symmetric), the integral term slowly builds up and removes this steady‐state offset.  

**Derivative term** reacts to how quickly the error is changing. It anticipates sharp turns and helps reduce overshoot and oscillations.  

PID control is well suited for line following because the system is relatively slow (a small ground robot), the measurements are noisy, and the exact dynamics of motors and friction are difficult to model. PID provides a good balance between simplicity and performance.

In the Arduino code the gains are stored in variables such as:
```cpp
double kP = 100.0;
double kI = 0.05;
double kD = 0.1;
```
which allow tuning by changing only a few numbers.

---
## PID Logic & Implementation

### Sensor Reading & Error Computation
At each iteration of the main loop the controller reads both IR sensors and computes the error:

$$
e(t) = L(t) - R(t)
$$

In code, this looks like:
```cpp
double e = leftSensor - rightSensor;
```
Because the raw sensor values are somewhat noisy, the error is low‐pass filtered using an exponential moving average:

$$
e_f(t) = \alpha \ e(t) + (1 - \alpha) \ e_f(t-\Delta t)
$$

where $0 < \alpha < 1$ is a smoothing factor. 

In the code, this is implemented as:
```cpp
ef = alpha * e + (1.0 - alpha) * ef;
```
where ef stores the filtered error $e_f(t)$.

### Proportional Term
The proportional contribution at step t is simply:

$$
P(t) = K_P \ e_f(t)
$$

In the implementation this is calculated as part of the total correction:
```cpp
double P = kP * ef;
```
This term provides an immediate steering response: the more the robot deviates from the center, the larger the correction.

### Integral Term
The integral term in discrete time can be approximated by a sum. If $\Delta t$ is the duration of one control cycle, then we have:

$$
I(t) = I(t-\Delta t) + K_I \ e_f(t) \ \Delta t
$$

In the code, the integral of the filtered error is stored in integralSum:
```cpp
integralSum += ef * dt; // dt in seconds
// anti-windup: keep the integral within bounds
integralSum = constrain(integralSum , -I_MAX , I_MAX);
double I = kI * integralSum;
```
The call to constrain implements a simple anti‐windup mechanism: it prevents the integral term from grow‐ing without bound when the robot cannot correct quickly enough (for example on very sharp turns). This keeps the controller stable and avoids excessively large corrections after saturation.

### Derivative Term
The derivative term is based on the rate of change of the filtered error:

$$
D(t) = K_D \ \frac{e_f(t) - e_f(t-\Delta t)}{\Delta t}
$$

In the code it is computed from the current and previous filtered errors:
```cpp
double de = (ef - previousError) / dt;
double D = kD * de;
previousError = ef;
```
Keeping the derivative logic in place makes it easy to adjust if the robot starts to oscillate around the line.

### Combining the PID Terms
The three contributions are added to obtain the final correction signal $\omega(t)\$:

$$
\omega(t) = P(t) + I(t) + D(t)
$$

In code this is simply:
```cpp
double correction = P + I + D;
```
### Motor Speed Computation
Let $v_0$ be the nominal forward speed of the robot. The commanded speeds for the left and right sides are:

$$
v_L(t) = v_0 - \omega(t)
$$

$$
v_R(t) = v_0 + \omega(t)
$$

If  $\omega(t)\$ is positive, the left speed is reduced and the right speed is increased, which makes the robot turn left.  
If $\omega(t)\$ is negative, the opposite happens and the robot turns right.  
When $\omega(t) \approx 0 \$, both sides run at the same speed and the robot drives straight ahead.

The corresponding code follows this pattern:
```cpp
double leftSpeed = baseSpeed - correction;
double rightSpeed = baseSpeed + correction;

// limit speeds to allowed range
leftSpeed = constrain(leftSpeed , minSpeed , maxSpeed);
rightSpeed = constrain(rightSpeed , minSpeed , maxSpeed);
```
These signed speeds are passed to a helper function that sets the motor driver pins for all four wheels.  
Internally, this function decides the direction (forward or backward) and maps the absolute speed to a PWM value:
```cpp
void setMotor(int pwmPin , int dirPin , double speedSigned , bool wiringForward) {
      bool forwardDesired = (speedSigned >= 0);
      bool dirState = forwardDesired ? wiringForward : !wiringForward;
      digitalWrite(dirPin , dirState);
      int pwmVal = map(abs(speedSigned), 0, maxAbsSpeed , 0, 255);
      analogWrite(pwmPin , pwmVal);
}
```
The four mecanum wheels are grouped into left and right sides, so the final call looks conceptually like:
```cpp
moveMotors(leftSpeed , rightSpeed);
```
which applies the same left speed to both left wheels and the same right speed to both right wheels.

---
## Summary
The line‐following algorithm combines a simple hardware setup (two front IR sensors and four mecanum wheels) with a PID feedback controller.

The sensors provide a scalar error that measures how far the robot is from the center of the black tape.

The PID loop filters this error, computes proportional, integral and optional derivative contributions, and turns them into a steering correction.

This correction biases the left and right wheel speeds in opposite directions, so the robot continuously adjusts its trajectory and remains centered on the tape.

---
