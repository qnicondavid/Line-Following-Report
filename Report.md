# Line Following Algorithm Report

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

We denote the left and right sensor readings at discrete time step *k* as $L[t]$ and $R[t]$.

A simple scalar error is then defined as:

$$
e[t] = L[t] - R[t]
$$

When the robot is perfectly centered, the two sensors see the same surface and the error satisfies  $e[k] \approx 0$.  

If the robot drifts to the **right**, the left sensor moves onto the tape while the right sensor sees more of the bright floor, making $e[k]$ **positive**.  
If the robot drifts to the **left**, the right sensor moves onto the tape while the left sensor sees more of the bright floor, making $e[k]$ **negative**.  

The controller must drive this error back to zero by adjusting the speed of the left and right wheels.

---
## PID control and its advantages
