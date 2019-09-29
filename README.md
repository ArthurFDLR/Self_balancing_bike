# Self_balancing_bike

The final objective is to stabilize a bike using a flywheel. For now, I'm focusing on the stabilisation part : reading IMU data, precise DC motor control, PID controller ...

## Mecanical parts
### Frame 
I wanted something as simple as possible. I initialy printed a single arm supported wheel. However the mass of the wheel was way to much for the frame to handle. I added a second arm to hold the wheel thanks to a bearing. The height of the rotationnal axis is variable. Once in place, toothpicks lock the position.

### Flywheel
Theorically I could determine the momentum of the wheel. However the equation depends on way to many factors to be really usable from the ground up. I've simply use an aluminum wheel I had laying around. I'll eventually change it to achieve a better efficiency.

## Electronics

Nothing to fancy here. A simple Nano v3 to compute the data, a MD13S motor driver with a Pololu 6V 25mm motor and an MPU6050. I'll certainly swap those componnent for a single board like a Storm32-BGC V1.32 usually used for gimbals. It has all I need (and even more) for this project.

## Code

The main goal is to get an efficiente code that can run as fast as possible on the Nano. For exemple I access directly to Atmega's registers during ISR. Currently runing at ~60Hz.
To ease tunings, a serial communication between the board and the IDE is setup to tune certain values and choose mode in real-time. 

| Char command     | Mode          |
| - | -------------- |
| 1 | Running stabilisation |
| 2 | Motor test w/ potentiometer |
| 3 | Setpoint variation viewer |
| 4 | IMU data viewer |
| p | KP tuning |
| i | KI tuning |
| d | KD tuning |
| s | KS tuning |

I use the incredible [library i2cdevlib from jrowberg](https://github.com/jrowberg/i2cdevlib) which takes advantage of the internal DMP of the board to read the IMU.

```c++
// IN   // NULL
// OUT  // ypr : float[3]
// DO   // Read last available data from MPU6050. Update {Yaw,Pitch,Roll} angular position in ypr
void Update_leanAngle();

// IN   // ypr                           : float[3]
        // IMU_time_now, IMU_time_prev   : uint32_t
// OUT  // leanAngle                     : float
        // leanAngle_smoothed            : float
        // leanAngleError                : float
        // leanAngle_Integer             : float
        // leanAngle_derivative          : float
        // leanAngle_derivative_smoothed : float
        // leanAngleSetPoint             : float
// TUNE // leanAngle_Filter              : float
        // leanAngle_Derivative_Filter   : float
        // leanAngle_HeavyFilter         : float
        // setPoint_KP, setPoint_KD      : uint32_t
// DO   // Calculates lean angle and its derivative and integral from ypr.
        // Filter results to eliminate noise (exponential filter)
void leanAngle_compute();

// IN   // motorCommandPwm : int16_t
        // motorSpeedRpm   : uint32_t
// OUT  // NULL
// TUNE // motorCommandPwm_Offset         : int16_t
        // motorCommandPwm_Starter_Offset : int16_t
        // motorLimitPwm                  : int16_t
// DO   // Filters command and send to motor. Smooth start, offset, limit ...
void Set_Motor_Speed();

// IN   // motorTickCount    : int32_t
// OUT  // motorSpeedRpm     : uint32_t
        // motorDirection    : int8_t
        // motorAccelerating : bool
// TUNE // motorFilter : float
// DO   // Interpret encoder ticks to give motor speed
void Update_MotorData();

// IN   // leanAngleError       : float
        // leanAngle_Integer    : float
        // leanAngle_derivative : float
        // motorDirection       : int8_t
        // motorSpeedRpm        : uint32_t
// OUT  // PID_output           : double
// TUNE // KP, KI, KD, KS       : float
        // motorFriction        : uint32_t
        // motorLimitPwm        : int16_t
// DO   // Compute data to give command w/PID controller
void Update_MotorData();
```

## Current state

[![IMU data viewer](http://img.youtube.com/vi/QBGMhSDM0Ms/0.jpg)](http://www.youtube.com/watch?v=QBGMhSDM0Ms "IMU data viewer")

I didn't find any way to start the motor without it giving a strong hit. It leads to an overshooting when trying to stabilize when the motor is still, for any KP ... I tried to set the standard rotation speed around PWM=40. I achieved some good reaction but only one falling side at a time. The braking power of the motor is way stronger than it's acceleration. It would recquiere an adaptive PID tuning given wheel rotation acceleration sign. Even then, it would be higly unefficient.
I'm now trying to fit a gimbal brushless motor - 120kv -. With a descent control method, I'll eventually overcome the jerkyness at low speed.
