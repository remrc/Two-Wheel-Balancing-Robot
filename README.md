# Two-Wheel-Balancing-Robot

Gimbal controller BGC 3.1 (use as Arduino), gimbal motors GBM2804H-100T, MPU6050, magnetic encoders AS5600 connected over analog inputs.

I use the Simple FOC library to control the BLDC motors.

For remote control I use Joy BT Commander.

Balancing controller can be tuned remotely over bluetooth.

Example:

Send p+ (or p+p+p+p+p+p+p+) for increase K1.

Send p- (or p-p-p-p-p-p-p-) for decrease K1.

The same for K2, K3, K4. Send "s", "d", "a".

<img src="/pictures/robot0.jpg" alt="Robot pic"/>
 
More about this:

https://youtu.be/p3CFaL55U08
