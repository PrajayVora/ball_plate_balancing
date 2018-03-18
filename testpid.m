x = 359.7170
global errSum;
errSum = 0;
global lastErr;
lastErr = 0;
global timeChange;
timeChange = 0.1;
Setpoint = 240;
kp_y = 10;
ki_y = 10;
kd_y = 10;

    while(1)
    z = computepid(x, kp_y, ki_y, kd_y, Setpoint)
    end