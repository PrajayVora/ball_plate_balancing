function Output = computepidx(Input, kp_x, ki_x, kd_x, Setpoint)

   global timeChange;
   global errSum_x;
   global lastErr_x;
  
   % time calculation start
   
   % Compute all the working error variables
   error = Setpoint - Input;
   errSum_x = (error * timeChange) + errSum_x;
   dErr = (error - lastErr_x) / timeChange;
  
   % Compute PID Output
   Output = kp_x * error + ki_x * errSum_x + kd_x * dErr;
   
   %/*Remember some variables for next time*/
   lastErr_x = error;

end