function Output = computepidy(Input, kp_y, ki_y, kd_y, Setpoint)

   global timeChange;
   global errSum_y;
   global lastErr_y;
  
   % time calculation start

   % Compute all the working error variables
   error = Setpoint - Input;
   errSum_y = (error * timeChange) + errSum_y;
   dErr = (error - lastErr_y) / timeChange;
  
   % Compute PID Output
   Output = kp_y * error + ki_y * errSum_y + kd_y * dErr;
   
   %/*Remember some variables for next time*/
   lastErr_y = error;
   
end