function Output = computepid(Input, kp, ki, kd, Setpoint)
   
   global errSum;
   global lastErr;
   global timeChange;  
 
   %/*How long since we last calculated*
   tic;
   %/*Compute all the working error variables*/
   error = Setpoint - Input;
   errSum = (error*timeChange) + errSum;
   dErr = (error - lastErr) / timeChange;
 
   %/*Compute PID Output*/
   Output = kp*error + ki*errSum + kd*dErr;
  
   %/*Remember some variables for next time*/
   lastErr = error;
   timeChange = toc;
end

