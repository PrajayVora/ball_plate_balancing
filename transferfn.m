s = tf('s');
H = (9.8*7)/(5*s^2)
pidTuner(H, 'pid')