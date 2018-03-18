b = arduino
sy = servo(b,'D9','MinPulseDuration',7.00e-4,'MaxPulseDuration',2.3e-3)
sx = servo(b,'D6','MinPulseDuration',7.00e-4,'MaxPulseDuration',2.3e-3)
writePosition(sx, 0.3)
writePosition(sy, 0.7)
writePosition(sx, 0.5)
writePosition(sy, 0.5)
