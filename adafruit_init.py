#!/usr/bin/env python
  
from adafruit_servokit import ServoKit


kit = ServoKit(channels=16)

#servo specifics
kit.servo[0].set_pulse_width_range(700, 2300)
kit.servo[1].set_pulse_width_range(700, 2300)
kit.servo[2].set_pulse_width_range(700, 2300)
kit.servo[3].set_pulse_width_range(700, 2300)
kit.servo[4].set_pulse_width_range(700, 2300)
kit.servo[5].set_pulse_width_range(700, 2300)
#kit.servo[6].set_pulse_width_range(700, 2300)
kit.servo[7].set_pulse_width_range(700, 2300)
kit.servo[8].set_pulse_width_range(700, 2300)
kit.servo[9].set_pulse_width_range(700, 2300)
kit.servo[10].set_pulse_width_range(700, 2300)
kit.servo[11].set_pulse_width_range(700, 2300)
kit.servo[12].set_pulse_width_range(700, 2300)
kit.servo[13].set_pulse_width_range(700, 2300)

kit.servo[0].angle = 90 #hla1
kit.servo[1].angle = 90 #hla2
kit.servo[2].angle = 90 #hra1
kit.servo[3].angle = 120 #hra2
kit.servo[4].angle = 90 #spine
kit.servo[5].angle = 90 #tail
kit.servo[8].angle = 90 #fla1
kit.servo[9].angle = 90 #fla2
kit.servo[10].angle = 90 #fra1
kit.servo[11].angle = 90 #fra2
kit.servo[7].angle = 180 #spine2
kit.servo[12].angle = 90 #Head LR
kit.servo[13].angle = 90 #HEAD UD

