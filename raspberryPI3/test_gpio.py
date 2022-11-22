'''
Test script for GPIOs on Raspberry Pi
Andrea P.
'''

import lgpio
import time
import numpy as np
#global variable for state
PackageIn = False
time = 0
i = 0

#define the interrupt call back
def button_callback(a, b, c, timestamp):
    global PackageIn
    global time
    global i 
    #print("in") 
    if (np.abs(timestamp - time) > 1000000000):
        PackageIn = not PackageIn
        print("Temps", time)
        print(PackageIn)
        print(i)
        i += 1
        time = timestamp
    
    #time.sleep(1)

#Set up the pin 6 en pull down
button = lgpio.gpiochip_open(0)
lgpio.gpio_claim_alert(button, 6, lgpio.FALLING_EDGE, lFlags=lgpio.SET_BIAS_PULL_DOWN)

#Add callbac for rising edge (0 to 1)
c =lgpio.callback(button, 6, lgpio.FALLING_EDGE, button_callback)

while(True):
    pass 


