'''
Test script for GPIOs on Raspberry Pi
Andrea P.
'''

import lgpio
#global variable for state
PackageIn = False

#define the interrupt call back
def button_callback():
    global PackageIn
    PackageIn = not PackageIn
    print("Changement de valeur")

#Set up the pin 6 en pull down
button = lgpio.gpiochip_open(0)
lgpio.gpio_claim_input(button, 6, lFlags=lgpio.SET_PULL_DOWN)

#Add callbac for rising edge (0 to 1)
lgpio.callback(0, 6, lgpio.RISING_EDGE, button_callback)

