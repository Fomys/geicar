'''
Test script for GPIOs on Raspberry Pi
Andrea P.
'''

import RPIO
#global variable for state
PackageIn = False

#define the interrupt call back
def button_callback():
    global PackageIn
    PackageIn = not PackageIn
    print("Changement de valeur")

#Set up the pin 6 en pull down
RPIO.setup(6, RPIO.IN, pull_up_down=RPIO.PUD_DOWN)

#Add callbac for rising edge (0 to 1)
RPIO.add_interrupt_callback(7, button_callback, edge='rising')

#wait for rising edge
RPIO.wait_for_interrupts()
