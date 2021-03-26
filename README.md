# TMC_2209_Raspberry_Pi
this is a libary to drive a stepper motor with a TMC2209 stepper driver and a Raspberry Pi

this code is still experimental, so use it on your own risk.

My TMC2209 is a driver from Watterott:
https://shop.watterott.com/SilentStepStick-TMC2209-V2_1

It has a rSense of 110 mOhm and it uses one Pin (PDN_UART) for UART RX and TX.
So the PD_UART-Pin needs to be connected to the Raspberrry Pis RX-Pin directly and to the TX-Pin with an 1kOhm resistor
You can read more about this in the datasheet from Trinamic:
https://www.trinamic.com/products/integrated-circuits/details/tmc2209-la/

Because the Raspberry Pis UART Receive Pin (RX) is connected to the transmit Pin of the Pi (TX), the Pi also receives what it sends.
If you have a different driver with TX and RX this is not the case.

Well, in my case the Pi receive 4 bits from itself and 8 bit from the driver.
When the driver has a shared TX/RX Pin the Pi receive 12 bit instead of 8.

the code to run the stepper motor is based on the code of the AccelStepper Libary from Mike McCauley
http://www.airspayce.com/mikem/arduino/AccelStepper/
