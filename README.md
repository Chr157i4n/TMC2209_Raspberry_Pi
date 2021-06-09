# TMC_2209_Raspberry_Pi
this is a libary to drive a stepper motor with a TMC2209 stepper driver and a Raspberry Pi

this code is still experimental, so use it on your own risk.

My TMC2209 is a driver from Watterott:
https://shop.watterott.com/SilentStepStick-TMC2209-V2_1

It has a rSense of 110 mOhm and it uses one Pin (PDN_UART) for UART RX and TX.
So the PD_UART-Pin needs to be connected to the Raspberrry Pis RX-Pin directly and to the TX-Pin with an 1kOhm resistor
You can read more about this in the datasheet from Trinamic:
https://www.trinamic.com/products/integrated-circuits/details/tmc2209-la/

Because the TMC2209 use one shared pin for transmit and receive in the UART communication line, the Raspberry Pi also receives what it sends,
Well, the Pi receive 8 bits from itself and 4 bit from the driver. So the Pi receives a total of 12 bits and only the last 4 needs to be used.

the code to run the stepper motor is based on the code of the AccelStepper Libary from Mike McCauley
https://github.com/adafruit/AccelStepper
http://www.airspayce.com/mikem/arduino/AccelStepper/

the code for the uart communicationis based on this code from troxel 
https://github.com/troxel/TMC_UART

the Documentation of the TMC2209 can be found here:
https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_rev1.06.pdf


## Usage
- clone this repo to your Raspberry Pi using "git clone https://github.com/Chr157i4n/TMC2209_Raspberry_Pi"
- install the python module bitstring with "pip3 install bitstring"
- enable the serial port in "raspi-config"
- run the script using "python3 test_script_tmc.py"
- test whether the UART communication works
- test whether the communication via STEP, DIR, EN pins work (with the function "testDirStepEn")
- test whether the motor runs 1 revolution forward and then 1 revolution backwards
