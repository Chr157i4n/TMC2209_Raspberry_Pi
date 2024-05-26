# Changelog

## version 0.5

- decoupled gpio access from gpio library
- added support for Raspberry Pi5 (gpiozero)
- added support for Luckfox Pico (python-periphery)

## version 0.4.5

- enhancement of logging module
- small bugfix

## version 0.4.4

- change logger to use logging module

## version 0.4.3

- small bugfix

## version 0.4.2

- added support for Nvidia Jetson
- added seperate file for GPIO board imports
- changed min python version to 3.7

## version 0.4.1

- removed dependency enum34
- removed dependency bitstring
- changed min python version to 3.6
- changed docstring format to google

## version 0.4

- split code into different files
- added logger class
- moved demo scripts into demo folder
- added unittest
- switched all string to f-strings

## version 0.3.4

- fixed do_homing()
- added minspeed to do_homing()
- added TMC_2209_math.py

## version 0.3.3

- added correct StallGuard min_speed calculation

## version 0.3.2

- add pylint github action
- fixed code to pass pylint check

## version 0.3.1

- added threaded movement
- added test_script_07_threads.py
- added softstop
- added get_movement_phase()

## version 0.3

- change code to snake_case

## version 0.2.2

- added set_deinitialize_true
- fixed ifcnt wrap around from 255 to 0

## version 0.2.1

- added setPDNdisable
- added setMaxSpeed_fullstep and setAcceleration_fullstep

## version 0.2

- Pin parameter order in constructor changed to EN, STEP, DIR
- STEP and DIR pins are optional parameters
- CRC check for read access reply datagrams
- if only zeroes are received an error will be thrown
- added ignore_delay to StallGuard callback
- implemented write access retry
- implemented velocity ramping with VActual
- add ability to print StallGuard results and TStep in VActual
- if write or read access fails, GSTAT will be checked for driver errors
- added CHANGELOG.md

## version 0.1.7

- updated README
- added number of revolutions as parameter for doHoming
- added output whether doHoming was successful or not

## version 0.1.6

- added ability to invert direction in setVActual_rps with negative revolutions
