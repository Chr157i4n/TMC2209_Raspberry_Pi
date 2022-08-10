# Changelog
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
-  added ability to invert direction in setVActual_rps with negative revolutions