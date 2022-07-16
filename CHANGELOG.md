# Changelog
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
- added number of revolutions as parameter for do_homing
- added output whether do_homing was successful or not

## version 0.1.6
-  added ability to invert direction in setVActual_rps with negative revolutions