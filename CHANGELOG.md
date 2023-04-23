# Change Log
All notable changes to this project will be documented in this file as of MCB 0.1.1

## MCB [0.1.2] - 2023-04-23
 
### Added

- HardwareHandler class added, classes should be able to access hardware through the abstracted functions
 
### Changed
  
- Core initialized some hardware components that should be left up to the respective classes so cleaned that up
- Core did not declare everything statically, so it would get cloned every time it was included

### Fixed

- Multiple declarations of initialization functions were removed

## MCB [0.1.1] - 2023-04-22
 
### Added

- Core.h added, so now any library class can now include the Core and have necessary dependancies
 
### Changed
  
- "TEST" robot has been refactored to slowly assimilate refactor changes and perform tests on them
