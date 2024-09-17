**Note**: Please be aware that this example app is provided for demonstration purposes only. This flight mode may not be suitable to fly on real hardware without further verification and testing.

# Go-to Multicopter App Example

This is an example app on how to use Auterion SDK to build a go-to mode for
a multicopter. 

## Expected Behavior

This flight mode commands the vehicle to reach 3 different local positions in a loop.

## Build

This app can be built with auterion-cli. E.g. for simulation, run

```
auterion-cli app build --simulation
```