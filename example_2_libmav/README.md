**Note**: Please be aware that this example app is provided for demonstration purposes only. This flight mode may not be suitable to fly on real hardware without further verification and testing.

# Libmav App Example

This is an example app on how to use Libmav within an Auterion SDK application. 

**Important:** Add libmav and mavlink as submodule in the head directory of your application to use it. 

```
cd to you app directory

git submodule add git@github.com:Auterion/libmav.git

git submodule add git@github.com:mavlink/mavlink.git

git submodule update --init --recursive
```
Also add the the `libmav` and `mavlink` related runtime dependencies to your Dockerfile (see [Dockerfile](./Dockerfile)).

## Expected Behavior

This flight mode commands the vehicle to go to local position (x=0, y=0, z=5) and send STATUSTEXT mavlink messages that appear in AMC.

## Build

This app can be built with auterion-cli. E.g. for simulation, run

`auterion-cli app build --simulation`