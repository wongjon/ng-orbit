astrobee_gripper
========
This repository contains the Astrobee gecko-adhesive gripper for testing with a mock-up of the perching-arm controller (PAC) to Teensy interface.

# Quickstart
1. First build the hex file
```
cd $PERCHING_ARM
sudo make
```
where *PERCHING_ARM=$ASTROBEE/freeflyer/submodules/avionics/src/perching_arm/perching_arm.X*.

2. Flash hex file to gripper
```
cd $PERCHING_ARM/../../../tools/
./flash_perching_arm.sh $PERCHING_ARM/dist/default/production/perching_arm.X.production.hex
```

3. Run perching arm tool
```
cd $PERCHING_ARM/../../tools/perching_arm
make
./perching_arm_tester -r raw.txt -o out.txt
```
