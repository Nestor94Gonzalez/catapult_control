# Catapult Control [In progress]

This ros package contains a gazebo plugin that changes the velocity and upper limit of a joint. 

Publishing from command line:
```bash
rostopic pub /catapult/chatter catapult_control/Catapult 300 2
```
