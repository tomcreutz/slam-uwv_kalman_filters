uwv_kalman_filters
=============

PoseUKF
---
The PoseUKF is a model aided inertial localization solution for autonomous underwater vehicles. As minimal input the filter relays on rotation rates and accelerations from an IMU and velocities from a DVL.  Given force and torque measurements an AUV motion model aids the velocity estimate during DVL drop outs.  ADCP measurements further aid the estimation in cases of DVL bottom-lock loss. Given gyroscopes capable of sensing the rotation of the earth (e.g. a fiber optic gyro) the filter is able to estimate it's true heading.

Reference
-------
The uwv_kalman_filters library is described/used in the following paper:

```
@inproceedings{arnold2018robust,
  title={Robust model-aided inertial localization for autonomous underwater vehicles},
  author={Arnold, Sascha and Medagoda, Lashika},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={4889--4896},
  year={2018},
  organization={IEEE}
}
```

License
-------
BSD 3-Clause License
