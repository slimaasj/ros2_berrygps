# ros2_berrygps
Ros2 package for the BerryGPS hat

---

## Hardware Info
![BerryGPS Hat](https://i1.wp.com/ozzmaker.com/wp-content/uploads/2018/11/BerryGPS-IMUv3PintOut.jpg?resize=768%2C509)

[OZZMAKER DOCS](http://ozzmaker.com/berrygps-berrygps-imu-quick-start-guide/#BerryGPS-IMUv3)

### BerryGPS-IMUv3

**LSM9DS1** - Magnetometer, accelerometer and gyroscope datasheet can be found [here](http://ozzmaker.com/wp-content/uploads/2017/11/LSM9DS1.pdf).
**BM280** - Pressure sensor datasheet can be found [here](http://ozzmaker.com/wp-content/uploads/2017/11/BMP280-DS001-11.pdf).

GPS related Datasheets
[CAM-M8-FW3_DataSheet_(UBX-15031574)](http://ozzmaker.com/wp-content/uploads/2016/08/CAM-M8-FW3_DataSheet_UBX-15031574.pdf)
[CAM-M8-FW3_HardwareIntegrationManual_(UBX-15030063)](http://ozzmaker.com/wp-content/uploads/2016/08/CAM-M8-FW3_HardwareIntegrationManual_UBX-15030063.pdf)


#### BerryGPS-IMU I2C Addresses
**BerryGPS-IMUv3**

* **0x6A** for the gyroscope and accelerometer
* **0x1C** for the magnetometer
* **0x77** for the pressure sensor

---

## ROS2 Package Info

### Dependencies
* RTIMULib (c++ and python)

## Setup
for now, RTIMULib should be installed via the instructions of the readme located: `Linux/python`
**note:**: python3 should be used

* change any setttings in the `config.yaml` and if need be
* the `RTIMULib.ini` file should be fine minus the need to calibrate the magnitometer

### Calibration

See [this Link](https://github.com/RTIMULib/RTIMULib2/blob/master/Calibration.pdf) for details

using RTIMULibCal:
* run the tool
* move the device around
* save .ini file


## Usage

can use the launch file
`ros2 launch ros2_berrygps berrygps.launch.py`

or run a the node directly
`ros2 run ros2_berrygps berrygps_node`

### TODO
* change to c++ instead of python
