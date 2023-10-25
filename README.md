# Setup

## install azure kinect sdk

[reference](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1263)

```
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
# sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-get update
# sudo apt install libk4a1.4-dev
# sudo apt install libk4abt1.1-dev
# sudo apt install k4a-tools=1.4.1
sudo apt install libk4a1.4-dev libk4abt1.1-dev k4a-tools=1.4.1
```
exec
```
sudo k4aviewer
```

## exec sample code
### Azure-Kinect-Samples
```
git clone git@github.com:microsoft/Azure-Kinect-Samples.git
sudo apt install libxi-dev
sudo apt install ninja-build
cd Azure-Kinect-Samples
mkdir build
cd build
cmake .. -GNinja
ninja
```
exec
```
sudo bin/simple_3d_viewer
```
[usage](https://github.com/microsoft/Azure-Kinect-Samples/tree/master/body-tracking-samples/simple_3d_viewer)

### SDK examples
```
git clone -b release/1.4.x git@github.com:microsoft/Azure-Kinect-Sensor-SDK.git
```
build examples

[examples](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/release/1.4.x/examples)


## access without root permission
[reference](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup)

```
curl -sSL https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules | sudo tee /etc/udev/rules.d/99-k4a.rules > /dev/null
```
Deattach and attach



## clone repository
```
git clone -b foxy-devel git@github.com:microsoft/Azure_Kinect_ROS_Driver.git
```
## install dependencies
```
# rosdep install --from-paths src --ignore-src -r -y
```

