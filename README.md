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

## check microphone array


```
pacmd list-sources 

```
Azure Kinect Microphone Array
- sample spec: s32le 7ch 48000Hz
- channel map: front-left,front-right,rear-left,rear-right,front-center,lfe,aux0






## install hark

### deb package
[reference](https://hark.jp/install/linux/)


### install nodejs
[reference](https://github.com/nodesource/distributions#installation-instructions)

```
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg
```

```
set NODE_MAJOR 18
echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | sudo tee /etc/apt/sources.list.d/nodesource.list
```

```
sudo apt update
```

### install hark

```
sudo apt install hark-base harkmw hark-core hark-designer harktool5 harktool5-gui kaldidecoder-hark
```
-> version 3.4.0

```
sudo apt install hark-linux hark-gtkplot
```

### how to use hark designer
[slides](https://www.hark.jp/document/HARK_Guide_jp_HowToUse-HarkDesigner.pdf)


microphone array
```
arecord -l
```
```
** List of CAPTURE Hardware Devices **
card 1: C960 [HD Webcam eMeet C960], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 2: Generic [HD-Audio Generic], device 0: ALC1220 Analog [ALC1220 Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 2: Generic [HD-Audio Generic], device 2: ALC1220 Alt Analog [ALC1220 Alt Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 3: Array [Azure Kinect Microphone Array], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0

```
-> manage device as "plughw:3,0"


### install hark-lib (PyHark)

[reference](https://hark.jp/packages/hark-acoustic-library-hark-lib/hark-lib-package-installation/)

### 
