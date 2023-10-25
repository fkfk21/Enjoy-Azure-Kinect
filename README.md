# Setup

## install azure kinect sdk
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


## clone repository
```
git clone -b foxy-devel git@github.com:microsoft/Azure_Kinect_ROS_Driver.git
```
## install dependencies
```
# rosdep install --from-paths src --ignore-src -r -y
```

