import sounddevice as sd 
from tools import get_device_number

print(sd.query_devices())
print("Device number of Azure Kinect: ", get_device_number("Azure Kinect"))
