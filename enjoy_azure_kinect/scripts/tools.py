import sounddevice as sd

def get_device_number(device_name_keyword):
    devices = sd.query_devices()
    for i, device in enumerate(devices):
        if device_name_keyword in device['name']:
            return i
    raise ValueError('Device not found')