from realsense_device_manager import*
from helpers import*


c = rs.config()
c.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
c.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)

device_manager = DeviceManager(rs.context(), c)
device_manager.enable_all_devices()

device_manager.load_settings_json_all('settings.json')
print(enumerate_connected_devices(rs.context()))


