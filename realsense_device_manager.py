import pyrealsense2 as rs
import numpy as np
from helpers import*
import cv2
from PIL import Image

class DeviceManager:
    def __init__(self, context, pipeline_configuration):
        """
        Class to manage the Intel RealSense devices
        Parameters:
        -----------
        context                 : rs.context()
                                  The context created for using the realsense library
        pipeline_configuration  : rs.config()
                                  The realsense library configuration to be used for the application
        """
        assert isinstance(context, type(rs.context()))
        assert isinstance(pipeline_configuration, type(rs.config()))
        self._context = context
        self._available_devices = enumerate_connected_devices(context)
        self._enabled_devices = {}
        self._config = pipeline_configuration
        self._frame_counter = 0

    def enable_device(self, device_serial, enable_ir_emitter=True):
        """
        Enable an Intel RealSense Device - ie. start the pipeline for that device
        Parameters:
        -----------
        device_serial     : string
                            Serial number of the realsense device
        enable_ir_emitter : bool
                            Enable/Disable the IR-Emitter of the device
        """
        pipeline = rs.pipeline()

        # Enable the device
        self._config.enable_device(device_serial)
        pipeline_profile = pipeline.start(self._config)

        # Set the acquisition parameters
        sensor = pipeline_profile.get_device().first_depth_sensor()
        sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
        self._enabled_devices[device_serial] = (Device(pipeline, pipeline_profile))

    def capture_color_array(self, device_serial):
        """
        returns a numpy array for color images for a specified device
                """
        device = self._enabled_devices[device_serial]
        for i in range(5):
            try:
                frames = device.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                color_array = np.asanyarray(color_frame.get_data())
                if color_array.any():
                    return color_array
            except RuntimeError:
                continue
        return None

    def display_color_image(self, device_serial):
        img = self.capture_color_array(device_serial)
        cv2.imwrite('color_img.jpg', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        cv2.imshow("image", cv2.cvtColor(img, cv2.COLOR_BGR2RGB));
        cv2.waitKey();

    def capture_depth_array(self, device_serial):
        """
               returns a numpy array for depth images for a specified device
                       """
        device = self._enabled_devices[device_serial]
        for i in range(5):
            try:
                frames = device.pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                depth_array = np.asanyarray(depth_frame.get_data())
                if depth_array.any():
                    return depth_array
            except RuntimeError:
                continue
        return None

    def capture_frames(self, device_serial):
        """
               returns color and depth frames for a specified device
                       """
        device = self._enabled_devices[device_serial]
        for i in range(5):
            try:
                frames = device.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color_frame and depth_frame:
                    return color_frame, depth_frame
            except RuntimeError:
                continue
        return None


    def enable_all_devices(self, enable_ir_emitter=True):
        """
        Enable all the Intel RealSense Devices which are connected to the PC - ie start the pipelines for all connected devices
        """
        print(str(len(self._available_devices)) + " devices have been found")

        for serial in self._available_devices:
            self.enable_device(serial, enable_ir_emitter)

    def enable_emitter(self, enable_ir_emitter):
        """
        Enable/disable the emitter of all of the intel realsense devices in the device manager
        """
        for (device_serial, device) in self._enabled_devices.items():
            # Get the active profile and enable the emitter for all the connected devices
            sensor = device.pipeline_profile.get_device().first_depth_sensor()
            sensor.set_option(rs.option.emitter_enabled, 1 if enable_ir_emitter else 0)
            if enable_ir_emitter:
                sensor.set_option(rs.option.laser_power, 330)

    def load_settings_json_all(self, path_to_settings_file):
        """
        Load the settings stored in the JSON file
        """
        print('Loading settings...')
        with open(path_to_settings_file, 'r') as file:
            json_text = file.read().strip()

        for (device_serial, device) in self._enabled_devices.items():
            # Get the active profile and load the json file which contains settings readable by the realsense
            print(device)
            device = device.pipeline_profile.get_device()
            advanced_mode = rs.rs400_advanced_mode(device)
            advanced_mode.load_json(json_text)

        print("Settings successfully loaded to all devices")

    def load_settings_json_single(self, device_serial, path_to_settings_file):
        """
        Load the settings stored in the JSON file to a single device
        """
        print('Loading settings...')
        with open(path_to_settings_file, 'r') as file:
            json_text = file.read().strip()
        device = self._enabled_devices[device_serial]
            # Get the active profile and load the json file which contains settings readable by the realsense
        device = device.pipeline_profile.get_device()
        advanced_mode = rs.rs400_advanced_mode(device)
        advanced_mode.load_json(json_text)
        print("Settings successfully loaded to device with serial: " + device_serial)

    def poll_frames(self):
        """
        Poll for frames from the enabled Intel RealSense devices. This will return at least one frame from each device.
        If temporal post processing is enabled, the depth stream is averaged over a certain amount of frames

        Parameters:
        -----------
        """
        frames = {}
        while len(frames) < len(self._enabled_devices.items()):
            for (serial, device) in self._enabled_devices.items():
                streams = device.pipeline_profile.get_streams()
                frameset = device.pipeline.poll_for_frames()  # frameset will be a pyrealsense2.composite_frame object
                if frameset.size() == len(streams):
                    frames[serial] = {}
                    for stream in streams:
                        if (rs.stream.infrared == stream.stream_type()):
                            frame = frameset.get_infrared_frame(stream.stream_index())
                            key_ = (stream.stream_type(), stream.stream_index())
                        else:
                            frame = frameset.first_or_default(stream.stream_type())
                            key_ = stream.stream_type()
                        frames[serial][key_] = frame

        return frames


    def get_depth_shape(self):
        """
        Retruns width and height of the depth stream for one arbitrary device
        Returns:
        -----------
        width : int
        height: int
        """
        width = -1
        height = -1
        for (serial, device) in self._enabled_devices.items():
            for stream in device.pipeline_profile.get_streams():
                if (rs.stream.depth == stream.stream_type()):
                    width = stream.as_video_stream_profile().width()
                    height = stream.as_video_stream_profile().height()
        return width, height

    def get_color_shape(self):
        """
        Returns width and height of the color stream for one arbitrary device
        """
        width = -1
        height = -1
        for (serial, device) in self._enabled_devices.items():
            for stream in device.pipeline_profile.get_streams():
                if (rs.stream.color == stream.stream_type()):
                    width = stream.as_video_stream_profile().width()
                    height = stream.as_video_stream_profile().height()
        return width, height

    def get_device_intrinsics(self, frames):
        """
        Get the intrinsics of the imager using its frame delivered by the realsense device
        Parameters:
        -----------
        frames : rs::frame
                 The frame grabbed from the imager inside the Intel RealSense for which the intrinsic is needed
        Return:
        -----------
        device_intrinsics : dict
        keys  : serial
                Serial number of the device
        values: [key]
                Intrinsics of the corresponding device
        """
        device_intrinsics = {}
        for (serial, frameset) in frames.items():
            device_intrinsics[serial] = {}
            for key, value in frameset.items():
                device_intrinsics[serial][key] = value.get_profile().as_video_stream_profile().get_intrinsics()
        return device_intrinsics

    def get_depth_to_color_extrinsics(self, frames):
        """
        Get the extrinsics between the depth imager 1 and the color imager using its frame delivered by the realsense device
        Parameters:
        -----------
        frames : rs::frame
                 The frame grabbed from the imager inside the Intel RealSense for which the intrinsic is needed
        Return:
        -----------
        device_intrinsics : dict
        keys  : serial
                Serial number of the device
        values: [key]
                Extrinsics of the corresponding device
        """
        device_extrinsics = {}
        for (serial, frameset) in frames.items():
            device_extrinsics[serial] = frameset[
                rs.stream.depth].get_profile().as_video_stream_profile().get_extrinsics_to(
                frameset[rs.stream.color].get_profile())
        return device_extrinsics

    def disable_streams(self):
        self._config.disable_all_streams()
