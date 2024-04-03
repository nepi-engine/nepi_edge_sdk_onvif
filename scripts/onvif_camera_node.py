#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import time
import rospy
import threading
import cv2

from nepi_edge_sdk_base.idx_sensor_if import ROSIDXSensorIF
from nepi_edge_sdk_onvif.onvif_cam_driver import ONVIF_GENERIC_DRIVER_ID, OnvifIFCamDriver
from nepi_edge_sdk_onvif.econ_routecam_driver import ECON_ROUTECAM_DRIVER_ID, EConRouteCamDriver

DRIVER_SPECIALIZATION_CONSTRUCTORS = {ONVIF_GENERIC_DRIVER_ID: OnvifIFCamDriver,
                                      ECON_ROUTECAM_DRIVER_ID: EConRouteCamDriver}
class OnvifCameraNode:
    DEFAULT_NODE_NAME = "onvif_camera_node"

    def __init__(self):
        # Launch the ROS node
        rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
        rospy.init_node(self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
        self.node_name = rospy.get_name().split('/')[-1]

        # Require the camera connection parameters to have been set
        if not rospy.has_param('~credentials/username'):
            rospy.logerr(self.node_name + ": Missing credentials/username parameter... cannot start")
            return
        if not rospy.has_param('~credentials/password'):
            rospy.logerr(self.node_name + ": Missing credentials/password parameter... cannot start")
            return
        if not rospy.has_param('~network/host'):
            rospy.logerr(self.node_name + ": Missing network/host parameter... cannot start")
            return
        if not rospy.has_param('~driver_id'):
            rospy.logerr(self.node_name + ": Missing driver_id parameter... cannot start")
            return
                
        username = str(rospy.get_param('~credentials/username'))
        password = str(rospy.get_param('~credentials/password'))
        host = str(rospy.get_param('~network/host'))
        
        # Allow a default for the port, since it is part of onvif spec.
        onvif_port = rospy.get_param('~network/port', 80)
        rospy.set_param('~/network/port', onvif_port)

        # Set up for specialized drivers here
        self.driver_id = rospy.get_param('~driver_id')
        if self.driver_id not in DRIVER_SPECIALIZATION_CONSTRUCTORS:
            rospy.logerr(self.node_name + ": unknown driver_id " + self.driver_id)
            return
        DriverConstructor = DRIVER_SPECIALIZATION_CONSTRUCTORS[self.driver_id]

        # Start the driver to connect to the camera
        rospy.loginfo(self.node_name + ": Launching " + self.driver_id + " driver")
        while not rospy.is_shutdown():
            try:
                self.driver = DriverConstructor(username, password, host, onvif_port)
                break
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                rospy.logerr_throttle(30, self.node_name + ": Failed to instantiate OnvifIFCamDriver... camera not online? bad credentials?: " + str(e))
                rospy.sleep(1)

        rospy.loginfo(self.node_name + ": ... Connected!")
        self.dev_info = self.driver.getDeviceInfo()
        self.logDeviceInfo()        
        # Configurable IDX parameter and data output remapping to support specific camera needs/capabilities
        # Don't edit this table directly -- do it through idx_remapping parameters
        idx_callback_names = {
            # IDX Standard
            "Resolution": self.setResolutionMode,
            "Framerate": self.setFramerateMode,
            "Contrast": self.setContrastRatio,
            "Brightness": self.setBrightnessRatio,
            "Thresholding": None, # No driver support, can be remapped though
            "Range": None, # No driver support, can be remapped though
            
            # Others are for remapping from non-standard IDX settings that are supported by the driver
            "ColorSaturation": self.setColorSaturation,
            "Sharpness": self.setSharpnessRatio,
            "ExposureTime": self.setExposureTimeRatio,
            "BacklightCompensation": self.setBacklightCompensationRatio,
            "WideDynamicRange": self.setWideDynamicRangeRatio,
                        
            # Data callbacks
            "Color2DImg": self.getColorImg,
            "StopColor2DImg": self.stopColorImg,
            "BW2DImg": self.getBWImg,
            "StopBW2DImg": self.stopBWImg,
            # Following have no driver support, can be remapped though
            "DepthMap": None, 
            "StopDepthMap": None,
            "DepthImg": None, 
            "StopDepthImg": None,
            "Pointcloud": None, 
            "StopPointcloud": None,
            "PointcloudImg": None, 
            "StopPointcloudImg": None
        }

        # Update available IDX callbacks based on capabilities that the driver reports
        if not self.driver.hasAdjustableResolution():
            idx_callback_names["Resolution"] = None # Clear the method
        else:
            self.createResolutionModeMapping()
        if not self.driver.hasAdjustableFramerate():
            idx_callback_names["Framerate"] = None # Clear the method
        else:
            self.createFramerateModeMapping()
        if not self.driver.hasAdjustableContrast():
            idx_callback_names["Contrast"] = None # Clear the method
        if not self.driver.hasAdjustableBrightness():
            idx_callback_names["Brightness"] = None # Clear the method
        if not self.driver.hasAdjustableSharpness():
            idx_callback_names["Sharpness"] = None # Clear the method
        if not self.driver.hasAdjustableExposureTime():
            idx_callback_names["ExposureTime"] = None # Clear the method
        if not self.driver.hasAdjustableBacklightCompensation():
            idx_callback_names["BacklightCompensation"] = None # Clear the method
        if not self.driver.hasAdjustableWideDynamicRange():
            idx_callback_names["WideDynamicRange"] = None # Clear the method

        # Now that we've updated the callbacks table, can apply the remappings
        idx_remappings = rospy.get_param('~idx_remappings', {})
        rospy.loginfo(self.node_name + ': Establishing IDX remappings')
        for from_name in idx_remappings:
            to_name = idx_remappings[from_name]
            if from_name not in idx_callback_names or to_name not in idx_callback_names:
                rospy.logwarn('\tInvalid IDX remapping: ' + from_name + '-->' + to_name)
            elif idx_callback_names[to_name] is None:
                rospy.logwarn('\tRemapping ' + from_name + ' to an unavailable adjustment (' + to_name + ')')
            else:
                idx_callback_names[from_name] = idx_callback_names[to_name]
                rospy.loginfo('\t' + from_name + '-->' + to_name)

        # Establish the URI indices (from ONVIF "Profiles") for the two image streams.
        # If these aren't the same, encoder param adjustments (resolution and framerate)
        # will only affect the first one.... so
        # TODO: Consider a scheme for adjusting parameters for separate streams independently
        # or in lock-step. Not sure if the uri_index and encoder_index have the same meaning
        self.img_uri_index = rospy.get_param('~/img_uri_index', 0)
        rospy.set_param('~/img_uri_index', self.img_uri_index)
        #self.bw_2d_img_uri_index = rospy.get_param('~/image_uris/bw_2d_img_uri_index', 0)
        #rospy.set_param('~/image_uris/bw_2d_img_uri_index', self.bw_2d_img_uri_index)

        # Create threading locks for each URI index (currently just 1) to provide threadsafety
        self.img_uri_lock = threading.Lock()
        self.color_image_acquisition_running = False
        self.bw_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None

        # Will hang forever waiting for the camera to be available
        rospy.loginfo(self.node_name + ": ... driver connected to camera")

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        rospy.loginfo(self.node_name + ": Launching NEPI IDX (ROS) interface...")
        self.idx_if = ROSIDXSensorIF(sensor_name=self.node_name,
                                     setResolutionModeCb=idx_callback_names["Resolution"], setFramerateModeCb=idx_callback_names["Framerate"], 
                                     setContrastCb=idx_callback_names["Contrast"], setBrightnessCb=idx_callback_names["Brightness"], 
                                     setThresholdingCb=idx_callback_names["Thresholding"], setRangeCb=idx_callback_names["Range"], 
                                     getColor2DImgCb=idx_callback_names["Color2DImg"], stopColor2DImgAcquisitionCb=idx_callback_names["StopColor2DImg"],
                                     getGrayscale2DImgCb=idx_callback_names["BW2DImg"], stopGrayscale2DImgAcquisitionCb=idx_callback_names["StopBW2DImg"],
                                     getDepthMapCb=idx_callback_names["DepthMap"], stopDepthMapAcquisitionCb=idx_callback_names["StopDepthMap"],
                                     getDepthImgCb=idx_callback_names["DepthImg"], stopDepthImgAcquisitionCb=idx_callback_names["StopDepthImg"],
                                     getPointcloudCb=idx_callback_names["Pointcloud"], stopPointcloudAcquisitionCb=idx_callback_names["StopPointcloud"],
                                     getPointcloudImgCb=idx_callback_names["PointcloudImg"], stopPointcloudImgAcquisitionCb=idx_callback_names["StopPointcloudImg"])
        rospy.loginfo(self.node_name + ": ... IDX interface running")
        
        # Now that all camera start-up stuff is processed, we can update the camera from the parameters that have been established
        self.idx_if.updateFromParamServer()

        # Now start the node
        rospy.spin()

    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        rospy.loginfo(dev_info_string)
                
    def createResolutionModeMapping(self):
        available_resolutions, _ = self.driver.getAvailableResolutions(video_encoder_id=0) # TODO: Configurable encoder selection? Multiples?
        # Check if this camera supports resolution adjustment
        if (len(available_resolutions) == 0):
            self.resolution_mode_map = {}
            return
        
        #available_resolutions is a list of dicts
        # Sort them by total pixel count from least to greatest
        available_resolutions = sorted(available_resolutions, key=lambda d: (int(d['Width'])*int(d['Height'])))
        
        # Current policy is to take the lowest res as LOW, highest as HIGH, then fill out the rest with evenly spaced intermediates
        available_resolution_count = len(available_resolutions)
        resolution_mode_count = ROSIDXSensorIF.RESOLUTION_MODE_MAX + 1
        self.resolution_mode_map = {0:available_resolutions[0]}
        self.resolution_mode_map[3] = available_resolutions[-1]
        if (available_resolution_count == 1):
            self.resolution_mode_map[1] = available_resolutions[0]
            self.resolution_mode_map[2] = available_resolutions[0]
        elif (available_resolution_count == 2):
            self.resolution_mode_map[1] = available_resolutions[0]
            self.resolution_mode_map[2] = available_resolutions[1]
        else:
            resolution_step = int(available_resolution_count / 3)            
            self.resolution_mode_map[1] = available_resolutions[resolution_step]
            self.resolution_mode_map[2] = available_resolutions[2*resolution_step]

        rospy.loginfo(self.node_name + ": Resolution Modes" )
        rospy.logerr('Debug ' + self.node_name + ": Resolution Modes" )
        for i in range(resolution_mode_count):
            rospy.loginfo("\t" + str(i) + ": " +  str(self.resolution_mode_map[i]['Width']) + 'x' + str(self.resolution_mode_map[i]['Height']))
    
    def createFramerateModeMapping(self):
        framerate_mode_count = ROSIDXSensorIF.FRAMERATE_MODE_MAX + 1
        framerate_range, _ = self.driver.getFramerateRange()
        if len(framerate_range) == 0:
            rospy.loginfo(self.node_name + ": detected framerate is not adjustable")
            # Indicates no settable framerate, so whatever the current config says is "max" is the only option
            max_framerate = self.driver.getFramerate()
            self.framerate_mode_map = {}
            for i in range(framerate_mode_count):
                self.framerate_mode_map[i] = max_framerate
        else:
            min_framerate = framerate_range["Min"]
            max_framerate = framerate_range["Max"]
            rospy.loginfo(self.node_name + ": detected framerate range = [" + str(min_framerate) + "," + str(max_framerate) + "]")
            self.framerate_mode_map = {0:min_framerate} # 0 (Low) is fixed at min framerate
            for i in range(1, framerate_mode_count):
                self.framerate_mode_map[framerate_mode_count - i] = int(round(float(max_framerate) / i))
        
        rospy.loginfo(self.node_name + ": Framerate Modes")
        for i in range(framerate_mode_count):
            rospy.loginfo("\t" + str(i) + ": " + str(self.framerate_mode_map[i]) + " fps")
    
    def setResolutionMode(self, mode):
        if (mode >= len(self.resolution_mode_map)):
            return False, "Invalid mode value"
        
        onvif_resolution = self.resolution_mode_map[mode]
        rospy.loginfo(self.node_name + ": Setting resolution to " + str(onvif_resolution['Width']) + "x" + str(onvif_resolution['Height']))
        
        # TODO: Stop and restart as part of the driver method, not here in the ROS node
        # Stop and  restart capture... seems many cameras require this, so just make it the universal
        img_acq_needs_restart = False
        if self.driver.imageAcquisitionRunning(uri_index=self.img_uri_index) is True: 
            self.img_uri_lock.acquire()
            self.driver.stopImageAcquisition(uri_index=self.img_uri_index)
            img_acq_needs_restart = True
                    
        ret, msg = self.driver.setResolution(onvif_resolution)
        
        if img_acq_needs_restart is True:
            while img_acq_needs_restart is True:
                ret, msg = self.driver.startImageAcquisition(uri_index=self.img_uri_index)
                if ret is True:
                    rospy.loginfo("Restarted image acquisition successfully")
                    img_acq_needs_restart = False
                else:
                    rospy.logwarn("Failed to restart image acquisition: " + msg)
                    rospy.sleep(1.0)
            self.img_uri_lock.release()

        return ret, msg
    
    def setFramerateMode(self, mode):
        if (mode >= len(self.framerate_mode_map)):
            return False, "Invalid mode value"
        
        onvif_framerate = self.framerate_mode_map[mode]
        rospy.loginfo(self.node_name + ": Setting framerate to " + str(onvif_framerate) + "fps")
        
        return (self.driver.setFramerate(onvif_framerate))
    
    def checkRatioBounds(self, ratio_val, has_auto_manual_setting = True):
        if ratio_val < 0.0 and has_auto_manual_setting is False:
            return False
        elif ratio_val > 1.0:
            return False
        
        return True

    def setContrastRatio(self, contrast_ratio):
        if (self.checkRatioBounds(contrast_ratio) is False):
            return False, "Invalid contrast ratio" + str(contrast_ratio)
        
        rospy.loginfo(self.node_name + ": Setting contrast ratio to " + str(contrast_ratio))
        return (self.driver.setScaledContrast(contrast_ratio))
    
    def setBrightnessRatio(self, brightness_ratio):
        if (self.checkRatioBounds(brightness_ratio) is False):
            return False, "Invalid brightness ratio"
        
        rospy.loginfo(self.node_name + ": Setting brightness ratio to " + str(brightness_ratio))
        return (self.driver.setScaledBrightness(brightness_ratio))
    
    def setColorSaturation(self, saturation_ratio):
        if (self.checkRatioBounds(saturation_ratio) is False):
            return False, "Invalid saturation ratio"
        
        rospy.loginfo(self.node_name + ": Setting saturation ratio to " + str(saturation_ratio))
        return (self.driver.setScaledColorSaturation(saturation_ratio))        
    
    def setSharpnessRatio(self, sharpness_ratio):
        if (self.checkRatioBounds(sharpness_ratio) is False):
            return False, "Invalid sharpness ratio"
        
        rospy.loginfo(self.node_name + ": Setting sharpness ratio to " + str(sharpness_ratio))
        return (self.driver.setScaledSharpness(sharpness_ratio))
    
    def setExposureTimeRatio(self, exposure_time_ratio):
        if (self.checkRatioBounds(exposure_time_ratio) is False):
            return False, "Invalid exposure time ratio"
        
        auto_mode = (exposure_time_ratio == -1.0)
        rospy.loginfo(self.node_name + ": Setting exposure time ratio to " + str(exposure_time_ratio))
        return (self.driver.setExposureSettings(auto_mode, exposure_time_ratio))
    
    def setBacklightCompensationRatio(self, backlight_compensation_ratio):
        if (self.checkRatioBounds(backlight_compensation_ratio) is False):
            return False, "Invalid backlight compensation ratio"
        
        rospy.loginfo(self.node_name + ": Setting backlight compensation ratio to " + str(backlight_compensation_ratio))
        return (self.driver.setBacklightCompensation(True, backlight_compensation_ratio))
    
    def setWideDynamicRangeRatio(self, wide_dynamic_range_ratio):
        if (self.checkRatioBounds(wide_dynamic_range_ratio) is False):
            return False, "Invalid wide dynamic range ratio"
        
        enabled = (wide_dynamic_range_ratio != -1.0)
        ratio = wide_dynamic_range_ratio if enabled is True else 0.0
        rospy.loginfo(self.node_name + ": Setting wide dynamic range ratio to " + (str(wide_dynamic_range_ratio) if enabled is True else "OFF"))
        return (self.driver.setWideDynamicRange(enabled, ratio))        
    
    def getColorImg(self):
        self.img_uri_lock.acquire()
        # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
        ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None, None
        
        self.color_image_acquisition_running = True

        timestamp = None
        
        start = time.time()
        frame, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
        stop = time.time()
        #print('GI: ', stop - start)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None, None
        
        if timestamp is not None:
            ros_timestamp = rospy.Time.from_sec(timestamp)
        else:
            ros_timestamp = rospy.Time.now()
        
        # Make a copy for the bw thread to use rather than grabbing a new frame
        if self.bw_image_acquisition_running:
            self.cached_2d_color_frame = frame
            self.cached_2d_color_frame_timestamp = ros_timestamp

        self.img_uri_lock.release()
        return ret, msg, frame, ros_timestamp
    
    def stopColorImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        if self.bw_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        else:
            ret = True
            msg = "Success"
        self.color_image_acquisition_running = False
        self.cached_2d_color_frame = None
        self.cached_2d_color_frame_timestamp = None
        self.img_uri_lock.release()
        return ret,msg
    
    def getBWImg(self):
        self.img_uri_lock.acquire()
        # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
        ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None, None
        
        self.bw_image_acquisition_running = True

        ros_timestamp = None
        
        # Only grab a frame if we don't already have a cached color frame... avoids cutting the update rate in half when
        # both image streams are running
        if self.color_image_acquisition_running is False or self.cached_2d_color_frame is None:
            #rospy.logwarn("Debugging: getBWImg acquiring")
            frame, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
            if timestamp is not None:
                ros_timestamp = rospy.Time.from_sec(timestamp)
            else:
                ros_timestamp = rospy.Time.now()
        else:
            #rospy.logwarn("Debugging: getBWImg reusing")
            frame = self.cached_2d_color_frame.copy()
            ros_timestamp = self.cached_2d_color_frame_timestamp
            self.cached_2d_color_frame = None # Clear it to avoid using it multiple times in the event that threads are running at different rates
            self.cached_2d_color_frame_timestamp = None
            ret = True
            msg = "Success: Reusing cached frame"

        self.img_uri_lock.release()

        # Abort if there was some error or issue in acquiring the image
        if ret is False or frame is None:
            return False, msg, None, None

        # Fix the channel count if necessary
        if frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        return ret, msg, frame, ros_timestamp
    
    def stopBWImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the color image is still being requested
        if self.color_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        else:
            ret = True
            msg = "Success"
        self.bw_image_acquisition_running = False
        self.img_uri_lock.release()
        return ret, msg

if __name__ == '__main__':
	node = OnvifCameraNode()

            


        

