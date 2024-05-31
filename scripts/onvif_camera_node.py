#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#
import sys
import time
import math
import rospy
import threading
import cv2

from nepi_edge_sdk_base.idx_sensor_if import ROSIDXSensorIF
from nepi_edge_sdk_onvif.onvif_cam_driver import ONVIF_GENERIC_DRIVER_ID, OnvifIFCamDriver
from nepi_edge_sdk_onvif.econ_routecam_driver import ECON_ROUTECAM_DRIVER_ID, EConRouteCamDriver

from nepi_edge_sdk_base import nepi_img
from nepi_edge_sdk_base import nepi_nex

DRIVER_SPECIALIZATION_CONSTRUCTORS = {ONVIF_GENERIC_DRIVER_ID: OnvifIFCamDriver,
                                      ECON_ROUTECAM_DRIVER_ID: EConRouteCamDriver}
class OnvifCameraNode:
    DEFAULT_NODE_NAME = "onvif_camera_node"

    FACTORY_SETTINGS = nepi_nex.TEST_SETTINGS

    #Factory Control Values 
    FACTORY_CONTROLS = dict( controls_enable = True,
    auto_adjust = False,
    brightness_ratio = 0.5,
    contrast_ratio =  0.5,
    threshold_ratio =  0.0,
    resolution_mode = 3, # LOW, MED, HIGH, MAX
    framerate_mode = 3, # LOW, MED, HIGH, MAX
    start_range_ratio = None, 
    stop_range_ratio = None,
    min_range_m = None,
    max_range_m = None,
    zoom_ratio = None, 
    rotate_ratio = None,
    frame_id = None
    )
 
    DEFAULT_CURRENT_FPS = 20 # Will be update later with actual

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
            "Controls" : {
                # IDX Standard
                "Controls_Enable":  self.setControlsEnable,
                "Auto_Adjust":  self.setAutoAdjust,
                "Brightness": self.setBrightness,
                "Contrast":  self.setContrast,
                "Thresholding": self.setThresholding,
                "Resolution": self.setResolutionMode,
                "Framerate":  self.setFramerateMode,
                "Range":  None
            },
            

            "Data" : {
                # Data callbacks
                "Color2DImg": self.getColorImg,
                "StopColor2DImg": self.stopColorImg,
                "BW2DImg": self.getBWImg,
                "StopBW2DImg": self.stopBWImg,
                "DepthMap": None, 
                "StopDepthMap": None,
                "DepthImg": None, 
                "StopDepthImg": None,
                "Pointcloud": None, 
                "StopPointcloud": None,
                "PointcloudImg": None, 
                "StopPointcloudImg": None,
                "GPS": None,
                "Odom": None,
                "Heading": None,
            }
        }

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


        # Initialize controls
        self.factory_controls = self.FACTORY_CONTROLS
        self.current_controls = self.factory_controls # Updateded during initialization
        self.current_fps = self.DEFAULT_CURRENT_FPS # Should be updateded when settings read

        # Initialize settings
        self.caps_settings = nepi_nex.TEST_CAP_SETTINGS 
        self.factory_settings = self.FACTORY_SETTINGS
        self.current_settings = [] # Updated 
        for setting in self.factory_settings:
          self.settingsUpdateFunction(setting)

        # Launch the IDX interface --  this takes care of initializing all the camera settings from config. file
        rospy.loginfo(self.node_name + ": Launching NEPI IDX (ROS) interface...")
        self.idx_if = ROSIDXSensorIF(sensor_name=self.node_name,
                                     capSettings = self.caps_settings,
                                     factorySettings = self.factory_settings,
                                     settingsUpdateFunction=self.settingsUpdateFunction,
                                     getSettings=self.getSettings,
                                     factoryControls = self.FACTORY_CONTROLS,
                                     setControlsEnable = idx_callback_names["Controls"]["Controls_Enable"],
                                     setAutoAdjust= idx_callback_names["Controls"]["Auto_Adjust"],
                                     setResolutionMode=idx_callback_names["Controls"]["Resolution"], 
                                     setFramerateMode=idx_callback_names["Controls"]["Framerate"], 
                                     setContrast=idx_callback_names["Controls"]["Contrast"], 
                                     setBrightness=idx_callback_names["Controls"]["Brightness"], 
                                     setThresholding=idx_callback_names["Controls"]["Thresholding"], 
                                     setRange=idx_callback_names["Controls"]["Range"], 
                                     getColor2DImg=idx_callback_names["Data"]["Color2DImg"], 
                                     stopColor2DImgAcquisition=idx_callback_names["Data"]["StopColor2DImg"],
                                     getBW2DImg=idx_callback_names["Data"]["BW2DImg"], 
                                     stopBW2DImgAcquisition=idx_callback_names["Data"]["StopBW2DImg"],
                                     getDepthMap=idx_callback_names["Data"]["DepthMap"], 
                                     stopDepthMapAcquisition=idx_callback_names["Data"]["StopDepthMap"],
                                     getDepthImg=idx_callback_names["Data"]["DepthImg"], 
                                     stopDepthImgAcquisition=idx_callback_names["Data"]["StopDepthImg"],
                                     getPointcloud=idx_callback_names["Data"]["Pointcloud"], 
                                     stopPointcloudAcquisition=idx_callback_names["Data"]["StopPointcloud"],
                                     getPointcloudImg=idx_callback_names["Data"]["PointcloudImg"], 
                                     stopPointcloudImgAcquisition=idx_callback_names["Data"]["StopPointcloudImg"],
                                     getGPSMsg=idx_callback_names["Data"]["GPS"],
                                     getOdomMsg=idx_callback_names["Data"]["Odom"],
                                     getHeadingMsg=idx_callback_names["Data"]["Heading"])
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
                
    def getCapSettings(self):
        cap_settings = nepi_nex.TEST_CAP_SETTINGS #.NONE_SETTINGS
        # Replace with get cap settings process
        return cap_settings

    def settingsUpdateFunction(self,setting):
        success = False
        self.current_settings = nepi_nex.update_setting_in_settings(setting,self.current_settings)
        success = True
        return success
    
    def getSettings(self):
        settings = self.current_settings
        # Replace with get settings process
        return settings

    def getCapSettings(self):
        cap_settings = nepi_nex.TEST_CAP_SETTINGS #.NONE_SETTINGS
        # Replace with get cap settings process
        return cap_settings

    def settingsUpdateFunction(self,setting):
        success = False
        self.current_settings = nepi_nex.update_setting_in_settings(setting,self.current_settings)
        success = True
        return success
    
    def getSettings(self):
        settings = self.current_settings
        # Replace with get settings process
        return settings
    
    def setControlsEnable(self, enable):
        self.current_controls["controls_enable"] = enable
        status = True
        err_str = ""
        return status, err_str
        
    def setAutoAdjust(self, enable):
        ret = self.current_controls["auto_adjust"] = enable
        status = True
        err_str = ""
        return status, err_str

    def setBrightness(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
        self.current_controls["brightness_ratio"] = ratio
        status = True
        err_str = ""
        return status, err_str

    def setContrast(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
        self.current_controls["contrast_ratio"] = ratio
        status = True
        err_str = ""
        return status, err_str

    def setThresholding(self, ratio):
        if ratio > 1:
            ratio = 1
        elif ratio < 0:
            ratio = 0
        self.current_controls["threshold_ratio"] = ratio
        status = True
        err_str = ""
        return status, err_str

    def setResolutionMode(self, mode):
        if (mode > self.idx_if.RESOLUTION_MODE_MAX):
            return False, "Invalid mode value"
        self.current_controls["resolution_mode"] = mode
        status = True
        err_str = ""
        return status, err_str
    
    def setFramerateMode(self, mode):
        if (mode > self.idx_if.FRAMERATE_MODE_MAX):
            return False, "Invalid mode value"
        self.current_controls["framerate_mode"] = mode
        status = True
        err_str = ""
        return status, err_str
    
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
        cv2_img, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
        stop = time.time()
        #print('GI: ', stop - start)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None, None
        
        if timestamp is not None:
            ros_timestamp = rospy.Time.from_sec(timestamp)
        else:
            ros_timestamp = rospy.Time.now()

        # Apply controls
        if self.current_controls.get("controls_enable") and cv2_img is not None:
          cv2_img = nepi_nex.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)        
        
        # Make a copy for the bw thread to use rather than grabbing a new cv2_img
        if self.bw_image_acquisition_running:
            self.cached_2d_color_frame = cv2_img
            self.cached_2d_color_frame_timestamp = ros_timestamp

        self.img_uri_lock.release()
        return ret, msg, cv2_img, ros_timestamp
    
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
            cv2_img, timestamp, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
            if timestamp is not None:
                ros_timestamp = rospy.Time.from_sec(timestamp)
            else:
                ros_timestamp = rospy.Time.now()
            # Apply controls
            if self.current_controls.get("controls_enable") and cv2_img is not None:
                cv2_img = nepi_nex.applyIDXControls2Image(cv2_img,self.current_controls,self.current_fps)
        else:
            #rospy.logwarn("Debugging: getBWImg reusing")
            cv2_img = self.cached_2d_color_frame.copy()
            ros_timestamp = self.cached_2d_color_frame_timestamp
            self.cached_2d_color_frame = None # Clear it to avoid using it multiple times in the event that threads are running at different rates
            self.cached_2d_color_frame_timestamp = None
            ret = True
            msg = "Success: Reusing cached cv2_img"

        self.img_uri_lock.release()

        # Abort if there was some error or issue in acquiring the image
        if ret is False or cv2_img is None:
            return False, msg, None, None

        # Fix the channel count if necessary
        if cv2_img.ndim == 3:
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
        
        return ret, msg, cv2_img, ros_timestamp
    
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

            


        

