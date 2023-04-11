#!/usr/bin/env python

import rospy
import threading

from nepi_edge_sdk_base.idx_sensor_if import ROSIDXSensorIF
from nepi_edge_sdk_onvif.onvif_cam_driver import OnvifIFCamDriver

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
        if not rospy.has_param('~network/camera_ip'):
            rospy.logerr(self.node_name + ": Missing network/camera_ip parameter... cannot start")
            return
                
        username = rospy.get_param('~credentials/username')
        password = rospy.get_param('~credentials/password')
        camera_ip = rospy.get_param('~network/camera_ip')
        
        # Allow a default for the camera_port, since it is part of onvif spec.
        onvif_port = rospy.get_param('~network/camera_port', 80)
        rospy.set_param('~/network/camera_port', onvif_port)

        # Start the driver to connect to the camera
        rospy.loginfo(self.node_name + ": Launching driver... ")
        try:
            self.driver = OnvifIFCamDriver(username, password, camera_ip, onvif_port)
        except Exception as e:
            rospy.logerr("Failed to instantiate OnvifIFCamDriver... bad credentials?: " + str(e))
            return

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
        
    def createResolutionModeMapping(self):
        available_resolutions, _ = self.driver.getAvailableResolutions(video_encoder_id=0) # TODO: Configurable encoder selection? Multiples?
        # Check if this camera supports resolution adjustment
        if (len(available_resolutions) == 0):
            self.resolution_mode_map = {}
            return
        
        #available_resolutions is a list of dicts
        # Sort them by width from least to greatest
        available_resolutions = sorted(available_resolutions, key=lambda d: d['Width'])

        # Current policy is to take the lowest res, then fill out the rest with the highest
        available_resolution_count = len(available_resolutions)
        resolution_mode_count = ROSIDXSensorIF.RESOLUTION_MODE_MAX + 1
        self.resolution_mode_map = {0:available_resolutions[0]}
        for i in range(1,resolution_mode_count):
            res_index = (available_resolution_count - i) if (i < available_resolution_count) else (available_resolution_count - 1)
            self.resolution_mode_map[resolution_mode_count - i] = available_resolutions[res_index]

        rospy.loginfo(self.node_name + ": Resolution Modes" )
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
        rospy.loginfo(self.node_name + ": Setting resolution to " + str(onvif_resolution.Width) + "x" + str(onvif_resolution.Height))
        
        # Experimental: Try stopping/restarting capture
        img_acq_needs_restart = False
        if self.driver.imageAcquisitionRunning(uri_index=self.img_uri_index) is True: 
            self.img_uri_lock.acquire()
            self.driver.stopImageAcquisition(uri_index=self.img_uri_index)
            img_acq_needs_restart = True
                    
        ret, msg = self.driver.setResolution(onvif_resolution)
        
        if img_acq_needs_restart is True:
            self.driver.startImageAcquisition(uri_index=self.img_uri_index)
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
            return ret, msg, None
        
        self.color_img_acquisition_running = True
        
        frame, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None
        
        self.img_uri_lock.release()
        return ret, msg, frame
    
    def stopColorImg(self):
        self.img_uri_lock.acquire()
        # Don't stop acquisition if the b/w image is still being requested
        if self.bw_image_acquisition_running is False:
            ret,msg = self.driver.stopImageAcquisition(uri_index = self.img_uri_index)
        else:
            ret = True
            msg = "Success"
        self.color_img_acquisition_running = False
        self.img_uri_lock.release()
        return ret,msg
    
    def getBWImg(self):
        self.img_uri_lock.acquire()
        # Always try to start image acquisition -- no big deal if it was already started; driver returns quickly
        ret, msg = self.driver.startImageAcquisition(uri_index = self.img_uri_index)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None
        
        self.bw_image_acquisition_running = True

        frame, ret, msg = self.driver.getImage(uri_index = self.img_uri_index)
        if ret is False:
            self.img_uri_lock.release()
            return ret, msg, None
        
        self.img_uri_lock.release()
        return ret, msg, frame
    
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

            


        

