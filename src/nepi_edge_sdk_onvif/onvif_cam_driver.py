#!/usr/bin/env python

import time
import cv2
import threading
import os
from onvif import ONVIFCamera # python-onvif

# Utilities
def computeScaledFromUnscaled(unscaled, min_unscaled, max_unscaled):
    scaled = (unscaled - min_unscaled) / (max_unscaled - min_unscaled)
    return scaled

def computeUnscaledFromScaled(scaled, min_unscaled, max_unscaled, force_integer=True):
    unscaled = (scaled * (max_unscaled - min_unscaled)) + min_unscaled
    if force_integer is True:
        unscaled = round(unscaled)
    return unscaled

ONVIF_GENERIC_DRIVER_ID = 'GenericONVIF'

class OnvifIFCamDriver(object):
    #WSDL_FOLDER = "/home/josh/Desktop/Work/tmp/onvif_practice/python-onvif/wsdl/" # TODO: Update to the real location as installed via python-onvif
    WSDL_FOLDER = "/opt/nepi/ros/etc/onvif/wsdl/"
    RTSP_PORT = 554
    MAX_CONSEC_FRAME_FAIL_COUNT = 3

    def __init__(self, username, password, ip_addr, port=80):
        self.username = username
        self.password = password

        # Connect to the camera
        self.cam = ONVIFCamera(ip_addr, port, username, password, self.WSDL_FOLDER)
        # The devicemgt service gets set up automatically
        self.device_info = self.cam.devicemgmt.GetDeviceInformation()
        self.device_capabilities = self.cam.devicemgmt.GetCapabilities()
        #print("Debugging: Sys Capabilities = " + str(self.device_capabilities))
        
        # Other services require manual setup
        # TODO: Are all of these services guaranteed to exist, or should we use self.cam.devicemgmt.GetServices() to check?
        self.media_service = self.cam.create_media_service()
        self.imaging_service = self.cam.create_imaging_service()
                
        # Get the video source For now, work with just the first video source
        self.video_source = self.media_service.GetVideoSources()[0]

        # And the imaging settings (brightness, saturation, etc.) and options associated with this video source
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        #self.imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)
        self.imaging_options = self.imaging_service.GetOptions(video_src_token_param)
        #print("Debugging: Imaging Options = " + str(self.imaging_options))
        
        # Debugging only
        #self.media_service.GetVideoEncoderConfigurations()))
        
        #print("Debugging: Current encoder configurations = " + str(self.media_service.GetVideoEncoderConfigurations()))
        
        # Get the encoder options
        # First time is just to get the count to see if encoder configuration is supported
        try:
            # Appears that you must query these for specific encoders -- providing no ConfigurationToken argument yields a much smaller set of options
            # ... so just query the first encoder
            video_encoder_token_param = {"ConfigurationToken":self.media_service.GetVideoEncoderConfigurations()[0]._token}
            self.encoder_options = self.media_service.GetVideoEncoderConfigurationOptions(video_encoder_token_param)
        except:
            # Probably "list index 0 out of range" due to no encoder configurations for this camera... that's okay, just means no framerate or resolution adj.
            self.encoder_options = []
        
        self.encoder_count = len(self.encoder_options) # Will just allow the primary (first) encoder for now
        #print("Debugging: Encoder Options = " + str(self.encoder_options))

        # Get the available media profiles
        self.profiles = self.media_service.GetProfiles()
        # And query the RTSP URLs for each
        self.rtsp_uris = []
        self.rtsp_caps = []
        self.img_uri_locks = []
        self.latest_frames = []
        self.latest_frame_timestamps = []
        self.latest_frame_successes = []
        for p in self.profiles:
            params = {"StreamSetup":{"Stream":"RTP-Unicast", "Transport":{"Protocol":"RTSP"}}, "ProfileToken":p._token}
            uri = self.media_service.GetStreamUri(params).Uri
            self.rtsp_uris.append(uri)
            self.rtsp_caps.append(None) # Empty until streaming is started
            self.img_uri_locks.append(threading.Lock())
            self.latest_frames.append(None)
            self.latest_frame_timestamps.append(None)
            self.latest_frame_successes.append(False)

        #print("Debugging: RTSP URIs = " + str(self.rtsp_uris))
        self.consec_failed_frames = {}

        # Ensure that we use UDP for image transport via the FFMPEG API Backend to cv2.VideoCapture
        #os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
        # Testing: Seems TCP better in terms of reducing decode errors, but what is the framerate/latency cost?
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

        # Start and stop the acquisition to "prime" it (seems to help with initial-frame decoding errors) and ensure that the RTSP server is actually reachable
        ret, msg = self.startImageAcquisition(0)
        if ret is False:
            raise RuntimeError("Failed to prime image acquisition: " + msg)
        self.stopImageAcquisition(0)

    def getDeviceInfo(self):
        return self.device_info

    def imageAcquisitionRunning(self, uri_index = 0):
        if uri_index < 0 or uri_index > len(self.rtsp_uris) - 1:
            return False
        
        return (self.rtsp_caps[uri_index] != None)

    def startImageAcquisition(self, uri_index = 0):
        if uri_index < 0 or uri_index > len(self.rtsp_uris) - 1:
            return False, "Invalid URI index: " + str(uri_index)
        
        self.img_uri_locks[uri_index].acquire()
        if self.rtsp_caps[uri_index] != None:
            self.img_uri_locks[uri_index].release()
            return True, "Already capturing from " + self.rtsp_uris[uri_index]
        
        # Create the OpenCV using FFMPEG as the backend API -- appears that the default anyway, but just force it here
        self.rtsp_caps[uri_index] = cv2.VideoCapture(self.rtsp_uris[uri_index], cv2.CAP_FFMPEG)
        
        # Doesn't seem to work with this GStreamer API instead -- just blocks forever on image acq
        # self.rtsp_caps[uri_index] = cv2.VideoCapture(self.rtsp_uris[uri_index], cv2.CAP_GSTREAMER) 
        
        #print("!!!!!!!! Debug: Backend is " + self.rtsp_caps[uri_index].getBackendName() + "!!!!!!!")
        if not self.rtsp_caps[uri_index].isOpened():
            # Try adding username and password to URI per RTSP standards
            uri_pre = self.rtsp_uris[uri_index].split('//')[0]
            uri_post = self.rtsp_uris[uri_index].split('//')[1]
            secure_uri = uri_pre + self.username + ":" + self.password + "@" + uri_post
            self.rtsp_caps[uri_index] = cv2.VideoCapture(secure_uri)
            if not self.rtsp_caps[uri_index].isOpened():
                self.rtsp_caps[uri_index].release()
                self.rtsp_caps[uri_index] = None
                self.img_uri_locks[uri_index].release()
                return False, "Failed to start capture from " + self.rtsp_uris[uri_index] + " or " + secure_uri
            # But if it worked, just update the URI
            self.rtsp_uris[uri_index] = secure_uri
                
        # Experimental: Try setting a small buffer size for reduced latency
        self.rtsp_caps[uri_index].set(cv2.CAP_PROP_BUFFERSIZE, 3) # Don't think this does anything

        self.img_uri_locks[uri_index].release()
        self.latest_frames[uri_index] = None
        self.latest_frame_timestamps[uri_index] = None
        self.latest_frame_successes[uri_index] = False
        # Experimental: Launch a separate thread to grab frames as quickly as possible so that the buffer is empty when downstream
        # client actually wants a real image
        self.img_acq_thread = threading.Thread(target=self.runImgAcqThread, args=[uri_index])
        self.img_acq_thread.daemon = True
        self.img_acq_thread.start()
        
        self.consec_failed_frames[uri_index] = 0
        return True, "Success"

    def runImgAcqThread(self, uri_index):
        if uri_index < 0 or uri_index > len(self.rtsp_uris) - 1:
            return

        if self.rtsp_caps[uri_index] is None or self.rtsp_caps[uri_index].isOpened() is False:
            return
        
        keep_going = True
        while(keep_going):
            self.img_uri_locks[uri_index].acquire()
            if self.rtsp_caps[uri_index] is None:
                keep_going = False
            else:
                # Acquire without decoding via grab(). Image is available via a subsequent retrieve()
                start = time.time()
                self.latest_frame_successes[uri_index] = self.rtsp_caps[uri_index].grab()
                self.latest_frame_timestamps[uri_index] = time.time()
                stop = self.latest_frame_timestamps[uri_index]
                #print('G: ', stop - start)
                                
            self.img_uri_locks[uri_index].release()


            time.sleep(0.01)
    
    def stopImageAcquisition(self, uri_index = 0):
        if uri_index < 0 or uri_index > len(self.rtsp_uris) - 1:
            return False, "Invalid URI index: " + str(uri_index)
        
        self.img_uri_locks[uri_index].acquire()
        if self.rtsp_caps[uri_index] == None:
            self.img_uri_locks[uri_index].release()
            return True, "No current capture from " + self.rtsp_uris[uri_index]
        
        self.rtsp_caps[uri_index].release()
        self.rtsp_caps[uri_index] = None
        self.img_uri_locks[uri_index].release()
        return True, "Success"
    
    def getImage(self, uri_index = 0):
        if uri_index < 0 or uri_index > len(self.rtsp_uris) - 1:
            return None, None, False, "Invalid URI index: " + str(uri_index)

        if self.rtsp_caps[uri_index] is None or self.rtsp_caps[uri_index].isOpened() is False:
           return None, None, False, "Capture for " + self.rtsp_uris[uri_index] + " not opened"
        
        #ret, frame = self.rtsp_caps[uri_index].read()
        
        # Experimental -- Just decode and return latest grabbed by acquisition thread
        self.img_uri_locks[uri_index].acquire()
        if self.latest_frame_successes[uri_index] is True:
            start = time.time()
            ret, self.latest_frames[uri_index] = self.rtsp_caps[uri_index].retrieve()
            stop = time.time()
            #print('R: ', stop - start)
        else:
            ret = False
            self.latest_frames[uri_index] = None
        self.img_uri_locks[uri_index].release()
        if not ret:
            self.consec_failed_frames[uri_index] += 1
            if self.consec_failed_frames < self.MAX_CONSEC_FRAME_FAIL_COUNT:
                return None, None, False, "Failed to read next frame for " + self.rtsp_uris[uri_index]
            else:
                self.stopImageAcquisition(uri_index)
                self.startImageAcquisition(uri_index)
                return None, None, False, "Failed to read next frame " + str(self.MAX_CONSEC_FRAME_FAIL_COUNT) + "times consec... auto-restarting image acquisition"
        
        return self.latest_frames[uri_index], self.latest_frame_timestamps[uri_index], True, "Success"
    
    def getEncoding(self, video_encoder_id=0):
        if self.encoder_count == 0:
            return "", None
        
        encoder_cfg = self.media_service.GetVideoEncoderConfigurations()[video_encoder_id]
        encoding = encoder_cfg["Encoding"]
        #print("Debugging: Encoder cfg = " + str(encoder_cfg))
        return encoding, encoder_cfg
     
    def hasAdjustableResolution(self):
        resolutions, _ = self.getAvailableResolutions()
        return (len(resolutions) > 1)
    
    def hasAdjustableFramerate(self):
        framerate_range, _ = self.getFramerateRange()
        return (len(framerate_range) > 1)

    def hasAdjustableImagingSetting(self, onvif_setting_name, onvif_subsetting_name=None):
        if not hasattr(self.imaging_options, onvif_setting_name):
            return False
        
        if (onvif_subsetting_name is not None) and (not hasattr(self.imaging_options[onvif_setting_name], onvif_subsetting_name)):
            return False
        
        return True                                        
                
    def hasAdjustableContrast(self):
        return self.hasAdjustableImagingSetting("Contrast")
        
    def hasAdjustableBrightness(self):
        return self.hasAdjustableImagingSetting("Brightness")
    
    def hasAdjustableSharpness(self):
        return self.hasAdjustableImagingSetting("Sharpness")
        
    def hasAdjustableExposureTime(self):
        return (self.hasAdjustableImagingSetting("Exposure", "Mode") and self.hasAdjustableImagingSetting("Exposure", "ExposureTime"))
        
    def hasAdjustableBacklightCompensation(self):
        return self.hasAdjustableImagingSetting("BacklightCompensation", "Mode") and self.hasAdjustableImagingSetting("BacklightCompensation" and "Level")
        
    def hasAdjustableWideDynamicRange(self):
        self.hasAdjustableImagingSetting("WideDynamicRange", "Mode") and self.hasAdjustableImagingSetting("WideDynamicRange" and "Level")

    def getAvailableResolutions(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return [], None
        
        # First, figure out what compression format is available for this encoder
        encoding, encoder_cfg = self.getEncoding(video_encoder_id)
                        
        if not hasattr(self.encoder_options, encoding):
            return [], encoder_cfg

        # Now figure out which resolutions are available for this compression... just use the member variable since "options" don't change
        available_resolutions = self.encoder_options[encoding].ResolutionsAvailable
        # available_resolution: List of dictionaries: [{Width:x,Height:y}, ...]
        return available_resolutions, encoder_cfg # Convenient to return the encoder config, too
        
    
    def getResolution(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return []
        
        encoder_cfg = self.media_service.GetVideoEncoderConfigurations()[video_encoder_id]
        
        # resolution: Dictionary: [{Width:x, Height:x}]
        resolution = encoder_cfg.Resolution
        return resolution

    # Method broken for SS400 -- camera nacks the request packets
    def setResolution(self, resolution, video_encoder_id=0, readback_check=True):
        if video_encoder_id >= self.encoder_count:
            return False, "Device does not support resolution adjustment"

        # First, get the available resolutions and current encoder_cfg so that we can update just the resolution
        available_resolutions, encoder_cfg = self.getAvailableResolutions(video_encoder_id)
        
        # Ensure that the specified resolution is available
        if resolution not in available_resolutions:
            return False, "Invalid resolution"

        # Check whether we need to make a change
        if resolution.Width == encoder_cfg.Resolution.Width and resolution.Height == encoder_cfg.Resolution.Height:
            return True, "Desired resolution already set"

        # Update the encoder_cfg
        encoder_cfg.Resolution = resolution

        # Create the request
        request = self.media_service.create_type('SetVideoEncoderConfiguration')
        request.Configuration = encoder_cfg
        self.media_service.SetVideoEncoderConfiguration(request)

        # Check
        if readback_check is True:
            encoder_cfg = self.media_service.GetVideoEncoderConfigurations()[video_encoder_id]
            if resolution.Width != encoder_cfg.Resolution.Width or resolution.Height != encoder_cfg.Resolution.Height:
                return False, "Camera failed update resolution"

        return True, "Success"
    
    def getFramerateRange(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return {}, None
        
        # First, figure out what compression format is available for this encoder
        encoding, encoder_cfg = self.getEncoding(video_encoder_id)
                
        if not hasattr(self.encoder_options, encoding):
            return {}, encoder_cfg 

        range = self.encoder_options[encoding].FrameRateRange
        
        #TODO: Framerate range can be a function of resolution, as ONWOTE camera displays in its encoder_cfg_extensions. How to handle?
        return range, encoder_cfg  
    
    def getFramerate(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return -1.0
        
        encoder_cfg = self.media_service.GetVideoEncoderConfigurations()[video_encoder_id]
        #print("Debugging: Encoder cfg = " + str(encoder_cfg))

        if not hasattr(encoder_cfg, "RateControl"):
            return -1.0
        
        max_fps = encoder_cfg.RateControl.FrameRateLimit
        return max_fps
    
    def setFramerate(self, max_fps, video_encoder_id=0, readback_check=True):
        if video_encoder_id >= self.encoder_count:
            return False, "Device does not support framerate adjustment"

        if max_fps < 0:
            return False, "Invalid negative value for max framerate"
        
        encoder_cfg = self.media_service.GetVideoEncoderConfigurations()[video_encoder_id]

        if not hasattr(encoder_cfg, "RateControl"):
            return False, "Camera does not support rate control"
        
        # Check whether we need to make a change
        if max_fps == encoder_cfg.RateControl.FrameRateLimit:
            return True, "Desired framerate already set"
        
        encoder_cfg.RateControl.FrameRateLimit = max_fps
        request = self.media_service.create_type('SetVideoEncoderConfiguration')
        request.Configuration = encoder_cfg
        try:
            self.media_service.SetVideoEncoderConfiguration(request)
        except Exception as e:
            return False, "Onvif Error: Failed to set the frame rate to " + str(max_fps) + " - " + e.what()
        
        # Create an update param set and submit
        #update_params = {"token":encoder_cfg._token, "RateControl":{"FrameRateLimit":max_fps}}
        # Just testing
        #update_params = {"Configuration": {"token":encoder_cfg._token}}
        #self.media_service.SetVideoEncoderConfiguration(update_params)

        if readback_check is True:
            encoder_cfg = self.media_service.GetVideoEncoderConfigurations()[video_encoder_id]
            if encoder_cfg.RateControl.FrameRateLimit != max_fps:
                return False, "Camera failed to update max framerate"
            
        return True, "Success"

    def getScaledImageSetting(self, onvif_setting_name, onvif_setting_subname=None):
        # Make sure to query the system first
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)

        if not hasattr(imaging_settings, onvif_setting_name):
            return -1.0
        
        unscaled = imaging_settings[onvif_setting_name]
        scaled = computeScaledFromUnscaled(unscaled, self.imaging_options[onvif_setting_name].Min, self.imaging_options[onvif_setting_name].Max)
        return scaled
    
    def setScaledImageSetting(self, onvif_setting_name, scaled_value, readback_check=True, force_integer=True):
        # Bounds checking:
        if scaled_value < 0.0 or scaled_value > 1.0:
            return False, "Invalid scaled value for " + onvif_setting_name

        # Make sure to query the system first
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)
        #print("Debugging: Imaging Settings: ")
        #print(imaging_settings)

        if not hasattr(imaging_settings, onvif_setting_name):
            return False, "Unavailable setting: " + onvif_setting_name
        
        unscaled = computeUnscaledFromScaled(scaled_value, self.imaging_options[onvif_setting_name].Min, self.imaging_options[onvif_setting_name].Max, force_integer)
                
        if unscaled == imaging_settings[onvif_setting_name]:
            return True, "Success: Desired " + onvif_setting_name + " already set"
        
        new_imaging_settings = {"VideoSourceToken":self.video_source._token, "ImagingSettings":{onvif_setting_name: unscaled}}
        try:
            self.imaging_service.SetImagingSettings(new_imaging_settings)
        except Exception as e:
            return False, "Onvif error: Failed to update " + onvif_setting_name + " - " + e.what()
        
        if readback_check is True:
            unscaled_readback = self.imaging_service.GetImagingSettings(video_src_token_param)[onvif_setting_name]
            if (unscaled_readback != unscaled):
                #print("Debugging: Attempted = " + str(unscaled) + ", Returned = " + str(unscaled_readback))
                #print("Debugging: Limits = [" + str(self.imaging_options[onvif_setting_name].Min) + "," + str(self.imaging_options[onvif_setting_name].Max) + "]")
                return False, onvif_setting_name + " not updated by camera"
                
        return True, "Success"

    def getEnabledAndScaledImageSetting(self, onvif_setting_name, onvif_setting_enabled_name, onvif_setting_enabled_val, onvif_setting_level_name):
        # Make sure to query the system first
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)

        if not hasattr(imaging_settings, onvif_setting_name):
            return -1.0
        
        enabled = (imaging_settings[onvif_setting_name][onvif_setting_enabled_name] == onvif_setting_enabled_val)
        unscaled = imaging_settings[onvif_setting_name][onvif_setting_level_name]
        scaled = computeScaledFromUnscaled(unscaled, 
                                           self.imaging_options[onvif_setting_name][onvif_setting_level_name].Min, 
                                           self.imaging_options[onvif_setting_name][onvif_setting_level_name].Max)
        return enabled, scaled
    
    def setEnabledAndScaledImageSetting(self, onvif_setting_name, onvif_setting_enabled_name, onvif_setting_enabled_val, 
                                        onvif_setting_level_name, onvif_setting_level_scaled_val,
                                        readback_check=True, force_integer=True):
        # Bounds checking:
        if (onvif_setting_level_scaled_val < 0.0 and onvif_setting_level_scaled_val != -1.0) or (onvif_setting_level_scaled_val > 1.0):
            return False, "Invalid scaled value for " + onvif_setting_name + "." + onvif_setting_level_name
        
        # Make sure to query the system first
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)
        
        if not hasattr(imaging_settings, onvif_setting_name):
            return False, "Unavailable setting: " + onvif_setting_name
        
        unscaled = computeUnscaledFromScaled(onvif_setting_level_scaled_val, 
                                             self.imaging_options[onvif_setting_name][onvif_setting_level_name].Min, 
                                             self.imaging_options[onvif_setting_name][onvif_setting_level_name].Max, force_integer)
        new_imaging_settings = {"VideoSourceToken":self.video_source._token, "ImagingSettings":{onvif_setting_name: {onvif_setting_enabled_name: onvif_setting_enabled_val, onvif_setting_level_name: unscaled}}}
        try:
            self.imaging_service.SetImagingSettings(new_imaging_settings)
        except Exception as e:
            return False, "Onvif error: Failed to update " + onvif_setting_name + " - " + e.what()

        if readback_check is True:
           unscaled_readback = self.imaging_service.GetImagingSettings(video_src_token_param)[onvif_setting_name]
           if (unscaled_readback[onvif_setting_enabled_name] != onvif_setting_enabled_val) or (unscaled_readback[onvif_setting_level_name] != unscaled):
               print("Debug: Readback failed: " + str(unscaled_readback) + "vs. " + str(new_imaging_settings["ImagingSettings"][onvif_setting_name]))
               return False, onvif_setting_name + " not updated by camera"
           
        return True, "Success"

    def getBacklightCompensation(self):
        return self.getEnabledAndScaledImageSetting(self, "BacklightCompensation", "Mode", "ON", "Level")
    
    def setBacklightCompensation(self, enabled, scaled_val):
        enabled_val = "ON" if enabled is True else "OFF"
        return self.setEnabledAndScaledImageSetting("BacklightCompensation", "Mode", enabled_val, "Level", scaled_val)
    
    def getWideDynamicRange(self):
        return self.getEnabledAndScaledImageSetting(self, "WideDynamicRange", "Mode", "ON", "Level")
    
    def setWideDynamicRange(self, enabled, scaled_val):
        enabled_val = "ON" if enabled is True else "OFF"
        return self.setEnabledAndScaledImageSetting("WideDynamicRange", "Mode", enabled_val, "Level", scaled_val)
    
    def getScaledBrightness(self):
        return self.getScaledImageSetting("Brightness")
    
    def setScaledBrightness(self, scaled_val):
        return self.setScaledImageSetting("Brightness", scaled_val)
    
    def getScaledColorSaturation(self):
        return self.getScaledImageSetting("ColorSaturation")
    
    def setScaledColorSaturation(self, scaled_val):
        return self.setScaledImageSetting("ColorSaturation", scaled_val)
    
    def getScaledContrast(self):
        return self.getScaledImageSetting("Contrast")
    
    def setScaledContrast(self, scaled_val):
        return self.setScaledImageSetting("Contrast", scaled_val)
    
    def getScaledSharpness(self):
        return self.getScaledImageSetting("Sharpness")
    
    def setScaledSharpness(self, scaled_val):
        return self.setScaledImageSetting("Sharpness", scaled_val)
    
    def getExposureSettings(self):
        return self.getEnabledAndScaledImageSetting("Exposure", "Mode", "AUTO", "ExposureTime")
     
    def setExposureSettings(self, auto_exposure, manual_exposure_time_scaled, readback_check=True):
        mode_val = "AUTO" if auto_exposure is True else "MANUAL"
        return self.setEnabledAndScaledImageSetting("Exposure", "Mode", mode_val, 
                                                    "ExposureTime", manual_exposure_time_scaled)
    
    def getWhiteBalanceSettings(self):
        # Make sure to query the system first
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)
        #print("Debugging: Image Settings = " + str(imaging_settings))

        # Default return values
        auto_wb = True
        manual_cr_gain = None
        manual_cb_gain = None
                
        if hasattr(imaging_settings, "WhiteBalance"):
            auto_wb=(imaging_settings.WhiteBalance.Mode == "AUTO")
            manual_cr_gain = computeScaledFromUnscaled(imaging_settings.WhiteBalance.CrGain, self.imaging_options.WhiteBalance.YrGain.Min, self.imaging_options.WhiteBalance.YrGain.Max)
            manual_cb_gain = computeScaledFromUnscaled(imaging_settings.WhiteBalance.CbGain, self.imaging_options.WhiteBalance.YbGain.Min, self.imaging_options.WhiteBalance.YbGain.Max)

        return auto_wb, manual_cr_gain, manual_cb_gain
    
    def setWhiteBalanceSettings(self, auto_wb, manual_cr_gain_scaled=None, manual_cb_gain_scaled=None, readback_check=True):
        video_src_token_param = {"VideoSourceToken":self.video_source._token}
        imaging_settings = self.imaging_service.GetImagingSettings(video_src_token_param)
        
        if not hasattr(imaging_settings, "WhiteBalance"):
            return False, "Unavailable setting: WhiteBalance"
        
        updated_imaging_settings = self.imaging_service.create_type('SetImagingSettings')
        updated_imaging_settings.ImagingSettings = imaging_settings
        #print("Debugging: updated_imaging_settings before = " + str(updated_imaging_settings))
        updated_imaging_settings.VideoSourceToken = self.video_source._token
        updated_imaging_settings.ImagingSettings.WhiteBalance.Mode = "AUTO" if (auto_wb == True) else "MANUAL"
        if manual_cr_gain_scaled is not None:
            manual_cr_gain_unscaled = computeUnscaledFromScaled(manual_cr_gain_scaled, self.imaging_options.WhiteBalance.YrGain.Min, self.imaging_options.WhiteBalance.YrGain.Max)
            updated_imaging_settings.ImagingSettings.WhiteBalance.CrGain = manual_cr_gain_unscaled
        if manual_cb_gain_scaled is not None:
            manual_cb_gain_unscaled = computeUnscaledFromScaled(manual_cb_gain_scaled, self.imaging_options.WhiteBalance.YbGain.Min, self.imaging_options.WhiteBalance.YbGain.Max)
            updated_imaging_settings.ImagingSettings.WhiteBalance.CbGain = manual_cb_gain_unscaled

        try:
            self.imaging_service.SetImagingSettings(updated_imaging_settings)
        except Exception as e:
            return False, "Onvif error: Failed to update WhiteBalance - " + e.what()

        if (readback_check is True):
            auto_wb_rb, manual_cr_gain_rb, manual_cb_gain_rb = self.getWhiteBalanceSettings()
            if (auto_wb_rb != auto_exposure or manual_cr_gain_rb != manual_cr_gain_scaled or manual_cb_gain_rb != manual_cb_gain_scaled):
                print("Debugging: White balance = " + str(self.getWhiteBalanceSettings()))
                return False, "White balance not updated by camera"
        
        return True, "Success"

if __name__ == '__main__':
    # Unit test with Sidus camera for now
    driver = OnvifIFCamDriver(username="admin", password="SN:5364", ip_addr="192.168.0.150", port=80)
    
    # Unit test with ONWOTE ONVIF
    #driver = OnvifIFCamDriver(username="admin", password="123456", ip_addr="192.168.68.35", port=80)
    #driver = OnvifIFCamDriver(username="admin", password="123456", ip_addr="192.168.0.123", port=80)

    TEST_SET_RESOLUTION = False
    TEST_SET_FRAMERATE = False
    TEST_SET_BRIGHTNESS = False
    TEST_SET_SATURATION = False
    TEST_SET_CONTRAST = False
    TEST_SET_EXPOSURE = False
    TEST_SET_WHITEBALANCE = False
    TEST_RTSP_IMG_ACQUIRE = False

    # Comment these values in/out to determine which settings will get updated (and read-back-checked) by this script
    #TEST_SET_RESOLUTION = True
    TEST_SET_FRAMERATE = True
    #TEST_SET_BRIGHTNESS = True
    #TEST_SET_SATURATION = True
    #TEST_SET_CONTRAST = True
    #TEST_SET_EXPOSURE = True
    #TEST_SET_WHITEBALANCE = True
    TEST_RTSP_IMG_ACQUIRE = True

    # Query Available and Current Resolution
    print("Available Resolutions: ")
    available_resolutions, _ = driver.getAvailableResolutions()
    print(available_resolutions)
    print("Current Resolution:")
    current_resolution = driver.getResolution()
    print(current_resolution)
    if TEST_SET_RESOLUTION:
        current_resolution_index = available_resolutions.index([x for x in available_resolutions if x.Width == current_resolution.Width and x.Height == current_resolution.Height][0])
        new_resolution_index = current_resolution_index + 1 if current_resolution_index < len(available_resolutions) - 1 else 0
        print("Updating resolution to " + str(available_resolutions[new_resolution_index]) + ":")
        print(driver.setResolution(available_resolutions[new_resolution_index]))

    # Query, adjust, and check framerate limit
    print("Max Framerate:")
    framerate = driver.getFramerate()
    print(str(framerate))
    if TEST_SET_FRAMERATE:
        new_framerate = 15
        print("Updating framerate to " + str(new_framerate))
        print(driver.setFramerate(max_fps=new_framerate))

    # Query, adjust, and check brightness
    print("Current (Scaled) Brightness: ")
    brightness = driver.getScaledBrightness()
    print(brightness)
    if TEST_SET_BRIGHTNESS:
        brightness = (brightness+0.1) if (brightness < 0.9) else 0.1
        print("Updating brightness to " + str(brightness))
        print(driver.setScaledBrightness(brightness))

    # Query, adjust, and check saturation
    print("Current (Scaled) Saturation: ")
    saturation = driver.getScaledColorSaturation()
    print(saturation)
    if TEST_SET_SATURATION:
        saturation = (saturation-0.1) if (saturation > 0.1) else 0.9
        print("Updating saturation to " + str(saturation))
        print(driver.setScaledColorSaturation(saturation))
    
    # Query adjust and check contrast
    print("Current (Scaled) Contrast: ")
    contrast = driver.getScaledContrast()
    print(contrast)
    if TEST_SET_CONTRAST:
        contrast = (contrast+0.8) if (contrast <= 0.2) else 0.0
        #contrast = (contrast-0.1) if (contrast > 0.1) else 0.9
        print("Updating contrast to " + str(contrast))
        print(driver.setScaledContrast(contrast))

    # Query, adjust, and check exposure
    print("Current Exposure:")
    auto_exposure, manual_time = driver.getExposureSettings()
    print(str(auto_exposure), str(manual_time))
    if TEST_SET_EXPOSURE:
        auto_exposure = False
        #auto_exposure = True
        manual_time = manual_time + 0.5 if manual_time < 0.5 else 0.1
        print("Updating Exposure to " + str(auto_exposure) + "," + str(manual_time))
        print(driver.setExposureSettings(auto_exposure, manual_time))

    # Query, adjust, and check white balance
    print("Current WhiteBalance:")
    auto_wb, manual_cr_gain, manual_cb_gain = driver.getWhiteBalanceSettings()
    print(str(auto_wb), str(manual_cr_gain), str(manual_cb_gain))
    if TEST_SET_WHITEBALANCE:
        auto_wb = False
        manual_cr_gain = manual_cr_gain + 0.8 if manual_cr_gain < 0.2 else 0.1
        manual_cb_gain = manual_cb_gain + 0.8 if manual_cb_gain < 0.2 else 0.1
        print("Updating White Balance to " + str(auto_wb) + "," + str(manual_cr_gain) + "," + str(manual_cb_gain))
        print(driver.setWhiteBalanceSettings(auto_wb, manual_cr_gain, manual_cb_gain))

    # Now acquire and display some images
    if TEST_RTSP_IMG_ACQUIRE:
        uri_index = 0
        print("Starting image acquisition for " + driver.rtsp_uris[uri_index] + "... press 'q' to stop")
        print(driver.startImageAcquisition(uri_index))
        img_count = 0
        start = time.time()
        while True:
            frame, status, msg = driver.getImage(uri_index)
            if status is False:
                print("Failed to acquire image: " + msg)
                break
            else:
                img_count += 1
                cv2.imshow('frame', frame)
                if cv2.waitKey(20) & 0xFF == ord('q'):
                    print("Stopping image acquisition by user reqest:")
                    break
        stop = time.time()
        print(driver.stopImageAcquisition(uri_index))
        cv2.destroyAllWindows()
        duration_s = stop - start
        fps = img_count / duration_s
        print("Captured " + str(img_count) + " images in " + str(duration_s) + "s (" + str(fps) + " FPS)")
