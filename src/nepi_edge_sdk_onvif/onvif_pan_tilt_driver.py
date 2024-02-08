#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_edge_sdk_onvif
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_onvif
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#

import time
import datetime

from onvif import ONVIFCamera # python-onvif

ONVIF_GENERIC_PTZ_DRIVER_ID = 'GenericONVIFPTZ'

class OnvifIFPanTiltDriver(object):
    WSDL_FOLDER = "/opt/nepi/ros/etc/onvif/wsdl/"
    PT_DIRECTION_POSITIVE = 1
    PT_DIRECTION_NEGATIVE = -1
    PT_DIRECTION_NONE = 0
    
    ONVIF_GENERIC_DRIVER_ID = 'GenericONVIF'

    def __init__(self, username, password, ip_addr, port=80):
        #print('Debug: Starting GenericONVIF driver')
        self.username = username
        self.password = password

        # Connect to the "camera" -- should work even if no actual camera, just the way python-onvif API is set up
        self.cam = ONVIFCamera(ip_addr, port, username, password, self.WSDL_FOLDER)
        #print('Debug: Created the camera')

        # The devicemgt service gets set up automatically
        self.device_info = self.cam.devicemgmt.GetDeviceInformation()
        #print('Debug: Device Info = ' + str(self.device_info))
        
        # Create required services (media, ptz)
        self.media_service = self.cam.create_media_service()
        self.ptz_service = self.cam.create_ptz_service()
        
        node = self.ptz_service.GetNodes()[0]
        #print('Debug: Node = ' + str(node))
        
        # Get the available media profiles
        self.media_profile = self.media_service.GetProfiles()[0]
        
        self.profile_token = self.media_profile.token
        self.ptz_configuration_token = self.media_profile.PTZConfiguration.token

        # Get PTZ configuration options for getting continuous move range
        request = self.ptz_service.create_type('GetConfigurationOptions')
        request.ConfigurationToken = self.ptz_configuration_token
        ptz_config_options = self.ptz_service.GetConfigurationOptions(request)
        #print('Debug: PTZ Config Options = ' + str(ptz_config_options))
        
        # And get the actual configuration
        ptz_config = self.ptz_service.GetConfiguration({'PTZConfigurationToken': self.ptz_configuration_token})
        #print('Debug: PTZ Config = ' + str(ptz_config))
        
        srv_caps_request = self.ptz_service.create_type('GetServiceCapabilities')
        # Seen GetServiceCapabilities throw exception (unsupported request?) on at least one device, Jennov
        try:
            ptz_caps = self.ptz_service.GetServiceCapabilities(srv_caps_request)
        except:
            ptz_caps = None
        #print('Debug: Service capabilities = ' + str(ptz_caps))
            
        # Extract useful parameters from config options, actual config, and capabilities
        speed_ratio_limits = ptz_config_options.Spaces.PanTiltSpeedSpace[0].XRange
        self.max_speed_ratio = speed_ratio_limits.Max
        self.min_speed_ratio = speed_ratio_limits.Min

        if "PanTiltSpeedSpace" in ptz_config_options.Spaces:
            self.supports_adjustable_speed = True # Per ONVIF-PTZ-Service-Spec, existence of this Space element indicates speed adjustment is possible
        else:
            self.supports_adjustable_speed = False
        
        
        if "AbsolutePanTiltPositionSpace" in ptz_config_options.Spaces:
            abs_position_ratio_limits = ptz_config_options.Spaces.AbsolutePanTiltPositionSpace[0]
            self.max_pan_position_ratio = abs_position_ratio_limits.XRange.Max
            self.min_pan_position_ratio = abs_position_ratio_limits.XRange.Min
            self.max_tilt_position_ratio = abs_position_ratio_limits.YRange.Max
            self.min_tilt_position_ratio = abs_position_ratio_limits.YRange.Min
            self.supports_absolute_position = True # Per ONVIF-PTZ-Service-Spec, existence of this Space element indicates absolute positioning is possible.
        else:
            self.supports_absolute_position = False

        timeout_limits = ptz_config_options.PTZTimeout
        self.max_timeout = timeout_limits.Max
        self.min_timeout = timeout_limits.Min

        self.reports_position = True if (ptz_caps != None) and (ptz_caps.StatusPosition is True) else False

        self.can_home = node.HomeSupported and (node.FixedHomePosition != None)
        self.home_position_adjustable = self.can_home and (not node.FixedHomePosition)
        self.max_preset_count = node.MaximumNumberOfPresets
        
        # Set up a reusable continuous move request
        self.continuous_move_request = self.ptz_service.create_type('ContinuousMove')
        self.continuous_move_request.ProfileToken = self.profile_token
        if self.continuous_move_request.Velocity is None:
            self.continuous_move_request.Velocity = self.ptz_service.GetStatus({'ProfileToken': self.profile_token}).Position
        if self.continuous_move_request.Timeout is None:
            self.continuous_move_request.Timeout = ptz_config.DefaultPTZTimeout
        #print('Debug: ' + str(self.continuous_move_request))

        # Set up a reusable absolute move request
        self.abs_move_request = self.ptz_service.create_type('AbsoluteMove')
        self.abs_move_request.ProfileToken = self.profile_token
        if self.abs_move_request.Position is None:
            self.abs_move_request.Position = self.ptz_service.GetStatus({'ProfileToken': self.profile_token}).Position
        if self.abs_move_request.Speed is None:
            self.abs_move_request.Speed = ptz_config.DefaultPTZSpeed

        # Set up a resuable goto home request
        self.goto_home_position_request = self.ptz_service.create_type('GotoHomePosition')
        self.goto_home_position_request.ProfileToken = self.profile_token
        if self.goto_home_position_request.Speed is None:
            self.goto_home_position_request.Speed = self.ptz_service.GetStatus({'ProfileToken': self.profile_token}).Position

        # Set up a reusable preset request
        self.set_preset_request = self.ptz_service.create_type('SetPreset')
        self.set_preset_request.ProfileToken = self.profile_token
        
        # Set up a reusable goto preset request
        self.goto_preset_request = self.ptz_service.create_type('GotoPreset')
        self.goto_preset_request.ProfileToken = self.profile_token
        if self.goto_preset_request.Speed is None:
            self.goto_preset_request.Speed = self.abs_move_request.Position = self.ptz_service.GetStatus({'ProfileToken': self.profile_token}).Position
        
        # Query existing presets
        unindexed_presets = self.ptz_service.GetPresets({'ProfileToken': self.profile_token})
        self.presets = {}
        next_index = 0
        for preset in unindexed_presets:
            self.presets[next_index] = preset
            next_index += 1
                
        #print('Debug: PTZStatus = ')
        #print(self.ptz_service.GetStatus({'ProfileToken': self.profile_token}))

    def stopMotion(self):
        request = self.ptz_service.create_type('Stop')
        request.ProfileToken = self.profile_token
        self.ptz_service.Stop(request)

    def jog(self, pan_direction, tilt_direction, speed_ratio, time_s = 1):
        self.continuous_move_request.Velocity.PanTilt.x = speed_ratio * pan_direction
        self.continuous_move_request.Velocity.PanTilt.y = speed_ratio * tilt_direction
        self.continuous_move_request.Timeout = datetime.timedelta(seconds = time_s)
        self.ptz_service.ContinuousMove(self.continuous_move_request)

    def moveToPosition(self, pan_position_ratio, tilt_position_ratio, speed_ratio):
        self.abs_move_request.Position.PanTilt.x = self.min_pan_position_ratio + (pan_position_ratio * (self.max_pan_position_ratio - self.min_pan_position_ratio))
        self.abs_move_request.Position.PanTilt.y = self.min_tilt_position_ratio + (tilt_position_ratio * (self.max_tilt_position_ratio - self.min_tilt_position_ratio))

        self.abs_move_request.Speed.PanTilt.x = speed_ratio
        self.abs_move_request.Speed.PanTilt.y = speed_ratio
        self.ptz_service.AbsoluteMove(self.abs_move_request)

    def getCurrentPosition(self):
        current_status = self.ptz_service.GetStatus({'ProfileToken': self.profile_token})
        current_pan_ratio = current_status.Position.PanTilt.x
        current_tilt_ratio = current_status.Position.PanTilt.y
        return current_pan_ratio, current_tilt_ratio
    
    def setHomeHere(self):
        self.ptz_service.SetHomePosition({'ProfileToken': self.profile_token})

    def goHome(self, speed_ratio):
        self.goto_home_position_request.Speed.PanTilt.x = speed_ratio
        self.goto_home_position_request.Speed.PanTilt.y = speed_ratio
        self.ptz_service.GotoHomePosition(self.goto_home_position_request)
    
    def setPresetHere(self, preset_index):
        if preset_index in self.presets:
            self.set_preset_request.PresetToken = self.presets[preset_index].token
            self.presets[preset_index].token = self.ptz_service.SetPreset(self.set_preset_request)
        else:
            self.set_preset_request.PresetToken = None
            self.presets[preset_index].token = self.ptz_service.SetPreset(self.set_preset_request)

    def gotoPreset(self, preset_index, speed_ratio):
        if preset_index not in self.presets:
            return False, "Invalid preset index"
        
        self.goto_preset_request.PresetToken = self.presets[preset_index].token
        self.goto_preset_request.Speed.PanTilt.x = speed_ratio
        self.goto_preset_request.Speed.PanTilt.y = speed_ratio
        self.ptz_service.GotoPreset(self.goto_preset_request)

    def printStatus(self):
        print('PTZStatus:')
        print(str(self.ptz_service.GetStatus({'ProfileToken': self.profile_token})))

    def getDeviceInfo(self):
        return self.device_info
    
    def getDeviceSerialNumber(self):
        return self.device_info.SerialNumber
    
    def getDeviceFirmwareVersion(self):
        return self.device_info.FirmwareVersion
    
    def getDeviceHardwareId(self):
        return self.device_info.HardwareId
    
    def hasAdjustableSpeed(self):
        return self.supports_adjustable_speed
    
    def reportsPosition(self):
        return self.reports_position
    
    def hasAbsolutePositioning(self):
        return self.supports_absolute_position
    
    def canHome(self):
        return self.can_home
    
    def homePositionAdjustable(self):
        return self.home_position_adjustable
    
    def hasWaypoints(self):
        return (self.max_preset_count > 0)
    
if __name__ == '__main__':
    # JENNOV
    USER = 'admin'
    PASSWORD = '123456'
    IP_ADDR = '192.168.179.111'
    PORT = 80 
    
    # ONWOTE
    #USER = 'admin'
    #PASSWORD = 'admin'
    #IP_ADDR = '192.168.0.33'
    #PORT = 80

    # ECON ROUTECAM
    #USER = 'x' # Doesn't matter
    #PASSWORD = 'x' # Doesn't matter
    #IP_ADDR = "192.168.0.41"
    #PORT = 8000

    driver = OnvifIFPanTiltDriver(USER, PASSWORD, IP_ADDR, PORT)

    print(driver.getDeviceInfo())
    driver.printStatus()

    driver.stopMotion()

    print("Jogging (+,+) for 2 seconds")
    driver.jog(pan_direction = driver.PT_DIRECTION_POSITIVE, tilt_direction = driver.PT_DIRECTION_POSITIVE, speed_ratio=1)
    time.sleep(1)
    print("Position:" + str(driver.getCurrentPosition()))
    time.sleep(1)
    driver.stopMotion()
    time.sleep(1)
    driver.setPresetHere(0)
    print("Jogging (-,-) for 2 seconds")
    driver.jog(pan_direction = driver.PT_DIRECTION_NEGATIVE, tilt_direction = driver.PT_DIRECTION_NEGATIVE, speed_ratio=1)
    time.sleep(1)
    print("Position:" + str(driver.getCurrentPosition()))
    time.sleep(1)
    driver.stopMotion()
    time.sleep(1)
    driver.setPresetHere(1)

    print("Returning to position 0")
    driver.gotoPreset(preset_index = 0, speed_ratio = 1)
    time.sleep(2)

    print("Returning to position 1")
    driver.gotoPreset(preset_index = 1, speed_ratio = 1)
    time.sleep(2)

    #print("Returning home")
    #driver.goHome(speed_ratio = 1)
    #time.sleep(2)
    #driver.jog(-0.5, -0.5)

    #driver.moveToPosition(-0.5,-0.5,0.5)

