#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

import rospy

from nepi_edge_sdk_ptx.ptx_if import ROSPTXActuatorIF
from nepi_edge_sdk_onvif.onvif_pan_tilt_driver import ONVIF_GENERIC_PTZ_DRIVER_ID, OnvifIFPanTiltDriver
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

class OnvifPanTiltNode:
    DEFAULT_NODE_NAME = "onvif_pan_tilt"
    
    DRIVER_SPECIALIZATION_CONSTRUCTORS = {ONVIF_GENERIC_PTZ_DRIVER_ID: OnvifIFPanTiltDriver} # Extend as necessary

    DEFAULT_STATUS_UPDATE_RATE_HZ = 10.0 # Hz

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
                
        username = str(rospy.get_param('~credentials/username'))
        password = str(rospy.get_param('~credentials/password'))
        host = str(rospy.get_param('~network/host'))

        # Allow a default for the pan_tilt_port, since it is part of onvif spec.
        onvif_port = int(rospy.get_param('~network/port', 80))
        rospy.set_param('~network/port', onvif_port)

        # Set up for specialized drivers here
        self.driver_id = rospy.get_param('~driver_id', ONVIF_GENERIC_PTZ_DRIVER_ID)
        rospy.set_param('~driver_id', self.driver_id)
        if self.driver_id not in self.DRIVER_SPECIALIZATION_CONSTRUCTORS:
            rospy.logerr(self.node_name + ": unknown driver_id " + self.driver_id)
            return
        DriverConstructor = self.DRIVER_SPECIALIZATION_CONSTRUCTORS[self.driver_id]

        # Start the driver to connect to the camera
        rospy.loginfo(self.node_name + ": Launching " + self.driver_id + " driver")
        while not rospy.is_shutdown():
            try:
                #self.driver = DriverConstructor(username, password, host, onvif_port)
                #rospy.logerr("DEBUG: Launching with hard-coded credentials")
                self.driver = DriverConstructor(username, password, host, onvif_port)
                break
            except Exception as e:
                # Only log the error every 30 seconds -- don't want to fill up log in the case that the camera simply isn't attached.
                rospy.logerr_throttle(30, self.node_name + ": Failed to instantiate OnvifIFPanTiltDriver... device not online? bad credentials?: " + str(e))
                rospy.sleep(1)

        rospy.loginfo(self.node_name + ": ... Connected!")
        self.dev_info = self.driver.getDeviceInfo()
        self.logDeviceInfo()

        # Now initialize the default settings, some of which require driver support -- can be overridden by config file
        default_settings = {
            'frame_id' : self.node_name + '_frame',
            'yaw_joint_name' : self.node_name + '_yaw_joint',
            'pitch_joint_name' : self.node_name + '_pitch_joint',
            'reverse_yaw_control' : False,
            'reverse_pitch_control' : False,
            'speed_ratio' : 1.0,
            'status_update_rate_hz' : self.DEFAULT_STATUS_UPDATE_RATE_HZ
        }

        # TODO: Absolute position limits (hard and soft) in default_settings (see ptx_if.py). 
        # ONVIF spec. reports absolute limits in terms of [-1.0,1.0] ratio so seems we'll need 
        # some additional config. params to map these to physical units (degrees, etc.)

        ptx_callback_names = {
            # PTX Standard
            "StopMoving": self.stopMoving,
            "MoveYaw": self.moveYaw,
            "MovePitch": self.movePitch,
            "SetSpeed": self.setSpeed,
            "GetSpeed": self.getSpeed,
            "GetCurrentPosition": self.getCurrentPosition,
            "GotoPosition": self.gotoPosition,
            "GoHome": self.goHome,
            "SetHomePosition": self.setHomePosition,
            "SetHomePositionHere": self.setHomePositionHere,
            "GotoWaypoint": self.gotoWaypoint,
            "SetWaypoint": self.setWaypoint,
            "SetWaypointHere": self.setWaypointHere
        }

        # Must pass a capabilities structure to ptx_interface constructor
        ptx_default_capabilities = {}

        # Now check with the driver if any of these PTX capabilities are explicitly not present
        if not self.driver.hasAdjustableSpeed():
            ptx_callback_names["GetSpeed"] = None # Clear the method
            ptx_callback_names["SetSpeed"] = None # Clear the method
            ptx_default_capabilities['has_speed_control'] = False
        else:
            ptx_default_capabilities['has_speed_control'] = True
            
        if not self.driver.reportsPosition():
            ptx_callback_names["GetCurrentPosition"] = None # Clear the method
            
        if not self.driver.hasAbsolutePositioning():
            rospy.logerr("Debug: Clearing GotoPosition")
            ptx_callback_names["GotoPosition"] = None # Clear the method
        
        self.has_absolute_positioning_and_feedback = self.driver.hasAbsolutePositioning() and self.driver.reportsPosition()
        ptx_default_capabilities['has_absolute_positioning'] = self.has_absolute_positioning_and_feedback
                
        if not self.driver.canHome() and not self.has_absolute_positioning_and_feedback:
            ptx_callback_names["GoHome"] = None
            ptx_default_capabilities['has_homing'] = False
        else:
            ptx_default_capabilities['has_homing'] = True
        
            
        if not self.driver.homePositionAdjustable() and not self.has_absolute_positioning_and_feedback:
            ptx_callback_names["SetHomePositionHere"] = None
        
        if not self.driver.hasWaypoints() and not self.has_absolute_positioning_and_feedback:
            ptx_callback_names["GotoWaypoint"] = None
            ptx_callback_names["SetWaypointHere"] = None
            ptx_default_capabilities['has_waypoints'] = False
        else:
            ptx_default_capabilities['has_waypoints'] = True
        
        # Following are implemented entirely in software because ONVIF has no support for setting HOME or PRESET by position
        if not self.has_absolute_positioning_and_feedback:
            ptx_callback_names["SetHomePosition"] = None    
            ptx_callback_names["SetWaypoint"] = None

        # Now that we've updated the callbacks table, can apply the remappings... this is particularly useful since
        # many ONVIF devices report capabilities that they don't actually have, so need a user-override mechanism. In
        # that case, assign these to null in the config file
        # TODO: Not sure we actually need remappings for PTX: Makes sense for IDX because there are lots of controllable params.
        ptx_remappings = rospy.get_param('~ptx_remappings', {})
        rospy.loginfo(self.node_name + ': Establishing PTX remappings')
        for from_name in ptx_remappings:
            to_name = ptx_remappings[from_name]
            if from_name not in ptx_callback_names or (to_name not in ptx_callback_names and to_name != None and to_name != 'None'):
                rospy.logwarn('\tInvalid PTX remapping: ' + from_name + '-->' + to_name)
            elif to_name is None or to_name == 'None':
                ptx_callback_names[from_name] = None
                rospy.loginfo('\Remapping %s to non-existence to remove capability', from_name)
            elif ptx_callback_names[to_name] is None:
                rospy.logwarn('\tRemapping ' + from_name + ' to an unavailable adjustment (' + to_name + ')')
            else:
                ptx_callback_names[from_name] = ptx_callback_names[to_name]
                rospy.loginfo('\t' + from_name + '-->' + to_name)

        # Launch the PTX interface --  this takes care of initializing all the ptx settings from config. file, subscribing and advertising topics and services, etc.
        rospy.loginfo(self.node_name + ": Launching NEPI PTX (ROS) interface...")
        self.ptx_if = ROSPTXActuatorIF(ptx_device_name=self.node_name,
                                       serial_num = self.driver.getDeviceSerialNumber(),
                                       hw_version = self.driver.getDeviceHardwareId(),
                                       sw_version = self.driver.getDeviceFirmwareVersion(),
                                       default_settings = default_settings,
                                       default_capabilities = ptx_default_capabilities,
                                       stopMovingCb = ptx_callback_names["StopMoving"],
                                       moveYawCb = ptx_callback_names["MoveYaw"],
                                       movePitchCb = ptx_callback_names["MovePitch"],
                                       setSpeedCb = ptx_callback_names["SetSpeed"],
                                       getSpeedCb = ptx_callback_names["GetSpeed"],
                                       getCurrentPositionCb = ptx_callback_names["GetCurrentPosition"],
                                       gotoPositionCb = ptx_callback_names["GotoPosition"],
                                       goHomeCb = ptx_callback_names["GoHome"],
                                       setHomePositionCb = ptx_callback_names["SetHomePosition"],
                                       setHomePositionHereCb = ptx_callback_names["SetHomePositionHere"],
                                       gotoWaypointCb = ptx_callback_names["GotoWaypoint"],
                                       setWaypointCb = ptx_callback_names["SetWaypoint"],
                                       setWaypointHereCb = ptx_callback_names["SetWaypointHere"])
        rospy.loginfo(self.node_name + ": ... PTX interface running")

        self.speed_ratio = 1.0
        self.home_yaw_deg = 0.0
        self.home_pitch_deg = 0.0
        self.waypoints = [] # List of dictionaries with waypoint_pitch, waypoint_yaw

        # Set up save_cfg interface
        self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.setCurrentSettingsAsDefault, paramsModifiedCallback=self.updateFromParamServer)

        rospy.spin()
    
    def logDeviceInfo(self):
        dev_info_string = self.node_name + " Device Info:\n"
        dev_info_string += "Manufacturer: " + self.dev_info["Manufacturer"] + "\n"
        dev_info_string += "Model: " + self.dev_info["Model"] + "\n"
        dev_info_string += "Firmware Version: " + self.dev_info["FirmwareVersion"] + "\n"
        dev_info_string += "Serial Number: " + self.dev_info["HardwareId"] + "\n"
        rospy.loginfo(dev_info_string)

    def stopMoving(self):
        self.driver.stopMotion()

    def moveYaw(self, direction, duration):
        driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.driver.PT_DIRECTION_NEGATIVE
        self.driver.jog(pan_direction = driver_direction, tilt_direction = self.driver.PT_DIRECTION_NONE, speed_ratio = self.speed_ratio, time_s = duration)

    def movePitch(self, direction, duration):
        driver_direction = self.driver.PT_DIRECTION_POSITIVE if direction == self.ptx_if.PTX_DIRECTION_POSITIVE else self.driver.PT_DIRECTION_NEGATIVE
        self.driver.jog(pan_direction = self.driver.PT_DIRECTION_NONE, tilt_direction = driver_direction, speed_ratio = self.speed_ratio, time_s = duration)

    def setSpeed(self, speed_ratio):
        # TODO: Limits checking and driver unit conversion?
        self.speed_ratio = speed_ratio

    def getSpeed(self):
        # TODO: Driver unit conversion?
        return self.speed_ratio
            
    def getCurrentPosition(self):
        pan_ratio, tilt_ratio = self.driver.getCurrentPosition()
        pan_deg = self.ptx_if.yawRatioToDeg(pan_ratio)
        tilt_deg = self.ptx_if.pitchRatioToDeg(tilt_ratio)
        return pan_deg, tilt_deg

    def gotoPosition(self, yaw_deg, pitch_deg):
        yaw_ratio = self.ptx_if.yawDegToRatio(yaw_deg)
        pitch_ratio = self.ptx_if.pitchRatioToDeg(pitch_deg)
        self.driver.moveToPosition(yaw_ratio, pitch_ratio, self.speed_ratio)
        
    def goHome(self):
        if self.driver.canHome() is True:
            self.driver.goHome(self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True:
            home_yaw_ratio = self.ptx_if.yawDegToRatio(self.home_yaw_deg)
            home_pitch_ratio = self.ptx_if.pitchDegToRatio(self.home_pitch_deg)
            self.driver.moveToPosition(home_yaw_ratio, home_pitch_ratio, self.speed_ratio)
        else:
            rospy.logwarn("Homing not supported by this PTX device... ignoring")

    def setHomePosition(self, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.home_yaw_deg = yaw_deg
        self.home_pitch_deg = pitch_deg

    def setHomePositionHere(self):
        if self.driver.reportsPosition() is True:
            curr_yaw_ratio, curr_pitch_ratio = self.driver.getCurrentPosition()
            self.home_yaw_deg = self.ptx_if.yawRatioToDeg(curr_yaw_ratio)
            self.home_pitch_deg = self.ptx_if.pitchRatioToDeg(curr_pitch_ratio) 

        if self.driver.homePositionAdjustable is True:
            self.driver.setHomeHere()

    def gotoWaypoint(self, waypoint_index):
        if self.driver.hasWaypoints() is True:
           self.driver.gotoPreset(waypoint_index, self.speed_ratio)
        elif self.driver.hasAbsolutePositioning() is True:
            if waypoint_index not in self.waypoints:
                rospy.logwarn("Requested waypoint index %u is invalid/unset... ignoring")
                return
            
            waypoint_yaw_deg = self.waypoints[waypoint_index]['yaw_deg']
            waypoint_pitch_deg = self.waypoints[waypoint_index]['pitch_deg']
            self.driver.moveToPosition(waypoint_yaw_deg, waypoint_pitch_deg, self.speed_ratio)
    
    def setWaypoint(self, waypoint_index, yaw_deg, pitch_deg):
        # Have to implement these fully in s/w since ONVIF doesn't support absolute home position setting
        self.waypoints[waypoint_index] = {'yaw_deg': yaw_deg, 'pitch_deg': pitch_deg}
        
    def setWaypointHere(self, waypoint_index):
        if self.driver.reportsPosition():
            yaw_ratio, pitch_ratio = self.driver.getCurrentPosition()
            current_yaw_deg = self.ptx_if.yawRatioToDeg(yaw_ratio)
            current_pitch_deg = self.ptx_if.pitchRatioToDeg(pitch_ratio)
            self.waypoints[waypoint_index] = {'yaw_deg': current_yaw_deg, 'pitch_deg': current_pitch_deg}

        if self.driver.hasWaypoints():
            self.driver.setPresetHere(waypoint_index)

        rospy.loginfo("Waypoint set to current position")

    def setCurrentSettingsAsDefault(self):
        # Don't need to worry about any of our params in this class, just child interfaces' params
        self.ptx_if.setCurrentSettingsToParamServer()

    def updateFromParamServer(self):
        self.ptx_if.updateFromParamServer()

if __name__ == '__main__':
	node = OnvifPanTiltNode()