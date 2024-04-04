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
import datetime

import requests
from xml.etree import ElementTree as ET
from requests.auth import HTTPDigestAuth

SIDUS_SS109HT_PTZ_DRIVER_ID = 'SidusSS109HT_PTZ'

#### Generic ONVIF - Adapted from Vendor-provided code ####
DEVICE_SERVICE_PATH = "/onvif/device_service"
SERVICES_PATH = "/onvif/services"
MEDIA_SERVICE_PATH = "/onvif/Media"

def send_soap_request(url, soap_env, headers, auth):
    #print('Debug: send_soap_request')
    # Prepare the request (without sending it)
    # Removing carriage return and newline characters
    sanitized_soap_env = soap_env.replace("\r", "").replace("\n", "").replace("\t", "")
    sanitized_soap_env = sanitized_soap_env.strip()
    req = requests.Request('POST', url, data=sanitized_soap_env, headers=headers, auth=auth)
    prepared = req.prepare()

    # Now send the prepared request
    with requests.Session() as session:
        response = session.send(prepared, timeout=30)
        #print('Debug: send_soap_request response received')
        return response
    
def soapGetCapabilities(globalip, globalport, username, password):
    soap_env = f"""<?xml version="1.0" encoding="UTF-8"?>
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
                    xmlns:t="http://www.onvif.org/ver10/device/wsdl">
            <s:Header>
                <Security s:mustUnderstand="true" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                <UsernameToken>
                    <Username>{username}</Username>
                    <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
                <t:GetCapabilities>
                <t:Category>All</t:Category>
                </t:GetCapabilities>
            </s:Body>
        </s:Envelope>
        """

    headers = {
            "Content-Type": "application/soap+xml",
            "charset": "utf-8",
            "Content-Length": str(len(soap_env)),
        }

    url = f"http://{globalip}:{globalport}{DEVICE_SERVICE_PATH}"
    return requests.post(url, data=soap_env, headers=headers, timeout=30)

def soapGetDeviceInformation(globalip, globalport, username, password):
    soap_env = f"""
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
                    xmlns:t="http://www.onvif.org/ver10/device/wsdl">
            <s:Header>
                <Security s:mustUnderstand="true" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{username}</Username>
                        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                    </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
                <t:GetDeviceInformation/>
            </s:Body>
        </s:Envelope>
        """
    headers = {
            "Content-Type": "application/soap+xml",
            "charset": "utf-8",
            "Content-Length": str(len(soap_env)),
        }

    # Send HTTP request
    url = f"http://{globalip}:{globalport}{DEVICE_SERVICE_PATH}"
    return requests.post(url, data=soap_env, headers=headers, timeout=30)

def soapGetProfileToken(globalip, globalport, username, password):
    # SOAP request body for retrieving profiles
    soap_body = f"""
        <s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">
            <s:Header>
                <Security s:mustUnderstand="true" xmlns="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd">
                    <UsernameToken>
                        <Username>{username}</Username>
                        <Password Type="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-username-token-profile-1.0#PasswordText">{password}</Password>
                    </UsernameToken>
                </Security>
            </s:Header>
            <s:Body>
                <GetProfiles xmlns="http://www.onvif.org/ver10/media/wsdl"/>
            </s:Body>
        </s:Envelope>
        """
    # Add WS-Security header for authentication (replace with your actual credentials)
    headers = {
        "Content-Type": "application/soap+xml; charset=utf-8",
        "Content-Length": str(len(soap_body)),
    }

    url = f"http://{globalip}:{globalport}{MEDIA_SERVICE_PATH}"
    response = requests.post(url, data=soap_body, headers=headers)
    #print("Got Response soapGetProfileToken")

    print("HTTP Status Code:", response.status_code)
    #print("Response Text:", response.text)

    # Check if request was successful
    if response.status_code == 200:
        root = ET.fromstring(response.content)

        # Namespace for parsing the response
        namespaces = {
            's': 'http://www.w3.org/2003/05/soap-envelope',
            'trt': 'http://www.onvif.org/ver10/media/wsdl',
            'tt': 'http://www.onvif.org/ver10/schema'
        }

        # Get all profiles
        profiles = root.findall('.//trt:Profiles', namespaces=namespaces)

        # Loop through each profile and check for PTZConfiguration
        for profile in profiles:
            ptz_config = profile.find('tt:PTZConfiguration', namespaces=namespaces)
            if ptz_config is not None:
                # This profile has PTZ capabilities
                profile_token = profile.get('token')
                #print(f"Profile {profile_token} supports PTZ.")
                return profile_token, response.text
    else:
        print("RESPONSE WAS NOT 200")

def soapPTZ(globalip, globalport, username, password, profile_token, speed, x_value, y_value):
    soap_env = f"""<?xml version="1.0" encoding="UTF-8"?>
    <s:Envelope
        xmlns:s="http://www.w3.org/2003/05/soap-envelope"
        xmlns:enc="http://www.w3.org/2003/05/soap-encoding"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:wsa="http://www.w3.org/2005/08/addressing"
        xmlns:ds="http://www.w3.org/2000/09/xmldsig#"
        xmlns:wsse="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd"
        xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd"
        xmlns:wsnt="http://docs.oasis-open.org/wsn/b-2"
        xmlns:tt="http://www.onvif.org/ver10/schema"
        xmlns:tds="http://www.onvif.org/ver10/device/wsdl"
        xmlns:trt="http://www.onvif.org/ver10/media/wsdl"
        xmlns:tr2="http://www.onvif.org/ver20/media/wsdl"
        xmlns:tev="http://www.onvif.org/ver10/events/wsdl"
        xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"
        xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl"
        xmlns:tan="http://www.onvif.org/ver20/analytics/wsdl"
        xmlns:pt="http://www.onvif.org/ver10/pacs"
        xmlns:tmd="http://www.onvif.org/ver10/deviceIO/wsdl"
        xmlns:trp="http://www.onvif.org/ver10/replay/wsdl"
        xmlns:tse="http://www.onvif.org/ver10/search/wsdl"
        xmlns:trc="http://www.onvif.org/ver10/recording/wsdl"
        xmlns:tac="http://www.onvif.org/ver10/accesscontrol/wsdl"
        xmlns:tdc="http://www.onvif.org/ver10/doorcontrol/wsdl"
        xmlns:tth="http://www.onvif.org/ver10/thermal/wsdl"
        xmlns:tcr="http://www.onvif.org/ver10/credential/wsdl"
        xmlns:tar="http://www.onvif.org/ver10/accessrules/wsdl"
        xmlns:tsc="http://www.onvif.org/ver10/schedule/wsdl"
        xmlns:trv="http://www.onvif.org/ver10/receiver/wsdl"
        xmlns:tpv="http://www.onvif.org/ver10/provisioning/wsdl"
        xmlns:ter="http://www.onvif.org/ver10/error">
        <s:Body>
        <tptz:AbsoluteMove xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl">
            <tptz:ProfileToken>{profile_token}</tptz:ProfileToken>
            <tptz:Position>
                <tt:PanTilt x="{x_value}" y="{y_value}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace" xmlns:tt="http://www.onvif.org/ver10/schema"/>
            </tptz:Position>
            <tptz:Speed>
                <tt:PanTilt x="{speed}" y="{speed}" xmlns:tt="http://www.onvif.org/ver10/schema"/>
            </tptz:Speed>
        </tptz:AbsoluteMove>
    </s:Body>
        </s:Envelope>
    """
    
    #print(soap_env)

    # Define the URL and payload
    url = "http://"+globalip+":"+globalport+"/onvif/services"
    #data = '''<YOUR_SOAP_PAYLOAD_HERE>'''  # Replace this with your SOAP XML payload

    headers = {
        "Host": ""+globalip+":"+globalport+"",
        "Content-Type": 'application/soap+xml; charset=utf-8; action="http://www.onvif.org/ver20/ptz/wsdl/ContinuousMove"',
        "Connection": "close"
    }

    # Use Digest Authentication
    auth = HTTPDigestAuth(username, password)  # Replace with your password

    # Send the POST request
    return send_soap_request(url, soap_env, headers, auth)

def soapPTZContinuous(globalip, globalport, username, password, profile_token, xspeed, yspeed):
    soap_env = f"""<?xml version="1.0" encoding="UTF-8"?>
    <s:Envelope
        xmlns:s="http://www.w3.org/2003/05/soap-envelope"
        xmlns:enc="http://www.w3.org/2003/05/soap-encoding"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:wsa="http://www.w3.org/2005/08/addressing"
        xmlns:ds="http://www.w3.org/2000/09/xmldsig#"
        xmlns:wsse="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd"
        xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd"
        xmlns:wsnt="http://docs.oasis-open.org/wsn/b-2"
        xmlns:tt="http://www.onvif.org/ver10/schema"
        xmlns:tds="http://www.onvif.org/ver10/device/wsdl"
        xmlns:trt="http://www.onvif.org/ver10/media/wsdl"
        xmlns:tr2="http://www.onvif.org/ver20/media/wsdl"
        xmlns:tev="http://www.onvif.org/ver10/events/wsdl"
        xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"
        xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl"
        xmlns:tan="http://www.onvif.org/ver20/analytics/wsdl"
        xmlns:pt="http://www.onvif.org/ver10/pacs"
        xmlns:tmd="http://www.onvif.org/ver10/deviceIO/wsdl"
        xmlns:trp="http://www.onvif.org/ver10/replay/wsdl"
        xmlns:tse="http://www.onvif.org/ver10/search/wsdl"
        xmlns:trc="http://www.onvif.org/ver10/recording/wsdl"
        xmlns:tac="http://www.onvif.org/ver10/accesscontrol/wsdl"
        xmlns:tdc="http://www.onvif.org/ver10/doorcontrol/wsdl"
        xmlns:tth="http://www.onvif.org/ver10/thermal/wsdl"
        xmlns:tcr="http://www.onvif.org/ver10/credential/wsdl"
        xmlns:tar="http://www.onvif.org/ver10/accessrules/wsdl"
        xmlns:tsc="http://www.onvif.org/ver10/schedule/wsdl"
        xmlns:trv="http://www.onvif.org/ver10/receiver/wsdl"
        xmlns:tpv="http://www.onvif.org/ver10/provisioning/wsdl"
        xmlns:ter="http://www.onvif.org/ver10/error">
        <s:Body>
            <tptz:ContinuousMove>
                <tptz:ProfileToken>{profile_token}</tptz:ProfileToken>
                <tptz:Velocity>
                    <tt:PanTilt x="{xspeed}" y="{yspeed}"></tt:PanTilt>
                    </tptz:Velocity>
                </tptz:ContinuousMove>
            </s:Body>
        </s:Envelope>
    """
    
    #print(soap_env)

    # Define the URL and payload
    url = "http://"+globalip+":"+globalport+"/onvif/services"
    #data = '''<YOUR_SOAP_PAYLOAD_HERE>'''  # Replace this with your SOAP XML payload

    headers = {
        "Host": ""+globalip+":"+globalport+"",
        "Content-Type": 'application/soap+xml; charset=utf-8; action="http://www.onvif.org/ver20/ptz/wsdl/ContinuousMove"',
        "Connection": "close"
    }

    # Use Digest Authentication
    auth = HTTPDigestAuth(username, password)  # Replace with your password

    # Send the POST request
    return send_soap_request(url, soap_env, headers, auth)

def soapGetPtzStatus(globalip, globalport, username, password, profile_token):
    #print("soapGetPtzStatus")
    soap_env = f"""<?xml version="1.0" encoding="UTF-8"?>
    <s:Envelope
        xmlns:s="http://www.w3.org/2003/05/soap-envelope"
        xmlns:enc="http://www.w3.org/2003/05/soap-encoding"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:wsa="http://www.w3.org/2005/08/addressing"
        xmlns:ds="http://www.w3.org/2000/09/xmldsig#"
        xmlns:wsse="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd"
        xmlns:wsu="http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd"
        xmlns:wsnt="http://docs.oasis-open.org/wsn/b-2"
        xmlns:tt="http://www.onvif.org/ver10/schema"
        xmlns:tds="http://www.onvif.org/ver10/device/wsdl"
        xmlns:trt="http://www.onvif.org/ver10/media/wsdl"
        xmlns:tr2="http://www.onvif.org/ver20/media/wsdl"
        xmlns:tev="http://www.onvif.org/ver10/events/wsdl"
        xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"
        xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl"
        xmlns:tan="http://www.onvif.org/ver20/analytics/wsdl"
        xmlns:pt="http://www.onvif.org/ver10/pacs"
        xmlns:tmd="http://www.onvif.org/ver10/deviceIO/wsdl"
        xmlns:trp="http://www.onvif.org/ver10/replay/wsdl"
        xmlns:tse="http://www.onvif.org/ver10/search/wsdl"
        xmlns:trc="http://www.onvif.org/ver10/recording/wsdl"
        xmlns:tac="http://www.onvif.org/ver10/accesscontrol/wsdl"
        xmlns:tdc="http://www.onvif.org/ver10/doorcontrol/wsdl"
        xmlns:tth="http://www.onvif.org/ver10/thermal/wsdl"
        xmlns:tcr="http://www.onvif.org/ver10/credential/wsdl"
        xmlns:tar="http://www.onvif.org/ver10/accessrules/wsdl"
        xmlns:tsc="http://www.onvif.org/ver10/schedule/wsdl"
        xmlns:trv="http://www.onvif.org/ver10/receiver/wsdl"
        xmlns:tpv="http://www.onvif.org/ver10/provisioning/wsdl"
        xmlns:ter="http://www.onvif.org/ver10/error">
        <s:Body>
            <tptz:GetStatus xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl">
                <tptz:ProfileToken>{profile_token}</tptz:ProfileToken>
            </tptz:GetStatus>
        </s:Body>
    </s:Envelope>
    """

    headers = {
        "Host": globalip+":"+globalport,
        "Content-Type": 'application/soap+xml; charset=utf-8; action="http://www.onvif.org/ver20/ptz/wsdl/ContinuousMove"',
        "Connection": "close"
    }

    # Use Digest Authentication
    auth = HTTPDigestAuth(username, password)  # Replace with your password
    services_path = "/onvif/services"
    url = f"http://{globalip}{services_path}"

    # Send the POST request
    return send_soap_request(url, soap_env, headers, auth)

class SidusSS109HTPTZ(object):
    PT_DIRECTION_POSITIVE = 1
    PT_DIRECTION_NEGATIVE = -1
    PT_DIRECTION_NONE = 0
        
    def __init__(self, username, password, ip_addr, port=80):
        self.username = username
        self.password = password
        self.ip_addr = ip_addr
        self.port_str = str(port)
        
        soap_device_info = soapGetDeviceInformation(self.ip_addr, self.port_str, self.username, self.password)
        self.parseSoapDeviceInfo(soap_device_info.text)
        #print('Debug: Device Info = ' + str(self.device_info))
        
        self.profile_token, dummy = soapGetProfileToken(self.ip_addr, self.port_str, username, password)
                    
        # Hard-coded parameters... could potentially be extracted via ONVIF, but for now just set them
        self.max_speed_ratio = 1.0
        self.min_speed_ratio = 0.0
        self.supports_adjustable_speed = True
        self.max_pan_position_ratio = 1.0
        self.min_pan_position_ratio = -1.0
        self.max_tilt_position_ratio = 1.0
        self.min_tilt_position_ratio = -1.0
        self.supports_absolute_position = True # Per ONVIF-PTZ-Service-Spec, existence of this Space element indicates absolute positioning is possible.
        self.max_timeout = 10.0
        self.min_timeout = 0.0
        self.reports_position = True
        self.can_home = False # Might be supported through ONVIF, but we'll just do it at the node level
        self.home_position_adjustable = False # Might be supported through ONVIF, but we'll just do it at the node level
        self.max_preset_count = 0 # Might be supported through ONVIF, but we'll just do it at the node level

    def stopMotion(self):
        soapPTZContinuous(self.ip_addr, self.port_str, self.username, self.password, self.profile_token, 0.0, 0.0)

    def jog(self, pan_direction, tilt_direction, speed_ratio, time_s = 1):
        pan_speed_ratio = pan_direction * speed_ratio
        tilt_speed_ratio = tilt_direction * speed_ratio
        soapPTZContinuous(self.ip_addr, self.port_str, self.username, self.password, self.profile_token, pan_speed_ratio, tilt_speed_ratio)

    def getPositionLimitsInDegrees(self):
        # For this generic driver, can only set these to some arbitrary value. Users can override in the device's config file (after saving one as necessary)
        position_limits_deg = {
            'max_yaw_hardstop_deg' : 180.0,
            'min_yaw_hardstop_deg' : -180.0,
            'max_pitch_hardstop_deg' : 180.0,
            'min_pitch_hardstop_deg' : -180.0,
            'max_yaw_softstop_deg' : 179.9,
            'min_yaw_softstop_deg' : -179.9,
            'max_pitch_softstop_deg' : 179.9,
            'min_pitch_softstop_deg' : -179.9
        }
        return position_limits_deg
    
    def moveToPosition(self, pan_position_ratio, tilt_position_ratio, speed_ratio):
        #print(f'Debug: sidus_driver moveToPosition ({pan_position_ratio},{tilt_position_ratio})')
        soapPTZ(self.ip_addr, self.port_str, self.username, self.password, self.profile_token, speed_ratio, pan_position_ratio, tilt_position_ratio)

    def getCurrentPosition(self):
        soap_status = soapGetPtzStatus(self.ip_addr, self.port_str, self.username, self.password, self.profile_token)
        soap_status_text = soap_status.text

        current_pan_ratio = 0.0
        current_tilt_ratio = 0.0
        # Don't want to go through the hassle of full SOAP parsing, so hack it here
        # Search for and parse lines of the form 	
        #    <tt:PanTilt x="-0.02583405" y="0"></tt:PanTilt>
        for line in soap_status_text.splitlines():
            if "<tt:PanTilt x=" in line:
               tokens = line.split('"') # Split on the quotation marks
               current_pan_ratio = float(tokens[1])
               current_tilt_ratio = float(tokens[3])
               break

        return current_pan_ratio, current_tilt_ratio
    
    def parseSoapDeviceInfo(self, soap_device_info):
        # Initialize to unknown
        self.device_info = {
            "Manufacturer" : "Unknown",
            "Model" : "Unknown",
            "FirmwareVersion" : "Unknown",
            "SerialNumber" : "Unknown",
            "HardwareId" : "Unknown"
        }

        for line in soap_device_info.splitlines():
            if "<tds:Manufacturer>" in line:
                self.device_info["Manufacturer"] = (line.split('>')[1]).split('<')[0]
            elif "<tds:Model>" in line:
                self.device_info["Model"] = (line.split('>')[1]).split('<')[0]
            elif "<tds:FirmwareVersion>" in line:
                self.device_info["FirmwareVersion"] = (line.split('>')[1]).split('<')[0]
            elif "<tds:SerialNumber>" in line:
                self.device_info["SerialNumber"] = (line.split('>')[1]).split('<')[0]
            elif "<tds:HardwareId>" in line:
                self.device_info["HardwareId"] = (line.split('>')[1]).split('<')[0]
    
    def getDeviceInfo(self):
        # Need to parse out of the soap response that is self.device_info
        return self.device_info
    
    def getDeviceSerialNumber(self):
        return self.device_info["SerialNumber"]
    
    def getDeviceFirmwareVersion(self):
        return self.device_info["FirmwareVersion"]
    
    def getDeviceHardwareId(self):
        return self.device_info["HardwareId"]
    
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
    # SIDUS SS109HT Provided for testing
    USER = 'admin'
    PASSWORD = 'SidusSidus123$'
    IP_ADDR = '192.168.0.120'
    PORT = 80 
    
    driver = SidusSS109HTPTZ(USER, PASSWORD, IP_ADDR, PORT)

    print(driver.getDeviceInfo())
    
    driver.stopMotion()

    print("Moving to (0, 0)")
    driver.moveToPosition(0,0,0.5)
    time.sleep(5)

    print("Jogging (+,+) for 2 seconds")
    driver.jog(pan_direction = driver.PT_DIRECTION_POSITIVE, tilt_direction = driver.PT_DIRECTION_POSITIVE, speed_ratio=0.5)
    time.sleep(2)
    driver.stopMotion()
    time.sleep(1)
    print("Position:" + str(driver.getCurrentPosition()))
    #driver.setPresetHere(0)
    print("Jogging (-,-) for 2 seconds")
    driver.jog(pan_direction = driver.PT_DIRECTION_NEGATIVE, tilt_direction = driver.PT_DIRECTION_NEGATIVE, speed_ratio=0.5)
    time.sleep(2)
    driver.stopMotion()
    print("Position:" + str(driver.getCurrentPosition()))
    time.sleep(1)
    
    #driver.setPresetHere(1)
    #print("Returning to position 0")
    #driver.gotoPreset(preset_index = 0, speed_ratio = 1)
    #time.sleep(2)

    #print("Returning to position 1")
    #driver.gotoPreset(preset_index = 1, speed_ratio = 1)
    #time.sleep(2)

    #print("Returning home")
    #driver.goHome(speed_ratio = 1)
    #time.sleep(2)
    #driver.jog(-0.5, -0.5)

    print("Moving to (0, 0)")
    driver.moveToPosition(0,0,0.5)
    time.sleep(5)
    print("Final Position:" + str(driver.getCurrentPosition()))

