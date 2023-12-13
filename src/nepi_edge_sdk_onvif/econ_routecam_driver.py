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

import socket
import struct

from nepi_edge_sdk_onvif.onvif_cam_driver import OnvifIFCamDriver

ECON_ROUTECAM_DRIVER_ID = 'EConRouteCam'

class EConRouteCamDriver(OnvifIFCamDriver):
    SOCKET_SERVER_PORT = 8080

    SET_VIDEO_PARAM_CMD = struct.pack('<B', 0x3)
    GET_VIDEO_PARAM_CMD = struct.pack('<B', 0x23)
    SET_GET_VIDEO_PARAM_LENGTH = struct.pack('<i', 512)

    VIDEO_ENC_H264_ARRAY = struct.pack('<4s16x', b'H264') # Zero-padded fixed-length string
    #VIDEO_ENC_H264_ARRAY = ['H','2','6','4',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # Fixed 20-characters
    VIDEO_ENC_HEVC_ARRAY = struct.pack('<4s16x', b'HEVC')
    #VIDEO_ENC_HEVC_ARRAY = ['H','E','V','C',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] # Fixed 20-characters

    # Resolution options are fixed
    RESOLUTIONS_AVAIL_LIST = [{'Width':320, 'Height':240}, {'Width':640, 'Height':480}, 
                              {'Width':1280, 'Height':720}, {'Width':1920, 'Height':1080}]

    VIDEO_PARAM_RESP_LENGTH = 517

    VIDEO_SETTINGS_PORT = struct.pack('<h', 5005) # Hard-coded

    def __init__(self, username, password, ip_addr, onvif_port=8000):
        super(EConRouteCamDriver, self).__init__(username, password, ip_addr, onvif_port)

        # Now set up the socket client for non-Onvif operations
        self.socket_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_client.connect((ip_addr, self.SOCKET_SERVER_PORT))

        # Gather initial Onvif-replaced params
        self.encoder_cfg = {}
        self.retrieveEncoderCfg()
        self.encoder_count = 1
        # Available resolutions are fixed -- no way to query for them
        self.encoder_options = [{'H264' : {'ResolutionsAvailable' : self.RESOLUTIONS_AVAIL_LIST} }]

        # For now, always start up in H264 encoding -- HEVC (i.e. H265) causes issues with OpenCV software decoding
        self.setEncoding(0, self.VIDEO_ENC_H264_ARRAY)
        
    def retrieveEncoderCfg(self):
        # Use socket client to replace Onvif, but otherwise do these just as if they were Onvif-reported to reuse as much other
        # driver code as possible
        get_video_params_packet = self.GET_VIDEO_PARAM_CMD + self.SET_GET_VIDEO_PARAM_LENGTH + bytearray(512) # Zero-padded payoad
        self.socket_client.send(get_video_params_packet)
        resp = self.socket_client.recv(self.VIDEO_PARAM_RESP_LENGTH)
        resp_list = struct.unpack('<Bi20shh488s', resp)
        encoding = resp_list[2]
        img_width = resp_list[3]
        img_height = resp_list[4]

        self.encoder_cfg = {'Encoding': encoding,
                            'Resolution': {'Width': img_width, 'Height': img_height}}
        
    def setEncoderCfg(self, encoding_str, img_width, img_height):
        encoding_packed = struct.pack('20s', encoding_str)
        resolution_packed = struct.pack('<hh', img_width, img_height)

        set_video_params_packet = self.SET_VIDEO_PARAM_CMD + self.SET_GET_VIDEO_PARAM_LENGTH + \
                                  encoding_packed + resolution_packed + \
                                  self.VIDEO_SETTINGS_PORT  + bytearray(512 - 26) # Zero Fill the rest of the 512-byte payload
        self.socket_client.send(set_video_params_packet)

    def getCompressionType(self, video_encoder_id):
        if video_encoder_id >= self.encoder_count:
            return '', None
        
        return self.encoder_cfg["Encoding"], self.encoder_cfg

    def getAvailableResolutions(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return [], None
        
               # Now figure out which resolutions are available for this compression... just use the member variable since "options" don't change
        available_resolutions = self.encoder_options[0]['H264']['ResolutionsAvailable']
        # available_resolution: List of dictionaries: [{Width:x,Height:y}, ...]
        return available_resolutions, self.encoder_cfg # Convenient to return the encoder config, too
    
    def getResolution(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return []
        
        self.retrieveEncoderCfg()
        
        # resolution: Dictionary: [{Width:x, Height:x}]
        resolution = self.encoder_cfg['Resolution']
        return resolution

    def setResolution(self, resolution, video_encoder_id=0, readback_check=True):
        if video_encoder_id >= self.encoder_count:
            return False, "Invalid encoder ID"

        # First, get the available resolutions and current encoder_cfg so that we can update just the resolution
        available_resolutions, _ = self.getAvailableResolutions(video_encoder_id)
        
        # Ensure that the specified resolution is available
        if resolution not in available_resolutions:
            return False, "Invalid resolution"

        # Check whether we need to make a change
        if resolution['Width'] == self.encoder_cfg['Resolution']['Width'] and resolution['Height'] == self.encoder_cfg['Resolution']['Height']:
            return True, "Desired resolution already set"

        # Update the encoder_cfg
        self.setEncoderCfg(self.encoder_cfg['Encoding'], resolution['Width'], resolution['Height'])

        # Check
        if readback_check is True:
            self.retrieveEncoderCfg()
            if resolution['Width'] != self.encoder_cfg['Resolution']['Width'] or resolution['Height'] != self.encoder_cfg['Resolution']['Height']:
                return False, "Camera failed update resolution"

        return True, "Success"
    
    def getAvailableEncodings(self, video_encoder_id):
        if video_encoder_id >= self.encoder_count:
            return []
        
        return [self.VIDEO_ENC_H264_ARRAY, self.VIDEO_ENC_HEVC_ARRAY]

    def getEncoding(self, video_encoder_id=0):
        if video_encoder_id >= self.encoder_count:
            return '', None
        self.retrieveEncoderCfg()
        return self.encoder_cfg['Encoding'].decode(), self.encoder_cfg
    
    def setEncoding(self, video_encoder_id, encoding_str):
        if video_encoder_id >= self.encoder_count:
            return False, "Invalid encoder ID"
        
        if self.encoder_cfg['Encoding'] == encoding_str:
            return True, "Desired encoding already set"

        # Update the encoder_cfg
        self.setEncoderCfg(encoding_str, self.encoder_cfg['Resolution']['Width'], self.encoder_cfg['Resolution']['Height'])

      
        



