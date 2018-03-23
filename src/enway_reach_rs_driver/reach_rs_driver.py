#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Enway GmbH
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__copyright__ = "Enway GmbH - All Rights reserved."
__license__ = "BSD"

import rospy
import socket
import diagnostic_updater
import diagnostic_msgs
import enway_reach_rs_driver.driver
from sensor_msgs.msg import NavSatFix, NavSatStatus
    
FixStatus = {-1 : 'No Fix',
              0 : 'Single Fix',
              1 : 'SBAS Fix',
              2 : 'GBAS Fix'}

class ReachRsDriver(object):
    def __init__(self):
        host = rospy.get_param('~reach_rs_host_or_ip', 'reach.local')
        port = rospy.get_param('~reach_rs_port')
        self.address = (host, port)

        self.socket = None

        self.frameId = rospy.get_param('~reach_rs_frame_id', 'reach_rs')
        self.fixTimeout = rospy.get_param('~fix_timeout', 0.5)
        
        self.driver = enway_reach_rs_driver.driver.RosNMEADriver()
        
        self.diagnostics = diagnostic_updater.Updater()
        self.diagnostics.setHardwareID('Emlid Reach RS')
        self.diagnostics.add('Receiver Status', self.addDiagnotics)
        
        self.connected = False
        self.connection_status = 'not connected'
        self.lastFix = None
        
    def __del__(self):
        if self.socket:
            self.socket.close()
        
    def update(self):
        self.diagnostics.update()
    
    def receivesFixes(self):
        if not self.lastFix:
            return False
        
        duration = (rospy.Time.now() - self.lastFix.header.stamp)
        return duration.to_sec() < self.fixTimeout
        
    def addDiagnotics(self, stat):
        if self.connected and self.receivesFixes():
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Reach RS driver is connected and has a fix')
        elif self.connected:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, 'Reach RS driver is connected but has no fix')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'Reach RS driver has a connection problem')
            
        stat.add('Connected', self.connected)
        stat.add('Connection status', self.connection_status)
        
        if self.lastFix:
            stat.add("Fix status", FixStatus[self.lastFix.status.status])
            stat.add("Seconds since last fix", (rospy.Time.now() - self.lastFix.header.stamp).to_sec())
        else:
            stat.add("Fix status", FixStatus[NavSatStatus.STATUS_NO_FIX])
            stat.add("Seconds since last fix", '-')
        
    def connectToDevice(self):
        rospy.loginfo('Connecting to {0}:{1}...'.format(*self.address))
        
        while not rospy.is_shutdown():
            self.update()
            
            if self.socket:
                self.socket.close()
                self.socket = None

            self.connected = False
            self.connection_status = 'not connected'

            self.socket = socket.socket()

            try:
                self.socket.settimeout(5)
                self.socket.connect(self.address)
                self.connected = True
                self.connection_status = 'connected'
                rospy.loginfo('Successfully connected to device!')
                return
            except socket.timeout:
                self.connection_status = 'connect timeout'
            except socket.error,  msg:
                self.connection_status = 'connect error ({0})'.format(msg)
            
        exit()
        
    def run(self):
        self.connectToDevice()
        
        while not rospy.is_shutdown():
            self.update()
            
            try:
                self.socket.settimeout(self.fixTimeout)
                data = self.socket.recv(1024)
                
                if data == '':
                    rospy.logwarn('Lost connection. Trying to reconnect...')
                    self.connectToDevice()
                else:
                    self.parseData(data)
                    self.connection_status = 'receiving NMEA messages'
            except socket.timeout as t:
                self.connection_status = 'no NMEA messages received'
            except socket.error:
                pass
        
    def parseData(self, data):
        data = data.strip().split()
        
        for sentence in data:
            if 'GPGGA' in sentence or 'RMC' in sentence:
                try:
                    fix = self.driver.add_sentence(sentence, self.frameId)
                    
                    if fix:
                        self.lastFix = fix
                except ValueError as e:
                    rospy.logwarn("Value error, likely due to missing fields in the NMEA message. Error was: %s. Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA sentences that caused it." % e)
