#!/usr/bin/env python
'''
Created January, 2011

@author: Dr. Rainer Hessmer

  arduino.py - gateway to Arduino based differential drive base
  Copyright (c) 2011 Dr. Rainer Hessmer.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib
import rospy
import sys

from std_msgs.msg import String

from SerialDataGateway import SerialDataGateway

class serial_comms_node(object):
	'''
	Helper class for communicating with an Arduino board over serial port
	'''

	def ReceivedLine(self,  line):
		self.Publisher.publish(line)
		

	def TransmitWheelSpeeds(self, wheelSpeeds):
		#message = "w " + str(wheelSpeeds) +"\n"
		message = str(wheelSpeeds.data)
		self.SerialDataGateway.Write(message)

		
	def __init__(self, port="/dev/ttyAMA0", baudrate=56700):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		rospy.init_node('serial_comms_node')

		port = rospy.get_param("~port", "/dev/ttyAMA0")
		baudRate = int(rospy.get_param("~baudRate", 57600))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))

		# subscriptions
		rospy.Subscriber("wheelSpeeds", String, self.TransmitWheelSpeeds)
		self.Publisher = rospy.Publisher('Sensors', String, queue_size = 10)

		# CREATE A SERIAL_DATA_GATEWAY OBJECT 
		# pass it a function pointer to _HandleReceivedLine
		self.SerialDataGateway = SerialDataGateway(port, baudRate,  self.ReceivedLine)

	def Start(self):
		#rospy.logdebug("Starting")
		self.SerialDataGateway.Start()

	def Stop(self):
		#rospy.logdebug("Stopping")
		self.SerialDataGateway.Stop()
		

if __name__ == '__main__':
	serial_comms_node = serial_comms_node()
	try:
		serial_comms_node.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		serial_comms_node.Stop()

