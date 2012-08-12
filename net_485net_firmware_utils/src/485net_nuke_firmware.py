#!/usr/bin/env python

import roslib
roslib.load_manifest('net_485net_firmware_utils')
import rospy
from packets_485net.msg import *
import binascii
import struct
import sys
import time

def main(what, addr):
	rospy.init_node('net_485net_nuke_firmware')
	pub = rospy.Publisher("net_485net_outgoing_dgram", packet_485net_dgram)
	
	reply = packet_485net_dgram()
	
	reply.header.stamp = rospy.Time.now()
	reply.source = 0xF0
	reply.destination = addr
	reply.sport = 7
	reply.dport = 7
	if what == "x":
		reply.data = "\x55\xaaRST!\xc3\x3c"
	elif what == "a":
		reply.data = "\x55\xaaABCD\xc3\x3c"
	elif what == "l":
		reply.data = "\x55\xaaLMNO\xc3\x3c"
	elif what == "m":
		reply.data = "\x55\xaaMU%s%s%s%s" % (binascii.unhexlify(sys.argv[3]), binascii.unhexlify(sys.argv[4]), binascii.unhexlify(sys.argv[5]), binascii.unhexlify(sys.argv[6]))
	
	while not rospy.is_shutdown():
		pub.publish(reply)

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print "Usage: %s x(everything)|a|l addr\n\tor m addr aa bb cc dd" % sys.argv[0]
		sys.exit(1)
	try:
		main(sys.argv[1], ord(binascii.unhexlify(sys.argv[2])))
	except rospy.ROSInterruptException: pass
