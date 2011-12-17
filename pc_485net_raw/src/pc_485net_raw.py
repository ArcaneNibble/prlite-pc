#!/usr/bin/env python

import roslib
roslib.load_manifest('pc_485net_raw')
import rospy
from packets_485net.msg import *
import serial
import binascii
import struct

ser = None

def do_checksum(data):
	csum = 0;
	for b in data:
		csum = csum + struct.unpack("B", b)[0]
		csum = csum & 0xFF
	csum = 0x100 - csum
	csum = csum & 0xFF
	return csum

def decode_blob(blob):
	out = []
	lastgoodbyte = 0;
	#this is a really horrible hack
	blob = decode_blob.leftovers + blob
	#print binascii.hexlify(blob)
	i = 0
	while i < len(blob):
	#for i in range(0, len(blob)):
		print binascii.hexlify(blob)
		print i
		if blob[i] == "\x00":
			i = i + 1
			continue
			#0 can never be a source address
			#otherwise we always pick up runs of 00 as valid packets
		for l in range(3, min(len(blob)-i, 64)):
			#the range needs to be at least 3 for normal packets
			#otherwise 10 f0 in the header registers as a valid packet
			print "from %d length %d" % (i, l)
			#this assumes all packets do have a checksum
			possiblepkt = blob[i:i+l]
			print binascii.hexlify(possiblepkt)
			possiblecsum = struct.unpack("B", blob[i+l])[0]
			checksum = do_checksum(possiblepkt)
			print "csum is %d %d" % (possiblecsum, checksum)
			if checksum == possiblecsum:
				print binascii.hexlify(possiblepkt)
				pkt = packet_485net_raw()
				pkt.header.stamp = rospy.Time.now()
				pkt.data = possiblepkt + blob[i+l]
				out.append(pkt)
				lastgoodbyte = i+l+1
				#even more hacky
				i = lastgoodbyte
				print "jumping to %d" % i
				break
		i = i + 1
	decode_blob.leftovers = blob[lastgoodbyte:]
	return out
			
#hack
decode_blob.leftovers = ""

def tx_packet(packet):
	#There is no good way to determine if the bus is free
	#We just transmit
	global ser
	ser.setRTS(False)
	ser.write(packet.data)
	ser.flush()
	ser.setRTS(True)

def main():
	global ser
	print "PC 485net interface (raw serial)"
	rospy.init_node('pc_485net_raw_node')
	serport = rospy.get_param("~serport", "/dev/magellan-i2c-serial")
	baud = rospy.get_param("~baud", 1000000)
	print "Using serial port %s at baud rate %d" % (serport, baud)
	
	ser = serial.Serial("/dev/magellan-i2c-serial", 1000000, timeout=0)
	ser.setRTS(True)
	ser.setDTR(True)
	
	pub = rospy.Publisher("net_485net_incoming_packets", packet_485net_raw)
	rospy.Subscriber("net_485net_outgoing_packets", packet_485net_raw, tx_packet)
	
	while not rospy.is_shutdown():
		bytes = ser.read(500)
		if len(bytes) > 0:
			packets = decode_blob(bytes)
			for p in packets:
				pub.publish(p)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
