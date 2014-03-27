#!/usr/bin/env python

import roslib
#roslib.load_manifest('net_485net_packet_handler')
roslib.load_manifest('pr2lite_moveit_config')
import rospy
#from packets_485net.msg import *
from pr2lite_moveit_config.msg import *
import binascii
import struct

stream_pub = None
dgram_pub = None
bootloader_pub = None
outgoing_pub = None

crc8_table = [
	#  0	 1	   2	 3	   4	 5	   6	 7	   8	 9	   A	 B	   C	 D	   E	 F
	0x00, 0xF7, 0xB9, 0x4E, 0x25, 0xD2, 0x9C, 0x6B, 0x4A, 0xBD, 0xF3, 0x04, 0x6F, 0x98, 0xD6, 0x21,
	0x94, 0x63, 0x2D, 0xDA, 0xB1, 0x46, 0x08, 0xFF, 0xDE, 0x29, 0x67, 0x90, 0xFB, 0x0C, 0x42, 0xB5,
	0x7F, 0x88, 0xC6, 0x31, 0x5A, 0xAD, 0xE3, 0x14, 0x35, 0xC2, 0x8C, 0x7B, 0x10, 0xE7, 0xA9, 0x5E,
	0xEB, 0x1C, 0x52, 0xA5, 0xCE, 0x39, 0x77, 0x80, 0xA1, 0x56, 0x18, 0xEF, 0x84, 0x73, 0x3D, 0xCA,
	0xFE, 0x09, 0x47, 0xB0, 0xDB, 0x2C, 0x62, 0x95, 0xB4, 0x43, 0x0D, 0xFA, 0x91, 0x66, 0x28, 0xDF,
	0x6A, 0x9D, 0xD3, 0x24, 0x4F, 0xB8, 0xF6, 0x01, 0x20, 0xD7, 0x99, 0x6E, 0x05, 0xF2, 0xBC, 0x4B,
	0x81, 0x76, 0x38, 0xCF, 0xA4, 0x53, 0x1D, 0xEA, 0xCB, 0x3C, 0x72, 0x85, 0xEE, 0x19, 0x57, 0xA0,
	0x15, 0xE2, 0xAC, 0x5B, 0x30, 0xC7, 0x89, 0x7E, 0x5F, 0xA8, 0xE6, 0x11, 0x7A, 0x8D, 0xC3, 0x34,
	0xAB, 0x5C, 0x12, 0xE5, 0x8E, 0x79, 0x37, 0xC0, 0xE1, 0x16, 0x58, 0xAF, 0xC4, 0x33, 0x7D, 0x8A,
	0x3F, 0xC8, 0x86, 0x71, 0x1A, 0xED, 0xA3, 0x54, 0x75, 0x82, 0xCC, 0x3B, 0x50, 0xA7, 0xE9, 0x1E,
	0xD4, 0x23, 0x6D, 0x9A, 0xF1, 0x06, 0x48, 0xBF, 0x9E, 0x69, 0x27, 0xD0, 0xBB, 0x4C, 0x02, 0xF5,
	0x40, 0xB7, 0xF9, 0x0E, 0x65, 0x92, 0xDC, 0x2B, 0x0A, 0xFD, 0xB3, 0x44, 0x2F, 0xD8, 0x96, 0x61,
	0x55, 0xA2, 0xEC, 0x1B, 0x70, 0x87, 0xC9, 0x3E, 0x1F, 0xE8, 0xA6, 0x51, 0x3A, 0xCD, 0x83, 0x74,
	0xC1, 0x36, 0x78, 0x8F, 0xE4, 0x13, 0x5D, 0xAA, 0x8B, 0x7C, 0x32, 0xC5, 0xAE, 0x59, 0x17, 0xE0,
	0x2A, 0xDD, 0x93, 0x64, 0x0F, 0xF8, 0xB6, 0x41, 0x60, 0x97, 0xD9, 0x2E, 0x45, 0xB2, 0xFC, 0x0B,
	0xBE, 0x49, 0x07, 0xF0, 0x9B, 0x6C, 0x22, 0xD5, 0xF4, 0x03, 0x4D, 0xBA, 0xD1, 0x26, 0x68, 0x9F
]

def do_checksum(data):
	csum = 0xFF;
	for b in data:
		byte = struct.unpack("B", b)[0]
		csum = crc8_table[(csum ^ byte) & 0xFF]
		csum = csum & 0xFF
	csum = csum ^ 0xFF
	return csum

def handler_incoming(packet):
	global stream_pub
	global dgram_pub
	global bootloader_pub

	#packet is a packet_485net_raw
	proto = struct.unpack("B", packet.data[2])[0] & 0b11000000
	
	actual_csum = do_checksum(packet.data[:-1])
	if actual_csum != struct.unpack("B", packet.data[-1])[0]:
		rospy.logwarn("Packet with bad checksum: %s" % binascii.hexlify(packet.data))
		return
	
	if proto == 0x00:
		#datagram
		if len(packet.data) < 4:
			rospy.logwarn("Impossibly short (but valid checksum) packet: %s" % binascii.hexlify(packet.data))
			return
		
		pkt = packet_485net_dgram()
		pkt.header		= packet.header
		pkt.source		= struct.unpack("B", packet.data[0])[0]
		pkt.destination	= struct.unpack("B", packet.data[1])[0]
		ports = struct.unpack("B", packet.data[2])[0]
		pkt.sport		= (ports & 0b00111000) >> 3
		pkt.dport		= (ports & 0b00000111)
		pkt.data		= packet.data[3:-1]
		pkt.checksum	= struct.unpack("B", packet.data[-1])[0]
		
		dgram_pub.publish(pkt)
	elif proto == 0x40:
		#stream
		if len(packet.data) < 7:
			rospy.logwarn("Impossibly short (but valid checksum) packet: %s" % binascii.hexlify(packet.data))
			return
		
		pkt = packet_485net_stream()
		pkt.header		= packet.header
		pkt.source		= struct.unpack("B", packet.data[0])[0]
		pkt.destination	= struct.unpack("B", packet.data[1])[0]
		ports = struct.unpack("B", packet.data[2])[0]
		pkt.sport		= (ports & 0b00111000) >> 3
		pkt.dport		= (ports & 0b00000111)
		pkt.type		= struct.unpack("B", packet.data[3])[0]
		pkt.seq			= struct.unpack("<H", packet.data[4:6])[0]
		pkt.data		= packet.data[6:-1]
		pkt.checksum	= struct.unpack("B", packet.data[-1])[0]
		
		stream_pub.publish(pkt)
	elif proto == 0xC0:
		#bootloader
		if len(packet.data) < 4:
			rospy.logwarn("Impossibly short (but valid checksum) packet: %s" % binascii.hexlify(packet.data))
			return
		
		pkt = packet_485net_bootloader()
		pkt.header		= packet.header
		pkt.source		= struct.unpack("B", packet.data[0])[0]
		pkt.destination	= struct.unpack("B", packet.data[1])[0]
		pkt.protocol	= struct.unpack("B", packet.data[2])[0]
		pkt.data		= packet.data[3:-1]
		pkt.checksum	= struct.unpack("B", packet.data[-1])[0]
		
		bootloader_pub.publish(pkt)
	else:
		#reserved
		rospy.logwarn("Packet with reserved protocol: %s" % binascii.hexlify(packet.data))

def handler_stream(packet):
	global outgoing_pub
	#packet is a packet_485net_stream
	pkt = packet_485net_raw()
	pkt.header = packet.header
	pkt.data = struct.pack("BBBB<H", packet.source, packet.destination, 0x40 | (packet.sport << 3) | packet.dport, packet.type, packet.seq) + packet.data
	checksum = do_checksum(pkt.data)
	pkt.data = pkt.data + struct.pack("B", checksum)
	
	outgoing_pub.publish(pkt)

def handler_dgram(packet):
	global outgoing_pub
	#packet is a packet_485net_dgram
	pkt = packet_485net_raw()
	pkt.header = packet.header
	pkt.data = struct.pack("BBB", packet.source, packet.destination, 0x00 | (packet.sport << 3) | packet.dport) + packet.data
	checksum = do_checksum(pkt.data)
	pkt.data = pkt.data + struct.pack("B", checksum)
	
	outgoing_pub.publish(pkt)

def handler_bootloader(packet):
	global outgoing_pub
	#packet is a packet_485net_bootloader
	pkt = packet_485net_raw()
	pkt.header = packet.header
	pkt.data = struct.pack("BBB", packet.source, packet.destination, 0xC0 | packet.protocol) + packet.data
	checksum = do_checksum(pkt.data)
	pkt.data = pkt.data + struct.pack("B", checksum)
	
	outgoing_pub.publish(pkt)

def main():
	global stream_pub
	global dgram_pub
	global bootloader_pub
	global outgoing_pub
	
	print "485net packet sorter"
	rospy.init_node('net_485net_packet_handler')
	
	stream_pub = rospy.Publisher("net_485net_incoming_stream", packet_485net_stream)
	dgram_pub = rospy.Publisher("net_485net_incoming_dgram", packet_485net_dgram)
	bootloader_pub = rospy.Publisher("net_485net_incoming_bootloader", packet_485net_bootloader)
	outgoing_pub = rospy.Publisher("net_485net_outgoing_packets", packet_485net_raw)
	
	rospy.Subscriber("net_485net_incoming_packets", packet_485net_raw, handler_incoming)
	rospy.Subscriber("net_485net_outgoing_stream", packet_485net_stream, handler_stream)
	rospy.Subscriber("net_485net_outgoing_dgram", packet_485net_dgram, handler_dgram)
	rospy.Subscriber("net_485net_outgoing_bootloader", packet_485net_bootloader, handler_bootloader)
	
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
