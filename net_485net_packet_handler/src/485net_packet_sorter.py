#!/usr/bin/env python

import roslib
roslib.load_manifest('net_485net_packet_handler')
import rospy
from packets_485net.msg import *
import binascii
import struct

stream_pub = None
dgram_pub = None
bootloader_pub = None
outgoing_pub = None

def do_checksum(data):
	csum = 0;
	for b in data:
		csum = csum + struct.unpack("B", b)[0]
		csum = csum & 0xFF
	csum = 0x100 - csum
	csum = csum & 0xFF
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
