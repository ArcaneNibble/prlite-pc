#!/usr/bin/env python

import roslib
roslib.load_manifest('net_485net_firmware_utils')
import rospy
from packets_485net.msg import *
import binascii
import struct
import sys
import time

def parse_uc_list(file):
	f = open(file, "r")
	uc_list = {}
	firm_list = {}
	while True:
		l = f.readline()
		if l == "":
			raise Exception("Premature end of uc.txt file")
		if l[0] != "#" and l[0] != "\n":
			l = l.strip()
			if l == "=":
				break
			vals = l.split("|")
			uc_list[ord(binascii.unhexlify(vals[0]))] = (vals[1], vals[2])
	while True:
		l = f.readline()
		if l == "":
			break
		if l[0] != "#" and l[0] != "\n":
			l = l.strip()
			vals = l.split("|")
			firm_list[vals[0]] = (vals[1], vals[2])
	f.close()
	#print uc_list
	#print firm_list
	return (uc_list, firm_list)

def download_lib(addr, uc_list, firm_list):
	type = uc_list[addr][0]
	firm = firm_list[type][1]
	
	print "%X is a %s which requires %s" % (addr, type, firm)
	
	f = open(firm, "rb")
	firm_data = f.read()
	f.close()
	
	pub = rospy.Publisher("net_485net_outgoing_bootloader", packet_485net_bootloader)
	
	done = [False]
	def feed_data(done, addr, firm_data, pub, reqpkt):
		if reqpkt.source != addr:
			return
		if reqpkt.protocol == 0xC2:
			done[0] = True
			return
		if reqpkt.protocol != 0xC1:
			return
		if len(reqpkt.data) != 1:
			return
		#32 bytes
		block = ord(reqpkt.data[0])
		print "Downloading block %d of 256" % block
		bytes = firm_data[block*32:(block+1)*32]
		reply = packet_485net_bootloader()
		
		reply.header.stamp = rospy.Time.now()
		reply.protocol = 0xC1
		reply.source = 0xF0
		reply.destination = addr
		reply.data = chr(block) + bytes
		
		pub.publish(reply)
	
	temp_sub = rospy.Subscriber("net_485net_incoming_bootloader", packet_485net_bootloader, lambda x: feed_data(done, addr, firm_data, pub, x))
	while not done[0]:
		rospy.sleep(1)
	
def download_app(addr, uc_list, firm_list):
	type = uc_list[addr][0]
	firm = firm_list[type][0]
	
	print "%X is a %s which requires %s" % (addr, type, firm)
	
	f = open(firm, "rb")
	firm_data = f.read()
	f.close()
	
	pub = rospy.Publisher("net_485net_outgoing_bootloader", packet_485net_bootloader)
	
	done = [False]
	def feed_data(done, addr, firm_data, pub, reqpkt):
		if reqpkt.source != addr:
			return
		if reqpkt.protocol != 0xC2:
			return
		if len(reqpkt.data) != 2:
			return
		#32 bytes
		block = struct.unpack("<H", reqpkt.data)[0]
		print "Downloading block %d of 640" % block
		bytes = firm_data[block*32:(block+1)*32]
		reply = packet_485net_bootloader()
		
		reply.header.stamp = rospy.Time.now()
		reply.protocol = 0xC2
		reply.source = 0xF0
		reply.destination = addr
		reply.data = struct.pack("<H", block) + bytes
		
		pub.publish(reply)
		
		#hack
		if block == 639:
			done[0] = True
	
	temp_sub = rospy.Subscriber("net_485net_incoming_bootloader", packet_485net_bootloader, lambda x: feed_data(done, addr, firm_data, pub, x))
	while not done[0]:
		rospy.sleep(1)

def main(addr):
	rospy.init_node('net_485net_firmware_tool')
	uc_list,firm_list = parse_uc_list(rospy.get_param("/uc_config_file", "uc.txt"))
	#print uc_list
	#print firm_list
	print "Using address %X" % addr
	if not addr in uc_list:
		print "Cannot find specified address in config list"
		sys.exit(1)
	
	library_or_app = [-1]
	def gather_information_callback(library_or_app, pkt):
		if pkt.source == addr or pkt.source == 0xFE:
			if pkt.protocol == 0xC0:
				#need to put in address
				pub = rospy.Publisher("net_485net_outgoing_bootloader", packet_485net_bootloader)
				reply = packet_485net_bootloader()
				
				reply.header.stamp = rospy.Time.now()
				reply.protocol = 0xC0
				reply.source = 0xF0
				reply.destination = 0xFE
				reply.data = struct.pack("<B", addr)
				
				pub.publish(reply)
				print "Sent address"
			if pkt.protocol == 0xC1:
				#print pkt
				library_or_app[0] = 1
			if pkt.protocol == 0xC2:
				library_or_app[0] = 0
	temp_sub = rospy.Subscriber("net_485net_incoming_bootloader", packet_485net_bootloader, lambda x: gather_information_callback(library_or_app, x))
	print "Listening..."
	rospy.sleep(5)
	temp_sub.unregister()
	if library_or_app[0] == -1:
		print "Specified address is not asking for a download?"
		sys.exit(1)
	#print library_or_app
	if library_or_app[0] == 1:
		print "Loading 485net library..."
		download_lib(addr, uc_list, firm_list)
	if library_or_app[0] == 0:
		print "Loading application..."
		download_app(addr, uc_list, firm_list)
	print "Done!"

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print "Usage: %s addr" % sys.argv[0]
		sys.exit(1)
	try:
		main(ord(binascii.unhexlify(sys.argv[1])))
	except rospy.ROSInterruptException: pass
