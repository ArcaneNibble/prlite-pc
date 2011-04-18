#!/usr/bin/env python

import roslib
roslib.load_manifest('i2c_serial_net_node')
import rospy
from i2c_net_utils.msg import *
from i2c_net_utils.srv import *
import serial
import string
import binascii
import threading

#is there something better than a global?
ser = serial.Serial('/dev/magellan-i2c-serial', 1000000)
read_lock = threading.Lock()
write_lock = threading.Lock()

#ugly
prepend_this = ''
append_this = ''

def callback(data):
	packet = packet_to_bcast(data)
	rospy.logdebug("TX: '%s'" % packet)
	write_lock.acquire()
	ser.write(packet)
	write_lock.release()

#get rid of anything except hex and \r \n
def remove_garbage(data):
	ret = ''
	for c in data:
		if c in (string.hexdigits + '\r\n'):
			ret += c
		else:
			rospy.logwarn("Threw away %s (%s)" % (c, binascii.hexlify(c)))
	return ret

def piece_to_packet(data):
	if len(data) % 2 != 0:
		rospy.logwarn("Packet %s is missing a character" % data)
		return i2c_packet(data='')
	rospy.logdebug("RX: %s" % data)
	bytes = binascii.unhexlify(data)
	return i2c_packet(data=bytes)

def packet_to_bcast(data):
	data = data.data
	hex = binascii.hexlify(data)
	packet = "00"+hex+" "
	return packet

def synchronous(req):
	global prepend_this
	global append_this
	packet = "%02X%s. " % (req.resplen, binascii.hexlify(req.req))
	rospy.logdebug("TX: '%s'" % packet)
	read_lock.acquire()
	write_lock.acquire()
	ser.write(packet)
	ser.flush()
	tmp = ''
	while tmp.find('s') == -1:
		tmp += ser.read(ser.inWaiting())
		if len(tmp) > 1000:
			rospy.logwarn("Couldn't get synchronous reply after 1000 bytes, giving up")
			prepend_this = tmp
			append_this = ''
			read_lock.release()
			write_lock.release()
			return ''
			
	stuff = tmp.split('s')
	prepend_this = stuff[0]
	#this will blow up with two s's but that should never happen
	tmp = stuff[1]
	while tmp.find('\r\n') == -1:
		tmp += ser.read(ser.inWaiting())
	stuff = tmp.split('\r\n',1)
	real_stuff = stuff[0]
	append_this = stuff[1]
	read_lock.release()
	write_lock.release()
	rospy.logdebug("RX: %s" % real_stuff)
	respbytes = binascii.unhexlify(real_stuff)
	return respbytes

def main():
	global prepend_this
	global append_this
	rospy.init_node('i2c_serial_net_node')
	rospy.Subscriber('i2c_in_bcast', i2c_packet, callback)
	rospy.Service('i2c_in_synch', i2c_sync_request, synchronous)
	pub = rospy.Publisher('i2c_out', i2c_packet)
	rxbuf = ''
	while not rospy.is_shutdown():
		read_lock.acquire()
		tmpbuf = prepend_this + append_this + ser.read(ser.inWaiting())
		prepend_this = ''
		append_this = ''
		read_lock.release()
		tmpbuf = remove_garbage(tmpbuf)
		rxbuf += tmpbuf
		pieces = rxbuf.split('\r\n')
		rxbuf = pieces[-1]
		pieces = pieces[:-1]
		for piece in pieces:
			packet = piece_to_packet(piece)
			pub.publish(packet)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
