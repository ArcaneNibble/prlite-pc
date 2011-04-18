#!/usr/bin/env python

import struct

#assume 1st byte = dst addr
#sum includes everything else
def packet_add_csum(data):
	sum = 0
	for c in data[1:]:
		sum += struct.unpack("B",c)[0]
	return data + struct.pack("B",sum & 0xFF)

#we can't actually rx 1st byte (dst addr)
#sum doesn't include that anyways
#the last byte should be the checksum
def packet_check_csum(data):
	sum = struct.unpack("B",data[-1])[0]
	stuff = data[:-1]
	real_sum = 0
	for c in stuff:
		real_sum += struct.unpack("B",c)[0]
	real_sum = real_sum & 0xFF
	return real_sum == sum

if __name__=='__main__':
	import binascii
	packet = '\x01\x02\x03\x04\x05'
	print binascii.hexlify(packet)
	packet = packet_add_csum(packet)
	print binascii.hexlify(packet)
	packet = packet[1:]
	print packet_check_csum(packet)
