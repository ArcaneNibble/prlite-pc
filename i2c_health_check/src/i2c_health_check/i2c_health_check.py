#!/usr/bin/env python

import roslib
roslib.load_manifest('i2c_health_check')
import rospy
from i2c_net_packets.msg import *
import threading
import time

counter_lock = threading.Lock()
counters = [-1] * 256
times = [-1] * 256

def timer():
	while not rospy.is_shutdown():
		rospy.logdebug("timer")
		
		time_now = time.time()
		
		counter_lock.acquire()
		
		for i in range(0,256):
			if times[i] != -1:
				time_last = times[i]
				rospy.logdebug("%02X last time %f" % (i, time_last))
				
				delta_time = time_now - time_last
				
				if delta_time < 0:
					rospy.logwarn("%02X last message in the future? (%f now %f)" % (i, time_last, time_now))
				if delta_time > 10:
					rospy.logwarn("%02X last message over 10 seconds ago (%f now %f)" % (i, time_last, time_now))
		
		counter_lock.release()
		
		time.sleep(5)

def callback(data):
	#rospy.loginfo("test")
	srcaddr = data.srcaddr
	interval_count = data.interval_count
	rospy.logdebug("%02X interval %d" % (srcaddr, interval_count))
	
	counter_lock.acquire()
	
	if counters[srcaddr] == -1:
		rospy.loginfo("First packet from %02X (%d)" % (srcaddr, interval_count))
	else:
		if (counters[srcaddr] + 1) == interval_count:
			rospy.logdebug("ok for %02X" % srcaddr)
		else:
			rospy.logwarn("ALERT: Interval count for %02X jumped from %d to %d" % (srcaddr, counters[srcaddr], interval_count))
	counters[srcaddr] = interval_count
	times[srcaddr] = time.time()

	counter_lock.release()

def main():
	rospy.init_node('i2c_health_check')
	rospy.Subscriber("wheel_status", wheel_status_packet, callback)
	rospy.Subscriber("linear_actuator_status", linact_position, callback)
	
	sanity_check_thread = threading.Thread(target=timer)
	sanity_check_thread.run()
	
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
