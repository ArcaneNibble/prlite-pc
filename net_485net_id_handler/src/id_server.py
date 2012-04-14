#!/usr/bin/env python

import roslib
roslib.load_manifest('net_485net_id_handler')
import rospy
from net_485net_id_handler.srv import *
import threading
import binascii
import sys

db_lock = threading.Lock()
db_uc_list = None
db_firm_list = None

def parse_db(file):
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

def update_db():
	global db_lock
	global db_uc_list
	global db_firm_list
	uc_list, firm_list = parse_db(rospy.get_param("~uc_config_file", "uc.txt"))
	db_lock.acquire()
	db_uc_list = uc_list
	db_firm_list = firm_list
	db_lock.release()

def search_id(type, desc):
	global db_lock
	global db_uc_list
	global db_firm_list
	db_lock.acquire()	#no, we probably don't need to lock the whole function but update is rarely called
	for addr,uc in db_uc_list.iteritems():
		if uc[1] == desc:
			if type == None or type == "" or type == uc[0]:
				#matched
				db_lock.release()
				return (addr, uc)
	#failed
	db_lock.release()
	return None

def main():
	rospy.init_node('net_485net_id_server')
	
	update_db()
	
	def search_id_shim(req):
		result = search_id(req.type, req.desc)
		if result == None:
			rospy.logwarn("Couldn't find '%s' of type '%s'" % (req.desc, req.type))
			return 0xFF
		return result[0]
	search_id_service = rospy.Service('search_id', SearchID, search_id_shim)
	
	def update_db_shim(req):
		update_db()
		return ()
	update_db_service = rospy.Service('reload_db', ReloadID, update_db_shim)
	
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
