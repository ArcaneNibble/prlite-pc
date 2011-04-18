#include "ros/ros.h"
#include "i2c_net_packets/wheel_status_packet.h"
#include "i2c_net_packets/i2c_ping.h"
#include "i2c_net_packets/wheel_pid_gains.h"
#include "i2c_net_packets/wheel_setpoints.h"
#include "i2c_net_packets/linact_position.h"
#include "i2c_net_packets/linact_target.h"
#include "i2c_net_utils/i2c_packet.h"
#include "i2c_net_utils/i2c_sync_request.h"

typedef uint32_t u32;
typedef int16_t s16;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int32_t s32;
typedef s32 FIXED1616;

#include "i2c-net-packets.h"

ros::ServiceClient i2c_sync;
ros::Publisher wheel_pub;
ros::Publisher linact_pub;

#define PC_ADDR	0xFE
#define RETRY_TIMES	5

bool check_csum(uint8_t *buf, int len)
{
	int i;
	uint8_t sum = 0;
	for(i=0;i<len-1;i++)
		sum += buf[i];
	sum = sum & 0xFF;
	return sum == buf[len-1];
}

void add_csum(uint8_t *buf, int len)
{
	int i;
	uint8_t sum = 0;
	for(i=1;i<len;i++)
		sum += buf[i];
	sum = sum & 0xFF;
	buf[len] = sum;
}

char *dump_packet(uint8_t *buf, int len)
{
	char *text = new char[len*2+1];
	const char *hexlut = "0123456789ABCDEF";
	int i;
	
	for(i=0;i<len;i++)
	{
		text[i*2+0] = hexlut[(buf[i] >> 4) & 0xF];
		text[i*2+1] = hexlut[(buf[i] >> 0) & 0xF];
	}
	text[i*2] = 0;
	
	return text;
}

bool i2c_ping(i2c_net_packets::i2c_ping::Request& req, i2c_net_packets::i2c_ping::Response& resp)
{
	i2c_net_utils::i2c_sync_request srv;
	uint8_t buf[4];
	
	buf[0] = req.addr;
	buf[1] = PC_ADDR;	//src addr
	buf[2] = ARE_YOU_ALIVE_REQUEST_TYPE;
	
	add_csum(buf, sizeof(buf) - 1);
	
	char *test = dump_packet(buf, sizeof(buf));
	ROS_DEBUG("ping: %s", test);
	delete test;
	
	srv.request.req.assign(buf, buf + sizeof(buf));
	srv.request.resplen = 4;
	if(!i2c_sync.call(srv))
	{
		ROS_WARN("Error requesting synchronous i2c access!");
		return false;
	}
	if(srv.response.resp.size() == 0)
	{
		resp.present = false;
		resp.responding = false;
		resp.reply = 0;
		return true;
	}
	if(!srv.response.resp[0])
	{
		ROS_WARN("Got a NACK pinging %02X!", req.addr);
		resp.present = true;
		resp.responding = false;
		resp.reply = 0;
		return true;
	}
	if(srv.response.resp.size() != srv.request.resplen)
	{
		ROS_WARN("Missing bytes pinging %02X!", req.addr);
		resp.present = true;
		resp.responding = false;
		resp.reply = 0;
		return true;
	}
	uint8_t ack = srv.response.resp[0];
	uint8_t type = srv.response.resp[1];
	uint8_t thing = srv.response.resp[2];
	uint8_t sum = srv.response.resp[3];
	if(sum != ((ack+type+thing) & 0xFF))
	{
		ROS_WARN("Bad checksum pinging %02X!", req.addr);
		resp.present = true;
		resp.responding = false;
		resp.reply = 0;
		return true;
	}
	if(type != I_AM_ALIVE_RESPONSE_TYPE)
	{
		ROS_WARN("WTF?! Got unexpected response type %02X pinging %02X!", type, req.addr);
		resp.present = true;
		resp.responding = false;
		resp.reply = 0;
		return true;
	}
	resp.present = true;
	resp.responding = true;
	resp.reply = thing;
	return true;
}

void i2c_callback(const i2c_net_utils::i2c_packet& msg)
{
	uint8_t *contents;
	i2c_net_packets::wheel_status_packet wheel_pkt;
	i2c_net_packets::linact_position linact_pkt;
	wheel_status_packet *wheel_pkt_data;
	linact_position *linact_pkt_data;
	
	int payload_size = msg.data.size() - 3;	//from, type, csum
	
	contents = new uint8_t[msg.data.size()];
	copy(msg.data.begin(), msg.data.end(), contents);
	
	if(msg.data.size() < 3)
	{
		char *bad = dump_packet(contents, msg.data.size());
		ROS_WARN("Received impossibly small packet: %s", bad);
		delete bad;
		delete contents;
		return;
	}
	
	if(!check_csum(contents, msg.data.size()))
	{
		char *bad = dump_packet(contents, msg.data.size());
		ROS_WARN("Received packet with bad checksum: %s", bad);
		delete bad;
		delete contents;
		return;
	}
	
	switch(contents[1])
	{
	case WHEEL_STATUS_ANNOUNCE_TYPE:
		if(payload_size != sizeof(wheel_status_packet))
		{
			char *bad = dump_packet(contents, msg.data.size());
			ROS_WARN("Recieved packet with correct checksum with wrong # of bytes: %s", bad);
			delete bad;
			delete contents;
			return;
		}
		wheel_pkt_data = (wheel_status_packet*)(&(contents[2]));
		wheel_pkt.srcaddr = contents[0];
		wheel_pkt.interval_count = wheel_pkt_data->interval_count;
		wheel_pkt.ticks0_interval = wheel_pkt_data->ticks0_interval;
		wheel_pkt.ticks1_interval = wheel_pkt_data->ticks1_interval;
		wheel_pkt.debug_p = ((float)(wheel_pkt_data->debug_p)) / 65536.0f;
		wheel_pkt.debug_i = ((float)(wheel_pkt_data->debug_i)) / 65536.0f;
		wheel_pkt.debug_d = ((float)(wheel_pkt_data->debug_d)) / 65536.0f;
		wheel_pkt.out = wheel_pkt_data->out;
		wheel_pkt.time = wheel_pkt_data->time;
		wheel_pub.publish(wheel_pkt);
		break;
		
	case LINACT_POS_ANNOUNCE_TYPE:
		if(payload_size != sizeof(linact_position))
		{
			char *bad = dump_packet(contents, msg.data.size());
			ROS_WARN("Recieved packet with correct checksum with wrong # of bytes: %s", bad);
			delete bad;
			delete contents;
			return;
		}
		linact_pkt_data = (linact_position*)(&(contents[2]));
		linact_pkt.srcaddr = contents[0];
		linact_pkt.interval_count = linact_pkt_data->interval_count;
		linact_pkt.pos0 = linact_pkt_data->pos0;
		linact_pkt.pos1 = linact_pkt_data->pos1;
		linact_pkt.arr0 = linact_pkt_data->arr0;
		linact_pkt.arr1 = linact_pkt_data->arr1;
		linact_pub.publish(linact_pkt);
		break;
	
	default:
		ROS_INFO("Unimplemeted message type %d", contents[1]);
		break;
	}
	
	delete contents;
}

void wheel_pid_callback(const i2c_net_packets::wheel_pid_gains& msg)
{
	uint8_t buf[sizeof(pid_gains_packet)+3+1]; //dst, src, type, csum
	pid_gains_packet *pg = (pid_gains_packet*)(buf+3);
	
	buf[0] = msg.dstaddr;
	buf[1] = PC_ADDR;
	buf[2] = SET_PID_GAINS_REQUEST_TYPE;
	
	pg->w0_p = (FIXED1616)(msg.p0 * 65536.0f);
	pg->w0_i = (FIXED1616)(msg.i0 * 65536.0f);
	pg->w0_d = (FIXED1616)(msg.d0 * 65536.0f);
	pg->w1_p = (FIXED1616)(msg.p1 * 65536.0f);
	pg->w1_i = (FIXED1616)(msg.i1 * 65536.0f);
	pg->w1_d = (FIXED1616)(msg.d1 * 65536.0f);
	pg->w0_reverse = !!msg.rev0;	//!! should force into 0 or 1
	pg->w1_reverse = !!msg.rev1;
	
	add_csum(buf, sizeof(buf) - 1);
	
	char *test = dump_packet(buf, sizeof(buf));
	ROS_DEBUG("%s", test);
	delete test;
	
	int retries = 0;
	
	do
	{
		retries++;
		
		i2c_net_utils::i2c_sync_request srv;
		srv.request.req.assign(buf, buf + sizeof(buf));
		srv.request.resplen = 1;
		if(!i2c_sync.call(srv))
		{
			ROS_WARN("Error requesting synchronous i2c access (try %d of %d)!", retries, RETRY_TIMES);
		}
		
		else if(srv.response.resp.size() == 0)
		{
			ROS_WARN("Wheel %02X ack totally disappeared for pid gains (try %d of %d)!", msg.dstaddr, retries, RETRY_TIMES);
		}
		else if(!srv.response.resp[0])
		{
			ROS_WARN("Wheel %02X NACK'd pid gains (try %d of %d)!", msg.dstaddr, retries, RETRY_TIMES);
		}
		else
		{
			return;
		}
	}
	while(retries < RETRY_TIMES);
}

void wheel_setpoints_callback(const i2c_net_packets::wheel_setpoints& msg)
{
	uint8_t buf[sizeof(setpoints_packet)+3+1]; //dst, src, type, csum
	setpoints_packet *pg = (setpoints_packet*)(buf+3);
	
	buf[0] = msg.dstaddr;
	buf[1] = PC_ADDR;
	buf[2] = SET_SETPOINTS_REQUEST_TYPE;
	
	pg->s0 = msg.s0;
	pg->s1 = msg.s1;
	
	add_csum(buf, sizeof(buf) - 1);
	
	char *test = dump_packet(buf, sizeof(buf));
	ROS_DEBUG("%s", test);
	delete test;
	
	int retries = 0;
	
	do
	{
		retries++;
		
		i2c_net_utils::i2c_sync_request srv;
		srv.request.req.assign(buf, buf + sizeof(buf));
		srv.request.resplen = 1;
		if(!i2c_sync.call(srv))
		{
			ROS_WARN("Error requesting synchronous i2c access (try %d of %d)!", retries, RETRY_TIMES);
		}
		
		else if(srv.response.resp.size() == 0)
		{
			ROS_WARN("Wheel %02X ack totally disappeared for setpoints (try %d of %d)!", msg.dstaddr, retries, RETRY_TIMES);
		}
		else if(!srv.response.resp[0])
		{
			ROS_WARN("Wheel %02X NACK'd setpoints (try %d of %d)!", msg.dstaddr, retries, RETRY_TIMES);
		}
		else
		{
			return;
		}
	}
	while(retries < RETRY_TIMES);
}

typedef struct
{
	uint16_t t0_min;
	uint16_t t0_max;
	uint16_t t1_min;
	uint16_t t1_max;
} linact_targets_workaround;

void linact_target_callback(const i2c_net_packets::linact_target& msg)
{
	uint8_t buf[sizeof(linact_target)+3+1]; //dst, src, type, csum
	linact_target *pg = (linact_target*)(buf+3);
	
	static linact_targets_workaround lulz[256];
	static bool init_lulz = false;
	
	if(!init_lulz)
	{
		init_lulz = true;
		for(int i = 0;i < 256; i++)
		{
			lulz[i].t0_min = 0;
			lulz[i].t1_min = 0;
			lulz[i].t0_max = 0x3FF;
			lulz[i].t1_max = 0x3FF;
		}
	}
	
	if(msg.which != 0 && msg.which != 1)
	{
		ROS_WARN("Some genius didn't read the comments");
		return;
	}
	
	if(msg.which == 0)
	{
		lulz[msg.dstaddr].t0_min = msg.min;
		lulz[msg.dstaddr].t0_max = msg.max;
	}
	else
	{
		lulz[msg.dstaddr].t1_min = msg.min;
		lulz[msg.dstaddr].t1_max = msg.max;
	}
	
	buf[0] = msg.dstaddr;
	buf[1] = PC_ADDR;
	buf[2] = LINACT_SET_POS;
	
	pg->t0_min = lulz[msg.dstaddr].t0_min;
	pg->t0_max = lulz[msg.dstaddr].t0_max;
	pg->t1_min = lulz[msg.dstaddr].t1_min;
	pg->t1_max = lulz[msg.dstaddr].t1_max;
	
	add_csum(buf, sizeof(buf) - 1);
	
	char *test = dump_packet(buf, sizeof(buf));
	ROS_DEBUG("%s", test);
	delete test;
	
	int retries = 0;
	
	do
	{
		retries++;
		
		i2c_net_utils::i2c_sync_request srv;
		srv.request.req.assign(buf, buf + sizeof(buf));
		srv.request.resplen = 1;
		if(!i2c_sync.call(srv))
		{
			ROS_WARN("Error requesting synchronous i2c access (try %d of %d)!", retries, RETRY_TIMES);
		}
		
		else if(srv.response.resp.size() == 0)
		{
			ROS_WARN("Linear actuator %02X ack totally disappeared for setpoints (try %d of %d)!", msg.dstaddr, retries, RETRY_TIMES);
		}
		else if(!srv.response.resp[0])
		{
			ROS_WARN("Linear actuator %02X NACK'd setpoints (try %d of %d)!", msg.dstaddr, retries, RETRY_TIMES);
		}
		else
		{
			return;
		}
	}
	while(retries < RETRY_TIMES);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "i2c_net_packet_translator");
	ros::NodeHandle n;
	
	wheel_pub = n.advertise<i2c_net_packets::wheel_status_packet>("wheel_status", 1000);
	linact_pub = n.advertise<i2c_net_packets::linact_position>("linear_actuator_status", 1000);
	ros::ServiceServer i2c_ping_service = n.advertiseService("i2c_net_ping", i2c_ping);
	ros::Subscriber i2c_sub = n.subscribe("i2c_out", 1000, i2c_callback);
	ros::Subscriber wheel_pid_sub = n.subscribe("wheel_pid", 1000, wheel_pid_callback);
	ros::Subscriber wheel_sp_sub = n.subscribe("wheel_setpoints", 1000, wheel_setpoints_callback);
	ros::Subscriber linact_target_sub = n.subscribe("linear_actuator_target", 1000, linact_target_callback);
	i2c_sync = n.serviceClient<i2c_net_utils::i2c_sync_request>("i2c_in_synch");
	
	ros::spin();
	
	return 0;
}
