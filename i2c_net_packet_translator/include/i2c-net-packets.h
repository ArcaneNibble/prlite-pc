/* warning, make sure you typedef the correct types before including */
#ifndef _I2C_NET_PACKETS_H_
#define _I2C_NET_PACKETS_H_

typedef struct
{
	u32 interval_count;
	s16 ticks0_interval;
	s16 ticks1_interval;

	FIXED1616 debug_p;
	FIXED1616 debug_i;
	FIXED1616 debug_d;
	u16 out;
	u16 time;
} __attribute__((__packed__)) wheel_status_packet;

typedef struct
{
	FIXED1616 w0_p;
	FIXED1616 w0_i;
	FIXED1616 w0_d;
	FIXED1616 w1_p;
	FIXED1616 w1_i;
	FIXED1616 w1_d;
	u8 w0_reverse;
	u8 w1_reverse;
} __attribute__((__packed__)) pid_gains_packet;

typedef struct
{
	s16 s0;
	s16 s1;
} __attribute__((__packed__)) setpoints_packet;

typedef struct
{
	u8 missed_interval;
	u8 arbitration_dropped;
	u8 busy_dropped;
	u8 i2c_bus_error;
	u8 state_machine_fubar;
	u8 bad_checksum_packet;
} __attribute__((__packed__)) wheel_complaints_packet;

typedef struct
{
	u32 interval_count;
	u16 pos0;
	u16 pos1;
	u8 arr0;
	u8 arr1;
} __attribute__((__packed__)) linact_position;

typedef struct
{
	u16 t0_min;
	u16 t0_max;
	u16 t1_min;
	u16 t1_max;
} __attribute__((__packed__)) linact_target;

#define ARE_YOU_ALIVE_REQUEST_TYPE	0
#define WHEEL_STATUS_ANNOUNCE_TYPE	1
#define SET_SETPOINTS_REQUEST_TYPE	2
#define GET_STATS_REQUEST_TYPE		3
#define I_AM_ALIVE_RESPONSE_TYPE	4
#define SET_PID_GAINS_REQUEST_TYPE	5
#define SET_I2C_TARGET_ADDR			6
#define STATS_RESPONSE_TYPE			7
#define LINACT_POS_ANNOUNCE_TYPE	8
#define LINACT_SET_POS				9

const u8 cmd_has_reply[] = {
	1,									//ARE_YOU_ALIVE_REQUEST_TYPE
	0,									//WHEEL_STATUS_ANNOUNCE_TYPE ???
	0,									//SET_SETPOINTS_REQUEST_TYPE
	sizeof(wheel_complaints_packet),	//GET_STATS_REQUEST_TYPE
	0,									//I_AM_ALIVE_RESPONSE_TYPE ???
	0,									//SET_PID_GAINS_REQUEST_TYPE
	0,						 			//SET_I2C_TARGET_ADDR
	0,									//STATS_RESPONSE_TYPE ???
	0,									//LINACT_POS_ANNOUNCE_TYPE ???
	0,									//LINACT_SET_POS
};

const u8 cmd_reply[] = {
	I_AM_ALIVE_RESPONSE_TYPE,	//ARE_YOU_ALIVE_REQUEST_TYPE
	I_AM_ALIVE_RESPONSE_TYPE,	//WHEEL_STATUS_ANNOUNCE_TYPE ???
	I_AM_ALIVE_RESPONSE_TYPE,	//SET_SETPOINTS_REQUEST_TYPE ???
	STATS_RESPONSE_TYPE,		//GET_STATS_REQUEST_TYPE
	I_AM_ALIVE_RESPONSE_TYPE,	//I_AM_ALIVE_RESPONSE_TYPE ???
	I_AM_ALIVE_RESPONSE_TYPE,	//SET_PID_GAINS_REQUEST_TYPE
	I_AM_ALIVE_RESPONSE_TYPE, 	//SET_I2C_TARGET_ADDR
	I_AM_ALIVE_RESPONSE_TYPE,	//STATS_RESPONSE_TYPE ???
	I_AM_ALIVE_RESPONSE_TYPE,	//LINACT_POS_ANNOUNCE_TYPE ???
	I_AM_ALIVE_RESPONSE_TYPE,	//LINACT_SET_POS ???
};

const u8 cmd_has_data[] = {
	0,							//ARE_YOU_ALIVE_REQUEST_TYPE
	0,							//WHEEL_STATUS_ANNOUNCE_TYPE ???
	sizeof(setpoints_packet),	//SET_SETPOINTS_REQUEST_TYPE
	0,							//GET_STATS_REQUEST_TYPE
	0,							//I_AM_ALIVE_RESPONSE_TYPE ???
	sizeof(pid_gains_packet),	//SET_PID_GAINS_REQUEST_TYPE
	1,							//SET_I2C_TARGET_ADDR
	0,							//STATS_RESPONSE_TYPE ???
	0,							//LINACT_POS_ANNOUNCE_TYPE ???
	sizeof(linact_target),		//LINACT_SET_POS
};

#endif
