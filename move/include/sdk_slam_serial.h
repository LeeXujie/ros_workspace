#ifndef _SDK_SLAM_SERIAL_
#define _SDK_SLAM_SERIAL_
#define SEND_PACK_START_FLAG 0xaa
#define SEND_PACK_START_INDEX 0x55
#define RECV_PACK_START_FLAG 0x55
#define RECV_PACK_START_FLAG_RT 0xa1
#define RECV_PACK_START_FLAG_NRT 0xa2

//#define BUF_MAX_LEN 256
#define BUF_MAX_LEN 60

#define COMM_STAT_WAIT 2
#define COMM_STAT_ERR 1
#define COMM_STAT_OK 0
#define OBSTACLE_AVOIDANCE 30
#define EXPVALUE 2.71828
typedef struct _SDK_Send_Pack{
	unsigned char startFlag;
	unsigned char index;
	unsigned char cmd;
	//unsigned char lv;
	char lv;
	unsigned char av;
	unsigned short distance;
	unsigned short degree;
	unsigned char msgId;
	unsigned char reserved1;
	unsigned char reserved2;
	unsigned char reserved3;
	char sum;
}SDK_Send_Pack;

typedef struct _SDK_Recv_Pack{
	unsigned char startFlag;
	unsigned char index;
	unsigned char cmdresult;
	unsigned char lv;
	unsigned char av;
	unsigned char obstacle;
	unsigned char errcode;
	unsigned char motionStatus;
	unsigned char msgId;
    unsigned int distance;
	unsigned char reserved1;
	char sum;
}SDK_Recv_Pack;

#define VEL_INCRE_2_DISTANCE 0.115 //รื
#define VEL_INCRE_5_DISTANCE 0.75   //รื
#define VEL_INCRE_10_DISTANCE 1.50  //รื

#define VEL_DECRE_2_DISTANCE 0.288
#define VEL_DECRE_5_DISTANCE 0.550
#define VEL_DECRE_10_DISTANCE 0.750

/*from DSP to PC, package to receive*/
#define RT_RECV_PACK_LEN 15    /*real time recv package length*/
#define NRT_RECV_PACK_LEN 20    /*not real time recv package length*/
#define RT_AND_NRT_PKG_LEN 28
#define EPSINON     0.00001f
#define IsFloatEqualZero(f) (( ((f) >= -EPSINON)&&((f) <= EPSINON) )? true : false)
#endif
