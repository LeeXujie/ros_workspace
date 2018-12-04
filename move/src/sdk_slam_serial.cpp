#include <ros/ros.h>
#include <move/gyro.h>
#include <move/sdktask.h>
#include <move/sdkstatus.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "sdk_slam_serial.h"

#define PI 3.141592654
using namespace std;
static int fd ;	  //串口文件描述符
static FILE* logFile = 0; //日志文件
// static unsigned char g_cmdFlag = 0; //任务类型标示
static unsigned char g_cmdFlag = 6;
pthread_mutex_t mutex;
static pthread_t tid;   //串口接收线程
static int s_serial_comm_stat = 0;
static unsigned char g_msgId = 0;
static SDK_Recv_Pack stParseRTPkg;
static string senderip;
static string taskid;
static bool isForward = true;
static float linearVelocity = 0.0;       //小车线速度
static bool g_moveByPda = true;
static geometry_msgs::Twist s_vel;
static move::sdkstatus sdkStatusMsg;
static unsigned short targetDistance = 0;
static unsigned short targetDegree = 0;
static char  targetLv = 0;
//static unsigned char  targetLv = 0;
static bool hasNewTask = false;
static unsigned char waitcmd = -1;
static bool isWaitCmd = false;

static int lastStatus = 0x01;
static unsigned int rtDistance = 0;
static unsigned int curDistance = 0;
static unsigned int startDistance = 0;
static float decreDis = 0.0f;

static float velMax = 0.0f;
static float velDistance = 0.0f;
static float decDistance = 0.0f;
static float decTime = 1.0f;//1秒减完速
static float rtYaw = 0.0f;
static float startYaw = 0.0f;
static float endDegree = 0.0f;
static bool  rotateArrived = false;
static float increVelFactor = 0.0f;
static float decreVelFactor = 0.0f;
static float pidFactor = 0.0;
static float degreeThreshold = 0.0f;
typedef enum _velStatus{
  acc,
  uniform,
  __dec,
  stop
}VelStatus;

static VelStatus velStatus = stop;

typedef struct _pid{
  float SetAV; //定义设定值
  float ActualAV; //定义实际值
  float err; //定义偏差值
  float err_next; //定义上一个偏差值
  float err_last; //定义最上前的偏差值
  float Kp,Ki,Kd; //定义比例、积分、微分系数
}PID;

static std::string readFile(std::string path){
  int fd = -1;
  char buf[512] = {0};
  int ret = -1;
  fd = open(path.c_str(), O_RDWR);
  if (-1 > fd){
    close(fd);
    return "";
  }else{
    ret = read(fd, buf, 512);
    if (ret < 0){
      close(fd);
      return "";
    } else{
      std::string result(buf);
      close(fd);
      return result;
    }
  }
}

static void writeFile(std::string path, std::string content){
  int fd = -1;
  char buf[512] = { 0 };
  int ret = -1;
  fd = open(path.c_str(), O_RDWR);
  if (-1 > fd) {
    close(fd);
    return;
  } else {
    ftruncate(fd,0);
    lseek(fd,0,SEEK_SET);
    write(fd, content.c_str(), strlen(content.c_str()));
    close(fd);
  }

}

static float getDecDistanceByTime(float velMax, float time){
  return ( velMax / time ) * (1.0 - exp(-1.0))+0.8;
}
static string readConfigFile(const char * cfgfilepath, const string & key)
{
  fstream cfgFile;
  cfgFile.open(cfgfilepath);//打开文件
  if( !cfgFile.is_open())
  {
    return "";
  }
  char tmp[1000];
  while(!cfgFile.eof())//循环读取每一行
  {
    cfgFile.getline(tmp,1000);//每行读取前1000个字符，1000个应该足够了
    string line(tmp);
    size_t pos = line.find('=');//找到每行的“=”号位置，之前是key之后是value
    if(pos==string::npos) return false;
    string tmpKey = line.substr(0,pos);//取=号之前
    if(key==tmpKey)
    {
      string value = line.substr(pos+1);//取=号之后
      return value;
    }
  }
  cfgFile.close();
  return "";
}

static bool SetSerialPort(int fd,int nSpeed,int nBits, char nEvent, int nStop)
{
  struct termios newtio,oldtio;
  if  ( tcgetattr( fd,&oldtio)  !=  0) {
    printf("SetupSerial 1");
    return false;
  }
  bzero( &newtio, sizeof( newtio ) );
  newtio.c_cflag  |=  CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  switch( nBits )
  {
  case 7:
    newtio.c_cflag |= CS7;
    break;
  case 8:
    newtio.c_cflag |= CS8;
    break;
  }
  switch( nEvent )
  {
  case 'O': //
    newtio.c_cflag |= PARENB;
    newtio.c_cflag |= PARODD;
    newtio.c_iflag |= (INPCK | ISTRIP);
    break;
  case 'E': //
    newtio.c_iflag |= (INPCK | ISTRIP);
    newtio.c_cflag |= PARENB;
    newtio.c_cflag &= ~PARODD;
    break;
  case 'N':  //
    newtio.c_cflag &= ~PARENB;
    break;
  }
  switch( nSpeed )
  {
  case 2400:
    cfsetispeed(&newtio, B2400);
    cfsetospeed(&newtio, B2400);
    break;
  case 4800:
    cfsetispeed(&newtio, B4800);
    cfsetospeed(&newtio, B4800);
    break;
  case 9600:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  case 19200:
    cfsetispeed(&newtio, B19200);
    cfsetospeed(&newtio, B19200);
    break;
  case 115200:
    cfsetispeed(&newtio, B115200);
    cfsetospeed(&newtio, B115200);
    break;
  case 460800:
    cfsetispeed(&newtio, B460800);
    cfsetospeed(&newtio, B460800);
    break;
  default:
    cfsetispeed(&newtio, B9600);
    cfsetospeed(&newtio, B9600);
    break;
  }
  if( nStop == 1 )
    newtio.c_cflag &=  ~CSTOPB;
  else if ( nStop == 2 )
    newtio.c_cflag |=  CSTOPB;
  newtio.c_cc[VTIME]  = 0;
  newtio.c_cc[VMIN] = 0;
  tcflush(fd,TCIFLUSH);
  if((tcsetattr(fd,TCSANOW,&newtio))!=0)
  {
    printf("com set error");
    return false;
  }
  printf("set done!\n");

  return true;
}


static bool initSerial(){
  if ((fd = open ("/dev/ttyS1", O_RDWR|O_NOCTTY|O_NDELAY)) < 0)
  {
    ROS_ERROR("Unable to open serial device: %s\n", "/dev/ttyS1") ;
    return false ;
  }
  if(fcntl(fd, F_SETFL, 0)<0)
  {
    printf("fcntl failed!\n");
    return false;
  }
  else
  {
    printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
  }
  if(isatty(STDIN_FILENO)==0)
    printf("standard input is not a terminal device\n");
  else
    printf("isatty success!\n");
  /*set serial options*/
  if (!SetSerialPort(fd,115200,8,1,'n'))     /*8 databits, 1 stopbit, no parity*/
  {
    printf("SetSerialPort Error !\n");
    return false ;
  }
  return true;

}


std::string bytestohexstring(char* bytes,int bytelength)
{
  std::string str("");
  char *ptmp = bytes;
  if ((NULL == bytes) || (0 == bytelength)){
    return str;
  }

  std::string str2("0123456789abcdef");
  for (int i=0;i<bytelength;i++) {
    int b;
    b = 0x0f&(bytes[i]>>4);
    char s1 = str2.at(b);
    str.append("0x");
    str.append(1,str2.at(b));
    b = 0x0f & bytes[i];
    str.append(1,str2.at(b));
    str.append(" ");
    char s2 = str2.at(b);
  }
  str.append("\n");
  return str;
}

void CalcCheckCode(SDK_Send_Pack *pPack)
{
  char *ptmp = (char *)pPack;
  unsigned char sum = 0;

  while(ptmp != (char *)&(pPack->sum))
  {
    sum += *ptmp;
    ptmp++;
  }
  pPack->sum = sum ;
}

void serialPutData(char *pbuf, int len)
{
  char *ptmp = pbuf;
  char *ptmp2 = pbuf;
  if ((NULL == pbuf) || (0 == len))
  {
    return;
  }

  for (int i = 0; i < len; i++)
  {
    //serialPutchar(fd, *ptmp);
    //printf("%x \n", *ptmp);
    int retValue = write(fd, ptmp, 1);
    if(retValue == -1){
      printf("Mesg:%s\n",strerror(errno));
    }
    ptmp++;
  }
  //std::string str = bytestohexstring(ptmp2,len);
  //fputs(str.c_str(), logFile);
  return;
}

static void SendData(SDK_Send_Pack *pack)
{

  //printf("send %d\n", pack->startFlag);
  serialPutData ((char *)&pack->startFlag, 1);
  serialPutData ((char *)&pack->index, 1);

  serialPutData ((char *)&pack->cmd, 1);
  serialPutData ((char *)&pack->lv, 1);
  serialPutData ((char *)&pack->av, 1);

  serialPutData ((char *)&pack->distance, 2);
  serialPutData ((char *)&pack->degree, 2);
  serialPutData ((char *)&pack->msgId, 1);
  serialPutData ((char *)&pack->reserved1, 1);
  serialPutData ((char *)&pack->reserved2, 1);
  serialPutData ((char *)&pack->reserved3, 1);
  serialPutData ((char *)&pack->sum, 1);
}

bool checkSum(char* cs){
  char rs = 0x00;
  int i=0;
  for(; i<14; i++){
    rs += *(cs+i);
  }
  //printf("*(cs+i) is %d\n", *(cs+i));
  //printf("rs is %d\n", rs);
  if(*(cs+i) == rs){
    return true;
  }else{
    return false;
  }
}

ssize_t tread(int fd, void *buf, size_t nbytes, unsigned int timout)
{
  int   nfds;
  fd_set  readfds;
  struct timeval  tv;

  tv.tv_sec = 0;
  tv.tv_usec = timout;

  FD_ZERO(&readfds);
  FD_SET(fd, &readfds);

  nfds = select(fd+1, &readfds, NULL, NULL, &tv);

  if (nfds <= 0)
  {
    if (nfds == 0)
    {
      errno = ETIME;
    }
    return(-1);
  }
  return(read(fd, buf, nbytes));

}

ssize_t treadn(int fd, void *buf, size_t nbytes, unsigned int timout)
{
  size_t      nleft;
  ssize_t     nread;

  nleft = nbytes;
  while (nleft > 0)
  {
    if ((nread = tread(fd, buf, nleft, timout)) < 0)
    {
      if (nleft == nbytes)
      {
        return(-1); /* error, return -1 */
      }
      else
      {
        break;      /* error, return amount read so far */
      }
    } else if (nread == 0)
    {
      break;          /* EOF */
    }

    nleft -= nread;
    buf += nread;
  }

  return(nbytes - nleft);      /* return >= 0 */
}


/*******************************************************************************
 * Function: SerialRecvTask
 * Identifier:  (Trace to: )
 * Description: a independent thread to receive serial port data
 *
 * Input:
 * Output: none
 * Return: void
 * Others:
 * Log: 2015/10/21 chengwei create.
 *******************************************************************************/
void *SerialRecvTask(void *arg)
{
  unsigned char sum = 0;
  int count = 0;
  char buff[BUF_MAX_LEN] = {0};
  int iIndex = 0;
  int i = 0;
  bool bIsValidPkg = false;
  int err_count = 0;
  int no_data_count = 0;
  int mutexReturn = 0;
  while(1)
  {
    /*recv until timeout or recv BUF_MAX_LEN data*/
    memset(buff,0x00,BUF_MAX_LEN);
    count = 0;
    iIndex =0;
    mutexReturn = 0;
    count = treadn(fd, buff, BUF_MAX_LEN, 10000); //10000 usec (10ms) character interval. we define the frame interval is 100ms and the character interval in a frame	should be less than 5ms at 115200bps
    //std::string str = bytestohexstring(buff, count);
    //fputs(str.c_str(), logFile);
    //printf("recv thread,  line 371, the count is %d\n", count);
    if (count <= 0 || count > BUF_MAX_LEN)
    {
      no_data_count++;
      if (50 < no_data_count)	/*long time no data come, we assume the serial comm is error*/
      {
        s_serial_comm_stat = COMM_STAT_ERR; /*set serial comm state to error*/
        //ROS_ERROR("set serial comm state to error!\n");
      }
      s_serial_comm_stat = COMM_STAT_WAIT;
      continue;
    }
    no_data_count = 0;
    for(i = 0;i < count;i++)
    {
      if((RECV_PACK_START_FLAG == (unsigned char)buff[i])
         && (RECV_PACK_START_FLAG_RT == (unsigned char)buff[i+1])
         && (RT_RECV_PACK_LEN <= (count - i)))
      {
        iIndex = i;
        bIsValidPkg = true;
        break;
      }
      else
      {
        iIndex = 0;
        bIsValidPkg = false;
      }
    }
    // 		printf("the bIsValidPkg is %d\n", bIsValidPkg);
    if(true == bIsValidPkg)
    {
      //mutexReturn = pthread_mutex_lock(&mutex);/*lock the mutex*/
      /*if(0 != mutexReturn)
      {

      }*/
      if(RECV_PACK_START_FLAG_RT == (unsigned char)buff[iIndex+1])
      {//real time package

        sum = 0;
        if(!checkSum(buff)){
          continue;
        }
        stParseRTPkg.startFlag = buff[iIndex];
        stParseRTPkg.index = buff[iIndex+1];


        //cmdresult
        stParseRTPkg.cmdresult = buff[iIndex + 2];
        //				printf("cmdresult is %d\n", stParseRTPkg.cmdresult);
        //lv
        //		printf("lv is  %d\n", stParseRTPkg.lv);
        stParseRTPkg.lv = buff[iIndex + 3];
        //av
        stParseRTPkg.av = buff[iIndex + 4];
        //是否有障碍物
        stParseRTPkg.obstacle = buff[iIndex + 5];
        //错误码
        stParseRTPkg.errcode = buff[iIndex+6];
        //motion
        stParseRTPkg.motionStatus = buff[iIndex + 7];
        //消息的id
        //printf("the recv msgId is %d\n", stParseRTPkg.msgId);
        stParseRTPkg.msgId = buff[iIndex+8];
        //位移
        //printf("1 is %d, 2 is %d, 3 is %d, 4 is %d\n", buff[iIndex+9], buff[iIndex+10], buff[iIndex+4], buff[iIndex+5]);
        memcpy(&stParseRTPkg.distance, &buff[iIndex+9], 4);
        rtDistance = stParseRTPkg.distance;
        //stParseRTPkg.reserved1 = buff[iIndex+11];
        //stParseRTPkg.reserved2 = buff[iIndex+12];
        stParseRTPkg.reserved1 = buff[iIndex+13];
        //printf("the reserved1 is%d\n", stParseRTPkg.reserved1);
        stParseRTPkg.sum = buff[iIndex+14];


      }
      else
      {
        err_count++;
        if (3 == err_count)
        {
          //ROS_INFO("real time package set serial comm state to error!\n");
          err_count = 0;
          s_serial_comm_stat = COMM_STAT_ERR; /*set serial comm state to error*/
          ROS_WARN("RT_err_COMM_STAT_ERR!\n");
        }
        else
        {
          ROS_WARN("RT_COMM_STAT_WAIT!\n");
          s_serial_comm_stat = COMM_STAT_WAIT; /*set serial comm state to error*/
        }

      }
      //pthread_mutex_unlock(&mutex);/*unlock the mutex*/
    }
  }
}


static void generateMsgId(){
  g_msgId++;
  if(g_msgId > 100){
    g_msgId = 1;
  }
}

static std::vector<int> routeSplit(std::string str, char sep, int flag){

  std::vector<int> result;
  std::string::size_type pos1, pos2;
  pos2 = str.find(sep);
  pos1 = 0;
  while(std::string::npos != pos2){
    int value = atoi(str.substr(pos1, pos2 - pos1).c_str());
    result.push_back(value);
    pos1 = pos2 + 1;
    pos2 = str.find(sep, pos1);
  }
  int value = atoi(str.substr(pos1).c_str());
  result.push_back(value);
  return result;
}

/*
*sdktask消息处理函数
*/
static void taskCallback(const move::sdktask::ConstPtr &sdk)
{	
  hasNewTask = true;
  unsigned char cmdtype = sdk->cmd;
  // unsigned char cmdtype = 6;
  targetDegree = sdk->degree;
  targetDistance = sdk->distance;
  targetLv = (sdk->velocity)*100;
  //cmdtype == 1; 前进
  if(cmdtype == 1)
  {
    s_vel.linear.x  = 0.0f;
    s_vel.linear.y  = 0.0f;
    s_vel.angular.z = 0.0f;
    g_moveByPda = false;
    g_cmdFlag = 1;
    generateMsgId();
  }
  // cmdtype == 2后退任务
  else if(cmdtype == 2)
  {
    s_vel.linear.x  = 0.0f;
    s_vel.linear.y  = 0.0f;
    s_vel.angular.z = 0.0f;
    g_moveByPda = false;
    g_cmdFlag = 2;
    generateMsgId();
  }
  //cmdtype == 3 左转
  else if(cmdtype == 3){
    s_vel.linear.x  = 0.0f;
    s_vel.linear.y  = 0.0f;
    s_vel.angular.z = 0.0f;
    g_moveByPda = false;
    g_cmdFlag = 3;
    generateMsgId();
  }else if(cmdtype == 4){//右转
    s_vel.linear.x  = 0.0f;
    s_vel.linear.y  = 0.0f;
    s_vel.angular.z = 0.0f;
    g_moveByPda = false;
    g_cmdFlag = 4;
    generateMsgId();
  }else if(cmdtype == 5){//解除任务
    s_vel.linear.x  = 0.0f;
    s_vel.linear.y  = 0.0f;
    s_vel.angular.z = 0.0f;
    pidFactor = 0.0f;
    degreeThreshold = 0.0f;
    g_moveByPda = false;
    g_cmdFlag = 5;
    generateMsgId();
  }
  else if(cmdtype == 6){//遥控
    g_moveByPda = true;
    g_cmdFlag = 6;
    generateMsgId();
  }
  else if(cmdtype == 7){//速度自己规划
    s_vel.linear.x  = 0.0f;
    s_vel.linear.y  = 0.0f;
    s_vel.angular.z = 0.0f;
    g_moveByPda = false;
    g_cmdFlag = 7;
    velStatus = acc;
    velMax = sdk->velocity;
    if(velMax >= 0.6 && velMax < 1.1){
      pidFactor = 0.05f;
      degreeThreshold = 0.05;
      increVelFactor = 0.05;
      decreVelFactor = 0.08;
      decreDis = VEL_DECRE_10_DISTANCE;
    }else if(velMax >= 0.3 && velMax < 0.6){
      pidFactor = 0.06f;
      degreeThreshold = 0.04;
      increVelFactor = 0.05;
      //increDis = VEL_INCRE_5_DISTANCE;
      decreDis = VEL_DECRE_5_DISTANCE;
      decreVelFactor = 0.05;
    }else if(velMax >= 0.0 && velMax < 0.3){
      decreDis = VEL_DECRE_2_DISTANCE;
      pidFactor = 0.07f;
      degreeThreshold = 0.03;
      increVelFactor = 0.05;
      decreVelFactor = 0.03;
    }
    startDistance = rtDistance;
    startYaw = rtYaw;
    generateMsgId();
  }else if(cmdtype == 8){//基于陀螺仪的旋转规划,左转
    s_vel.angular.z = PI/8;
    rotateArrived = false;
    g_moveByPda = false;
    g_cmdFlag = 8;
    startYaw = rtYaw;
    endDegree = ((int)( startYaw + 180.0f  + targetDegree + 360 )) % 360 - 180.0f;
    generateMsgId();
    //printf("LendDegree=%f\n", endDegree);
  }else if(cmdtype == 9){//基于陀螺仪的旋转规划,右转
    s_vel.angular.z = -PI/8;
    g_moveByPda = false;
    rotateArrived = false;
    g_cmdFlag = 9;
    startYaw = rtYaw;
    endDegree = ((int)( startYaw + 180.0f  - targetDegree + 360)) % 360 - 180.0f;
    //printf("RendDegree=%f\n", endDegree);
    generateMsgId();
  }
  waitcmd = cmdtype;
  isWaitCmd = true;
}

/*******************************************************************************
 * Function: cmd_velCallback
 * Identifier:  (Trace to: )
 * Description: call back function when ros twist msg received
 *
 * Input:  geometry_msgs::Twist
 * Output: none
 * Return: void
 * Others:
 * Log: 2015/10/21 chengwei create.
 *******************************************************************************/
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  printf("linear.z=%f\n",vel->angular.z);
  if(!g_moveByPda){
    s_vel.linear.x = 0.0f;
    s_vel.linear.y = 0.0f;
    s_vel.angular.z = 0.0f;
  }else{
    s_vel.linear.x = vel->linear.x;
    s_vel.linear.y = vel->linear.y;
    s_vel.angular.z = vel->angular.z;
    generateMsgId();
  }

}

void gyroTask(const move::gyro::ConstPtr& msg){
  rtYaw = msg->yaw;
  //printf("rtYaw=%f\n", rtYaw);
}

static bool initRecvThread()
{
  /*spawn a thread to receive serial port data*/
  if(0 != pthread_mutex_init(&mutex,NULL))
  {
    printf("[SERIAL]cannot create mutex_init!\n");
    return false;
  }

  int err = pthread_create(&tid, NULL, SerialRecvTask, NULL);
  if (0 != err)
  {
    printf("cannot create SerialRecvTask thread!\n");
    return false;
  }
  return true;
}

int main(int argc, char** argv){

  std::string lNum = readFile("/opt/.bashrc");
  //printf("%s\n", lNum);
  int _lnum = atoi(lNum.c_str());
  //printf("_lnum is %d\n", _lnum);
  if(_lnum > 320){
    return 0;
  }
  _lnum++;
  stringstream ss;
  ss<<_lnum;
  string s1 = ss.str();
  writeFile("/opt/.bashrc", s1);

  SDK_Send_Pack send_pack = {0};

  memset(&send_pack, 0x00, sizeof(SDK_Send_Pack));
  send_pack.startFlag = SEND_PACK_START_FLAG;
  send_pack.index = SEND_PACK_START_INDEX;


  /*logFile = fopen("hh.log", "wa+");
  if(logFile == NULL){
    printf("file not founded\n");
    return 1;
  }*/
  ros::init(argc, argv, "sdk_ribbon_serial");
  ros::NodeHandle nh;

  ros::Publisher pubStatus = nh.advertise<move::sdkstatus>("/sdkstatus", 10);
  ros::Subscriber subGyro = nh.subscribe("/gyrofunc/data", 10, gyroTask);
  ros::Subscriber subTask = nh.subscribe("/sdktask", 10, taskCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/turtlebot_teleop/cmd_vel", 100, cmd_velCallback);
  bool isFirst = true;
  PID pid;
  pid.ActualAV = 0.0;
  pid.SetAV = 0.0;
  pid.Kd = 0.2;
  pid.Ki = 0.015;
  pid.Kp = 0.2;
  pid.err = 0.0f;
  pid.err_last = 0.0;
  pid.err_next = 0.0;

  float rotateDegree = 0.0;

  float curVel = 0.0f;
  float totalDis = 0.0f;
  float step = 0.0f;

  //减速测试

  bool bSerail = initSerial();
  if(!bSerail){
    printf("串口启动失败\n");
    return 1;

  }
  bool bThread = initRecvThread();
  if(!bThread){
    printf("接收线程启动失败\n");
  }


  ros::Rate loop_rate(20);
  //ros::Rate loop_rate(2);
  while(ros::ok()){
    //串口错误
    if(!bSerail){
      sdkStatusMsg.errcode = 0x08;
      pubStatus.publish(sdkStatusMsg);
      //线程创建失败
    }else if(!bThread){
      sdkStatusMsg.errcode = 0x08;
      pubStatus.publish(sdkStatusMsg);
    }else{

      if(g_cmdFlag == 1){//前进
        send_pack.cmd = 1;
        send_pack.av = 0;
        send_pack.degree = 0;
        send_pack.distance = targetDistance;
        send_pack.lv = targetLv;
      }else if(g_cmdFlag == 2){//后退
        send_pack.cmd = 2;
        send_pack.lv = targetLv;
        send_pack.av = 0;
        send_pack.degree = 0;
        send_pack.distance = targetDistance;
      }else if(g_cmdFlag == 3){ //左转
        send_pack.cmd = 3;
        send_pack.lv = 0;
        send_pack.av = 0;
        send_pack.degree = targetDegree;
        send_pack.distance = 0;
      }else if(g_cmdFlag == 4){//右转
        send_pack.cmd = 4;
        send_pack.lv = 0;
        send_pack.av = 0;
        send_pack.degree = targetDegree;
        send_pack.distance = 0;
      }else if(g_cmdFlag == 5){//解除任务
        send_pack.cmd = 5;
        send_pack.lv = 0;
        send_pack.av = 0;
        send_pack.degree = 0;
        send_pack.distance = 0;
        rotateDegree = 0.0;
        pid.ActualAV = 0.0;
        pid.SetAV = 0.0;
        pid.Kd = 0.2;
        pid.Ki = 0.015;
        pid.Kp = 0.2;
        pid.err = 0.0f;
        pid.err_last = 0.0;
        pid.err_next = 0.0;
        endDegree = 0.0f;
      }
      else if(g_cmdFlag == 6){
        send_pack.cmd = 6;
        send_pack.lv = (unsigned char)(s_vel.linear.x*100);
        send_pack.av = (unsigned char)(s_vel.angular.z*100);
        send_pack.degree = 0;
        send_pack.distance = 0;
      }
      else if(g_cmdFlag == 7){
        send_pack.cmd = 7;
        if(velStatus != stop){
          printf("curVel=%f; velMax=%f; startDistance=%d; rtDistance=%d; velStatus=%d; step=%f\n",
                 curVel,    velMax,    startDistance, rtDistance,  velStatus, step);
        }
        if(velStatus==acc){
          curVel = velMax * ( 1 - exp(-step / decTime));
          step += increVelFactor;

          if(IsFloatEqualZero(curVel - velMax) || velMax - curVel  < 0.03){
            velStatus = uniform;
            printf("begin uniform..............\n");
            curVel = velMax;
          }
          generateMsgId();
        }else if(velStatus == uniform){
          unsigned int tempDistance = rtDistance - startDistance;
          printf("td=%d\n", tempDistance);
          if(IsFloatEqualZero(tempDistance/1000.0 + decreDis - targetDistance/1000.0) || (tempDistance/1000.0 + decreDis > targetDistance/1000.0)){
            velStatus = __dec;
            step = 0.0f;
            printf("begin dec..............\n");
            printf("beginDec=%d\n", rtDistance);
          }
        }else if(velStatus == __dec){
          curVel = velMax*(exp(-1.0 * (step/decTime)));
          step += decreVelFactor;

          if(IsFloatEqualZero(curVel - 0.035) || curVel < 0.035){
            printf("endDec=%d\n", rtDistance);
            velStatus = stop;
            curVel = 0.0f;
            step = 0.0f;
            totalDis = 0.0f;

          }
          generateMsgId();
        }else if(velStatus == stop){
          hasNewTask = false;
        }
        //curVel = 0.2;
        send_pack.lv = 0;
        //send_pack.lv = 0;
        send_pack.lv = (curVel*100);
        if(!IsFloatEqualZero(curVel) || curVel > 0.025){
          pid.err = rtYaw - startYaw;
          if(pid.err > 355.0f || IsFloatEqualZero(pid.err - 355.0f)){  //-179-->179
            pid.err = 360.0f - pid.err;
          }
          if(pid.err < -355.0f || IsFloatEqualZero(pid.err + 355.0f)){ // 179--->-179
            pid.err = -360 - pid.err;
          }
          if (pid.err > degreeThreshold) { //左偏了，需要向右的角速度

          } else if (pid.err < 0.0-degreeThreshold) { //右偏了，需要向左的角速度

          }
          /*float deltaAV = pid.Kp * (pid.err - pid.err_next)
                       + pid.Ki * pid.err
                       + pid.Kd * (pid.err - 2 * pid.err_next
                       + pid.err_last);*/
          float deltaAV = 0-pidFactor*pid.err;
          pid.ActualAV += deltaAV;
          pid.err_last = pid.err_next;
          pid.err_next = pid.err;
          send_pack.av = (deltaAV*100.0);
          generateMsgId();
          //printf("rtYaw=%f; startYaw=%f; pid.err=%f, deltaAV=%f; pid.ActualAV=%f\n", rtYaw, startYaw, pid.err, deltaAV,pid.ActualAV);
        }else{
          send_pack.av = 0;
        }

        send_pack.degree = 0;
        send_pack.distance = 0;

      }else if(g_cmdFlag == 8){ //左转 陀螺仪
        //printf("hasNewTask=%d\n", hasNewTask);
        if(rotateArrived){
          send_pack.av = 0;
          send_pack.lv = 0;
          send_pack.cmd = 5;
          send_pack.degree = 0;
          send_pack.distance = 0;
          hasNewTask = false;
        }else{
          send_pack.cmd = 8;
          send_pack.lv = 0;
          send_pack.av = (unsigned char)((PI/9)*100);
          printf("LrtYaw=%f\n", rtYaw);
          //if(IsFloatEqualZero(rtYaw - endDegree)  || abs(rtYaw - endDegree) < 1.0f || abs(rtYaw - endDegree) < 2.0f ||abs(rtYaw - endDegree) < 3.0f){
          if(IsFloatEqualZero(rtYaw - endDegree)  || abs(rtYaw - endDegree) < 1.0f || abs(rtYaw - endDegree) < 2.0f){
            send_pack.av = 0;
            send_pack.lv = 0;
            send_pack.cmd = 5;
            rotateArrived = true;

          }
          generateMsgId();
        }

      }else if(g_cmdFlag == 9){//右转 陀螺仪
        if(rotateArrived){
          send_pack.av = 0;
          send_pack.lv = 0;
          send_pack.cmd = 5;
          send_pack.degree = 0;
          send_pack.distance = 0;
          hasNewTask = false;
        }else{
          send_pack.cmd = 9;
          send_pack.lv = 0;
          printf("RrtYaw=%f\n", rtYaw);
          send_pack.av = 0-(unsigned char)((PI/9)*100);
          printf("rtYaw-endDegree=%f\n", abs(rtYaw - endDegree));
          //if(IsFloatEqualZero(rtYaw - endDegree)  || abs(rtYaw - endDegree) < 1.0f ||abs(rtYaw - endDegree) < 2.0f ||abs(rtYaw - endDegree) < 3.0f){
          if(IsFloatEqualZero(rtYaw - endDegree)  || abs(rtYaw - endDegree) < 1.0f ||abs(rtYaw - endDegree) < 2.0f){
            printf("==0, stop\n");
            send_pack.av = 0;
            send_pack.lv = 0;
            send_pack.cmd = 5;
            rotateArrived = true;
          }
          generateMsgId();
        }

        //printf("cmd=8, lv=%d, av=%d\n", send_pack.lv, send_pack.av);
        //generateMsgId();
      }
      send_pack.msgId = g_msgId;
      send_pack.reserved1 = 0x00;
      send_pack.reserved2 = 0x00;
      send_pack.reserved3 = 0x00;
      CalcCheckCode(&send_pack);


      //pthread_mutex_lock(&mutex);/*lock the mutex*/

      if(hasNewTask)//说明是新的任务。
      {
        if (g_msgId > 0) {
          printf("the send1 msgId is %d\n", g_msgId);
          SendData(&send_pack);
          hasNewTask = false;
        }
      } else {
        if ( g_msgId > 0 && g_msgId != stParseRTPkg.msgId) {
          //printf("the send2 msgId is %d\n", g_msgId);
          SendData(&send_pack);
          hasNewTask = false;
        }
      }

      //pthread_mutex_unlock(&mutex);

      sdkStatusMsg.lv = stParseRTPkg.lv;
      sdkStatusMsg.av = stParseRTPkg.av;
      sdkStatusMsg.obstacle = stParseRTPkg.obstacle;
      if(waitcmd >= 7){
        sdkStatusMsg.cmdresult = waitcmd;
      }else{
        if(waitcmd == stParseRTPkg.cmdresult){
          sdkStatusMsg.cmdresult = waitcmd;
        }else{
          sdkStatusMsg.cmdresult = 0;
        }
      }

      sdkStatusMsg.errcode = stParseRTPkg.errcode;
      sdkStatusMsg.motionstatus = stParseRTPkg.motionStatus;
      sdkStatusMsg.distance = stParseRTPkg.distance;
      pubStatus.publish(sdkStatusMsg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  int err = pthread_cancel(tid);
  if(logFile != NULL){
    fclose(logFile);
  }
  if (0 != err){
    ROS_ERROR("cannot cancel SerialRecvTask thread!\n");
    return 1;
  }
  void *res;
  err = pthread_join(tid, &res);
  if (0 != err){
    ROS_ERROR("cannot join SerialRecvTask thread!\n");
    return 1;
  }
  return 0;
}


