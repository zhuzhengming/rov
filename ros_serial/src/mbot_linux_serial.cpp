//created by zzm
//针对my_rov的串口通信协议

#include "mbot_linux_serial.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};//定义消息头
const unsigned char header[2] = {0x55, 0xaa};



/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()//串口初始化
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/********************************************************
发送协议：数据头55aa + 数据字节数size + 6位数据 +校验crc8 +数据尾0d0a
55 aa size 00 00 00 00 00 00 crc8 0d 0a
函数功能：将对ROV的不同推进器的PWM波，打包发送给下位机。目前推进器的数量为6,PWM波的范围-400～400,当前按比例缩小4倍
入口参数：ROV6个推进器的PWM波
出口参数：无
********************************************************/
void writeSpeed(uint8_t PWM[])
{
    unsigned char buf[12] = {0};//定义发送数据变量
    int i, length = 0;

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 6;
    buf[2] = length;                    //buf[2] size为6
    

        buf[3] = PWM[0];  //buf[3]
        buf[4] = PWM[1];  //buf[4]
        buf[5] = PWM[2];  //buf[5]
        buf[6] = PWM[3];  //buf[6]    控制数据
        buf[7] = PWM[4];  //buf[7]
        buf[8] = PWM[5];  //buf[8]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[9]
    buf[3 + length + 1] = ender[0];     //buf[10]
    buf[3 + length + 2] = ender[1];     //buf[11]

    // 通过串口下发数据
    if(boost::asio::write(sp, boost::asio::buffer(buf))!=0)
    {
        ROS_INFO("successfully!");//发送成功打印successfully
    }
}


/********************************************************
（暂时用不到）
函数功能：从下位机读取数据
入口参数：机器人
出口参数：bool
********************************************************/
/*
bool readSpeed(double &Left_v,double &Right_v,double &Angle,unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[150]={0};
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buf); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        

    // 检查信息头
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    length = buf[2];                                 //buf[2]

    // 检查信息校验值
    checkSum = getCrc8(buf, 3 + length);             //buf[10] 计算得出
    if (checkSum != buf[3 + length])                 //buf[10] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }    

    // 读取速度值
    for(i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
        rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
        angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
    }

    // 读取控制标志位
    ctrlFlag = buf[9];
    
    Left_v  =leftVelNow.d;
    Right_v =rightVelNow.d;
    Angle   =angleNow.d;

    return true;
}
*/
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
