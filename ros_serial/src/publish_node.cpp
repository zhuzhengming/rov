/**************************

用遥控器通过上位机串口通讯控制stm32来控制6个推进器运动
zzm
//安装推进器注意：1   头    2
//             5         6
//             3         4
*************************/

//遥控器操作方式：左摇杆负责平面运动，右摇杆竖直方向为沉浮，水平方向为原地转动


#include "ros/ros.h"
#include "std_msgs/String.h" //use data struct of std_msgs/String  
#include "mbot_linux_serial.h"
#include "sensor_msgs/Joy.h"

double msg_joy[4];//保存手柄数据 （4个通道分别为前进后退，左右横移，原地旋转，上浮下沉）
uint8_t PWM[6]={80,80,80,80,80,80};//初始化到中间位置
uint8_t startbuf[6];
uint8_t balance_force = 50;//初始上升推力
int state = 0;
double k = 0.8;//减小速度比例系数

void control(void)//根据手柄信息处理发送数据
{
   
    if(msg_joy[1]>0)
    {
         //前进,3,4号电机推进
        PWM[2]=(uint8_t)(k*(100*msg_joy[1]+100));
        PWM[3]=(uint8_t)(k*(100*msg_joy[1]+100));
    }
    
    if(msg_joy[1]<0)
    {
        //后退,1，2号电机推进
        PWM[0]=(uint8_t)(k*(-(100*msg_joy[1])+100));//推进器正转
        PWM[1]=(uint8_t)(k*(-(100*msg_joy[1])+100));
    }
    
    if(msg_joy[0]>0)
    {
        //左横移,2,4号电机推进
        PWM[1]=(uint8_t)(k*(100*msg_joy[0]+100));
        PWM[3]=(uint8_t)(k*(100*msg_joy[0]+100));
    }
    
    if(msg_joy[0]<0)
    {
        //右横移，1,3号电机推进
        PWM[0]=(uint8_t)(k*(-(100*msg_joy[0])+100));//推进器正转
        PWM[2]=(uint8_t)(k*(-(100*msg_joy[0])+100));
    }
    
    if(msg_joy[2]>0)
    {
        //原地左方向旋转,2,3号电机推进
        PWM[1]=(uint8_t)(k*(100*msg_joy[2]+100));
        PWM[2]=(uint8_t)(k*(100*msg_joy[2]+100));
    }
   
    if(msg_joy[2]<0)
    {
        //原地右方向旋转,1,4号电机推进
        PWM[0]=(uint8_t)(k*(-(100*msg_joy[2])+100));//推进器正转
        PWM[3]=(uint8_t)(k*(-(100*msg_joy[2])+100));
    }

        //上浮,5,6号电机正向推进,下沉,5,6号电机反向推进
        //由于5，6号推进器需要持续转动来提供平衡升力
        PWM[4]=(uint8_t)(k*(100*msg_joy[3]+100+balance_force));
        PWM[5]=(uint8_t)(k*(100*msg_joy[3]+100+balance_force));

}

void start(void)
{
    if(state == 1)
	    {
            
            for(int j=0;j<6;j++)
            {
                startbuf[j]=1;
            }
            
            writeSpeed(startbuf);//发送开始信号 数据位内容全为1
            return;
        }
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
        for (int i=0; i < 4; i++)
        {
            msg_joy[i]=msg->axes[i];//保存手柄发送的数据
        }
        state = msg->buttons[1];//开始

        start();//发送开始遥控信号
        control();//处理手柄数据
        writeSpeed(PWM);//发送控制指令
	return;
}


int main(int agrc,char **argv)
{
    ros::init(agrc,argv,"public_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joy", 1000, &joy_callback);//订阅手柄
    ros::Rate loop_rate(100);
    
    //串口初始化
    serialInit();
    //竖直推进器速度初始化
    PWM[4] = 80+balance_force;
    PWM[5] = 80+balance_force;


    while(ros::ok())
    {
        ros::spinOnce();
        //向STM32端发送数据
        //ROS_INFO("%d,%d,%d,%d,%d,%d",startbuf[0],startbuf[1],startbuf[2],startbuf[3],startbuf[4],startbuf[5]);//打印发送数据
        ROS_INFO("%d,%d,%d,%d,%d,%d",PWM[0],PWM[1],PWM[2],PWM[3],PWM[4],PWM[5]);//打印发送数据
        
        loop_rate.sleep();
    }
    return 0;
}
 



