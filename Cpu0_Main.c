/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC364DP
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-11-23
 ********************************************************************************************************************/

#include "Cpu0_Main.h"
#include "headfile.h"
#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中


//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译

//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因为需要我们自己手动调用enableInterrupts();来开启中断的响应。


#define BEEP_PIN   P33_10       //定义蜂鸣器引脚
#define MOTOR1_PWM   ATOM0_CH3_P21_5    //定义1电机PWM引脚
#define MOTOR2_PWM   ATOM0_CH0_P21_2    //定义2电机PWM引脚
#define MOTOR3_PWM   ATOM0_CH2_P21_4    //定义3电机PWM引脚
#define MOTOR4_PWM   ATOM0_CH1_P21_3   //定义4电机PWM引脚
#define S_MOTOR_PIN  ATOM1_CH1_P33_9    //定义舵机引脚


//定义五向按键引脚
#define KeyRight    P23_1
#define KeyUp       P22_2
#define KeyTip      P22_3
#define KeyDown     P22_0
#define KeyLeft     P22_1

////////////////////////////////////////五向按键/////////////////////////////////////////
//开关状态变量
uint8 keyTip_status = 1;
uint8 keyUp_status = 1;
uint8 keyDown_status = 1;
uint8 keyLeft_status = 1;
uint8 keyRight_status = 1;
//上一次开关状态变量
uint8 keyTip_last_status;
uint8 keyUp_last_status;
uint8 keyDown_last_status;
uint8 keyLeft_last_status;
uint8 keyRight_last_status;
//开关标志位
uint8 keyTip_flag;
uint8 keyUp_flag;
uint8 keyDown_flag;
uint8 keyLeft_flag;
uint8 keyRight_flag;


#define routine 3

void All_Pit_Handler(void);
void Speed_control(void);

//舵机的偏角
int SERVOLEFT   = 850; //(+80)
int SERVOMIDDLE = 775;
int SERVORIGHT  = 695; //(-80)

int right_bian[200];
uint8 bin_mt9v03x_image[MT9V03X_H][MT9V03X_W];

int state = 0;

int cf_state=0;//侧方状态位
float cf_want_distance;
int Count_Black=0;
int get_color=0,get_white=0;

int32 all_lenth=0;
int32 now_lenth=0;
int32 state_0_lenth=0;

float all_angle=0;
float state_1_angle=0;
float state_3_angle=0;
float now_angle=0;
int cf_want_servo;

int hop = 233;

int Key_Check(PIN_enum pin, int time_s);
int Key_Check1(PIN_enum pin, int time_s);
int Key_Check2(PIN_enum pin, int time_s);
int Key_Check3(PIN_enum pin, int time_s);
int flag1,flag2,flag3,flag4;
int order = 8;
int page = 0;

int sta1_line = 70;

int xianshi[20];

float daoku_angle = 0;
float chuku_angle = 0;

float distance=0;

int All_State=0;
void clean_icm20602(void);

int core0_main(void)
{
	get_clk();//获取时钟频率  务必保留
	//用户在此处调用各种初始化函数等

	/********************---------------ICM20602 初始化-----------------********************/
    icm20602_init_spi();

    gpio_init(BEEP_PIN, GPO, 0, PUSHPULL);  //蜂鸣器初始化
    //初始化摄像头
    mt9v03x_init();
    /********************----------------ips200-----------------********************/
    ips200_init(1);
    /********************----------------五向按键-----------------********************/
    //五个按键初始化GPIO为输入
    //按键需要对引脚进行初始化，并且把引脚置成输入、高电平（高低电平实际无所谓，讲究则根据硬件上下拉情况）
    gpio_init(KeyTip,  GPI, 1, PULLUP);
    gpio_init(KeyUp,   GPI, 1, PULLUP);
    gpio_init(KeyDown, GPI, 1, PULLUP);
    gpio_init(KeyLeft, GPI, 1, PULLUP);
    gpio_init(KeyRight,GPI, 1, PULLUP);


    //编码器初始化
    gpt12_init(GPT12_T5,GPT12_T5INB_P10_3,GPT12_T5EUDB_P10_1);  //左
    gpt12_init(GPT12_T4,GPT12_T4INA_P02_8,GPT12_T4EUDA_P00_9);  //右
    ///********************----------------PWM初始化-----------------********************/
    //    //舵机居中
    gtm_pwm_init(S_MOTOR_PIN, 50, SERVOMIDDLE);  //中750 右520（-145）左980（+145）
    //    //电机初始化
    gtm_pwm_init(MOTOR1_PWM, 17000, 0);
    gtm_pwm_init(MOTOR2_PWM, 17000, 0);
    gtm_pwm_init(MOTOR3_PWM, 17000, 0);
    gtm_pwm_init(MOTOR4_PWM, 17000, 0);
    /********************----------------中断初始化----------------********************/
    pit_interrupt_ms(CCU6_0, PIT_CH0, routine); //电机闭环
    //使用CCU6_1模块的通道0 产生一个5ms的周期中断
    pit_interrupt_ms(CCU6_1, PIT_CH0, 2); //ICM20602

    //等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();
	systick_delay_ms(STM0,1000);
	clean_icm20602();
	while (TRUE)
	{
//        flag1 = Key_Check(KeyUp,100);
//        flag2 = Key_Check1(KeyDown,100);
//        flag3 = Key_Check2(KeyRight,100);
//        flag4 = Key_Check3(KeyLeft,100);
//        ips200_showstr(0,order,">");
	    ips200_displayimage032(bin_mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
//	    ips200_showint32(68,8,hop,3);
//	    ips200_showint32(68,9,state,3);
//	    ips200_showint32(68,10,xianshi[0],3);
//	    ips200_showint32(68,11,xianshi[1],3);
//	    ips200_showfloat(68,12,daoku_angle,3,3);
//	    ips200_showfloat(68,13,distance,3,3);
//	    ips200_showint32(68,14,xianshi[4],3);
//	    ips200_showint32(68,15,xianshi[5],3);
//	    ips200_showint32(68,16,xianshi[6],3);
//	    ips200_showint32(68,17,xianshi[7],3);
//	    ips200_showint32(68,18,xianshi[8],3);

	    ips200_showint32(68,12,Count_Black,3);
	    ips200_showint32(68,13,get_color,3);
	    ips200_showint32(68,14,all_lenth,5);
	    ips200_showint32(68,15,distance,3);
	    ips200_showfloat(68,16,all_angle,3,3);
		//用户在此处编写任务代码
	}
}
/***********************************************************************************
姿态解算
***********************************************************************************/
float angle_re_last[2]={0};
float icm_gyro_y_last[3]={0};
float icm_gyro_z_last[3]={0};
float icm_gyro_x_clean=0.22;
float icm_gyro_y_clean=0.88;
float icm_gyro_z_clean=1.38;
float w_y_re_use=0;                         //利用的y轴角速度
float w_z_re_use=0;                         //利用的z轴角速度
float angle_acc=0;
float angle_gyro=0;
float angle_re=0;
float angle_re_use=0;
float acc_unit_conversion = 57.3;           //加速度单位转换（度）
float gyro_unit_conversion=-0.50;           //角速度单位转换（度）
float gyro_integral_time  = 0.002;          //角速度积分时间
float angle_k1=0.007;                       //加速度与角速度融合，加速度占比权重
/*********************************************************************************** */
void clean_icm20602(void){
    for(int i=0;i<500;i++){
        gpio_set(BEEP_PIN,1);//打开蜂鸣器
        get_icm20602_gyro_spi();
        get_icm20602_accdata_spi();

        icm_gyro_y_clean+= icm_gyro_y;
        icm_gyro_z_clean+= icm_gyro_z;
        systick_delay_ms(STM0,2);
    }
    icm_gyro_y_clean/=500;
    icm_gyro_y_clean/=8;
    icm_gyro_z_clean/=500;
    icm_gyro_z_clean/=8;
    gpio_set(BEEP_PIN,0);//打开蜂鸣器


}



/***************************************************************************************
函 数 名 :void Angle_Detection(void)
功     能  :检测转角（俯仰，翻滚）
传     入  :无
返 回  值 :无
整定参数 :无
***************************************************************************************/
void Angle_Detection(void)
{
    angle_re_last[1]=angle_re_last[0];
    angle_re_last[0]=angle_re;

    icm_gyro_y_last[2]=icm_gyro_y_last[1];
    icm_gyro_y_last[1]=icm_gyro_y_last[0];
    icm_gyro_y_last[0]=icm_gyro_y;

    icm_gyro_z_last[2]=icm_gyro_z_last[1];
    icm_gyro_z_last[1]=icm_gyro_z_last[0];
    icm_gyro_z_last[0]=icm_gyro_z;

    get_icm20602_gyro_spi();
    get_icm20602_accdata_spi();


    w_y_re_use=icm_gyro_y/8.0;
    w_z_re_use=icm_gyro_z/8.0;

//  icm_gyro_x=icm_gyro_x-icm_gyro_x_clean;
    w_y_re_use=w_y_re_use-icm_gyro_y_clean;  //消除零飘
    w_z_re_use=w_z_re_use-icm_gyro_z_clean;

    angle_acc=(atan2(icm_acc_x,icm_acc_z))*acc_unit_conversion; //加速度得角度 单位转换
    angle_acc=(angle_acc+angle_re_last[0]+angle_re_last[1])/3; //滤波
    angle_gyro=w_y_re_use*gyro_unit_conversion ;

    angle_re=angle_k1*angle_acc+(1-angle_k1)*(angle_re+angle_gyro*0.002);       //0.002为pit时间！！！！


    /***************************************************************************************************************************************/
    if(All_State==0){
    //记角度
    if(state == 2) daoku_angle += w_z_re_use*gyro_unit_conversion*0.002;
    else daoku_angle=0;
    if(state == 6) chuku_angle += w_z_re_use*gyro_unit_conversion*0.002;
    else chuku_angle=0;
    }

    /***************************************************************************************************************************************/

    all_angle += w_z_re_use*gyro_unit_conversion*0.003;
}

int16 encoder_speedleft=0;
int16 encoder_speedright=0;

int Target_Left_Speed=7;
int Target_Right_Speed=7;

float Left_Kp=250,Left_Ki=1,Right_Kp=250,Right_Ki=1;
float Left_inter=0;Right_inter=0;

void Speed_control(void){
    encoder_speedleft = -gpt12_get(GPT12_T5);//左编码器读取
    encoder_speedright = gpt12_get(GPT12_T4);//右编码器读取
    gpt12_clear(GPT12_T5);                   //左编码器清零
    gpt12_clear(GPT12_T4);                   //右编码器清零
    all_lenth = all_lenth + (encoder_speedleft + encoder_speedright)/2;


    encoder_speedleft /= routine*1.0;
    encoder_speedright /= routine*1.0;//归一化

    float Left_Err = Target_Left_Speed - encoder_speedleft;
    float Right_Err = Target_Right_Speed - encoder_speedright;//误差

    if(abs(Left_Err)<Target_Left_Speed/2)//积分分离
    Left_inter += Left_Err;
    if(abs(Right_Err)<Target_Right_Speed/2)//积分分离
    Right_inter += Right_Err;

    Left_inter = Left_inter<-10000?-10000:Left_inter;
    Right_inter = Right_inter<-10000?-10000:Right_inter;
    Left_inter = Left_inter>10000?10000:Left_inter;
    Right_inter = Right_inter>10000?10000:Right_inter;//积分限幅

    int MotorOutLeft=0,MotorOutRight=0;
    MotorOutLeft =  Left_Kp * Left_Err + Left_Ki * Left_inter;
    MotorOutRight =  Right_Kp * Right_Err + Right_Ki * Right_inter;//位置式pid

    MotorOutLeft = MotorOutLeft<-6000?-6000:MotorOutLeft;
    MotorOutRight = MotorOutRight<-6000?-6000:MotorOutRight;
    MotorOutLeft = MotorOutLeft>6000?6000:MotorOutLeft;
    MotorOutRight = MotorOutRight>6000?6000:MotorOutRight;//pwm限幅

    if((MotorOutLeft>=0))//左电机
        {
            MotorOutLeft = MotorOutLeft>10000?10000:MotorOutLeft;//分母为10000（FTM.H）
            pwm_duty(MOTOR1_PWM, (int)MotorOutLeft); //电机正转
            pwm_duty(MOTOR2_PWM, 0); //电机反转
        }
        else if((MotorOutLeft<0))
        {
            MotorOutLeft = MotorOutLeft<-10000?-10000:MotorOutLeft;//分母为10000（FTM.H）
            pwm_duty(MOTOR1_PWM, 0); //电机正转
            pwm_duty(MOTOR2_PWM, (int)(-MotorOutLeft)); //电机反转
        }
        if((MotorOutRight>=0))//右电机
        {
            MotorOutRight = MotorOutRight>10000?10000:MotorOutRight;//分母为10000（FTM.H）
            pwm_duty(MOTOR3_PWM, (int)MotorOutRight); //电机正转
            pwm_duty(MOTOR4_PWM, 0); //电机反转
        }
        else if((MotorOutRight<0))
        {
            MotorOutRight = MotorOutRight<-10000?-10000:MotorOutRight;//分母为10000（FTM.H）
            pwm_duty(MOTOR3_PWM, 0); //电机正转
            pwm_duty(MOTOR4_PWM, (int)(-MotorOutRight)); //电机反转
        }


}
///***************************************************************************************
//函 数 名 :void Pingmuceshi(void)
//功     能  :按键测试(图像信息分析处理)
//传     入  :无
//返 回  值 :无
//整定参数 :无
//说明：五向按键控制IPS屏幕
//***************************************************************************************/
void Pingmuceshi(void)
{
    //保存按键状态
    keyTip_last_status = keyTip_status;
    keyUp_last_status = keyUp_status;
    keyDown_last_status = keyDown_status;
    keyLeft_last_status = keyLeft_status;
    keyRight_last_status = keyRight_status;
    //读取当前按键状态
    keyTip_status = gpio_get(KeyTip);
    keyUp_status = gpio_get(KeyUp);
    keyDown_status = gpio_get(KeyDown);
    keyLeft_status = gpio_get(KeyLeft);
    keyRight_status = gpio_get(KeyRight);
    //检测到按键按下之后  并放开置位标志位
    if(keyTip_status && !keyTip_last_status)    keyTip_flag = 0;
    if(keyUp_status && !keyUp_last_status)    keyUp_flag = 0;
    if(keyDown_status && !keyDown_last_status)    keyDown_flag = 0;
    if(keyLeft_status && !keyLeft_last_status)    keyLeft_flag = 0;
    if(keyRight_status && !keyRight_last_status)    keyRight_flag = 0;
    //标志位置位之后，可以使用标志位执行自己想要做的事件
//    if(!keyTip_flag)
//    {
//        keyTip_flag = 1;//使用按键之后，应该清除标志位
//        testTip++;
//    }
//
//    if(!keyUp_flag)
//    {
//        keyUp_flag = 1;//使用按键之后，应该清除标志位
//        testUp++;
//    }
//
//    if(!keyDown_flag)
//    {
//        keyDown_flag = 1;//使用按键之后，应该清除标志位
//        testDown++;
//    }
//
//    if(!keyLeft_flag)
//    {
//        keyLeft_flag = 1;//使用按键之后，应该清除标志位
//        pwm_duty(S_MOTOR_PIN,Duty); //测试左右极限时用
//        Duty-=10;
//    }
//    if(!keyRight_flag)
//    {
//        keyRight_flag = 1;//使用按键之后，应该清除标志位
//        pwm_duty(S_MOTOR_PIN,Duty); //测试左右极限时用
//        Duty+=10;
//    }


//    ips200_showstr(0,8,"Duty:");
//    ips200_showint32(68,8,Duty,3);
//    ips200_showstr(0,9,"Error:");
//    ips200_showint32(68,9,Duty-SERVOMIDDLE,3);
    }
/***************************************************************************************
函 数 名 :short GetOSTU (unsigned char tmImage[120][188])
功    能 :灰度值大津法处理
传    入 :所选取的图像
返 回 值 :最佳阈值
整定参数 :无
说明：
***************************************************************************************/
int GetOSTU (unsigned char tmImage[120][188])
{
    signed int i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelshortegralBack = 0;
    unsigned long Pixelshortegral = 0;
    signed long PixelshortegralFore = 0;
    signed long PixelFore = 0;
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed int MinValue, MaxValue;
    signed int Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < 120; j++)
    {
        for (i = 0; i < 188; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)
        return MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)
        return MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  像素总数

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //前景像素点数
        PixelFore = Amount - PixelBack;           //背景像素点数
        OmegaBack = (float) PixelBack / Amount;   //前景像素百分比
        OmegaFore = (float) PixelFore / Amount;   //背景像素百分比
        PixelshortegralBack += HistoGram[j] * j;  //前景灰度值
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //背景灰度值
        MicroBack = (float) PixelshortegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (float) PixelshortegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}
/***************************************************************************************
函 数 名 :void Get_Bin_Image (unsigned char mode)
功    能 :二值化图像
传    入 :所选取的图像
返 回 值 :无
整定参数 :无
说明：
***************************************************************************************/

void Get_Bin_Image (int threshold)
{
    unsigned short i = 0, j = 0;
//    unsigned short Threshold = 0;


//        Threshold =mt9v03x_image[0];
    /* 二值化 */
    for (i = 0; i < 120; i++)
    {
        for (j = 0; j < 188; j++)
        {
            if ( mt9v03x_image[i][j] > threshold) //数值越大，显示的内容越多，较浅的图像也能显示出来
                {
                    bin_mt9v03x_image[i][j] = 255;
//                    ips200_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
                }
            else
                {
                    bin_mt9v03x_image[i][j] = 0;
//                    ips200_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
                }

        }
    }

//    ips200_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
}

void Get_Lenth(void)
{
    int bz;
    int start_search = MT9V03X_H - 3;
    for(int j = 60 ; j <= 120;j++){
                bz = 0;
                for(int i = start_search ; i > 10 ; i--){
                    for(int k = 0 ; k < 3 ; k++){
                        if (bin_mt9v03x_image[i - k][j] == 0 && i - k > 0){
                            if(k == 2)
                            {
                                right_bian[j] = MT9V03X_H - i;
                                start_search = i + 3;
                                bz = 1;
                            }
                        }else{
//                            bin_mt9v03x_image[i - k][j]=0;
                            break;
                        }
                    }
                    if (bz) break;
                }
            }

    for(int i=110;i<120;i++){
        distance += right_bian[i];
    }

    distance /= 10.0;
}

// 检测车库离边线距离
void Get_Lenth1(void)
{
    int i,j;
    distance = 0;
    for(int j = 110 ; j <= 120;j++){
        for(int i = MT9V03X_H - 1 ; i > 80 ; i--){
            if (bin_mt9v03x_image[i][j] == 0)
            {
                distance += MT9V03X_H - i;
                break;
            }
        }
    }
    distance /= 10.0;
}

// 检测倒库点
int pd_start()
{
    int i,j;
    int sum;
    for (j = sta1_line; j <= sta1_line + 2; j ++)
    {
        sum = 0;
        for (i = MT9V03X_H - right_bian[j]; i >= 0; i --)
        {
            if(bin_mt9v03x_image[i][j] == 0)
            {
                sum ++;
                if (sum >= 15) break;
            } else
                return 0;
        }
        if (sum < 10) return 0;
    }

    return 1;
}

// 检测垂直入库点
int pd_daoku()
{
    int i,j;
    for (i = 55; i <= 65; i ++)
    {
        for (j = 20; j <= 173; j ++)
        {
            if (bin_mt9v03x_image[i][j] == 0 && bin_mt9v03x_image[i][j + 1] == 0 && bin_mt9v03x_image[i][j + 2] == 0 && j < 132)
                return 0;
//            if (bin_mt9v03x_image[i][j] == 0 && i == 55)
//                xianshi[0] = j;
//            if (bin_mt9v03x_image[i][j] == 0 && i == 65)
//                xianshi[1] = j;
        }
    }
    return 1;
}

// 检测停车点
int pd_stop()
{
    int i,j;
    int pd = 0;
    for (i = 75; i <= 85; i ++)
    {
        for (j = 10; j <= 52; j ++)
        {
            if (bin_mt9v03x_image[i][j] == 0 && bin_mt9v03x_image[i][j + 1] == 0)
            {
                pd ++;
                break;
            }
//            if (bin_mt9v03x_image[i][j] == 0 && i == 75)
//                xianshi[0] = j;
//            if (bin_mt9v03x_image[i][j] == 0 && i == 85)
//                xianshi[1] = j;
        }
    }
    if (pd >= 7) return 1;
    else return 0;
}

// 检测出库打脚点
int pd_chu()
{
    int i,j;
    int pd = 0;
    for (i = 105; i <= 115; i ++)
    {
        for (j = 20; j <= 115; j ++)
        {
            if (bin_mt9v03x_image[i][j] == 0 && bin_mt9v03x_image[i][j + 1] == 0 && j <= 80)
            {
                return 0;
            }
//            if (bin_mt9v03x_image[i][j] == 0 && i == 75)
//                xianshi[0] = j;
//            if (bin_mt9v03x_image[i][j] == 0 && i == 85)
//                xianshi[1] = j;
        }
    }
    return 1;
}

int black_count_yu=13;

void Get_Black_Lenth(void){

    int black_count=0;


        for(int j = MT9V03X_H-5 ;j > 30;j--){
            black_count=0;
            if(bin_mt9v03x_image[j][100]==0){
                for(int k = 0;k < black_count_yu; k++){
                    if(bin_mt9v03x_image[j-k][100]==0)
                        black_count++;
                    else
                        break;
                }
                if(black_count>=black_count_yu-1){
                    get_color=0;
                }else{
                    get_color=1;
                    get_white=1;
                }

                break;

           }else{
            bin_mt9v03x_image[j][100]=2;
           }
        }



     if(get_color==0&&get_white){
         Count_Black++;
         get_white=0;
     }



}

int js = 0;
int tc1 = 0;

// state 0 没啥用
// state 1 检测倒库点
// state 2 固定打交倒车
// state 3 微调入库
// state 4 停5秒
// state 5 直走出库
// state 6 固定打脚出库
// state 7 循迹
int count_start=0;

void Picture_Processing(void)
{

    if(mt9v03x_finish_flag)
    {

        hop = GetOSTU (mt9v03x_image);
        Get_Bin_Image (hop);

/***************************************************************************************************************************************/
        if(All_State==0){

        if (state != 3) Get_Lenth();
        else if (state == 3)
            Get_Lenth1();

        if (state == 0)
        {
            if (pd_start())
            {
                state = 1;
                Target_Left_Speed = 6;
                Target_Right_Speed = 6;
            }
        } else if (state == 1)
        {
            if (pd_daoku() && tc1 == 0)
            {
                js = 0;
                tc1 = 1;
                gpio_set(BEEP_PIN,1);//打开蜂鸣器
                Target_Left_Speed = 0;
                Target_Right_Speed = 0;
            }
            if (tc1 && js >= 333)
            {
                state = 2;
                gpio_set(BEEP_PIN,0);//关闭蜂鸣器
                Target_Left_Speed = -17;
                Target_Right_Speed = -5;
            }
        } else if (state == 2)
        {
            if (daoku_angle <= -80)
            {
                state = 3;
                Target_Left_Speed = -10;
                Target_Right_Speed = 3;
            }
        } else if (state == 3)
        {
            if (pd_stop())
            {
                Target_Left_Speed = 0;
                Target_Right_Speed = 0;
                state = 4;
                gpio_set(BEEP_PIN,1);//打开蜂鸣器
                js = 0;
            }
        } else if (state == 4)
        {
            if (js >= 333)
            {
                gpio_set(BEEP_PIN,0);//关闭蜂鸣器
            }
            if (js >= 1666)
            {
                state = 5;
                Target_Left_Speed = 6;
                Target_Right_Speed = 6;
            }
        } else if (state == 5)
        {
            if (pd_chu())
            {
                state = 6;
            }
        } else if (state == 6)
        {
            // 出库
            if (chuku_angle >= 80)
            {
                state = 7; // 7出完库
                All_State=1;
            }
        }
        }

 /***************************************************************************************************************************************/

        else if(All_State==1){
            static int count=0;
//            if(count_start)
//                count+=3;

            switch(cf_state){
                case 0:
                    if(Count_Black<=1){
                        Target_Left_Speed = 7;
                        Target_Right_Speed = 7;
                    }else{
                        Target_Left_Speed = 6;
                        Target_Right_Speed = 6;
                    }
                    Get_Lenth();
                    cf_want_distance=30;
                    Get_Black_Lenth();
                    if(Count_Black==3){
                        cf_state++;
                        state_0_lenth=all_lenth;
                    }
                    break;
                case 1:
                    Target_Left_Speed = 6;
                    Target_Right_Speed = 6;
                    Get_Lenth();
                    cf_want_distance=30;
                    now_lenth=all_lenth;
                    if((now_lenth-state_0_lenth)>3200){
                        Target_Left_Speed = 0;
                        Target_Right_Speed = 0;
                        gpio_set(BEEP_PIN,1);//打开蜂鸣器
                        count+=3;
                        if(count>300){
                            count=0;
                            gpio_set(BEEP_PIN,0);//关闭蜂鸣器
                            count=0;
                            state_1_angle=all_angle;
                            cf_state++;
                        }
                    }

                    break;
                case 2:
                    Target_Left_Speed = -14;
                    Target_Right_Speed = -6;
                    now_angle=all_angle;
                    cf_want_servo = SERVORIGHT;
                    if((now_angle-state_1_angle)<-75){
                        cf_state++;
                    }

                    break;
                case 3:
                    now_angle=all_angle;
                    Target_Left_Speed = -6;
                    Target_Right_Speed = -14;
                    cf_want_servo = SERVOLEFT;
                    if((now_angle-state_1_angle)>-10){
                        Target_Left_Speed = 0;
                        Target_Right_Speed = 0;
                        gpio_set(BEEP_PIN,1);//打开蜂鸣器
                        count+=3;
                        if(count>100)
                            gpio_set(BEEP_PIN,0);//关闭蜂鸣器
                        if(count>500){
                            count=0;
                            state_3_angle=all_angle;
                            cf_state++;
                        }
                    }
                    break;
                case 4:
                    now_angle=all_angle;
                    Target_Left_Speed = 6;
                    Target_Right_Speed = 14;
                    cf_want_servo = SERVOLEFT;
                    if((now_angle-state_1_angle)<-80){
                        cf_state++;
                    }
                    break;
                case 5:
                    now_angle=all_angle;
                    Target_Left_Speed = 14;
                    Target_Right_Speed = 6;
                    cf_want_servo = SERVORIGHT;
                    if((now_angle-state_1_angle)>-15){
                        Target_Left_Speed = 0;
                        Target_Right_Speed = 0;

                    }
                    break;




            }



        }
        mt9v03x_finish_flag = 0;


    }

}

/***************************************************************************************************************************************/
float err;
float last_err;
float kp = 3.0;
float kd = 7.0;
/***************************************************************************************************************************************/
float cf_err;
float cf_last_err;
float cf_kp = 3.0;
float cf_kd = 6.0;
float cf_want_distance;
int cf_want_servo=0;
void servo_control(void){
    /***************************************************************************************************************************************/
    if(All_State==0){
    int want_servo=0;
    if (state == 3 || state == 4 || state == 5) err = distance - 1.0;
    else err = distance - 42.0;
    if (state == 3 || state == 4 || state == 5)
        want_servo = SERVOMIDDLE - 2 * err;
    else
        want_servo = SERVOMIDDLE - (kp * err + kd * (err - last_err));
    if(want_servo>SERVOLEFT)
        want_servo=SERVOLEFT;
    if(want_servo<SERVORIGHT)
        want_servo=SERVORIGHT;
    if (state == 2 || state == 6)
        pwm_duty(S_MOTOR_PIN,SERVORIGHT);
    else if (state == 5)
        pwm_duty(S_MOTOR_PIN,SERVOMIDDLE);
    else pwm_duty(S_MOTOR_PIN,want_servo);

    last_err = err;
    }
    /***************************************************************************************************************************************/
    else if(All_State==1){
        if(cf_state!=2&&cf_state!=3&&cf_state!=4&&cf_state!=5){
            cf_err = distance - cf_want_distance;
            cf_want_servo = SERVOMIDDLE - cf_kp * cf_err;
        }

            if(cf_want_servo>SERVOLEFT)
                cf_want_servo=SERVOLEFT;
            if(cf_want_servo<SERVORIGHT)
                cf_want_servo=SERVORIGHT;

            pwm_duty(S_MOTOR_PIN,cf_want_servo);

    }


}
void All_Pit_Handler(void){
    /***************************************************************************************************************************************/

    js ++;
    /***************************************************************************************************************************************/
    Speed_control();
//    printf("left:%d  right:%d\n",encoder_speedleft,encoder_speedright);
    Picture_Processing();
    servo_control();




}
//箭头往上
int Key_Check(PIN_enum pin, int time_s)
{
    if(!gpio_get(pin))
    {
        systick_delay_ms(STM0,time_s);
        ips200_showstr(0,order," ");
        order --;
//        if (order == -1)
//        {
//            page = 0;
//            ips200_clear(WHITE);
//            order = 14;
//        }
//        send = 0;
        return 1;
    }
    else
        return 0;
}

//箭头往下
int Key_Check1(PIN_enum pin, int time_s)
{
    if(!gpio_get(pin))
    {
        systick_delay_ms(STM0,time_s);
        ips200_showstr(0,order," ");
        order ++;
//        if (order == 20)
//        {
//            page = 1;
//            ips200_clear(WHITE);
//            order = 0;
//        }
//        send = 1;
//        start ++;
        return 1;
    }
    else
        return 0;
}

//箭头指的值加一
int Key_Check2(PIN_enum pin, int time_s)
{
    if(!gpio_get(pin))
    {
        systick_delay_ms(STM0,time_s);
//        expectation += 100;
        if (page == 0)
        {
            if (order == 8) hop += 1;
//            if (order == 12) js1 += 0.1;
//            if (order == 13) open = 200*240;
//            if (order == 14) kp += 0.5;
//            if (order == 15) kd += 0.1;
//            if (order == 16) tag[1] ++;
//            if (order == 17) jiasu += 5;
//            if (order == 18) start = 400;
//            if (order == 19) speed += 10;
        } else
        {
//            if (order == 1) gdj += 10;
//            if (order == 2) hsd += 10;
//            if (order == 3) cirangle += 5;
//            if (order == 4) MID += 50;
//            if (order == 5) MAXMID += 50;
//            if (order == 6) MINDG += 10;
//            if (order == 7) MIDX += 1;
//            if (order == 8) CIRQ += 50;
//            if (order == 9) HDG += 50;
//            if (order == 10) XDG1 += 50;
//            if (order == 11) XDG2 += 50;
//            if (order == 12) XDGS += 1;
//            if (order == 13) TURNL += 10;
//            if (order == 14) TURNR += 10;
//            if (order == 15) TIME += 5;
//            if (order == 16) DXMID += 50;
//            if (order == 17) DXSDG += 50;
//            if (order == 18) CHD += 5;
//            if (order == 19) start = 400;
        }

        return 1;
    }
    else
        return 0;
}

//箭头指的值减一
int Key_Check3(PIN_enum pin, int time_s)
{
    if(!gpio_get(pin))
    {
        systick_delay_ms(STM0,time_s);
//        expectation -= 100;
        if (page == 0)
        {
            if (order == 8) hop -= 1;
//            if (order == 12) js1 -= 0.1;
//            if (order == 13) open = 0;
//            if (order == 14) kp -= 0.5;
//            if (order == 15) kd -= 0.1;
//            if (order == 16) tag[1] --;
//            if (order == 17) jiasu -= 5;
//            if (order == 18) start = 400;
//            if (order == 19) speed -= 10;
        } else
        {
//            if (order == 1) gdj -= 10;
//            if (order == 2) hsd -= 10;
//            if (order == 3) cirangle -= 5;
//            if (order == 4) MID -= 50;
//            if (order == 5) MAXMID -= 50;
//            if (order == 6) MINDG -= 10;
//            if (order == 7) MIDX -= 1;
//            if (order == 8) CIRQ -= 50;
//            if (order == 9) HDG -= 50;
//            if (order == 10) XDG1 -= 50;
//            if (order == 11) XDG2 -= 50;
//            if (order == 12) XDGS -= 1;
//            if (order == 13) TURNL -= 10;
//            if (order == 14) TURNR -= 10;
//            if (order == 15) TIME -= 5;
//            if (order == 16) DXMID -= 50;
//            if (order == 17) DXSDG -= 50;
//            if (order == 18) CHD -= 5;
//            if (order == 19) start = 400;
        }

        return 1;
    }
    else
        return 0;
}


#pragma section all restore
