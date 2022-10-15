/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC364DP
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-11-23
 ********************************************************************************************************************/

#include "Cpu0_Main.h"
#include "headfile.h"
#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��


//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���

//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ���Ϊ��Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��


#define BEEP_PIN   P33_10       //�������������
#define MOTOR1_PWM   ATOM0_CH3_P21_5    //����1���PWM����
#define MOTOR2_PWM   ATOM0_CH0_P21_2    //����2���PWM����
#define MOTOR3_PWM   ATOM0_CH2_P21_4    //����3���PWM����
#define MOTOR4_PWM   ATOM0_CH1_P21_3   //����4���PWM����
#define S_MOTOR_PIN  ATOM1_CH1_P33_9    //����������


//�������򰴼�����
#define KeyRight    P23_1
#define KeyUp       P22_2
#define KeyTip      P22_3
#define KeyDown     P22_0
#define KeyLeft     P22_1

////////////////////////////////////////���򰴼�/////////////////////////////////////////
//����״̬����
uint8 keyTip_status = 1;
uint8 keyUp_status = 1;
uint8 keyDown_status = 1;
uint8 keyLeft_status = 1;
uint8 keyRight_status = 1;
//��һ�ο���״̬����
uint8 keyTip_last_status;
uint8 keyUp_last_status;
uint8 keyDown_last_status;
uint8 keyLeft_last_status;
uint8 keyRight_last_status;
//���ر�־λ
uint8 keyTip_flag;
uint8 keyUp_flag;
uint8 keyDown_flag;
uint8 keyLeft_flag;
uint8 keyRight_flag;


#define routine 3

void All_Pit_Handler(void);
void Speed_control(void);

//�����ƫ��
int SERVOLEFT   = 850; //(+80)
int SERVOMIDDLE = 775;
int SERVORIGHT  = 695; //(-80)

int right_bian[200];
uint8 bin_mt9v03x_image[MT9V03X_H][MT9V03X_W];

int state = 0;

int cf_state=0;//�෽״̬λ
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
	get_clk();//��ȡʱ��Ƶ��  ��ر���
	//�û��ڴ˴����ø��ֳ�ʼ��������

	/********************---------------ICM20602 ��ʼ��-----------------********************/
    icm20602_init_spi();

    gpio_init(BEEP_PIN, GPO, 0, PUSHPULL);  //��������ʼ��
    //��ʼ������ͷ
    mt9v03x_init();
    /********************----------------ips200-----------------********************/
    ips200_init(1);
    /********************----------------���򰴼�-----------------********************/
    //���������ʼ��GPIOΪ����
    //������Ҫ�����Ž��г�ʼ�������Ұ������ó����롢�ߵ�ƽ���ߵ͵�ƽʵ������ν�����������Ӳ�������������
    gpio_init(KeyTip,  GPI, 1, PULLUP);
    gpio_init(KeyUp,   GPI, 1, PULLUP);
    gpio_init(KeyDown, GPI, 1, PULLUP);
    gpio_init(KeyLeft, GPI, 1, PULLUP);
    gpio_init(KeyRight,GPI, 1, PULLUP);


    //��������ʼ��
    gpt12_init(GPT12_T5,GPT12_T5INB_P10_3,GPT12_T5EUDB_P10_1);  //��
    gpt12_init(GPT12_T4,GPT12_T4INA_P02_8,GPT12_T4EUDA_P00_9);  //��
    ///********************----------------PWM��ʼ��-----------------********************/
    //    //�������
    gtm_pwm_init(S_MOTOR_PIN, 50, SERVOMIDDLE);  //��750 ��520��-145����980��+145��
    //    //�����ʼ��
    gtm_pwm_init(MOTOR1_PWM, 17000, 0);
    gtm_pwm_init(MOTOR2_PWM, 17000, 0);
    gtm_pwm_init(MOTOR3_PWM, 17000, 0);
    gtm_pwm_init(MOTOR4_PWM, 17000, 0);
    /********************----------------�жϳ�ʼ��----------------********************/
    pit_interrupt_ms(CCU6_0, PIT_CH0, routine); //����ջ�
    //ʹ��CCU6_1ģ���ͨ��0 ����һ��5ms�������ж�
    pit_interrupt_ms(CCU6_1, PIT_CH0, 2); //ICM20602

    //�ȴ����к��ĳ�ʼ�����
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
		//�û��ڴ˴���д�������
	}
}
/***********************************************************************************
��̬����
***********************************************************************************/
float angle_re_last[2]={0};
float icm_gyro_y_last[3]={0};
float icm_gyro_z_last[3]={0};
float icm_gyro_x_clean=0.22;
float icm_gyro_y_clean=0.88;
float icm_gyro_z_clean=1.38;
float w_y_re_use=0;                         //���õ�y����ٶ�
float w_z_re_use=0;                         //���õ�z����ٶ�
float angle_acc=0;
float angle_gyro=0;
float angle_re=0;
float angle_re_use=0;
float acc_unit_conversion = 57.3;           //���ٶȵ�λת�����ȣ�
float gyro_unit_conversion=-0.50;           //���ٶȵ�λת�����ȣ�
float gyro_integral_time  = 0.002;          //���ٶȻ���ʱ��
float angle_k1=0.007;                       //���ٶ�����ٶ��ںϣ����ٶ�ռ��Ȩ��
/*********************************************************************************** */
void clean_icm20602(void){
    for(int i=0;i<500;i++){
        gpio_set(BEEP_PIN,1);//�򿪷�����
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
    gpio_set(BEEP_PIN,0);//�򿪷�����


}



/***************************************************************************************
�� �� �� :void Angle_Detection(void)
��     ��  :���ת�ǣ�������������
��     ��  :��
�� ��  ֵ :��
�������� :��
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
    w_y_re_use=w_y_re_use-icm_gyro_y_clean;  //������Ʈ
    w_z_re_use=w_z_re_use-icm_gyro_z_clean;

    angle_acc=(atan2(icm_acc_x,icm_acc_z))*acc_unit_conversion; //���ٶȵýǶ� ��λת��
    angle_acc=(angle_acc+angle_re_last[0]+angle_re_last[1])/3; //�˲�
    angle_gyro=w_y_re_use*gyro_unit_conversion ;

    angle_re=angle_k1*angle_acc+(1-angle_k1)*(angle_re+angle_gyro*0.002);       //0.002Ϊpitʱ�䣡������


    /***************************************************************************************************************************************/
    if(All_State==0){
    //�ǽǶ�
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
    encoder_speedleft = -gpt12_get(GPT12_T5);//���������ȡ
    encoder_speedright = gpt12_get(GPT12_T4);//�ұ�������ȡ
    gpt12_clear(GPT12_T5);                   //�����������
    gpt12_clear(GPT12_T4);                   //�ұ���������
    all_lenth = all_lenth + (encoder_speedleft + encoder_speedright)/2;


    encoder_speedleft /= routine*1.0;
    encoder_speedright /= routine*1.0;//��һ��

    float Left_Err = Target_Left_Speed - encoder_speedleft;
    float Right_Err = Target_Right_Speed - encoder_speedright;//���

    if(abs(Left_Err)<Target_Left_Speed/2)//���ַ���
    Left_inter += Left_Err;
    if(abs(Right_Err)<Target_Right_Speed/2)//���ַ���
    Right_inter += Right_Err;

    Left_inter = Left_inter<-10000?-10000:Left_inter;
    Right_inter = Right_inter<-10000?-10000:Right_inter;
    Left_inter = Left_inter>10000?10000:Left_inter;
    Right_inter = Right_inter>10000?10000:Right_inter;//�����޷�

    int MotorOutLeft=0,MotorOutRight=0;
    MotorOutLeft =  Left_Kp * Left_Err + Left_Ki * Left_inter;
    MotorOutRight =  Right_Kp * Right_Err + Right_Ki * Right_inter;//λ��ʽpid

    MotorOutLeft = MotorOutLeft<-6000?-6000:MotorOutLeft;
    MotorOutRight = MotorOutRight<-6000?-6000:MotorOutRight;
    MotorOutLeft = MotorOutLeft>6000?6000:MotorOutLeft;
    MotorOutRight = MotorOutRight>6000?6000:MotorOutRight;//pwm�޷�

    if((MotorOutLeft>=0))//����
        {
            MotorOutLeft = MotorOutLeft>10000?10000:MotorOutLeft;//��ĸΪ10000��FTM.H��
            pwm_duty(MOTOR1_PWM, (int)MotorOutLeft); //�����ת
            pwm_duty(MOTOR2_PWM, 0); //�����ת
        }
        else if((MotorOutLeft<0))
        {
            MotorOutLeft = MotorOutLeft<-10000?-10000:MotorOutLeft;//��ĸΪ10000��FTM.H��
            pwm_duty(MOTOR1_PWM, 0); //�����ת
            pwm_duty(MOTOR2_PWM, (int)(-MotorOutLeft)); //�����ת
        }
        if((MotorOutRight>=0))//�ҵ��
        {
            MotorOutRight = MotorOutRight>10000?10000:MotorOutRight;//��ĸΪ10000��FTM.H��
            pwm_duty(MOTOR3_PWM, (int)MotorOutRight); //�����ת
            pwm_duty(MOTOR4_PWM, 0); //�����ת
        }
        else if((MotorOutRight<0))
        {
            MotorOutRight = MotorOutRight<-10000?-10000:MotorOutRight;//��ĸΪ10000��FTM.H��
            pwm_duty(MOTOR3_PWM, 0); //�����ת
            pwm_duty(MOTOR4_PWM, (int)(-MotorOutRight)); //�����ת
        }


}
///***************************************************************************************
//�� �� �� :void Pingmuceshi(void)
//��     ��  :��������(ͼ����Ϣ��������)
//��     ��  :��
//�� ��  ֵ :��
//�������� :��
//˵�������򰴼�����IPS��Ļ
//***************************************************************************************/
void Pingmuceshi(void)
{
    //���水��״̬
    keyTip_last_status = keyTip_status;
    keyUp_last_status = keyUp_status;
    keyDown_last_status = keyDown_status;
    keyLeft_last_status = keyLeft_status;
    keyRight_last_status = keyRight_status;
    //��ȡ��ǰ����״̬
    keyTip_status = gpio_get(KeyTip);
    keyUp_status = gpio_get(KeyUp);
    keyDown_status = gpio_get(KeyDown);
    keyLeft_status = gpio_get(KeyLeft);
    keyRight_status = gpio_get(KeyRight);
    //��⵽��������֮��  ���ſ���λ��־λ
    if(keyTip_status && !keyTip_last_status)    keyTip_flag = 0;
    if(keyUp_status && !keyUp_last_status)    keyUp_flag = 0;
    if(keyDown_status && !keyDown_last_status)    keyDown_flag = 0;
    if(keyLeft_status && !keyLeft_last_status)    keyLeft_flag = 0;
    if(keyRight_status && !keyRight_last_status)    keyRight_flag = 0;
    //��־λ��λ֮�󣬿���ʹ�ñ�־λִ���Լ���Ҫ�����¼�
//    if(!keyTip_flag)
//    {
//        keyTip_flag = 1;//ʹ�ð���֮��Ӧ�������־λ
//        testTip++;
//    }
//
//    if(!keyUp_flag)
//    {
//        keyUp_flag = 1;//ʹ�ð���֮��Ӧ�������־λ
//        testUp++;
//    }
//
//    if(!keyDown_flag)
//    {
//        keyDown_flag = 1;//ʹ�ð���֮��Ӧ�������־λ
//        testDown++;
//    }
//
//    if(!keyLeft_flag)
//    {
//        keyLeft_flag = 1;//ʹ�ð���֮��Ӧ�������־λ
//        pwm_duty(S_MOTOR_PIN,Duty); //�������Ҽ���ʱ��
//        Duty-=10;
//    }
//    if(!keyRight_flag)
//    {
//        keyRight_flag = 1;//ʹ�ð���֮��Ӧ�������־λ
//        pwm_duty(S_MOTOR_PIN,Duty); //�������Ҽ���ʱ��
//        Duty+=10;
//    }


//    ips200_showstr(0,8,"Duty:");
//    ips200_showint32(68,8,Duty,3);
//    ips200_showstr(0,9,"Error:");
//    ips200_showint32(68,9,Duty-SERVOMIDDLE,3);
    }
/***************************************************************************************
�� �� �� :short GetOSTU (unsigned char tmImage[120][188])
��    �� :�Ҷ�ֵ��򷨴���
��    �� :��ѡȡ��ͼ��
�� �� ֵ :�����ֵ
�������� :��
˵����
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
    float OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // ��䷽��;
    signed int MinValue, MaxValue;
    signed int Threshold = 0;
    unsigned char HistoGram[256];              //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //��ʼ���Ҷ�ֱ��ͼ

    for (j = 0; j < 120; j++)
    {
        for (i = 0; i < 188; i++)
        {
            HistoGram[tmImage[j][i]]++; //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
        return MaxValue;         // ͼ����ֻ��һ����ɫ
    if (MinValue + 1 == MaxValue)
        return MinValue;        // ͼ����ֻ�ж�����ɫ

    for (j = MinValue; j <= MaxValue; j++)
        Amount += HistoGram[j];        //  ��������

    Pixelshortegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        Pixelshortegral += HistoGram[j] * j;        //�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];     //ǰ�����ص���
        PixelFore = Amount - PixelBack;           //�������ص���
        OmegaBack = (float) PixelBack / Amount;   //ǰ�����ذٷֱ�
        OmegaFore = (float) PixelFore / Amount;   //�������ذٷֱ�
        PixelshortegralBack += HistoGram[j] * j;  //ǰ���Ҷ�ֵ
        PixelshortegralFore = Pixelshortegral - PixelshortegralBack;  //�����Ҷ�ֵ
        MicroBack = (float) PixelshortegralBack / PixelBack;   //ǰ���ҶȰٷֱ�
        MicroFore = (float) PixelshortegralFore / PixelFore;   //�����ҶȰٷֱ�
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);   //������䷽��
        if (Sigma > SigmaB)                    //����������䷽��g //�ҳ������䷽���Լ���Ӧ����ֵ
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold;                        //���������ֵ;
}
/***************************************************************************************
�� �� �� :void Get_Bin_Image (unsigned char mode)
��    �� :��ֵ��ͼ��
��    �� :��ѡȡ��ͼ��
�� �� ֵ :��
�������� :��
˵����
***************************************************************************************/

void Get_Bin_Image (int threshold)
{
    unsigned short i = 0, j = 0;
//    unsigned short Threshold = 0;


//        Threshold =mt9v03x_image[0];
    /* ��ֵ�� */
    for (i = 0; i < 120; i++)
    {
        for (j = 0; j < 188; j++)
        {
            if ( mt9v03x_image[i][j] > threshold) //��ֵԽ����ʾ������Խ�࣬��ǳ��ͼ��Ҳ����ʾ����
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

// ��⳵������߾���
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

// ��⵹���
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

// ��ⴹֱ����
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

// ���ͣ����
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

// �������ŵ�
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

// state 0 ûɶ��
// state 1 ��⵹���
// state 2 �̶��򽻵���
// state 3 ΢�����
// state 4 ͣ5��
// state 5 ֱ�߳���
// state 6 �̶���ų���
// state 7 ѭ��
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
                gpio_set(BEEP_PIN,1);//�򿪷�����
                Target_Left_Speed = 0;
                Target_Right_Speed = 0;
            }
            if (tc1 && js >= 333)
            {
                state = 2;
                gpio_set(BEEP_PIN,0);//�رշ�����
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
                gpio_set(BEEP_PIN,1);//�򿪷�����
                js = 0;
            }
        } else if (state == 4)
        {
            if (js >= 333)
            {
                gpio_set(BEEP_PIN,0);//�رշ�����
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
            // ����
            if (chuku_angle >= 80)
            {
                state = 7; // 7�����
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
                        gpio_set(BEEP_PIN,1);//�򿪷�����
                        count+=3;
                        if(count>300){
                            count=0;
                            gpio_set(BEEP_PIN,0);//�رշ�����
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
                        gpio_set(BEEP_PIN,1);//�򿪷�����
                        count+=3;
                        if(count>100)
                            gpio_set(BEEP_PIN,0);//�رշ�����
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
//��ͷ����
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

//��ͷ����
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

//��ͷָ��ֵ��һ
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

//��ͷָ��ֵ��һ
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
