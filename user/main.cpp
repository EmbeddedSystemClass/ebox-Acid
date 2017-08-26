/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2016/08/14

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */


#include "ebox.h"
#include "ads7816.h"
#include "ds18b20.h"
//Timer timer2(TIM2);


Ads7816 adc(&PB3,&PB7,&PB4);//PB4��jtag���š���Ҫ��20pin��jlinkȥ���Գ���  Ads7816(Gpio *cs,Gpio *clk,Gpio *din)
uint16_t val;
float voltage;
uint8_t adc_ch,_485_ch;
uint64_t last_updata_time;
uint16_t   val_ch1,val_ch2,val_ch3,val_ch4;
float   voltage_ch1,voltage_ch2,voltage_ch3,voltage_ch4 ;
float   detector_urrent_ch1,detector_temperature_ch2,flue_temperature_ch3,ambient_temperature_ch4 ;
# define EN  PB5
# define A0  PB8
# define A1  PB9
# define LED0 PC14  //D4
# define LED1 PC13 //D11
# define PWM  PB6
//��������
void _485_tx_mode();
void _485_rx_mode();

void _485_send_ch(uint8_t ch);
void adc_read_ch(uint8_t ch);

Pwm pwm1(&PB6);//��ʼ��PWM�ӿ�����ģ�⿪�ص�ַ���������������ź�
Ds18b20 ds(&PA4);//��ʼ���¶ȴ�����

Flash flash;

//void t2it()
//{
//    PA0.toggle();
//}

//************************���Իع�������ʾ����*********************//   
//FLUKE 5502AУ׼��������С55uA����С��׼��������������û�в�����ʵ��Ӧ���д��ڳ��ߴ�������µ�����ġ�
double data1[11][2] = {
//    X      Y
    {163 , 60},//60uA
    {270, 100},//100
    {542, 200},//200
    {811, 300},//300
    {1347, 500},//500
    {1887, 700},//700
    {2425, 900},//900
    {2964, 1100},//1100
    {3503, 1300},//1300
    {3772, 1400},//1400
    {4042, 1500},//1500
};
//S�ȵ�ż�ο�FLUK 5502AУ׼�����0���϶ȣ�����ģ�����ʱ��Ҫ����,ѹ��㶨������ʵ���������
//������У׼ϵ��
//double data2[12][2] = {
////    X      Y
//    {1025, 0.328},//60��
//    {1138, 0.390},//70
//    {1485, 0.608},//100
//    {1724, 0.758},//120
//    {1978, 0.909},//140
//    {2237, 1.068},//160
//    {2507, 1.231},//180
//    {2780, 1.397},//200
//    {3344, 1.742},//240
//    {3635, 1.924},//260
//    {3928, 2.099},//280
//    {4024, 2.15}//285  290����
//};
//����У׼ϵ����У׼�������ñ��б䶯�����Զ�������������ñ�ʱ��ȡ�����������ݣ������ֿܷ���ȡ��������ƫ��
double data2[12][2] = {
//    X      Y
    {1025, 0.319},//60��
    {1142, 0.391},//70
    {1490, 0.603},//100
    {1740, 0.754},//120
    {1989, 0.907},//140
    {2252, 1.068},//160
    {2524, 1.233},//180
    {2795, 1.400},//200
    {3360, 1.744},//240
    {3647, 1.921},//260
    {3946, 2.103},//280
    {4024, 2.15}//285  290���ϱ���
};
double data3[14][2] = {
//    X      Y
    {98 , 0.396},//10   �� 
    {199 , 1.006},//25
    {369 , 2.035},//50
    {711, 4.108},//100
    {1050, 6.154},//150
    {1380, 8.157},//200
    {1713, 10.167},//250
    {2051, 12.219},//300
    {2395, 14.298},//350
    {2743, 16.396},//400
    {3092, 18.513},//450
    {3444, 20.643},//500
    {3797, 22.775},//550
    {4079, 24.487},//590  595���ϱ���
};
double data4[14][2] = {
//    X      Y
    {61 , 10.002},//10mv
    {292, 49.995},//50
    {582, 100},//100
    {871, 150},//150
    {1162, 200},//200
    {1451 , 250},//250
    {1741, 300},//300
    {2031, 350},//350
    {2322, 400},//400
    {2901, 500},//500
    {3481 , 600},//600
    {3771, 650},//650
    {3943, 680},//680
    {4061, 700}//700
};
//���ݸ�ʽת������
typedef union
{
    double val;
    uint8_t byte[8];
}Xdata;  

 
double answer_ch1[2];
double answer_ch2[2];
double answer_ch3[2];
double answer_ch4[2];
double SquarePoor[4];
Xdata ch1_ratio,ch1_off,ch2_ratio,ch2_off,ch3_ratio,ch3_off,ch4_ratio,ch4_off;
void display(double *dat, double *Answer, double *SquarePoor, int rows, int cols)
{
    double v, *p;
    int i, j;
    uart1.printf("�ع鷽��ʽ:    Y = %.5lf", Answer[0]);
    for (i = 1; i < cols; i ++)
        uart1.printf(" + %.5lf*X%d\r\n", Answer[i], i);
    uart1.printf("�ع������Լ���: \r\n");
    uart1.printf("�ع�ƽ���ͣ�%12.4lf  �ع鷽�%12.4lf \r\n", SquarePoor[0], SquarePoor[2]);
    uart1.printf("ʣ��ƽ���ͣ�%12.4lf  ʣ�෽�%12.4lf \r\n", SquarePoor[1], SquarePoor[3]);
    uart1.printf("���ƽ���ͣ�%12.4lf  ��׼��%12.4lf \r\n", SquarePoor[0] + SquarePoor[1], sqrt(SquarePoor[3]));
    uart1.printf("F   ��  �飺%12.4lf  ���ϵ����%12.4lf \r\n", SquarePoor[2] /SquarePoor[3],
           sqrt(SquarePoor[0] / (SquarePoor[0] + SquarePoor[1])));
    uart1.printf("ʣ�����: \r\n");
    uart1.printf("      �۲�ֵ      ����ֵ      ʣ��ֵ    ʣ��ƽ�� \r\n");
    for (i = 0, p = dat; i < rows; i ++, p ++)
    {
        v = Answer[0];
        for (j = 1; j < cols; j ++, p ++)
            v += *p * Answer[j];
        uart1.printf("%12.2lf%12.2lf%12.2lf%12.2lf \r\n", *p, v, *p - v, (*p - v) * (*p - v));
    }
}



//*************��ʼ������*************//
void setup()
{
    ebox_init();
    uart1.begin(115200);//���ô��ڲ�����Uart uart1(USART1, &PA9, &PA10);  PA9λTX  PA10λRX  ����1
    adc.begin();       
//    timer2.begin(100); //����ADC������
//    timer2.attach(t2it);
//    timer2.interrupt(ENABLE);
//    timer2.start();    
    //485�ܽų�ʼ��
    PA11.mode(OUTPUT_PP);  //485�����շ�ʹ�ܿ��ƹܽ�  
    //DS18B20����
    PA4.mode(OUTPUT_PP);   //
   //״ָ̬ʾ�Ƴ�ʼ��
    PC13.mode(OUTPUT_PP);
    PC14.mode(OUTPUT_PP);   
    //ģ�⿪�ش�
    EN.mode(OUTPUT_PP);
    A0.mode(OUTPUT_PP);
    A1.mode(OUTPUT_PP);
    //���Թܽ�
    PA0.mode(OUTPUT_PP);
    PA1.mode(OUTPUT_PP);

    //����©�������������źţ�Ƶ��100HZ,ռ�ձ�50%�������Ŵ������7.5V����������¶�㴫����
    pwm1.begin(100, 500);
    pwm1.set_oc_polarity(1);//set output polarity after compare

//    uart1.printf("core:%d\r\n",cpu.clock.core);
//    uart1.printf("core:%d\r\n",cpu.clock.core);
//    uart1.printf("hclk:%d\r\n",cpu.clock.hclk);
//    uart1.printf("pclk1:%d\r\n",cpu.clock.pclk1);
//    uart1.printf("pclk2:%d\r\n",cpu.clock.pclk2);
//************���Իع���㺯��****************************//  
    linear_regression((double*)data1,11,&answer_ch1[0],&answer_ch1[1],&SquarePoor[0]);
    ch1_ratio.val = answer_ch1[1];  //˫������CHARת��
    ch1_off.val = answer_ch1[0];
    
    linear_regression((double*)data2,12,&answer_ch2[0],&answer_ch2[1],&SquarePoor[0]);
    ch2_ratio.val = answer_ch2[1];//˫������CHARת��
    ch2_off.val = answer_ch2[0];
    
    linear_regression((double*)data3,14,&answer_ch3[0],&answer_ch3[1],&SquarePoor[0]);
    ch3_ratio.val = answer_ch3[1];//˫������CHARת��
    ch3_off.val = answer_ch3[0];
    
    linear_regression((double*)data4,14,&answer_ch4[0],&answer_ch4[1],&SquarePoor[0]);
    ch4_ratio.val = answer_ch4[1];
    ch4_off.val = answer_ch4[0];

//***************���ݶ�дFLASH����*****************//

//    flash.write(0x800E000,ch1_ratio.byte,16);
//    flash.write(0x800E010,ch1_off.byte,16);    
//    flash.write(0x800E020,ch2_ratio.byte,16);
//    flash.write(0x800E030,ch2_off.byte,16);
//    flash.write(0x800E040,ch3_ratio.byte,16);
//    flash.write(0x800E050,ch3_off.byte,16);  
//    flash.write(0x800E060,ch4_ratio.byte,16);
//    flash.write(0x800E070,ch4_off.byte,16);  

    flash.read(0x800E000,ch1_ratio.byte,16);//Y = -0.30744 + 0.00061*X1
    flash.read(0x800E010,ch1_off.byte,16);   
    flash.read(0x800E020,ch2_ratio.byte,16);//Y = -0.30518 + 0.00061*X1  ������㻹ԭ����// ԭ����  Y = -0.31382 + 0.00061*X1;
    flash.read(0x800E030,ch2_off.byte,16);
    flash.read(0x800E040,ch3_ratio.byte,16);// Y = -0.19654 + 0.00605*X1
    flash.read(0x800E050,ch3_off.byte,16);
    flash.read(0x800E060,ch4_ratio.byte,16);//Y = -0.38432 + 0.17249*X1
    flash.read(0x800E070,ch4_off.byte,16);
//*************������Լ��㺯��************************************************//
//    display((double*)data1,answer_ch1,&SquarePoor[0],11,2);
//    display((double*)data2,answer_ch2,&SquarePoor[0],12,2);
//    display((double*)data3,answer_ch3,&SquarePoor[0],14,2);
//    display((double*)data4,answer_ch4,&SquarePoor[0],14,2);
//    while(1);
}

double I_voltage_ch1,S_voltage_ch2,K_voltage_ch3,LM35_voltage_ch4;
int main(void)
{

    setup();
    last_updata_time = millis();
    while(1)
    {
        //��ȡADCͨ�������ݣ�ͨ��ģ�⿪���л���50HZƵ��
        for(int i = 0; i < 4; i++)
        { 
            adc_read_ch(i);
        }
//**************���㻹ԭϵ��******************//       
        detector_urrent_ch1 = voltage_ch1/2/0.823;//825ŷķ�������裬1.98Ϊ�˷ŷŴ�������λuA����¶©����,���������ҪУ׼��ʵ�ʲ���A=2������R4=823
        detector_temperature_ch2 = voltage_ch2/1020;//�˷ŷŴ���1020������ʵ�������������¶�������¶�S�ȵ�ż,��λ//mv����Ҫ�Ʊ��ѯ����       
        flue_temperature_ch3 = (voltage_ch3-19)/101;//�˷ŷŴ���101,�����¶�K�ȵ�ż,��λ//mv����Ҫ�Ʊ��ѯ���㣬��ȥ���ƫ��19mV,ƫ�ÿ���ͨ��
                                                        //����0��ѹ��ã�101��A=1+R25/R37��ʵ�������ҪУ׼���ƫ�ú�Kϵ����
        ambient_temperature_ch4 = voltage_ch4/3.56/10;//�˷ŷŴ���3.56,�����¶ȼ��LM35��10mV/�棬��λ�棬��Ҫ��㲹��У׼
        
//        detector_urrent_ch1 = voltage_ch1;//
//        detector_temperature_ch2 = voltage_ch2;//
////      flue_temperature_ch3 = voltage_ch3;//
//        ambient_temperature_ch4 = voltage_ch4;//    
        I_voltage_ch1 = ch1_off.val + ch1_ratio.val*val_ch1;
        S_voltage_ch2 = ch2_off.val + ch2_ratio.val*val_ch2; 
        K_voltage_ch3 = ch3_off.val + ch3_ratio.val*val_ch3;  
        LM35_voltage_ch4 = ch4_off.val + ch4_ratio.val*val_ch4;
//*********���ڷ��������������ص�Ƭ���ͽ������ص�Ƭ����������ָ��*********//
        if(millis() - last_updata_time >= 1000) //����485��������ʱ��Ϊ1000msÿ��
        {
            last_updata_time = millis();
//          _485_tx_mode();
//          uart1.printf("val_ch1 = 0x%04x\r\nval_ch2 = 0x%04x\r\nval_ch3 = 0x%04x\r\n val_ch4 = 0x%04x\r\n",val_ch1,val_ch2,val_ch3,val_ch4);
//          _485_rx_mode();           
            _485_tx_mode(); 
//            uart1.printf("%04d\t%0.4f\t%0.4f\r\n",val_ch2,S_voltage_ch2,K_voltage_ch3);     
//            uart1.printf("%04d\t%0.4f\t%0.4f\r\n",val_ch4,LM35_voltage_ch4,ambient_temperature_ch4);    
//          uart1.printf("%04d\r\t %4.3f\r\n",val_ch3,flue_temperature_ch3);               
          uart1.printf("I_voltage_ch1 = %4.3f\t S_voltage_ch2 = %4.3f\t K_voltage_ch3 = %4.3f\t LM35_voltage_ch4 = %4.1f\r\n",I_voltage_ch1,S_voltage_ch2,K_voltage_ch3,LM35_voltage_ch4);
//          uart1.printf("detector_urrent_ch1 = %4.3f\r\n detector_temperature_ch2 = %4.3f\r\n flue_temperature_ch3 = %4.3f\r\n ambient_temperature_ch4 = %4.1f\r\n",detector_urrent_ch1,detector_temperature_ch2,flue_temperature_ch3,ambient_temperature_ch4);
//          uart1.printf("voltage_ch1 = %4.3f\r\n voltage_ch2 = %4.3f\r\n voltage_ch3 = %4.3f\r\n voltage_ch4 = %4.1f\r\n",voltage_ch1,voltage_ch2,voltage_ch3,voltage_ch4);
            _485_rx_mode();
            LED0.toggle();//���ڷ���״ָ̬ʾ�ƣ�D4
            PA1.toggle();   //���Կ�        
        }
//          delay_ms(1000);             
    }
}

//485���
void _485_tx_mode()//�������������������÷�ת
{
    PA11.set();
}
void _485_rx_mode()
{
    PA11.reset();  
}
//ͨ��ģ�⿪��ѡ��
void adc_read_ch(uint8_t ch)
{
    LED1.toggle();//ADC�ɼ״̬�ָʾ��
    PA0.toggle();
    switch(ch)
    {
        case 0: 
            EN.set();
            A0.reset(); 
            A1.reset();
            delay_ms(10);
            val_ch1 = adc.read();
            voltage_ch1 = adc.read_voltage(); //mv  
            break;
        case 1:
            EN.set();
            A0.set(); 
            A1.reset();
            delay_ms(10);
            val_ch2 = adc.read();
            voltage_ch2 = adc.read_voltage();//mv
            break;
         case 2:
            EN.set();
            A0.reset(); 
            A1.set();
            delay_ms(10);
            val_ch3 = adc.read();
            voltage_ch3 = adc.read_voltage();//mv
            break;
         case 3:
            EN.set();
            A0.set(); 
            A1.set();
            delay_ms(10);
            val_ch4 = adc.read();
            voltage_ch4 = adc.read_voltage();//mv
            break;
        default:
            break;
    }    
}
