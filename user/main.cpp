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


Ads7816 adc(&PB3,&PB7,&PB4);//PB4是jtag引脚。不要用20pin的jlink去调试程序  Ads7816(Gpio *cs,Gpio *clk,Gpio *din)
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
//函数申明
void _485_tx_mode();
void _485_rx_mode();

void _485_send_ch(uint8_t ch);
void adc_read_ch(uint8_t ch);

Pwm pwm1(&PB6);//初始化PWM接口配置模拟开关地址，产生方波激励信号
Ds18b20 ds(&PA4);//初始化温度传感器

Flash flash;
//void t2it()
//{
//    PA0.toggle();
//}
//   Y = -0.31382 + 0.00061*X1;
double data1[5][2] = {
//    X      Y
    {1260 , 0.462},
    {2095, 0.974},
    {2750, 1.365},
    {3446, 1.805},
    {4027, 2.16},
};
double data2[5][2] = {
//    X      Y
    {1260 , 0.462},
    {2095, 0.974},
    {2750, 1.365},
    {3446, 1.805},
    {4027, 2.16},
};
double data3[5][2] = {
//    X      Y
    {1260 , 0.462},
    {2095, 0.974},
    {2750, 1.365},
    {3446, 1.805},
    {4027, 2.16},
};
double data4[5][2] = {
//    X      Y
    {1260 , 0.462},
    {2095, 0.974},
    {2750, 1.365},
    {3446, 1.805},
    {4027, 2.16},
};

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
void display(double *dat, double *Answer, double *SquarePoor, int rows, int cols)
{
    double v, *p;
    int i, j;
    uart1.printf("回归方程式:    Y = %.5lf", Answer[0]);
    for (i = 1; i < cols; i ++)
        uart1.printf(" + %.5lf*X%d\r\n", Answer[i], i);
    uart1.printf("回归显著性检验: \r\n");
    uart1.printf("回归平方和：%12.4lf  回归方差：%12.4lf \r\n", SquarePoor[0], SquarePoor[2]);
    uart1.printf("剩余平方和：%12.4lf  剩余方差：%12.4lf \r\n", SquarePoor[1], SquarePoor[3]);
    uart1.printf("离差平方和：%12.4lf  标准误差：%12.4lf \r\n", SquarePoor[0] + SquarePoor[1], sqrt(SquarePoor[3]));
    uart1.printf("F   检  验：%12.4lf  相关系数：%12.4lf \r\n", SquarePoor[2] /SquarePoor[3],
           sqrt(SquarePoor[0] / (SquarePoor[0] + SquarePoor[1])));
    uart1.printf("剩余分析: \r\n");
    uart1.printf("      观察值      估计值      剩余值    剩余平方 \r\n");
    for (i = 0, p = dat; i < rows; i ++, p ++)
    {
        v = Answer[0];
        for (j = 1; j < cols; j ++, p ++)
            v += *p * Answer[j];
        uart1.printf("%12.2lf%12.2lf%12.2lf%12.2lf \r\n", *p, v, *p - v, (*p - v) * (*p - v));
    }
}

Xdata ch1_ratio,ch1_off,ch2_ratio,ch2_off,ch3_ratio,ch3_off,ch4_ratio,ch4_off;

//*************初始化设置*************//
void setup()
{
    ebox_init();
    uart1.begin(115200);//设置串口波特率Uart uart1(USART1, &PA9, &PA10);  PA9位TX  PA10位RX  串口1
    adc.begin();       
//    timer2.begin(100); //设置ADC采样率
//    timer2.attach(t2it);
//    timer2.interrupt(ENABLE);
//    timer2.start();    
    //485管脚初始化
    PA11.mode(OUTPUT_PP);  //485驱动收发使能控制管脚  
    //DS18B20检测口
    PA4.mode(OUTPUT_PP);   //
   //状态指示灯初始化
    PC13.mode(OUTPUT_PP);
    PC14.mode(OUTPUT_PP);   
    //模拟开关打开
    EN.mode(OUTPUT_PP);
    A0.mode(OUTPUT_PP);
    A1.mode(OUTPUT_PP);
    //测试管脚
    PA0.mode(OUTPUT_PP);
    PA1.mode(OUTPUT_PP);

    //设置漏电流激励脉冲信号，频率100HZ,占空比50%。经过放大输出±7.5V方波驱动酸露点传感器
    pwm1.begin(100, 500);
    pwm1.set_oc_polarity(1);//set output polarity after compare

//    uart1.printf("core:%d\r\n",cpu.clock.core);
//    uart1.printf("core:%d\r\n",cpu.clock.core);
//    uart1.printf("hclk:%d\r\n",cpu.clock.hclk);
//    uart1.printf("pclk1:%d\r\n",cpu.clock.pclk1);
//    uart1.printf("pclk2:%d\r\n",cpu.clock.pclk2);
    linear_regression((double*)data2,5,&answer_ch2[0],&answer_ch2[1],&SquarePoor[0]);
    ch2_ratio.val = answer_ch2[1];  //双精度与CHAR转换
    ch2_off.val = answer_ch2[0];
    linear_regression((double*)data1,5,&answer_ch1[0],&answer_ch1[1],&SquarePoor[0]);
    ch1_ratio.val = answer_ch1[1];
    ch1_off.val = answer_ch1[0];
    linear_regression((double*)data3,5,&answer_ch1[0],&answer_ch1[1],&SquarePoor[0]);
    ch3_ratio.val = answer_ch1[1];
    ch3_off.val = answer_ch1[0];
    linear_regression((double*)data4,5,&answer_ch1[0],&answer_ch1[1],&SquarePoor[0]);
    ch4_ratio.val = answer_ch1[1];
    ch4_off.val = answer_ch1[0];



//    
//    flash.write(0x800E000,ch1_ratio.byte,16);
//    flash.write(0x800E100,ch1_off.byte,16);
    flash.read(0x800E000,ch1_ratio.byte,16);
    flash.read(0x800E010,ch1_off.byte,16);   
    flash.read(0x800E020,ch2_ratio.byte,16);
    flash.read(0x800E030,ch2_off.byte,16);
    flash.read(0x800E040,ch3_ratio.byte,16);
    flash.read(0x800E050,ch3_off.byte,16);
    flash.read(0x800E060,ch4_ratio.byte,16);
    flash.read(0x800E070,ch4_off.byte,16);
//    display((double*)data1,answer,&SquarePoor[0],12,2);
//    while(1);
}

double y;
int main(void)
{

    setup();
    last_updata_time = millis();
    while(1)
    {
        for(int i = 0; i < 4; i++)
            adc_read_ch(i);
        detector_urrent_ch1 = voltage_ch1/2/0.823;//825欧姆采样电阻，1.98为运放放大倍数，单位uA，酸露漏电流,具体情况需要校准，实际测试A=2，电阻R4=823
//        detector_temperature_ch2 = voltage_ch2/1020;//运放放大倍数1020，根据实际情况调整，酸露传感器温度S热电偶,单位//mv，需要制表查询计算
        flue_temperature_ch3 = (voltage_ch3-19)/101;//运放放大倍数101,烟气温度K热电偶,单位//mv，需要制表查询计算，减去零点偏置19mV,偏置可以通过输入0电压获得，101由A=1+R25/R37，实际情况需要校准
        //零点偏置和K系数。
//        ambient_temperature_ch4 = voltage_ch4/3.56/10;//运放放大倍数3.56,环境温度检测LM35，10mV/℃，单位℃，需要零点补偿校准
        
//        detector_urrent_ch1 = voltage_ch1;///(0.823*2);//825欧姆采样电阻，1.98为运放放大倍数，单位uA，酸露漏电流
        detector_temperature_ch2 = voltage_ch2;///1020;//运放放大倍数1020，根据实际情况调整，酸露传感器温度S热电偶
//        flue_temperature_ch3 = voltage_ch3;///101;//运放放大倍数101,烟气温度K热电偶
//        ambient_temperature_ch4 = voltage_ch4;///3.56;//运放放大倍数3.56,环境温度检测LM35，10mV/℃      
        if(millis() - last_updata_time >= 1000) //发送485更新速率时间为500ms每次
        {
           last_updata_time = millis();
//            _485_tx_mode();
//            uart1.printf("val_ch1 = 0x%04x\r\nval_ch2 = 0x%04x\r\nval_ch3 = 0x%04x\r\n val_ch4 = 0x%04x\r\n",val_ch1,val_ch2,val_ch3,val_ch4);
//            _485_rx_mode();           
            _485_tx_mode();
//            uart1.printf("%d\r\n",millis());
//            uart1.printf("%4.3f\r\n ",voltage_ch2); 
//            y = -0.31382 + 0.00061*val_ch2;   
            y = ch2_off.val + ch2_ratio.val*val_ch2;  
//            y2 = answer2[0] + answer2[1]*val_ch2;               
          uart1.printf("%04d\t%0.4f\r\n",val_ch2,y);            
//            uart1.printf("detector_urrent_ch1 = %4.3f\r\n detector_temperature_ch2 = %4.3f\r\n flue_temperature_ch3 = %4.3f\r\n ambient_temperature_ch4 = %4.1f\r\n",detector_urrent_ch1,detector_temperature_ch2,flue_temperature_ch3,ambient_temperature_ch4);
//           uart1.printf("voltage_ch1 = %4.3f\r\n voltage_ch2 = %4.3f\r\n voltage_ch3 = %4.3f\r\n voltage_ch4 = %4.1f\r\n",voltage_ch1,voltage_ch2,voltage_ch3,voltage_ch4);
            _485_rx_mode();
            LED0.toggle();//串口发送状态指示灯，D4
            PA1.toggle();   //测试口        
        }
//          delay_ms(1000);             
    }
}

//485输出
void _485_tx_mode()//如果不输出，将引脚设置翻转
{
    PA11.set();
}
void _485_rx_mode()
{
    PA11.reset();  
}
//通道模拟开关选择
void adc_read_ch(uint8_t ch)
{
    LED1.toggle();//ADC采甲刺指示灯
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
