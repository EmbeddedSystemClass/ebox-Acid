/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2017/09/17

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */


#include "ebox.h"
#include "ads7816.h"
#include "ds18b20.h"
//Timer timer2(TIM2);
#include "../core/math/math_misc.h"


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
const float k_600_table[] = 
{
//0,		1,			2,			3,			4,			5,			6,			7,			8,			9
0,		    0.039,		0.079,		0.119,		0.158,		0.198,		0.238,		0.277,		0.317,		0.357,		//0
0.397,		0.437,		0.477,		0.517,		0.557,		0.597,		0.637,		0.677,		0.718,		0.758,		//10
0.798,		0.838,		0.879,		0.919,		0.96,		    1,		1.041,		1.081,		1.122,		1.136,		//20
1.203,		1.244,		1.285,		1.326,		1.366,		1.407,		1.448,		1.489,		1.53,		1.571,		//30
1.612,		1.653,		1.694,		1.735,		1.776,		1.817,		1.858,		1.899,		1.941,		1.982,		//40
2.023,		2.064,		2.106,		2.147,		2.188,		2.23,		2.271,		2.312,		2.354,		2.395,		//50
2.436,		2.478,		2.519,		2.561,		2.602,		2.644,		2.685,		2.727,		2.768,		2.81,		//60
2.851,		2.893,		2.934,		2.976,		3.017,		3.059,		3.1,		3.142,		3.184,		3.225,		//70
3.267,		3.308,		3.35,		3.391,		3.433,		3.474,		3.516,		3.557,		3.599,		3.64,		//80
3.682,		3.723,		3.765,		3.806,		3.848,		3.889,		3.931,		3.972,		4.013,		4.055,		//90
4.096,		4.138,		4.179,		4.22,		4.262,		4.303,		4.344,		4.385,		4.427,		4.468,		//100
4.509,		4.55,		4.591,		4.633,		4.674,		4.715,		4.756,		4.797,		4.838,		4.879,		//110
4.92,		4.961,		5.002,		5.043,		5.084,		5.124,		5.165,		5.206,		5.247,		5.288,		//120
5.328,		5.369,		5.41,		5.45,		5.491,		5.532,		5.572,		5.613,		5.653,		5.694,		//130
5.735,		5.775,		5.815,		5.856,		5.896,		5.937,		5.977,		6.017,		6.058,		6.098,		//140
6.138,		6.179,		6.219,		6.259,		6.299,		6.339,		6.38,		6.42,		6.46,		6.5,		//150
6.54,		6.58,		6.62,		6.66,		6.701,		6.741,		6.781,		6.821,		6.861,		6.901,		//160
6.941,		6.981,		7.021,		7.06,		7.1,		7.14,		7.18,		7.22,		7.26,		7.3,		//170
7.34,		7.38,		7.42,		7.46,		7.5,		7.54,		7.579,		7.619,		7.659,		7.699,		//180
7.739,		7.779,		7.819,		7.859,		7.899,		7.939,		7.979,		8.019,		8.059,		8.099,		//190
8.138,		8.178,		8.218,		8.258,		8.298,		8.338,		8.378,		8.418,		8.458,		8.499,		//200
8.539,		8.579,		8.619,		8.659,		8.699,		8.739,		8.779,		8.819,		8.86,		8.9,		//210
8.94,		8.98,		9.02,		9.061,		9.101,		9.141,		9.181,		9.222,		9.262,		9.302,		//220
9.343,		9.383,		9.423,		9.464,		9.504,		9.545,		9.585,		9.626,		9.666,		9.707,		//230
9.747,		9.788,		9.828,		9.869,		9.909,		9.95,		9.991,		10.031,		10.072,		10.113,		//240
10.153,		10.194,		10.235,		10.276,		10.316,		10.357,		10.398,		10.439,		10.48,		10.52,		//250
10.561,		10.602,		10.643,		10.684,		10.725,		10.766,		10.807,		10.848,		10.889,		10.93,		//260
10.971,		11.012,		11.053,		11.094,		11.135,		11.176,		11.217,		11.259,		11.3,		11.341,		//270
11.382,		11.423,		11.465,		11.506,		11.547,		11.588,		11.63,		11.671,		11.712,		11.753,		//280
11.795,		11.836,		11.877,		11.919,		11.96,		12.001,		12.043,		12.084,		12.126,		12.167,		//290
12.209,		12.25,		12.291,		12.333,		12.374,		12.416,		12.457,		12.499,		12.54,		12.582,		//300
12.624,		12.665,		12.707,		12.748,		12.79,		12.831,		12.873,		12.915,		12.956,		12.998,		//310
13.04,		13.081,		13.123,		13.165,		13.206,		13.248,		13.29,		13.331,		13.373,		13.415,		//320
13.457,		13.498,		13.54,		13.582,		13.624,		13.665,		13.707,		13.749,		13.791,		13.833,		//330
13.874,		13.916,		13.958,		14,		    14.042,		14.084,		14.126,		14.167,		14.209,		14.251,		//340
14.293,		14.335,		14.377,		14.419,		14.461,		14.503,		14.545,		14.587,		14.629,		14.671,		//350
14.713,		14.755,		14.797,		14.839,		14.881,		14.923,		14.965,		15.007,		15.049,		15.091,		//360
15.133,		15.175,		15.217,		15.259,		15.301,		15.343,		15.385,		15.427,		15.469,		15.511,		//370
15.554,		15.596,		15.638,		15.68,		15.722,		15.764,		15.806,		15.849,		15.891,		15.933,		//380
15.975,		16.071,		16.059,		16.102,		16.144,		16.186,		16.228,		16.27,		16.313,		16.355,		//390
16.397,		16.439,		16.482,		16.524,		16.566,		16.608,		16.651,		16.693,		16.735,		16.778,		//400
16.82,		16.862,		16.904,		16.947,		16.989,		17.031,		17.074,		17.116,		17.158,		17.201,		//410
17.243,		17.285,		17.328,		17.37,		17.413,		17.455,		17.497,		17.54,		17.582,		17.624,		//420
17.667,		17.709,		17.752,		17.794,		17.837,		17.879,		17.921,		17.964,		18.006,		18.049,		//430
18.091,		18.134,		18.176,		18.218,		18.261,		18.303,		18.346,		18.388,		18.431,		18.473,		//440
18.516,		18.558,		18.601,		18.643,		18.686,		18.728,		18.771,		18.813,		18.856,		18.898,		//450
18.941,		18.983,		19.026,		19.068,		19.111,		19.154,		19.196,		19.239,		19.281,		19.324,		//460
19.366,		19.409,		19.451,		19.494,		19.537,		19.579,		19.622,		19.664,		19.707,		19.75,		//470
19.792,		19.835,		19.877,		19.92,		19.962,		20.005,		20.048,		20.09,		20.133,		20.175,		//480
20.218,		20.261,		20.303,		20.346,		20.389,		20.431,		20.474,		20.516,		20.559,		20.602,		//490
20.644,		20.687,		20.73,		20.772,		20.815,		20.857,		20.9,		20.943,		20.985,		21.028,		//500
21.071,		21.113,		21.156,		21.199,		21.241,		21.284,		21.326,		21.369,		21.412,		21.454,		//510
21.497,		21.54,		21.582,		21.625,		21.668,		21.71,		21.753,		21.796,		21.838,		21.881,		//520
21.924,		21.966,		22.009,		22.052,		22.094,		22.137,		22.179,		22.222,		22.265,		22.307,		//530
22.35,		22.393,		22.435,		22.478,		22.521,		22.563,		22.606,		22.649,		22.691,		22.734,		//540
22.776,		22.819,		22.862,		22.904,		22.947,		22.99,		23.032,		23.075,		23.117,		23.16,		//550
23.203,		23.245,		23.288,		23.331,		23.373,		23.416,		23.458,		23.501,		23.544,		23.586,		//560
23.629,		23.671,		23.714,		23.757,		23.799,		23.842,		23.884,		23.927,		23.97,		24.012,		//570
24.055,		24.097,		24.14,		24.182,		24.225,		24.267,		24.31,		24.353,		24.395,		24.438,		//580
24.48,		24.523,		24.565,		24.608,		24.65,		24.693,		24.735,		24.778,		24.82,		24.863,		//590
24.905,		24.948,		24.99,		25.033,		25.075,		25.118,		25.16,		25.203,		25.245,		25.288,		//600

};

const float s_300_table[] = 
{
 //0,		1,			2,			3,			4,			5,			6,			7,			8,			9
0.000,      0.005,      0.011,      0.016,      0.022,      0.027,      0.033,      0.038,      0.044,      0.050,      //0
0.055,		0.061,		0.067,		0.072,		0.078,		0.084,		0.090,		0.095,		0.101,		0.107,		//10
0.113,		0.119,		0.125,		0.131,		0.137,		0.142,		0.148,		0.154,		0.161,		0.167,		//20
0.173,		0.179,		0.185,		0.191,		0.197,		0.203,		0.210,		0.216,		0.222,		0.228,		//30
0.235,		0.241,		0.247,		0.254,		0.260,		0.266,		0.273,		0.279,		0.286,		0.292,		//40
0.299,		0.305,		0.312,		0.318,		0.325,		0.331,		0.338,		0.345,		0.351,		0.358,		//50
0.365,		0.371,		0.378,		0.385,		0.391,		0.398,		0.405,		0.412,		0.419,		0.425,		//60
0.432,		0.439,		0.446,		0.453,		0.460,		0.467,		0.474,		0.481,		0.488,		0.495,		//70
0.502,		0.509,		0.516,		0.523,		0.530,		0.537,		0.544,		0.551,		0.558,		0.566,		//80
0.573,		0.580,		0.587,		0.594,		0.602,		0.609,		0.616,		0.623,		0.631,		0.638,		//90
0.654,		0.653,		0.660,		0.667,		0.675,		0.682,		0.690,		0.697,		0.704,		0.712,		//100
0.719,		0.727,		0.734,		0.742,		0.749,		0.757,		0.764,		0.772,		0.780,		0.787,		//110
0.795,		0.802,		0.810,		0.818,		0.825,		0.833,		0.841,		0.848,		0.856,		0.864,		//120
0.872,		0.879,		0.887,		0.895,		0.903,		0.910,		0.918,		0.926,		0.934,		0.942,		//130
0.950,		0.957,		0.965,		0.973,		0.981,		0.989,		0.997,		0.1005,		0.1013,		0.1021,		//140
1.029,		1.037,		1.045,		1.053,		1.061,		1.069,		1.077,		1.085,		1.093,		1.101,		//150
1.109,		1.117,		1.125,		1.133,		1.141,		1.149,		1.158,		1.166,		1.174,		1.182,		//160
1.190,		1.198,		1.207,		1.215,		1.223,		1.231,		1.240,		1.248,		1.256,		1.264,		//170
1.273,		1.281,		1.289,		1.297,		1.306,		1.314,		1.322,		1.331,		1.339,		1.347,		//180
1.356,		1.364,		1.373,		1.381,		1.389,		1.398,		1.406,		1.415,		1.423,		1.432,		//190
1.440,		1.448,		1.457,		1.465,		1.474,		1.482,		1.491,		1.499,		1.508,		1.516,		//200
1.525,		1.534,		1.542,		1.551,		1.559,		1.568,		1.576,		1.585,		1.594,		1.602,		//210
1.611,		1.620,		1.628,		1.637,		1.645,		1.654,		1.663,		1.671,		1.680,		1.689,		//220
1.698,		1.706,		1.715,		1.724,		1.732,		1.741,		1.750,		1.759,		1.767,		1.776,		//230
1.785,		1.794,		1.802,		1.811,		1.820,		1.829,		1.838,		1.846,		1.855,		1.864,		//240
1.873,		1.882,		1.891,		1.899,		1.908,		1.917,		1.926,		1.935,		1.944,		1.953,		//250
1.962,		1.971,		1.979,		1.988,		1.997,		2.006,		2.015,		2.024,		2.033,		2.042,		//260
2.051,		2.060,		2.069,		2.078,		2.087,		2.096,		2.105,		2.114,		2.123,		2.132,		//270
2.141,		2.150,		2.159,		2.168,		2.177,		2.186,		2.195,		2.204,		2.213,		2.222,		//280
2.232,		2.241,		2.250,		2.259,		2.268,		2.277,		2.286,		2.295,		2.304,		2.314,		//290
2.323,		2.332,		2.341,		2.350,		2.359,		2.368,		2.378,		2.387,		2.396,		2.405,		//300
};
//void t2it()
//{
//    PA0.toggle();
//}

//************************线性回归计算和显示函数*********************// 
//FLUKE 5502A校准电流，最小55uA，再小不准，由于硬件电路底噪造成，改进电路或者分档处理  Y = -20.77718 + 0.32010*X1  
//校准电流系数
double data1[14][2] = {
//    X      Y
    {163 , 60},//60uA
    {189, 70},//70
    {218, 80},//80
    {248, 90},//90
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
//S热电偶参考FLUK 5502A校准,接入万用表测试有偏差，直接读取仪表参数即可。
//级联后校准系数，偏小4摄氏度

//double data2[14][2] = {
////    X      Y
//    {658, 0.0024},//30  
//    {871, 0.1285},//50   
//    {978, 0.1944},//60℃
//    {1086, 0.2621},//70
//    {1430, 0.4752},//100
//    {1688, 0.6247},//120
//    {2080,0.8587},//150
//    {2222, 0.9390},//160
//    {2467, 1.1028},//180
//    {2742, 1.2703},//200
//    {3323, 1.6151},//240
//    {3620, 1.7917},//260
//    {3914, 1.9709},//280
//    {4063, 2.0613}//285  290以上饱和
//};
//2017.11.21重新校准S热电偶
double data2[14][2] = {
//    X      Y
    {592, 0.0272},//30  
    {798, 0.1532},//50   
    {909, 0.2191},//60℃
    {1019, 0.2869},//70
    {1369, 0.4999},//100
    {1617, 0.6495},//120
    {2000,0.8835},//150
    {2135,0.9638},//160
    {2406, 1.1276},//180
    {2684, 1.2951},//200
    {3250, 1.6399},//240
    {3536, 1.8165},//260
    {3825, 1.9927},//280
    {3900, 2.0378}//285  290以上饱和
};

//校准K热电偶
double data3[14][2] = {
//    X      Y
    {98 , 0.396},//10   ℃ 
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
    {4079, 24.487},//590  595以上饱和
};
//校准LM35温度
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
//电流小电流区域校准数据
double data5[14][2] = {
//    X      Y
    {130 , 0},//0uA
    {133 , 5},//5uA
    {136 , 8},//8uA
    {137 , 13},//13uA
    {138, 15},//15
    {139, 20},//20
    {140, 25},//25
    {141, 30},//30
    {142, 35},//35
    {143, 40},//40
    {146, 45},//45
    {150, 50},//50
    {156, 55},//55
    {163, 60},//60
};
//数据格式转换函数
typedef union
{
    double val;
    uint8_t byte[8];
}Xdata;  

 
double answer_ch1[2];
double answer_ch2[2];
double answer_ch3[2];
double answer_ch4[2];
double answer_ch5[2];
double SquarePoor[4];
Xdata ch1_ratio,ch1_off,ch2_ratio,ch2_off,ch3_ratio,ch3_off,ch4_ratio,ch4_off,ch5_ratio,ch5_off;
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
////////////////////按照特定格式发送串口数据///////////////////////////////
uint8_t send_buf[20];
typedef union
{
    uint8_t byte[4];
    float value;
}DataF32_t; 

typedef union
{
    uint8_t byte[2];
    uint16_t value;
}DataU16_t; 

uint8_t make_frame(float ch1,float ch2,float ch3,float ch4)
{
    int j = 0;
    DataU16_t crc;
    DataF32_t x1,x2,x3,x4;
    x1.value = ch1;
    x2.value = ch2;
    x3.value = ch3;
    x4.value = ch4;
    
    send_buf[j++] = '$';
    for(int i = 0; i < 4; i++)
        send_buf[j++] = x1.byte[i];
    for(int i = 0; i < 4; i++)
        send_buf[j++] = x2.byte[i];
    for(int i = 0; i < 4; i++)
        send_buf[j++] = x3.byte[i];
    for(int i = 0; i < 4; i++)
        send_buf[j++] = x4.byte[i];
    crc.value = crc16(send_buf,17);
    send_buf[j++] = crc.byte[0];
    send_buf[j++] = crc.byte[1];
    return j;
}
////////////////////////////////////////////////////////////////////////////////


//////////////////////
int32_t voltage_to_temperature(float voltage,float ref,const float *table)
{
    //k_600_table[temp] = (k_600_table[ref] + voltage)
    float temp_voltage;
    int32_t i_max;
    if(table == k_600_table)
        i_max = 600; 
    else if(table == s_300_table)
        i_max = 300;
    

    
    temp_voltage = table[round(ref/10.0)] + voltage;
    for(uint16_t i = 0; i < i_max; i++)
    {
            if(temp_voltage < table[round(ref/10.0)]) return (ref/10.0);//修改判断阈值

            if((temp_voltage > table[i]) && (temp_voltage < table[i + 1]))
                return i;
            
            if(temp_voltage > table[i_max]) 
            {
                return i_max;
            }

    }
}
///////////////////////////
//void rx_event()
//{
//    char c;
//    c = uart1.receive();
//    uart1.put_char(c);
//}

//*************初始化设置*************//
void setup()
{
    ebox_init();
    uart1.begin(115200);//设置串口波特率Uart uart1(USART1, &PA9, &PA10);  PA9位TX  PA10位RX  串口1
//    uart1.attach_rx_interrupt(rx_event);
//    uart1.interrupt(TcIrq,ENABLE);

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
//************线性回归计算函数****************************//  
    linear_regression((double*)data1,14,&answer_ch1[0],&answer_ch1[1],&SquarePoor[0]);
    ch1_ratio.val = answer_ch1[1];  //双精度与CHAR转换
    ch1_off.val = answer_ch1[0];
    
    linear_regression((double*)data2,14,&answer_ch2[0],&answer_ch2[1],&SquarePoor[0]);
    ch2_ratio.val = answer_ch2[1];//双精度与CHAR转换
    ch2_off.val = answer_ch2[0];
    
    linear_regression((double*)data3,14,&answer_ch3[0],&answer_ch3[1],&SquarePoor[0]);
    ch3_ratio.val = answer_ch3[1];//双精度与CHAR转换
    ch3_off.val = answer_ch3[0];
    
    linear_regression((double*)data4,14,&answer_ch4[0],&answer_ch4[1],&SquarePoor[0]);
    ch4_ratio.val = answer_ch4[1];
    ch4_off.val = answer_ch4[0];
    
    linear_regression((double*)data5,14,&answer_ch5[0],&answer_ch5[1],&SquarePoor[0]);
    ch5_ratio.val = answer_ch5[1];
    ch5_off.val = answer_ch5[0];

//***************数据读写FLASH操作*****************//

    flash.write(0x800E000,ch1_ratio.byte,16);
    flash.write(0x800E010,ch1_off.byte,16);    
    flash.write(0x800E020,ch2_ratio.byte,16);
    flash.write(0x800E030,ch2_off.byte,16);
    flash.write(0x800E040,ch3_ratio.byte,16);
    flash.write(0x800E050,ch3_off.byte,16);  
    flash.write(0x800E060,ch4_ratio.byte,16);
    flash.write(0x800E070,ch4_off.byte,16);  
    flash.write(0x800E080,ch5_ratio.byte,16);
    flash.write(0x800E090,ch5_off.byte,16);  

    flash.read(0x800E000,ch1_ratio.byte,16);//Y = -0.30744 + 0.00061*X1     
    flash.read(0x800E010,ch1_off.byte,16);   
    flash.read(0x800E020,ch2_ratio.byte,16);//Y = -0.30518 + 0.00061*X1  软件计算还原函数// 原来的  Y = -0.31382 + 0.00061*X1;
    flash.read(0x800E030,ch2_off.byte,16);
    flash.read(0x800E040,ch3_ratio.byte,16);// Y = -0.19654 + 0.00605*X1
    flash.read(0x800E050,ch3_off.byte,16);
    flash.read(0x800E060,ch4_ratio.byte,16);//Y = -0.38432 + 0.17249*X1
    flash.read(0x800E070,ch4_off.byte,16);
    flash.read(0x800E080,ch5_ratio.byte,16);// Y = -264.12479 + 2.05554*X1   电流测试的低电流函数
    flash.read(0x800E090,ch5_off.byte,16);
//*************输出线性计算函数************************************************//
//    display((double*)data1,answer_ch1,&SquarePoor[0],12,2);
//    display((double*)data2,answer_ch2,&SquarePoor[0],12,2);
//    display((double*)data3,answer_ch3,&SquarePoor[0],14,2);
//    display((double*)data4,answer_ch4,&SquarePoor[0],14,2);
//    display((double*)data5,answer_ch5,&SquarePoor[0],14,2);
//    while(1);
}

double I_voltage_ch1,S_voltage_ch2,K_voltage_ch3,LM35_voltage_ch4;
int main(void)
{

    setup();
    last_updata_time = millis();
    while(1)
    {
        //读取ADC通道的数据，通过模拟开关切换，50HZ频率
        for(int i = 0; i < 4; i++)
        { 
            adc_read_ch(i);
        }
//**************计算还原系数******************//       
        detector_urrent_ch1 = voltage_ch1;///2/0.823;//825欧姆采样电阻，1.98为运放放大倍数，单位uA，酸露漏电流,具体情况需要校准，实际测试A=2，电阻R4=823
        detector_temperature_ch2 = voltage_ch2;///1020;//运放放大倍数1020，根据实际情况调整，酸露传感器温度S热电偶,单位//mv，需要制表查询计算       
        flue_temperature_ch3 = (voltage_ch3-19)/101;//运放放大倍数101,烟气温度K热电偶,单位//mv，需要制表查询计算，减去零点偏置19mV,偏置可以通过
                                                        //输入0电压获得，101由A=1+R25/R37，实际情况需要校准零点偏置和K系数。
        ambient_temperature_ch4 = voltage_ch4/3.56/10;//运放放大倍数3.56,环境温度检测LM35，10mV/℃，单位℃，需要零点补偿校准
        
//        detector_urrent_ch1 = voltage_ch1;//
//        detector_temperature_ch2 = voltage_ch2;//
////      flue_temperature_ch3 = voltage_ch3;//
//        ambient_temperature_ch4 = voltage_ch4;//  
        if(val_ch1 >= 159 )
        {
            I_voltage_ch1 = ch1_off.val + ch1_ratio.val*val_ch1;
        }
        else if ((val_ch1 < 159)&(val_ch1 > 131))
        {
             I_voltage_ch1 = ch5_off.val + ch5_ratio.val*val_ch1;
        }
        else if (val_ch1 <= 131)
        {
            I_voltage_ch1=0;
        }
        
//        S_voltage_ch2 = ch2_off.val + 0.1+ ch2_ratio.val*val_ch2; //增加100uV修正，重新校准系数时需要去除0.1
        S_voltage_ch2 = ch2_off.val + ch2_ratio.val*val_ch2; 
        K_voltage_ch3 = ch3_off.val + ch3_ratio.val*val_ch3;  
        LM35_voltage_ch4 = ch4_off.val + ch4_ratio.val*val_ch4;

//        int rx_temp;
        
//*********串口发送输出结果给主控单片机和接收主控单片机参数配置指令*********//
        if(millis() - last_updata_time >= 200) //发送485更新速率时间为200ms每次
        {
            last_updata_time = millis();
          _485_tx_mode();
//         uart1.printf("val_ch1 = 0x%04x\r\nval_ch2 = 0x%04x\r\nval_ch3 = 0x%04x\r\n val_ch4 = 0x%04x\r\n",val_ch1,val_ch2,val_ch3,val_ch4);
//          uart1.printf("val_ch1 = 0x%04x\r\nval_ch2 = 0x%04x\r\nval_ch3 = 0x%04x\r\n val_ch4 = 0x%04x\r\n",I_voltage_ch1,val_ch2,val_ch3,val_ch4);
  
 
 //查表得出热电偶温度值         
           int32_t temp1 = voltage_to_temperature(K_voltage_ch3,LM35_voltage_ch4,k_600_table);
           int32_t temp2 = voltage_to_temperature(S_voltage_ch2,LM35_voltage_ch4,s_300_table);
         //  temp2 = temp2 - 3 ;//修正温度校准误差
//           LM35_voltage_ch4 = LM35_voltage_ch4 /10.0;
//uart1.printf("temp1:%d\ttemp2:%d\tvoltage:%0.2f\r\n",temp1,temp2,K_voltage_ch3);
 //485发送数据方式   
 //测试程序
//           uart1.printf("I_voltage_ch1 = %4.3f\t S_voltage_ch2 = %4.3f\t K_voltage_ch3 = %4.3f\t LM35_voltage_ch4 = %4.1f\r\n",I_voltage_ch1,S_voltage_ch2,K_voltage_ch3,LM35_voltage_ch4); 

//**************最终上位机485通信程序**********************************
            uint8_t len = make_frame(I_voltage_ch1,temp2,temp1,LM35_voltage_ch4);
            uart1.write(send_buf,len);
//*******************最终上位机485通信程序  ********************************************               
                 PA1.toggle();   //测试口 

                 
//**************校准使用输出字符串，用于校准S等热电偶参数输出**************************************
//            uart1.printf("%04d\t%0.4f\t%0.4f\r\n",val_ch2,S_voltage_ch2,ambient_temperature_ch4);     
//            uart1.printf("%04d\t%0.4f\t%0.4f\r\n",val_ch4,LM35_voltage_ch4,ambient_temperature_ch4);    
//          uart1.printf("%04d\r\t %4.3f\r\n",val_ch3,flue_temperature_ch3);               
//            uart1.printf("I_voltage_ch1 = %4.3f\t S_voltage_ch2 = %4.3f\t K_voltage_ch3 = %4.3f\t LM35_voltage_ch4 = %4.1f\r\n",I_voltage_ch1,S_voltage_ch2,K_voltage_ch3,LM35_voltage_ch4);
////          uart1.printf("detector_urrent_ch1 = %4.3f\r\n detector_temperature_ch2 = %4.3f\r\n flue_temperature_ch3 = %4.3f\r\n ambient_temperature_ch4 = %4.1f\r\n",detector_urrent_ch1,detector_temperature_ch2,flue_temperature_ch3,ambient_temperature_ch4);
//          uart1.printf("voltage_ch1 = %4.3f\r\n voltage_ch2 = %4.3f\r\n voltage_ch3 = %4.3f\r\n voltage_ch4 = %4.1f\r\n",voltage_ch1,voltage_ch2,voltage_ch3,voltage_ch4);
//            _485_rx_mode();
//**************校准使用输出字符串，用于校准S等热电偶参数输出**************************************
            LED0.toggle();//串口发送状态指示灯，D4
//  
               PA1.toggle();   //测试口     
               
        }
//          delay_ms(1000);             
    }
}

//485输出
void _485_tx_mode()//如果不输出，将引脚设置翻转
{
    PA11.set();
    delay_us(10);
}
void _485_rx_mode()
{
    PA11.reset();
    delay_us(10);
    
}
//通道模拟开关选择
void adc_read_ch(uint8_t ch)
{
    LED1.toggle();//ADC采甲刺指示灯
   
    switch(ch)
    {
        case 0: 
            EN.set();
            A0.reset(); 
            A1.reset();
            delay_ms(10);
            val_ch1 = adc.read();
            voltage_ch1 = adc.read_voltage(); //mv  
         PA0.toggle();
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
