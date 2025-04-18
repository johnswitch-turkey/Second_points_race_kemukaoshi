/**
  ******************************************************************************
  * @file    control.c
  * @author  amkl
  * @version V1.0
  * @date    2022-09-22
  * @brief   控制函数
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "control.h"

#include "main.h"  // 包含HAL库主头文件
#include "tim.h"   // 定时器相关
#include "gpio.h"  // GPIO相关
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "key.h"
#include "math.h"
#include "mpu6050.h"
#include "uart.h"
#include "decode.h"

/* 全局变量 ------------------------------------------------------------------*/
Param_InitTypedef Param;
Flag_InitTypedef Flag;

float pitch, roll, yaw; // 欧拉角
#define T 0.158f       // 轴距参数
#define L 0.1545f      // 轮距参数
//#define SERVO_INIT 1500 // 舵机中位值
#define M_PI 3.14159265358979323846f

int raspi_rxData = 0;    //从树莓派获取的数据
uint8_t ModeChoose=0;

/* 硬件定义 ------------------------------------------------------------------*/


// 执行器控制
//#define LED_TOGGLE HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
//#define BEEP_ON HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)
//#define BEEP_OFF HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)

/* 私有函数声明 --------------------------------------------------------------*/
static void Back_Parking_Control(void);
static void Side_Parking_Control(void);
static void Back_Side_Parking_Control(void);
static void Timer_Count_Handler(void);

/* 函数实现 ------------------------------------------------------------------*/

/**
  * @brief 外部中断回调函数
  * @param GPIO_Pin: 触发中断的引脚
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint16_t Time_Cnt = 0;
    
    // 陀螺仪中断处理 (PB5)
    if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
        // 获取姿态数据
        if(Flag.Is_Go_straight || Flag.Is_Turn_Car) {
            mpu_dmp_get_data(&pitch, &roll, &yaw);
        }
        
        // 读取编码器脉冲
        Param.UnitTime_Motor1Pluse = (short)Read_Speed(2);
        Param.UnitTime_Motor2Pluse = (short)Read_Speed(4);
        
        // 控制计算
        Position_PID_Servo_Realize();
        Param.Motor1_PWM_Out = VelocityRing_MOTOR1_Control();
        Param.Motor2_PWM_Out = VelocityRing_MOTOR2_Control();
        
        // 输出限制和执行
        Limit(&Param.Motor2_PWM_Out, &Param.Motor1_PWM_Out, &Param.Servo_Target_Position);
        Load(Param.Motor2_PWM_Out, Param.Motor1_PWM_Out, Param.Servo_Target_Position);
        
        // 状态指示灯（10次中断翻转一次）
        if(++Time_Cnt >= 10) {
            Time_Cnt = 0;
            LED_TOGGLE;
        }
        
        // 定时器计数处理
        Timer_Count_Handler();
    }
		/*
    // 按键1中断
    else if(GPIO_Pin == KEY1_PIN) {
        HAL_Delay(15);
        if(HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_SET) {
            Param.ModeChoose = BACK_PACKING;
            Flag.Run_Step = 1;
        }
    }
    // 按键2中断
    else if(GPIO_Pin == KEY2_PIN) {
        HAL_Delay(15);
        if(HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_SET) {
            Param.ModeChoose = SIDE_PACKING;
            Flag.Run_Step = 1;
        }
    }
    // 按键3中断
    else if(GPIO_Pin == KEY3_PIN) {
        HAL_Delay(15);
        if(HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_SET) {
            Param.ModeChoose = BACK_SIDE_PACKING;
            Flag.Run_Step = 1;
        }
    }
		*/
}

/**
  * @brief 主控制流程
  */
void Control_Proc(void)
{
		modechoose(&ModeChoose);
    switch(ModeChoose)
    {
        case BACK_PACKING:
            Back_Parking_Control();
            break;
            
        case SIDE_PACKING:
            Side_Parking_Control();
            break;
            
        case BACK_SIDE_PACKING:
            Back_Side_Parking_Control();
            break;
            
        default:
            // 默认模式处理
            Kinematic_Analysis(0, 0, 0);
            break;
    }
}

/**
  * @brief 倒车入库控制
  */
static void Back_Parking_Control(void)
{
    switch(Flag.Run_Step)
    {
        case 1: // 直行检测
            if(Flag.Is_Go_straight) {
                Kinematic_Analysis(60, 60, yaw);
                //openMv_Proc();
								raspi_rxData=raspi_rx();
                if(raspi_rxData == 1) {
                    Flag.Is_Go_straight = 0;
                    Flag.Is_Stop_Car = 1;
                    Flag.Run_Step = 2;
                }
            }
            break;
            
        case 2: // 停车1秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 100;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Is_Start_Astern = 1;
                Flag.Run_Step = 3;
            }
            break;
            
        case 3: // 匀速后退0.72s
            if(Flag.Is_Start_Astern == 1 && !Flag.Start_Count) {
                Kinematic_Analysis(-60, -60, SERVO_INIT);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 72;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Run_Step = 4;
            }
            break;
            
        case 4: // 右转60°倒车
            if(Flag.Is_Start_Astern == 1 && !Flag.Start_Count) {
                Kinematic_Analysis(-20, -100, SERVO_INIT + 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) >= 80) {
                Flag.Is_Turn_Car = 0;
                Flag.Is_Start_Astern = 2;
                Flag.Run_Step = 5;
            }
            break;
            
        case 5: // 车头归正倒车1.5s
            if(Flag.Is_Start_Astern == 2 && !Flag.Start_Count) {
                Kinematic_Analysis(-100, -100, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 150;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Start_Astern = 0;
                Flag.Is_Stop_Car = 1;
                Flag.Run_Step = 6;
            }
            break;
            
        case 6: // 停车5秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 500;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Run_Step = 7;
            }
            break;
            
        case 7: // 直行出库0.85s
            if(!Flag.Is_Stop_Car) {
                Kinematic_Analysis(100, 100, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 85;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Run_Step = 8;
            }
            break;
            
        case 8: // 右转60°出库
            if(!Flag.Start_Count) {
                Kinematic_Analysis(100, 200, SERVO_INIT + 300);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 150;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 1;
                Flag.Run_Step = 9;
            }
            break;
            
        case 9: // 最终停车
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
							Usart2_SendString("1finish");
            }
            break;
            
        default:
            Flag.Run_Step = 1;
            break;
    }
}

/**
  * @brief 侧方停车控制
  */
static void Side_Parking_Control(void)
{
    switch(Flag.Run_Step)
    {
        case 1: // 直行检测
            if(Flag.Is_Go_straight) {
                Kinematic_Analysis(100, 100, yaw);
//                openMv_Proc();
									raspi_rxData=raspi_rx();
                if(raspi_rxData == 1) {
                    Flag.Is_Go_straight = 0;
                    Flag.Is_Stop_Car = 1;
                    Flag.Run_Step = 2;
                }
            }
            break;
            
        case 2: // 停车1秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 100;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Is_Start_Astern = 1;
                Flag.Run_Step = 3;
            }
            break;
            
        case 3: // 右转60°倒车
            if(Flag.Is_Start_Astern == 1 && !Flag.Start_Count) {
                Kinematic_Analysis(0, -200, SERVO_INIT + 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) >= 40) {
                Flag.Is_Turn_Car = 0;
                Flag.Is_Start_Astern = 2;
                Flag.Run_Step = 4;
            }
            break;
            
        case 4: // 回正倒车1s
            if(Flag.Is_Start_Astern == 2 && !Flag.Start_Count) {
                Kinematic_Analysis(-100, -100, SERVO_INIT);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 100;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Start_Astern = 3;
                Flag.Run_Step = 5;
            }
            break;
            
        case 5: // 左转60°倒车
            if(Flag.Is_Start_Astern == 3 && !Flag.Start_Count) {
                Kinematic_Analysis(-230, -100, SERVO_INIT - 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) <= 5) {
                Flag.Is_Turn_Car = 0;
                Flag.Is_Start_Astern = 0;
                Flag.Run_Step = 6;
            }
            break;
            
        case 6: // 直行调整位置
            if(Flag.Is_Start_Astern == 0 && !Flag.Start_Count) {
                Kinematic_Analysis(100, 100, SERVO_INIT);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 100;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 1;
                Flag.Run_Step = 7;
            }
            break;
            
        case 7: // 停车5秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 500;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Run_Step = 8;
            }
            break;
            
        case 8: // 后退0.6s
            if(!Flag.Is_Timer_Up && !Flag.Start_Count) {
                Kinematic_Analysis(-100, -100, SERVO_INIT);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 60;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Run_Step = 9;
            }
            break;
            
        case 9: // 左转60°出库
            if(!Flag.Is_Stop_Car) {
                Kinematic_Analysis(200, 0, SERVO_INIT - 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) >= 40) {
                Flag.Is_Turn_Car = 0;
                Flag.Run_Step = 10;
            }
            break;
            
        case 10: // 回正直行0.8s
            if(!Flag.Is_Timer_Up && !Flag.Start_Count) {
                Kinematic_Analysis(100, 100, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 80;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Run_Step = 11;
            }
            break;
            
        case 11: // 右转60°归位
            if(!Flag.Start_Count) {
                Kinematic_Analysis(100, 200, SERVO_INIT + 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) <= 5) {
                Flag.Is_Turn_Car = 0;
                Flag.Is_Stop_Car = 1;
                Flag.Run_Step = 12;
            }
            break;
            
        case 12: // 最终停车
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
								Usart2_SendString("2finish");     
						}
            break;
            
        default:
            Flag.Run_Step = 1;
            break;
    }
}

/**
  * @brief 连续停车控制
  */
static void Back_Side_Parking_Control(void)
{
    switch(Flag.Run_Step)
    {
        case 1: // 直行检测
            if(Flag.Is_Go_straight) {
                Kinematic_Analysis(100, 100, yaw);
//                openMv_Proc();
								raspi_rxData=raspi_rx();
                if(raspi_rxData == 1) {
                    Flag.Is_Go_straight = 0;
                    Flag.Is_Stop_Car = 1;
                    Flag.Run_Step = 2;
                }
            }
            break;
            
        case 2: // 停车1秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 100;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Is_Start_Astern = 1;
                Flag.Run_Step = 3;
            }
            break;
            
        case 3: // 倒车0.72s
            if(Flag.Is_Start_Astern == 1 && !Flag.Start_Count) {
                Kinematic_Analysis(-100, -100, SERVO_INIT);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 72;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Run_Step = 4;
            }
            break;
            
        case 4: // 右转60°倒车
            if(Flag.Is_Start_Astern == 1 && !Flag.Start_Count) {
                Kinematic_Analysis(-50, -150, SERVO_INIT + 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) >= 80) {
                Flag.Is_Turn_Car = 0;
                Flag.Is_Start_Astern = 2;
                Flag.Run_Step = 5;
            }
            break;
            
        case 5: // 车头归正倒车1.5s
            if(Flag.Is_Start_Astern == 2 && !Flag.Start_Count) {
                Kinematic_Analysis(-100, -100, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 150;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Start_Astern = 0;
                Flag.Is_Stop_Car = 1;
                Flag.Run_Step = 6;
            }
            break;
            
        case 6: // 停车5秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 500;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Run_Step = 7;
            }
            break;
            
        case 7: // 直行出库0.75s
            if(!Flag.Is_Stop_Car) {
                Kinematic_Analysis(100, 100, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 75;
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Run_Step = 8;
            }
            break;
            
        case 8: // 右转60°准备侧方
            if(!Flag.Start_Count) {
                Kinematic_Analysis(0, 200, SERVO_INIT + 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) <= 5) {
                Flag.Is_Turn_Car = 0;
                Flag.Is_Go_straight = 1;
                //Param.openMV_Data = 0;
							//发出执行测方倒车检测的信号
                Usart2_SendString("1finish");
                Flag.Run_Step = 9;
            }
            break;
            
        case 9: // 直行检测侧方
            if(Flag.Is_Go_straight) {
                Kinematic_Analysis(100, 100, yaw);
//                openMv_Proc();
									raspi_rxData=raspi_rx();
                if(raspi_rxData == 2) {
                    Flag.Is_Go_straight = 0;
                    Flag.Is_Stop_Car = 1;
                    Flag.Run_Step = 10;
                }
            }
            break;
            
        case 10: // 停车1秒
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1;
                Param.Timer_threshold_value = 100;
                BEEP_ON;
            }
            if(Flag.Is_Timer_Up) {
                BEEP_OFF;
                Flag.Start_Count = 0;
                Flag.Is_Timer_Up = 0;
                Flag.Is_Stop_Car = 0;
                Flag.Is_Start_Astern = 1;
                Flag.Run_Step = 11;
            }
            break;
            
        case 11: // 右转60°倒车
            if(Flag.Is_Start_Astern == 1 && !Flag.Start_Count) {
                Kinematic_Analysis(0, -150, SERVO_INIT + 300);
                Flag.Is_Turn_Car = 1;
            }
            if(abs((int)yaw) == 40) {
                Flag.Is_Turn_Car=0;//不开启转弯
//								Flag.Start_Count=0;//不开启计数
//								Flag.Is_Timer_Up=0;//定时时间到标志位清零
								Flag.Is_Start_Astern=2;//开始倒车第二步
								Flag.Run_Step=12;//跳转下一步
            }
            break;
            
        case 12: // 回正倒车1s
            if(Flag.Is_Start_Astern == 2 && !Flag.Start_Count) {
                Kinematic_Analysis(-100,-100,SERVO_INIT);								
							  Flag.Start_Count=1;//开始计时
							  Param.Timer_threshold_value=100;//定时1s
            }
            if(Flag.Is_Timer_Up) { //定时时间到
                Flag.Start_Count=0;//不开启计数
								Flag.Is_Timer_Up=0;//定时时间到标志位清零
								Flag.Is_Start_Astern=3;//清零倒车步骤
                Flag.Run_Step = 13;
            }
            break;
            
        case 13: // 左转60°倒车
            if(Flag.Is_Start_Astern == 3 && !Flag.Start_Count) {
                Kinematic_Analysis(-200, -100, SERVO_INIT - 300);
                Flag.Is_Turn_Car = 1;  //清零倒车步骤//开始转弯
            }
            if(abs((int)yaw) <= 5) {
//                Flag.Is_Turn_Car = 0; //不开启转弯
//							  Flag.Is_Timer_Up=0;//定时时间到标志位清零
							  Flag.Start_Count=0; //不开启计数
                Flag.Is_Start_Astern = 0; //清零倒车步骤
                Flag.Run_Step = 14;
            }
            break;
            
        case 14: // 直行调整位置
            if(Flag.Is_Start_Astern == 0 && !Flag.Start_Count) {
                Kinematic_Analysis(100, 100, SERVO_INIT);
                Flag.Start_Count = 1; //开始计时
                Param.Timer_threshold_value = 100; //定时1s
            }
            if(Flag.Is_Timer_Up) { //定时时间到
                Flag.Start_Count = 0; //不开启计数
                Flag.Is_Timer_Up = 0; //定时时间到标志位清零
                Flag.Is_Stop_Car = 1; //不停车
                Flag.Run_Step = 15;
            }
            break;
            
        case 15: // 停车5秒 //停车，回正，蜂鸣器响，定时5s
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
                Flag.Start_Count = 1; //开始计时
                Param.Timer_threshold_value = 500; //定时5s
                BEEP_ON; //蜂鸣器响
            }
            if(Flag.Is_Timer_Up) { //定时时间到
                BEEP_OFF; //关蜂鸣器
                Flag.Start_Count = 0; //不开启计数
                Flag.Is_Timer_Up = 0; //定时时间到标志位清零
                Flag.Is_Stop_Car = 0; //不停车
                Flag.Run_Step = 16; //跳转下一步
            }
            break;
						
            /*出库*/
        case 16: // 后退0.6s
            if(!Flag.Is_Timer_Up && !Flag.Start_Count) {
                Kinematic_Analysis(-100, -100, SERVO_INIT);
                Flag.Start_Count = 1; //开始计时
                Param.Timer_threshold_value = 60; //定时0.6s
            }
            if(Flag.Is_Timer_Up) {
                Flag.Start_Count = 0; //不开启计数
                Flag.Is_Timer_Up = 0; //定时时间到标志位清零
							  Flag.Is_Stop_Car=0; //不停车
                Flag.Run_Step = 17;
            }
            break;
            
        case 17: // 左转60°，前进，差速转弯出库
            if(!Flag.Is_Stop_Car) {
                Kinematic_Analysis(150, 0, SERVO_INIT - 300);
                Flag.Is_Turn_Car = 1; // 开始转弯
            }
            if(abs((int)yaw) >= 40) { // 角度为-40°结束
							  Flag.Is_Turn_Car=0; // 不开启转弯
//								Flag.Start_Count=0; // 不开启计数
//                Flag.Is_Turn_Car = 0; // 定时时间到标志位清零
                Flag.Run_Step = 18; // 跳转下一步
            }
            break;
            
        case 18: // 回正，直行，不差速转弯，定时0.8s
            if(!Flag.Is_Timer_Up && !Flag.Start_Count) {
                Kinematic_Analysis(100, 100, 0.0);
                Flag.Start_Count = 1; // 开始计时
                Param.Timer_threshold_value = 80; // 定时0.8s
            }
            if(Flag.Is_Timer_Up) { // 定时时间到
                Flag.Start_Count = 0; // 不开启计数
                Flag.Is_Timer_Up = 0; // 定时时间到标志位清零
                Flag.Run_Step = 19; // 跳转下一步
            }
            break;
            
        case 19: // 右转60°归位
            if(!Flag.Start_Count) {
                Kinematic_Analysis(100, 150, SERVO_INIT + 300.0);
                Flag.Is_Turn_Car = 1; // 开始转弯
            }
            if(abs((int)yaw) <= 5) {
                Flag.Is_Turn_Car = 0; // 不开启转弯
//                Flag.Is_Stop_Car = 1; // 不开启计数
//							  Flag.Is_Timer_Up=0; // 定时时间到标志位清零
								Flag.Is_Stop_Car=1; // 停车	
                Flag.Run_Step = 20; // 跳转下一步
            }
            break;
            
        case 20: // 最终停车
            if(Flag.Is_Stop_Car && !Flag.Start_Count) {
                Kinematic_Analysis(0, 0, 0.0);
							Usart2_SendString("2finish");
            }
            break;
            
        default:
            Flag.Run_Step = 1;
            break;
    }
}

/**
  * @brief 舵机PID控制
  */
void Position_PID_Servo_Realize(void)
{
    Param.Servo_Speed = Position_PID_Servo(Param.Servo_Target_Val);
    Param.Servo_Target_Position += Param.Servo_Speed;
}

/**
  * @brief 运动学分析
  */
void Kinematic_Analysis(float velocity1, float velocity2, float angle)
{
    // 角度限幅 (±60度)
    angle = angle > 60.0f ? 60.0f : (angle < -60.0f ? -60.0f : angle);
    
    // 直行模式
    if(Flag.Is_Go_straight || Flag.Is_Start_Astern != 1) {
        PID.Motor1_Velocity_Target_Val = velocity1;
        PID.Motor2_Velocity_Target_Val = velocity2;
    }
    // 倒车转向模式
    else if(Flag.Is_Start_Astern == 1) {
        float tan_angle = tanf(angle * M_PI / 180.0f);
        PID.Motor1_Velocity_Target_Val = velocity1 * (1 - T * tan_angle / (2 * L));
        PID.Motor2_Velocity_Target_Val = velocity1 * (1 + T * tan_angle / (2 * L));
    }
    
    // 舵机目标位置计算
    Param.Servo_Target_Val = SERVO_INIT + angle * 5;
}

/**
  * @brief 定时计数处理
  */
static void Timer_Count_Handler(void)
{
    if(Flag.Start_Count) {
        static uint16_t Timer_Cnt = 0;
        if(++Timer_Cnt >= Param.Timer_threshold_value) {
            Timer_Cnt = 0;
            Flag.Is_Timer_Up = 1;
        }
    }
}

/**
  * @brief 限制函数
  */
void Limit(int *motoA, int *motoB, float *servo)
{
    *motoA = (*motoA > 6500) ? 6500 : ((*motoA < -6500) ? -6500 : *motoA);
    *motoB = (*motoB > 6500) ? 6500 : ((*motoB < -6500) ? -6500 : *motoB);
    *servo = (*servo > 1900) ? 1900 : ((*servo < 1800) ? 1800 : *servo);
}



/*************************************END OF FILE************************************/
