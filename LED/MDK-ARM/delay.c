/**
  ******************************************************************************
  * @file           : delay.c
  * @brief          : delay function including us and ns
  ******************************************************************************
  */
 #include "delay.h"

 static uint32_t g_fac_us = 0;       /* us延时倍乘数 */



 void delay_init(uint16_t sysclk){       /* 初始化延迟函数 */
    g_fac_us=sysclk;
 }
void delay_ms(uint16_t nms){            /* 延时nms */
    delay_us((uint32_t)(1000*nms));
    return;
}
void delay_us(uint32_t nus){            /* 延时nus */
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
    ticks = nus * g_fac_us;                 /* 需要的节拍数 */


    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)                /*现有系统值较旧值低，说明往下数了*/
            {
                tcnt += told - tnow;        /* SYSTICK是一个递减的计数器,所以是旧减新 */
            }
            else                            /*现有系统值比旧值高，说明tick重置了*/
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;                    /*update*/
            if (tcnt >= ticks)
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
    return;
}
