/**
  ******************************************************************************
  * @file           : delay.h
  * @brief          : delay function including us and ns
  ******************************************************************************
  Tip:this is a user driver.you need to add the path "../MDK-ARM"
  */
#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"


void delay_init(uint16_t sysclk);       /* 初始化延迟函数 */
void delay_ms(uint16_t nms);            /* 延时nms */
void delay_us(uint32_t nus);            /* 延时nus */


#endif