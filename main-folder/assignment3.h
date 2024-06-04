/******************************************************************************
* Class::       CSC-615-01 Spring 2024
* Name::        Bryan Yan, Natalie Yam, Westley Cho, Griffin Evans
* Student ID::  916956188, 920698945, 922841683,922498210 
* Github-Name:: yan-bryan, ynataliee, westleyc30, griffinevans 
* Project::     Term Project 
* 
* File::        assignment3.h
* 
* Description:: Header definitions for our self-driving car program. 
*
******************************************************************************/
#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdio.h>      //printf()
#include <stdlib.h>     //exit()
#include <signal.h>

#include <time.h>
#include "DEV_Config.h"
//#include "MotorDriver.h"

#define BUTTON_IN	14

#include "PCA9685.h"

#define PWMA	PCA_CHANNEL_0
#define AIN1	PCA_CHANNEL_1
#define AIN2	PCA_CHANNEL_2
#define PWMB	PCA_CHANNEL_5
#define BIN1	PCA_CHANNEL_3
#define BIN2	PCA_CHANNEL_4

#define SLAVE_ADD_ONE	0x40
#define SLAVE_ADD_TWO	0x50

#define MOTORA	0
#define MOTORB	1

typedef enum {
    FORWARD = 1,
    BACKWARD  ,

} DIR;

#endif 
