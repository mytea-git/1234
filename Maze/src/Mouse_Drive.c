


/*********************************************************************************************************
  包含头文件
*********************************************************************************************************/
#include "Mouse_Drive.h"


/*********************************************************************************************************
  定义全局变量
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0};              /*  保存电脑鼠当前位置坐标      */

uchar             GucMouseDir                     = UP;                 /*  保存电脑鼠当前方向          */

uchar             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
                                                                        /*  x,横坐标;y,纵坐标;          */
                                                                        /*  bit3~bit0分别代表左下右上   */
                                                                        /*  0:该方向无路，1:该方向有路  */

static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0};    /*  定义并初始化左电机状态      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0};    /*  定义并初始化右电机状态      */

static uchar    __GucMouseState                   = __STOP;             /*  保存电脑鼠当前运行状态      */
static uint     __GuiAccelTable[400]              = {0};                 /*  电机加减速各阶段定时器值    */
static int      __GiMaxSpeed                      = SEARCHSPEED;        /*  保存允许运行的最大速度      */
static uchar    __GucDistance[5]                  = {0};                /*  记录传感器状态              */


/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        延时函数
** input parameters:    uiD :延时参数，值越大，延时越久
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __delay (uint  uiD)
{
    for (; uiD; uiD--);
}


/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        右步进电机驱动时序
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __rightMotorContr (void)
{
    static char cStep = 0;                                              /*  保存电机当前位置            */
    
    switch (__GmRight.cDir) {

    case __MOTORGOAHEAD:                                                /*  向前步进                    */
        cStep = (cStep + 1) % 8;
        break;

    case __MOTORGOBACK:                                                 /*  向后步进                    */
        cStep = (cStep + 7) % 8;
        break;

    default:
        break;
    }
    
    switch (cStep) {

    case 0:                                                             /*  A2B2                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2);
        break;

    case 1:                                                             /*  B2                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRA1 | __PHRA2);
        break;

    case 2:                                                             /*  A1B2                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRA1 | __PHRA2 | __PHRB2);
        break;

    case 3:                                                             /*  A1                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRB2);
        break;

    case 4:                                                             /*  A1B1                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRA2 | __PHRB2);
        break;

    case 5:                                                             /*  B1                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRA2);
        break;

    case 6:                                                             /*  A2B1                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRA2 | __PHRB1 | __PHRB2);
        break;

    case 7:                                                             /*  A2                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     __PHRB1 | __PHRB2);
        break;

    default:
        break;
    }
}


/*********************************************************************************************************
** Function name:       __leftMotorContr
** Descriptions:        左步进电机驱动时序
** input parameters:    __GmLeft.cDir :电机运行方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __leftMotorContr (void)
{
    static char cStep = 0;                                              /*  保存电机当前位置            */
    
    switch (__GmLeft.cDir) {
        
    case __MOTORGOAHEAD:                                                /*  向前步进                    */
        cStep = (cStep + 1) % 8;
        break;
        
    case __MOTORGOBACK:                                                 /*  向后步进                    */
        cStep = (cStep + 7) % 8;
        break;
        
    default:
        break;
    }
    
    switch (cStep) {

    case 0:                                                             /*  A2B2                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2);
        break;

    case 1:                                                             /*  B2                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLB1 | __PHLB2);
        break;

    case 2:                                                             /*  A1B2                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLA2 | __PHLB1 | __PHLB2);
        break;

    case 3:                                                             /*  A1                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLA2);
        break;

    case 4:                                                             /*  A1B1                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLA2 | __PHLB2);
        break;

    case 5:                                                             /*  B1                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLB2);
        break;

    case 6:                                                             /*  A2B1                        */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLA1 | __PHLA2 | __PHLB2);
        break;

    case 7:                                                             /*  A2                          */
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     __PHLA1 | __PHLA2);
        break;

    default:
        break;
    }
}


/*********************************************************************************************************
** Function name:       __speedContrR
** Descriptions:        右电机速度调节
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __speedContrR (void)
{
    int iDPusle;
    
    iDPusle = __GmRight.uiPulse - __GmRight.uiPulseCtr;                 /*  统计电机还剩余的步数        */
    if (iDPusle <= __GmRight.iSpeed) {
        __GmRight.iSpeed--;
    } else {                                                            /*  非减速区间，则加速到最大值  */
        if (__GmRight.iSpeed < __GiMaxSpeed) {
            __GmRight.iSpeed++;
        } else {
            __GmRight.iSpeed--;
        }
    }
    if (__GmRight.iSpeed < 0) {                                         /*  设置速度下限                */
        __GmRight.iSpeed = 0;
    }
    TimerLoadSet(TIMER0_BASE,TIMER_A,__GuiAccelTable[__GmRight.iSpeed]);/*  设置定时时间                */
}


/*********************************************************************************************************
** Function name:       __speedContrL
** Descriptions:        左电机速度调节
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __speedContrL (void)
{
    int iDPusle;
    
    iDPusle = __GmLeft.uiPulse - __GmLeft.uiPulseCtr;                   /*  统计电机还剩余的步数        */
    if (iDPusle <= __GmLeft.iSpeed) {
        __GmLeft.iSpeed--;
    } else {                                                            /*  非减速区间，则加速到最大值  */
        if (__GmLeft.iSpeed < __GiMaxSpeed) {
            __GmLeft.iSpeed++;
        }
    }
    if (__GmLeft.iSpeed < 0) {                                          /*  设置速度下限                */
        __GmLeft.iSpeed = 0;
    }
    TimerLoadSet(TIMER1_BASE,TIMER_A,__GuiAccelTable[__GmLeft.iSpeed]); /*  设置定时时间                */
}


/*********************************************************************************************************
** Function name:       Timer0A_ISR
** Descriptions:        Timer0中断服务函数
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer0A_ISR(void)
{
    static char n = 0,m = 0;
    
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                     /*  清除定时器0中断。           */
    switch (__GmRight.cState) {
        
    case __MOTORSTOP:                                                   /*  停止，同时清零速度和脉冲值  */
        __GmRight.iSpeed     = 0;
        __GmRight.uiPulse    = 0;
        __GmRight.uiPulseCtr = 0;
        break;

    case __WAITONESTEP:                                                 /*  暂停一步                    */
        __GmRight.cState     = __MOTORRUN;
        break;

    case __MOTORRUN:                                                    /*  电机运行                    */
        if (__GucMouseState == __GOAHEAD) {                             /*  根据传感器状态微调电机位置  */
            if (__GucDistance[__FRONTL] && (__GucDistance[__FRONT] == 0)) { /*左前方检测到墙壁，前方无挡板姿势调整*/
                if (n == 1) {
                    __GmRight.cState = __WAITONESTEP;
                }
                n++;
                n %= 2;                                                   /*运行一步，暂停一步,相当于速度减半*/
            } else {
                n = 0;
            }
            
            if ((__GucDistance[__RIGHT] == 1) && (__GucDistance[__LEFT] == 0))/*右边有墙，且距离较远，左方无挡板  */
            {
                if(m == 1) 
                {
                    __GmRight.cState = __WAITONESTEP;
                }
                m++;
                m %= 2;                                                   /*运行3步，暂停3步           */
            } else 
            {
                m  = 0;
            }
        }
        __rightMotorContr();                                            /*  电机驱动程序                */
        break;

    default:
        break;
    }
    /*
     *  是否完成任务判断
     */
    if (__GmRight.cState != __MOTORSTOP) {
        __GmRight.uiPulseCtr++;                                         /*  运行脉冲计数                */
        __speedContrR();                                                /*  速度调节                    */
        if (__GmRight.uiPulseCtr >= __GmRight.uiPulse) {
            __GmRight.cState      = __MOTORSTOP;
            __GmRight.uiPulseCtr  = 0;
            __GmRight.uiPulse     = 0;
            __GmRight.iSpeed      = 0;
        }
    }
}


/*********************************************************************************************************
** Function name:       Timer1A_ISR
** Descriptions:        Timer1中断服务函数
** input parameters:    __GmLeft.cState :驱动步进电机的时序状态
**                      __GmLeft.cDir   :步进电机运动的方向
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Timer1A_ISR(void)
{
    static char n = 0, m = 0;
    
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                     /*  清除定时器1中断。           */
    switch (__GmLeft.cState) {
        
    case __MOTORSTOP:                                                   /*  停止，同时清零速度和脉冲值  */
        __GmLeft.iSpeed     = 0;
        __GmLeft.uiPulse    = 0;
        __GmLeft.uiPulseCtr = 0;
        break;
        
    case __WAITONESTEP:                                                 /*  暂停一步                    */
        __GmLeft.cState     = __MOTORRUN;
        break;

    case __MOTORRUN:                                                    /*  电机运行                    */
        if (__GucMouseState == __GOAHEAD) {                             /*  根据传感器状态微调电机位置  */
            if (__GucDistance[__FRONTR] &&(__GucDistance[__FRONT]==0)) {
                if (n == 1) {
                    __GmLeft.cState = __WAITONESTEP;
                }
                n++;
                n %= 2;
            } else {
                n = 0;
            }
            if ((__GucDistance[__LEFT] == 1) && (__GucDistance[__RIGHT] == 0)) {
                if(m == 1) {
                    __GmLeft.cState = __WAITONESTEP;
                }
                m++;
                m %= 2;
            } else {
                m  = 0;
            }
        }
        __leftMotorContr();                                             /*  电机驱动程序                */
        break;

    default:
        break;
    }
    /*
     *  是否完成任务判断
     */
    if (__GmLeft.cState != __MOTORSTOP) {
        __GmLeft.uiPulseCtr++;                                          /*  运行脉冲计数                */
        __speedContrL();                                                /*  速度调节                    */
        if (__GmLeft.uiPulseCtr >= __GmLeft.uiPulse) {
            __GmLeft.cState      = __MOTORSTOP;
            __GmLeft.uiPulseCtr  = 0;
            __GmLeft.uiPulse     = 0;
            __GmLeft.iSpeed      = 0;
        }
    }
}


/*********************************************************************************************************
** Function name:       mouseGoahead
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseGoahead (char  cNBlock)
{
    char cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState)            //电机非停止状态
    {
        cCoor = 0;
    }
    /*
     *  设定运行任务
     */
    __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   MAXSPEED;
    __GmRight.cDir    = __MOTORGOAHEAD;
    __GmLeft.cDir     = __MOTORGOAHEAD;
    __GmRight.uiPulse = __GmRight.uiPulse + cNBlock * ONEBLOCK - 6;
    __GmLeft.uiPulse  = __GmLeft.uiPulse  + cNBlock * ONEBLOCK - 6;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
    while (__GmLeft.cState != __MOTORSTOP) {
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                cNBlock--;
                __mouseCoorUpdate();                                    /*  更新坐标                    */
            } 
            else 
            {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (__GucDistance[__FRONT]) {                                   /*  前方有墙，则跳出程序        */
            __GmRight.uiPulse = __GmRight.uiPulseCtr + 70;
            __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 70;
            while (__GucDistance[ __FRONT]) {
                if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                    goto End;
                }
            }
        }
        if (cNBlock < 2) {
            if (cL) {                                                   /*  是否允许检测左边            */
                if ((__GucDistance[ __LEFT] & 0x01) == 0) {             /*  左边有支路，跳出程序        */
                    __GmRight.uiPulse = __GmRight.uiPulseCtr + 74;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 74;
                    while ((__GucDistance[ __LEFT] & 0x01) == 0) {
                        if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                            goto End;
                        }
                    }
                }
            } else {                                                    /*  左边有墙时开始允许检测左边  */
                if ( __GucDistance[ __LEFT] & 0x01) {
                    cL = 1;
                }
            }
            if (cR) {                                                   /*  是否允许检测右边            */
                if ((__GucDistance[__RIGHT] & 0x01) == 0) {             /*  右边有支路，跳出程序        */
                    __GmRight.uiPulse = __GmRight.uiPulseCtr + 74;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 74;
                    while ((__GucDistance[ __RIGHT] & 0x01) == 0) {
                        if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                            goto End;
                        }
                    }
                }
            } else {
                if ( __GucDistance[__RIGHT] & 0x01) {                   /*  右边有墙时开始允许检测右边  */
                    cR = 1;
                }
            }
        }
    }
    /*
     *  设定运行任务，让电脑鼠走到支路的中心位置
     */
End:    __mouseCoorUpdate();                                            /*  更新坐标                    */
}

/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        前进N格
** input parameters:    iNblock: 前进的格数
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mazeSearch(void)
{
    char cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    /*
     *  设定运行任务
     */
    __GucMouseState   = __GOAHEAD;
    __GiMaxSpeed      =   SEARCHSPEED;
    __GmRight.cDir    = __MOTORGOAHEAD;
    __GmLeft.cDir     = __MOTORGOAHEAD;
    __GmRight.uiPulse =   MAZETYPE * ONEBLOCK;
    __GmLeft.uiPulse  =   MAZETYPE * ONEBLOCK;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    
    while (__GmLeft.cState != __MOTORSTOP) {
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  判断是否走完一格            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                __mouseCoorUpdate();                                    /*  更新坐标                    */
            } else {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  判断是否走完一格            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (__GucDistance[__FRONT]) {                                   /*  前方有墙，则跳出程序        */
            __GmRight.uiPulse = __GmRight.uiPulseCtr + 70;
            __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 70;
            while (1) {
                if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                    goto End;
                }
            }
        }
        if (cL) {                                                       /*  是否允许检测左边            */
            if ((__GucDistance[ __LEFT] & 0x01) == 0) {                 /*  左边有支路，跳出程序        */
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 74;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 74;
                while ((__GucDistance[ __LEFT] & 0x01) == 0) {
                    if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                        goto End;
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
            }
        } else {                                                        /*  左边有墙时开始允许检测左边  */
            if ( __GucDistance[ __LEFT] & 0x01) {
                cL = 1;
            }
        }
        if (cR) {                                                       /*  是否允许检测右边            */
            if ((__GucDistance[__RIGHT] & 0x01) == 0) {                 /*  右边有支路，跳出程序        */
                __GmRight.uiPulse = __GmRight.uiPulseCtr + 74;
                __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 74;
                while ((__GucDistance[ __RIGHT] & 0x01) == 0) {
                    if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                        goto End;
                    }
                }
                __GmRight.uiPulse = MAZETYPE * ONEBLOCK;
                __GmLeft.uiPulse  = MAZETYPE * ONEBLOCK;
            }
        } else {
            if ( __GucDistance[__RIGHT] & 0x01) {                       /*  右边有墙时开始允许检测右边  */
                cR = 1;
            }
        }
    }
End:    __mouseCoorUpdate();                                            /*  更新坐标                    */
}


/*********************************************************************************************************
** Function name:       SysTick_ISR
** Descriptions:        定时中断扫描。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SysTick_ISR(void)
{
    static int iL = 0, iR = 0;
    
    /*
     *  如果左电机长时间停止，则断电
     */
    if (__GmLeft.cState == __MOTORSTOP) {
        iL++;
    } else {
        iL = 0;
    }
    if (iL >= 500) {
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                     0x00);
    }
    /*
     *  如果右电机长时间停止，则断电
     */
    if (__GmRight.cState == __MOTORSTOP) {
        iR++;
    } else {
        iR = 0;
    }
    if (iR >= 500) {
        GPIOPinWrite(GPIO_PORTD_BASE,
                     __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                     0x00);
    }
    /*
     *  红外线检测
     */
    __irCheck();
}


/*********************************************************************************************************
** Function name:       __irSendFreq
** Descriptions:        发送红外线。
** input parameters:    __uiFreq:  红外线调制频率
**                      __cNumber: 选择需要设置的PWM模块
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irSendFreq (uint  __uiFreq, char  __cNumber)
{
    __uiFreq = SysCtlClockGet() / __uiFreq;
    switch (__cNumber) {

    case 1:
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                 /*  设置PWM发生器1的周期        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, __uiFreq / 2);            /*  设置PWM2输出的脉冲宽度      */
        PWMGenEnable(PWM_BASE, PWM_GEN_1);                              /*  使能PWM发生器1              */
        break;

    case 2:
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, __uiFreq);                 /*  设置PWM发生器2的周期        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, __uiFreq / 2);            /*  设置PWM4输出的脉冲宽度      */
        PWMGenEnable(PWM_BASE, PWM_GEN_2);                              /*  使能PWM发生器2              */
        break;

    default:
        break;
    }
}


/*********************************************************************************************************
** Function name:       __irCheck
** Descriptions:        红外线传感器检测。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __irCheck (void)
{
    static uchar ucState = 0;
    static uchar ucIRCheck;
    
    switch (ucState) {

    case 0:
        __irSendFreq(32200, 2);                                         /*  探测左右两侧近距            */
        __irSendFreq(35000, 1);                                         /*  驱动斜角上的传感器检测      */
        break;
        
    case 1:
        ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x3e);                 /*  读取传感器状态              */
        PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  禁止PWM发生器2              */
        PWMGenDisable(PWM_BASE, PWM_GEN_1);                             /*  禁止PWM发生器1              */
        if (ucIRCheck & __RIGHTSIDE) {
            __GucDistance[__RIGHT]  &= 0xfd;
        } else {
            __GucDistance[__RIGHT]  |= 0x02;
        }
        if (ucIRCheck & __LEFTSIDE) {
            __GucDistance[__LEFT]   &= 0xfd;
        } else {
            __GucDistance[__LEFT]   |= 0x02;
        }
        if (ucIRCheck & __FRONTSIDE_R) {
            __GucDistance[__FRONTR]  = 0x00;
        } else {
            __GucDistance[__FRONTR]  = 0x01;
        }
        if (ucIRCheck & __FRONTSIDE_L) {
            __GucDistance[__FRONTL]  = 0x00;
        } else {
            __GucDistance[__FRONTL]  = 0x01;
        }
        break;

    case 2:
        __irSendFreq(36000, 2);                                         /*  驱动检测左前右三个方向远距  */
        break;
        
    case 3:
        ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x2a);                 /*  读取传感器状态              */
        PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  禁止PWM发生器2              */
        break;

    case 4:
        __irSendFreq(36000, 2);                                         /*  重复检测左前右三个方向远距  */
        break;
        
    case 5:
        ucIRCheck &= GPIOPinRead(GPIO_PORTB_BASE, 0x2a);                /*  读取传感器状态              */
        PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  禁止PWM发生器2              */
        if (ucIRCheck & __RIGHTSIDE) {
            __GucDistance[__RIGHT] &= 0xfe;
        } else {
            __GucDistance[__RIGHT] |= 0x01;
        }
        if (ucIRCheck & __LEFTSIDE) {
            __GucDistance[__LEFT]  &= 0xfe;
        } else {
            __GucDistance[__LEFT]  |= 0x01;
        }
        if (ucIRCheck & __FRONTSIDE) {
            __GucDistance[__FRONT] &= 0xfe;
        } else {
            __GucDistance[__FRONT] |= 0x01;
        }
        break;

    default:
        break;
    }
    ucState = (ucState + 1) % 6;                                        /*  循环检测                    */
}


/*********************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        右转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnright(void)
{
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    /*
     *  开始右转
     */
     __GucMouseState   = __TURNRIGHT;
    __GmRight.cDir    = __MOTORGOBACK;
    __GmRight.uiPulse = 45;
    
    __GmLeft.cDir     = __MOTORGOAHEAD;
    __GmLeft.uiPulse  = 45;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  方向标记                    */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    __mazeInfDebug();
    __delay(100000);
}


/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        左转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnleft(void)
{
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    /*
     *  开始左转
     */
    __GucMouseState   = __TURNLEFT;
    __GmRight.cDir    = __MOTORGOAHEAD;
    __GmRight.uiPulse = 47;
    
    __GmLeft.cDir     = __MOTORGOBACK;
    __GmLeft.uiPulse  = 47;
    
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  方向标记                    */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    __mazeInfDebug();
    __delay(100000);
}


/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        后转
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseTurnback(void)
{
    /*
     *  等待停止
     */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    /*
     *  开始后转
     */
    __GucMouseState   = __TURNBACK;
    __GmRight.cDir    = __MOTORGOBACK;
    __GmRight.uiPulse = 90;//162 * 10;
    
    __GmLeft.cDir     = __MOTORGOAHEAD;
    __GmLeft.uiPulse  = 90;//162 * 10;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;                                /*  方向标记                    */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    __mazeInfDebug();
    __delay(100000);
}


/*********************************************************************************************************
** Function name:       __mouseCoorUpdate
** Descriptions:        根据当前方向更新坐标值
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mouseCoorUpdate (void)
{
    switch (GucMouseDir) {

    case 0:
        GmcMouse.cY++;
        break;

    case 1:
        GmcMouse.cX++;
        break;

    case 2:
        GmcMouse.cY--;
        break;

    case 3:
        GmcMouse.cX--;
        break;

    default:
        break;
    }
    __mazeInfDebug();
    __wallCheck();
}


/*********************************************************************************************************
** Function name:       __wallCheck
** Descriptions:        根据传感器检测结果判断是否存在墙壁
** input parameters:    无
** output parameters:   无
** Returned value:      cValue: 低三位从左到右一次代表左前右。1为有墙，0为没墙。
*********************************************************************************************************/
void __wallCheck (void)
{
    uchar ucMap = 0;
    ucMap |= MOUSEWAY_B;
    
    if (__GucDistance[__LEFT]  & 0x01) {
        ucMap &= ~MOUSEWAY_L;
    }else {
        ucMap |=  MOUSEWAY_L;
    }
    if (__GucDistance[__FRONT] & 0x01) {
        ucMap &= ~MOUSEWAY_F;
    }else {
        ucMap |=  MOUSEWAY_F;
    }
    if (__GucDistance[__RIGHT] & 0x01) {
        ucMap &= ~MOUSEWAY_R;
    }else {
        ucMap |=  MOUSEWAY_R;
    }
    if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] == 0x00) {
        GucMapBlock[GmcMouse.cX][GmcMouse.cY] = ucMap;
    } else {
        if (GucMapBlock[GmcMouse.cX][GmcMouse.cY] != ucMap) {
            Download_7289(1,4,0,GucMapBlock[GmcMouse.cX][GmcMouse.cY]);
            Download_7289(1,5,0,ucMap);
            //while(keyCheck() == false);
        }
    }
}


/*********************************************************************************************************
** Function name:       SensorDebug
** Descriptions:        用数码管显示出传感器状态，方便调试
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void sensorDebug (void)
{
    Download_7289(2, 0, 0, __GucDistance[__LEFT  ]);
    Download_7289(2, 1, 0, __GucDistance[__FRONTL]);
    Download_7289(2, 2, 0, __GucDistance[__FRONT ]);
    Download_7289(2, 3, 0, __GucDistance[__FRONTR]);    
    Download_7289(2, 4, 0, __GucDistance[__RIGHT ]);    
}


/*********************************************************************************************************
** Function name:       __mazeInfDebug
** Descriptions:        用数码管显示出当前电脑鼠前进方向和坐标
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __mazeInfDebug (void)
{
    /*
     *  显示方向
     */
    switch (GucMouseDir) {
        
    case 0:
        Download_7289(2, 3, 0, 0x47);                                /*  向前，用F表示               */
        break;
        
    case 1:
        Download_7289(2, 3, 0, 0x77);                                /*  向右，用R表示               */
        break;
        
    case 2:
        Download_7289(2, 3, 0, 0x1f);                                /*  向后，用b表示               */
        break;
        
    case 3:
        Download_7289(2, 3, 0, 0x0e);                                /*  向左，用L表示               */
        break;
        
    default :
        Download_7289(2, 3, 0, 0x4f);                                /*  错误，用E表示               */
        break;
    }
    /*
     *  显示坐标
     */
    Download_7289(1, 0, 0, GmcMouse.cX / 10);
    Download_7289(1, 1, 0, GmcMouse.cX % 10);
    Download_7289(1, 6, 0, GmcMouse.cY / 10);
    Download_7289(1, 7, 0, GmcMouse.cY % 10);
}


/*********************************************************************************************************
** Function name:       keyCheck
** Descriptions:        读取按键
** input parameters:    无
** output parameters:   无
** Returned value:      true:  按键已按下
**                      false: 按键未按下
*********************************************************************************************************/
uchar keyCheck (void)
{
    if (GPIOPinRead(GPIO_PORTC_BASE, __KEY) == 0) {
        __delay(50);
        while(GPIOPinRead(GPIO_PORTC_BASE, __KEY) == 0);
        return(true);
    }else {
        return(false);
    }
}


/*********************************************************************************************************
** Function name:       voltageDetect
** Descriptions:        电压检测，检测结果在7289 EX BOARD 上显示出来
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void voltageDetect (void)
{
    unsigned long ulVoltage;
    
    ADCProcessorTrigger(ADC_BASE, 0);                                   /*  处理器触发一次A/D转换       */
    while (!ADCIntStatus(ADC_BASE, 0, false));                          /*  等待转换结束                */
    ADCIntClear(ADC_BASE, 0);                                           /*  清除中断标准位              */
    ADCSequenceDataGet(ADC_BASE, 0, &ulVoltage);                        /*  读取转换结果                */
    
    ulVoltage = ulVoltage * 3000 / 1023;                                /*  计算实际检测到的电压值(mV)  */
    ulVoltage = ulVoltage * 3 + 350;                                    /*  计算电池电压值(mV)          */
    
    Download_7289(0,6,1,(ulVoltage % 10000) / 1000);                  /*  显示电压值整数部分，单位V   */
    Download_7289(0,7,0,(ulVoltage % 1000 ) / 100 );                  /*  显示电压值小数部分，单位V   */
}


/*********************************************************************************************************
** Function name:       mouseInit
** Descriptions:        对LM3S615处理器进行初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void mouseInit (void)
{
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_6MHZ );                                 /*  使能PLL，50M                */

    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );                      /*  使能GPIO B口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );                      /*  使能GPIO C口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );                      /*  使能GPIO D口外设            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );                      /*  使能GPIO E口外设            */
    __keyInit();                                                        /*  按键初始化                  */
    __sensorInit();                                                     /*  传感器初始化                */
    __stepMotorIint();                                                  /*  步进电机控制初始化          */
    __sysTickInit();                                                    /*  系统时钟初始化              */
    __ADCInit();
    GucMapBlock[0][0] = 0x01;
}


/*********************************************************************************************************
** Function name:       __sensorInit
** Descriptions:        传感器控制初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __sensorInit (void)
{
    /*
     *  设置连接到传感器信号输出脚的I/O口为输入模式
     */
    GPIODirModeSet(GPIO_PORTB_BASE,
                   __LEFTSIDE    |
                   __FRONTSIDE_L |
                   __FRONTSIDE   |
                   __FRONTSIDE_R |
                   __RIGHTSIDE,  
                   GPIO_DIR_MODE_IN);
    /*
     *  用PWM驱动红外线发射头产生调制的红外线信号
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  使能PWM模块                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                 /*  PWM时钟配置：不分频         */
    /*
     *  初始化PWM2，该PWM驱动斜角红外发射头
     */
    GPIOPinTypePWM(GPIO_PORTB_BASE, __IRSEND_BEVEL);                    /*  PB0配置为PWM功能            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_1,                                /*  配置PWM发生器1              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */

    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, true);                      /*  使能PWM2输出                */
    PWMGenDisable(PWM_BASE, PWM_GEN_1);                                 /*  禁止PWM发生器1              */
    /*
     *  初始化PWM4，该PWM驱动左前右正方向红外发射头
     */
    GPIOPinTypePWM(GPIO_PORTE_BASE, __IRSEND_SIDE);                     /*  PE0配置为PWM功能            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_2,                                /*  配置PWM发生器2              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  加计数，立即更新            */

    PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);                      /*  使能PWM4输出                */
    PWMGenDisable(PWM_BASE, PWM_GEN_2);                                 /*  禁止PWM发生器2              */
}


/*********************************************************************************************************
** Function name:       __stepMotorIint
** Descriptions:        步进电机控制初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __stepMotorIint (void)
{
    uint n = 0;
    /*
     *  设置驱动步进电机的八个I/O口为输出模式
     */
    GPIODirModeSet(GPIO_PORTD_BASE,
                   __PHRA1 |
                   __PHRA2 |
                   __PHRB1 |
                   __PHRB2 |
                   __PHLA1 |
                   __PHLA2 |
                   __PHLB1 |
                   __PHLB2,
                   GPIO_DIR_MODE_OUT);
    /*
     *  对左右电机转动的位置初始化
     */
    GPIOPinWrite(GPIO_PORTD_BASE,
                 __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                 __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2);
    
    GPIOPinWrite(GPIO_PORTD_BASE,
                 __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                 __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2);
    /*
     *  初始化加速/减速时定时器加载值的数据表
     */
    __GuiAccelTable[0] = 2236068;
    __GuiAccelTable[1] = 906949;
    for(n = 2; n < 400; n++) {
        __GuiAccelTable[n] = __GuiAccelTable[n - 1] - (2 * __GuiAccelTable[n - 1] / (4 * n + 1));
    }
    /*
     *  初始化定时器0，用来控制右电机的转速
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                       /*  使能定时器0模块             */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);                  /*  配置为32位周期计数模式      */
    TimerLoadSet(TIMER0_BASE, TIMER_A, __GuiAccelTable[0]);             /*  设置定时时间                */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                    /*  设置为溢出中断              */

    IntEnable(INT_TIMER0A);                                             /*  使能定时器0中断             */
    TimerEnable(TIMER0_BASE, TIMER_A);                                  /*  使能定时器0                 */
    
    /*
     *  初始化定时器1，用来控制电机的转速
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);                       /*  使能定时器1模块             */
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);                  /*  配置为32位周期计数模式      */
    TimerLoadSet(TIMER1_BASE, TIMER_A, __GuiAccelTable[0]);             /*  设置定时时间                */
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                    /*  设置为溢出中断              */

    IntEnable(INT_TIMER1A);                                             /*  使能定时器1中断             */
    TimerEnable(TIMER1_BASE, TIMER_A);                                  /*  使能定时器1                 */
}


/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        对连接按键的GPIO口初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __keyInit (void)
{
    GPIODirModeSet(GPIO_PORTC_BASE, __KEY, GPIO_DIR_MODE_IN);           /*  设置按键口为输入            */
}


/*********************************************************************************************************
** Function name:       __sysTickInit
** Descriptions:        系统节拍定时器初始化。
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __sysTickInit (void)
{
    SysTickPeriodSet(SysCtlClockGet() / 1600);                          /*  设置定时时钟为1ms           */
    SysTickEnable();                                                    /*  使能系统时钟                */
    SysTickIntEnable();                                                 /*  使能系统时钟中断            */
}


/*********************************************************************************************************
** Function name:       __ADCInit
** Descriptions:        对连接按键的GPIO口初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void __ADCInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);                          /*  使能ADC模块                 */
    SysCtlADCSpeedSet(SYSCTL_ADCSPEED_125KSPS);                         /*  125KSps采样率               */

    ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);        /*  序列0为处理器触发，优先级为0*/
    ADCSequenceStepConfigure(ADC_BASE, 0, 0,
                             ADC_CTL_CH0  | 
                             ADC_CTL_IE   | 
                             ADC_CTL_END);                              /*  配置采样序列发生器的步进    */
    
    ADCHardwareOversampleConfigure(ADC_BASE, 16);                       /*  设置ADC采样平均控制寄存器   */
    ADCSequenceEnable(ADC_BASE, 0);                                     /*  使能采样序列0               */
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
