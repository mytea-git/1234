


/*********************************************************************************************************
  ����ͷ�ļ�
*********************************************************************************************************/
#include "Mouse_Drive.h"
#include "astar_core.h"
#include "path_optimizer.h"
#include "motion_params.h"

#define USE_ASTAR_ALGORITHM  1


/*********************************************************************************************************
  ����ȫ�ֱ���
*********************************************************************************************************/
MAZECOOR          GmcMouse                        = {0,0};              /*  ���������ǰλ������      */

uchar             GucMouseDir                     = UP;                 /*  ���������ǰ����          */

uchar             GucMapBlock[MAZETYPE][MAZETYPE] = {0};                /*  GucMapBlock[x][y]           */
                                                                        /*  x,������;y,������;          */
                                                                        /*  bit3~bit0�ֱ������������   */
                                                                        /*  0:�÷�����·��1:�÷�����·  */

static __MOTOR  __GmLeft                          = {0, 0, 0, 0, 0};    /*  ���岢��ʼ������״̬      */
static __MOTOR  __GmRight                         = {0, 0, 0, 0, 0};    /*  ���岢��ʼ���ҵ��״̬      */

static uchar    __GucMouseState                   = __STOP;             /*  ���������ǰ����״̬      */
static uint     __GuiAccelTable[400]              = {0};                 /*  ����Ӽ��ٸ��׶ζ�ʱ��ֵ    */
static int      __GiMaxSpeed                      = SEARCHSPEED;        /*  �����������е�����ٶ�      */
static uchar    __GucDistance[5]                  = {0};                /*  ��¼������״̬              */


/*********************************************************************************************************
** Function name:       __delay
** Descriptions:        ��ʱ����
** input parameters:    uiD :��ʱ������ֵԽ����ʱԽ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __delay (uint  uiD)
{
    for (; uiD; uiD--);
}


/*********************************************************************************************************
** Function name:       __rightMotorContr
** Descriptions:        �Ҳ����������ʱ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __rightMotorContr (void)
{
    static char cStep = 0;                                              /*  ��������ǰλ��            */
    
    switch (__GmRight.cDir) {

    case __MOTORGOAHEAD:                                                /*  ��ǰ����                    */
        cStep = (cStep + 1) % 8;
        break;

    case __MOTORGOBACK:                                                 /*  ��󲽽�                    */
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
** Descriptions:        �󲽽��������ʱ��
** input parameters:    __GmLeft.cDir :������з���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __leftMotorContr (void)
{
    static char cStep = 0;                                              /*  ��������ǰλ��            */
    
    switch (__GmLeft.cDir) {
        
    case __MOTORGOAHEAD:                                                /*  ��ǰ����                    */
        cStep = (cStep + 1) % 8;
        break;
        
    case __MOTORGOBACK:                                                 /*  ��󲽽�                    */
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
** Descriptions:        �ҵ���ٶȵ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __speedContrR (void)
{
    int iDPusle;
    
    iDPusle = __GmRight.uiPulse - __GmRight.uiPulseCtr;                 /*  ͳ�Ƶ����ʣ��Ĳ���        */
    if (iDPusle <= __GmRight.iSpeed) {
        __GmRight.iSpeed--;
    } else {                                                            /*  �Ǽ������䣬����ٵ����ֵ  */
        if (__GmRight.iSpeed < __GiMaxSpeed) {
            __GmRight.iSpeed++;
        } else {
            __GmRight.iSpeed--;
        }
    }
    if (__GmRight.iSpeed < 0) {                                         /*  �����ٶ�����                */
        __GmRight.iSpeed = 0;
    }
    TimerLoadSet(TIMER0_BASE,TIMER_A,__GuiAccelTable[__GmRight.iSpeed]);/*  ���ö�ʱʱ��                */
}


/*********************************************************************************************************
** Function name:       __speedContrL
** Descriptions:        �����ٶȵ���
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __speedContrL (void)
{
    int iDPusle;
    
    iDPusle = __GmLeft.uiPulse - __GmLeft.uiPulseCtr;                   /*  ͳ�Ƶ����ʣ��Ĳ���        */
    if (iDPusle <= __GmLeft.iSpeed) {
        __GmLeft.iSpeed--;
    } else {                                                            /*  �Ǽ������䣬����ٵ����ֵ  */
        if (__GmLeft.iSpeed < __GiMaxSpeed) {
            __GmLeft.iSpeed++;
        }
    }
    if (__GmLeft.iSpeed < 0) {                                          /*  �����ٶ�����                */
        __GmLeft.iSpeed = 0;
    }
    TimerLoadSet(TIMER1_BASE,TIMER_A,__GuiAccelTable[__GmLeft.iSpeed]); /*  ���ö�ʱʱ��                */
}


/*********************************************************************************************************
** Function name:       Timer0A_ISR
** Descriptions:        Timer0�жϷ�����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer0A_ISR(void)
{
    static char n = 0,m = 0;
    
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                     /*  �����ʱ��0�жϡ�           */
    switch (__GmRight.cState) {
        
    case __MOTORSTOP:                                                   /*  ֹͣ��ͬʱ�����ٶȺ�����ֵ  */
        __GmRight.iSpeed     = 0;
        __GmRight.uiPulse    = 0;
        __GmRight.uiPulseCtr = 0;
        break;

    case __WAITONESTEP:                                                 /*  ��ͣһ��                    */
        __GmRight.cState     = __MOTORRUN;
        break;

    case __MOTORRUN:                                                    /*  �������                    */
        if (__GucMouseState == __GOAHEAD) {                             /*  ���ݴ�����״̬΢�����λ��  */
            if (__GucDistance[__FRONTL] && (__GucDistance[__FRONT] == 0)) { /*��ǰ����⵽ǽ�ڣ�ǰ���޵������Ƶ���*/
                if (n == 1) {
                    __GmRight.cState = __WAITONESTEP;
                }
                n++;
                n %= 2;                                                   /*����һ������ͣһ��,�൱���ٶȼ���*/
            } else {
                n = 0;
            }
            
            if ((__GucDistance[__RIGHT] == 1) && (__GucDistance[__LEFT] == 0))/*�ұ���ǽ���Ҿ����Զ�����޵���  */
            {
                if(m == 1) 
                {
                    __GmRight.cState = __WAITONESTEP;
                }
                m++;
                m %= 2;                                                   /*����3������ͣ3��           */
            } else 
            {
                m  = 0;
            }
        }
        __rightMotorContr();                                            /*  �����������                */
        break;

    default:
        break;
    }
    /*
     *  �Ƿ���������ж�
     */
    if (__GmRight.cState != __MOTORSTOP) {
        __GmRight.uiPulseCtr++;                                         /*  �����������                */
        __speedContrR();                                                /*  �ٶȵ���                    */
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
** Descriptions:        Timer1�жϷ�����
** input parameters:    __GmLeft.cState :�������������ʱ��״̬
**                      __GmLeft.cDir   :��������˶��ķ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Timer1A_ISR(void)
{
    static char n = 0, m = 0;
    
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                     /*  �����ʱ��1�жϡ�           */
    switch (__GmLeft.cState) {
        
    case __MOTORSTOP:                                                   /*  ֹͣ��ͬʱ�����ٶȺ�����ֵ  */
        __GmLeft.iSpeed     = 0;
        __GmLeft.uiPulse    = 0;
        __GmLeft.uiPulseCtr = 0;
        break;
        
    case __WAITONESTEP:                                                 /*  ��ͣһ��                    */
        __GmLeft.cState     = __MOTORRUN;
        break;

    case __MOTORRUN:                                                    /*  �������                    */
        if (__GucMouseState == __GOAHEAD) {                             /*  ���ݴ�����״̬΢�����λ��  */
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
        __leftMotorContr();                                             /*  �����������                */
        break;

    default:
        break;
    }
    /*
     *  �Ƿ���������ж�
     */
    if (__GmLeft.cState != __MOTORSTOP) {
        __GmLeft.uiPulseCtr++;                                          /*  �����������                */
        __speedContrL();                                                /*  �ٶȵ���                    */
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
** Descriptions:        ǰ��N��
** input parameters:    iNblock: ǰ���ĸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseGoahead (char  cNBlock)
{
    char cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState)            //�����ֹͣ״̬
    {
        cCoor = 0;
    }
    /*
     *  �趨��������
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
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  �ж��Ƿ�����һ��            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) 
            {
                cNBlock--;
                __mouseCoorUpdate();                                    /*  ��������                    */
            } 
            else 
            {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (__GucDistance[__FRONT]) {                                   /*  ǰ����ǽ������������        */
            __GmRight.uiPulse = __GmRight.uiPulseCtr + 70;
            __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 70;
            while (__GucDistance[ __FRONT]) {
                if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                    goto End;
                }
            }
        }
        if (cNBlock < 2) {
            if (cL) {                                                   /*  �Ƿ�����������            */
                if ((__GucDistance[ __LEFT] & 0x01) == 0) {             /*  �����֧·����������        */
                    __GmRight.uiPulse = __GmRight.uiPulseCtr + 74;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 74;
                    while ((__GucDistance[ __LEFT] & 0x01) == 0) {
                        if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                            goto End;
                        }
                    }
                }
            } else {                                                    /*  �����ǽʱ��ʼ����������  */
                if ( __GucDistance[ __LEFT] & 0x01) {
                    cL = 1;
                }
            }
            if (cR) {                                                   /*  �Ƿ���������ұ�            */
                if ((__GucDistance[__RIGHT] & 0x01) == 0) {             /*  �ұ���֧·����������        */
                    __GmRight.uiPulse = __GmRight.uiPulseCtr + 74;
                    __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 74;
                    while ((__GucDistance[ __RIGHT] & 0x01) == 0) {
                        if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                            goto End;
                        }
                    }
                }
            } else {
                if ( __GucDistance[__RIGHT] & 0x01) {                   /*  �ұ���ǽʱ��ʼ��������ұ�  */
                    cR = 1;
                }
            }
        }
    }
    /*
     *  �趨���������õ������ߵ�֧·������λ��
     */
End:    __mouseCoorUpdate();                                            /*  ��������                    */
}

/*********************************************************************************************************
** Function name:       mazeSearch
** Descriptions:        ǰ��N��
** input parameters:    iNblock: ǰ���ĸ���
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mazeSearch(void)
{
    char cL = 0, cR = 0, cCoor = 1;
    if (__GmLeft.cState) {
        cCoor = 0;
    }
    /*
     *  �趨��������
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
        if (__GmLeft.uiPulseCtr >= ONEBLOCK) {                          /*  �ж��Ƿ�����һ��            */
            __GmLeft.uiPulse    -= ONEBLOCK;
            __GmLeft.uiPulseCtr -= ONEBLOCK;
            if (cCoor) {
                __mouseCoorUpdate();                                    /*  ��������                    */
            } else {
                cCoor = 1;
            }
        }
        if (__GmRight.uiPulseCtr >= ONEBLOCK) {                         /*  �ж��Ƿ�����һ��            */
            __GmRight.uiPulse    -= ONEBLOCK;
            __GmRight.uiPulseCtr -= ONEBLOCK;
        }
        if (__GucDistance[__FRONT]) {                                   /*  ǰ����ǽ������������        */
            __GmRight.uiPulse = __GmRight.uiPulseCtr + 70;
            __GmLeft.uiPulse  = __GmLeft.uiPulseCtr  + 70;
            while (1) {
                if ((__GmLeft.uiPulseCtr + 20) > __GmLeft.uiPulse) {
                    goto End;
                }
            }
        }
        if (cL) {                                                       /*  �Ƿ�����������            */
            if ((__GucDistance[ __LEFT] & 0x01) == 0) {                 /*  �����֧·����������        */
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
        } else {                                                        /*  �����ǽʱ��ʼ����������  */
            if ( __GucDistance[ __LEFT] & 0x01) {
                cL = 1;
            }
        }
        if (cR) {                                                       /*  �Ƿ���������ұ�            */
            if ((__GucDistance[__RIGHT] & 0x01) == 0) {                 /*  �ұ���֧·����������        */
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
            if ( __GucDistance[__RIGHT] & 0x01) {                       /*  �ұ���ǽʱ��ʼ��������ұ�  */
                cR = 1;
            }
        }
    }
End:    __mouseCoorUpdate();                                            /*  ��������                    */
}


/*********************************************************************************************************
** Function name:       SysTick_ISR
** Descriptions:        ��ʱ�ж�ɨ�衣
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void SysTick_ISR(void)
{
    static int iL = 0, iR = 0;
    
    /*
     *  ���������ʱ��ֹͣ����ϵ�
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
     *  ����ҵ����ʱ��ֹͣ����ϵ�
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
     *  �����߼��
     */
    __irCheck();
}


/*********************************************************************************************************
** Function name:       __irSendFreq
** Descriptions:        ���ͺ����ߡ�
** input parameters:    __uiFreq:  �����ߵ���Ƶ��
**                      __cNumber: ѡ����Ҫ���õ�PWMģ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __irSendFreq (uint  __uiFreq, char  __cNumber)
{
    __uiFreq = SysCtlClockGet() / __uiFreq;
    switch (__cNumber) {

    case 1:
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, __uiFreq);                 /*  ����PWM������1������        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, __uiFreq / 2);            /*  ����PWM2������������      */
        PWMGenEnable(PWM_BASE, PWM_GEN_1);                              /*  ʹ��PWM������1              */
        break;

    case 2:
        PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, __uiFreq);                 /*  ����PWM������2������        */
        PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, __uiFreq / 2);            /*  ����PWM4������������      */
        PWMGenEnable(PWM_BASE, PWM_GEN_2);                              /*  ʹ��PWM������2              */
        break;

    default:
        break;
    }
}


/*********************************************************************************************************
** Function name:       __irCheck
** Descriptions:        �����ߴ�������⡣
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __irCheck (void)
{
    static uchar ucState = 0;
    static uchar ucIRCheck;
    
    switch (ucState) {

    case 0:
        __irSendFreq(32200, 2);                                         /*  ̽�������������            */
        __irSendFreq(35000, 1);                                         /*  ����б���ϵĴ��������      */
        break;
        
    case 1:
        ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x3e);                 /*  ��ȡ������״̬              */
        PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  ��ֹPWM������2              */
        PWMGenDisable(PWM_BASE, PWM_GEN_1);                             /*  ��ֹPWM������1              */
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
        __irSendFreq(36000, 2);                                         /*  ���������ǰ����������Զ��  */
        break;
        
    case 3:
        ucIRCheck = GPIOPinRead(GPIO_PORTB_BASE, 0x2a);                 /*  ��ȡ������״̬              */
        PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  ��ֹPWM������2              */
        break;

    case 4:
        __irSendFreq(36000, 2);                                         /*  �ظ������ǰ����������Զ��  */
        break;
        
    case 5:
        ucIRCheck &= GPIOPinRead(GPIO_PORTB_BASE, 0x2a);                /*  ��ȡ������״̬              */
        PWMGenDisable(PWM_BASE, PWM_GEN_2);                             /*  ��ֹPWM������2              */
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
    ucState = (ucState + 1) % 6;                                        /*  ѭ�����                    */
}


/*********************************************************************************************************
** Function name:       mouseTurnright
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnright(void)
{
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    /*
     *  ��ʼ��ת
     */
     __GucMouseState   = __TURNRIGHT;
    __GmRight.cDir    = __MOTORGOBACK;
    __GmRight.uiPulse = 45;
    
    __GmLeft.cDir     = __MOTORGOAHEAD;
    __GmLeft.uiPulse  = 45;
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    GucMouseDir     = (GucMouseDir + 1) % 4;                            /*  ������                    */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    __mazeInfDebug();
    __delay(100000);
}


/*********************************************************************************************************
** Function name:       mouseTurnleft
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnleft(void)
{
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    /*
     *  ��ʼ��ת
     */
    __GucMouseState   = __TURNLEFT;
    __GmRight.cDir    = __MOTORGOAHEAD;
    __GmRight.uiPulse = 47;
    
    __GmLeft.cDir     = __MOTORGOBACK;
    __GmLeft.uiPulse  = 47;
    
    __GmRight.cState  = __MOTORRUN;
    __GmLeft.cState   = __MOTORRUN;
    GucMouseDir     = (GucMouseDir + 3) % 4;                            /*  ������                    */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    __mazeInfDebug();
    __delay(100000);
}


/*********************************************************************************************************
** Function name:       mouseTurnback
** Descriptions:        ��ת
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseTurnback(void)
{
    /*
     *  �ȴ�ֹͣ
     */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    /*
     *  ��ʼ��ת
     */
    __GucMouseState   = __TURNBACK;
    __GmRight.cDir    = __MOTORGOBACK;
    __GmRight.uiPulse = 90;//162 * 10;
    
    __GmLeft.cDir     = __MOTORGOAHEAD;
    __GmLeft.uiPulse  = 90;//162 * 10;
    __GmLeft.cState   = __MOTORRUN;
    __GmRight.cState  = __MOTORRUN;
    GucMouseDir = (GucMouseDir + 2) % 4;                                /*  ������                    */
    while (__GmLeft.cState  != __MOTORSTOP);
    while (__GmRight.cState != __MOTORSTOP);
    __mazeInfDebug();
    __delay(100000);
}


/*********************************************************************************************************
** Function name:       __mouseCoorUpdate
** Descriptions:        ���ݵ�ǰ�����������ֵ
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ���ݴ�����������ж��Ƿ����ǽ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      cValue: ����λ������һ�δ�����ǰ�ҡ�1Ϊ��ǽ��0Ϊûǽ��
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
** Descriptions:        ���������ʾ��������״̬���������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ���������ʾ����ǰ������ǰ�����������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __mazeInfDebug (void)
{
    /*
     *  ��ʾ����
     */
    switch (GucMouseDir) {
        
    case 0:
        Download_7289(2, 3, 0, 0x47);                                /*  ��ǰ����F��ʾ               */
        break;
        
    case 1:
        Download_7289(2, 3, 0, 0x77);                                /*  ���ң���R��ʾ               */
        break;
        
    case 2:
        Download_7289(2, 3, 0, 0x1f);                                /*  �����b��ʾ               */
        break;
        
    case 3:
        Download_7289(2, 3, 0, 0x0e);                                /*  ������L��ʾ               */
        break;
        
    default :
        Download_7289(2, 3, 0, 0x4f);                                /*  ������E��ʾ               */
        break;
    }
    /*
     *  ��ʾ����
     */
    Download_7289(1, 0, 0, GmcMouse.cX / 10);
    Download_7289(1, 1, 0, GmcMouse.cX % 10);
    Download_7289(1, 6, 0, GmcMouse.cY / 10);
    Download_7289(1, 7, 0, GmcMouse.cY % 10);
}


/*********************************************************************************************************
** Function name:       keyCheck
** Descriptions:        ��ȡ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      true:  �����Ѱ���
**                      false: ����δ����
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
** Descriptions:        ��ѹ��⣬�������7289 EX BOARD ����ʾ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void voltageDetect (void)
{
    unsigned long ulVoltage;
    
    ADCProcessorTrigger(ADC_BASE, 0);                                   /*  ����������һ��A/Dת��       */
    while (!ADCIntStatus(ADC_BASE, 0, false));                          /*  �ȴ�ת������                */
    ADCIntClear(ADC_BASE, 0);                                           /*  ����жϱ�׼λ              */
    ADCSequenceDataGet(ADC_BASE, 0, &ulVoltage);                        /*  ��ȡת�����                */
    
    ulVoltage = ulVoltage * 3000 / 1023;                                /*  ����ʵ�ʼ�⵽�ĵ�ѹֵ(mV)  */
    ulVoltage = ulVoltage * 3 + 350;                                    /*  �����ص�ѹֵ(mV)          */
    
    Download_7289(0,6,1,(ulVoltage % 10000) / 1000);                  /*  ��ʾ��ѹֵ�������֣���λV   */
    Download_7289(0,7,0,(ulVoltage % 1000 ) / 100 );                  /*  ��ʾ��ѹֵС�����֣���λV   */
}


/*********************************************************************************************************
** Function name:       mouseInit
** Descriptions:        ��LM3S615���������г�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void mouseInit (void)
{
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                    SYSCTL_XTAL_6MHZ );                                 /*  ʹ��PLL��50M                */

    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );                      /*  ʹ��GPIO B������            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOC );                      /*  ʹ��GPIO C������            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );                      /*  ʹ��GPIO D������            */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );                      /*  ʹ��GPIO E������            */
    __keyInit();                                                        /*  ������ʼ��                  */
    __sensorInit();                                                     /*  ��������ʼ��                */
    __stepMotorIint();                                                  /*  ����������Ƴ�ʼ��          */
    __sysTickInit();                                                    /*  ϵͳʱ�ӳ�ʼ��              */
    __ADCInit();
    GucMapBlock[0][0] = 0x01;

#if USE_ASTAR_ALGORITHM
    astarInit();                                                        /*  ��ʼ��A*�㷨ģ��            */
    pathOptimizerInit();                                                /*  ��ʼ��·���Ż�ģ��          */
    motionParamsInit();                                                 /*  ��ʼ���˶�����ģ��        */
#endif
    IntMasterEnable();                                                  /*  ʹ��ȫ���ж�                */
}


/*********************************************************************************************************
** Function name:       __sensorInit
** Descriptions:        ���������Ƴ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __sensorInit (void)
{
    /*
     *  �������ӵ��������ź�����ŵ�I/O��Ϊ����ģʽ
     */
    GPIODirModeSet(GPIO_PORTB_BASE,
                   __LEFTSIDE    |
                   __FRONTSIDE_L |
                   __FRONTSIDE   |
                   __FRONTSIDE_R |
                   __RIGHTSIDE,  
                   GPIO_DIR_MODE_IN);
    /*
     *  ��PWM���������߷���ͷ�������Ƶĺ������ź�
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);                          /*  ʹ��PWMģ��                 */
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);                                 /*  PWMʱ�����ã�����Ƶ         */
    /*
     *  ��ʼ��PWM2����PWM����б�Ǻ��ⷢ��ͷ
     */
    GPIOPinTypePWM(GPIO_PORTB_BASE, __IRSEND_BEVEL);                    /*  PB0����ΪPWM����            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_1,                                /*  ����PWM������1              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  �Ӽ�������������            */

    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, true);                      /*  ʹ��PWM2���                */
    PWMGenDisable(PWM_BASE, PWM_GEN_1);                                 /*  ��ֹPWM������1              */
    /*
     *  ��ʼ��PWM4����PWM������ǰ����������ⷢ��ͷ
     */
    GPIOPinTypePWM(GPIO_PORTE_BASE, __IRSEND_SIDE);                     /*  PE0����ΪPWM����            */
    PWMGenConfigure(PWM_BASE, PWM_GEN_2,                                /*  ����PWM������2              */
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);       /*  �Ӽ�������������            */

    PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);                      /*  ʹ��PWM4���                */
    PWMGenDisable(PWM_BASE, PWM_GEN_2);                                 /*  ��ֹPWM������2              */
}


/*********************************************************************************************************
** Function name:       __stepMotorIint
** Descriptions:        ����������Ƴ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __stepMotorIint (void)
{
    uint n = 0;
    /*
     *  ����������������İ˸�I/O��Ϊ���ģʽ
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
     *  �����ҵ��ת����λ�ó�ʼ��
     */
    GPIOPinWrite(GPIO_PORTD_BASE,
                 __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2,
                 __PHRA1 | __PHRA2 | __PHRB1 | __PHRB2);
    
    GPIOPinWrite(GPIO_PORTD_BASE,
                 __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2,
                 __PHLA1 | __PHLA2 | __PHLB1 | __PHLB2);
    /*
     *  ��ʼ������/����ʱ��ʱ������ֵ�����ݱ�
     */
    __GuiAccelTable[0] = 2236068;
    __GuiAccelTable[1] = 906949;
    for(n = 2; n < 400; n++) {
        __GuiAccelTable[n] = __GuiAccelTable[n - 1] - (2 * __GuiAccelTable[n - 1] / (4 * n + 1));
    }
    /*
     *  ��ʼ����ʱ��0�����������ҵ����ת��
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                       /*  ʹ�ܶ�ʱ��0ģ��             */
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);                  /*  ����Ϊ32λ���ڼ���ģʽ      */
    TimerLoadSet(TIMER0_BASE, TIMER_A, __GuiAccelTable[0]);             /*  ���ö�ʱʱ��                */
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                    /*  ����Ϊ����ж�              */

    IntEnable(INT_TIMER0A);                                             /*  ʹ�ܶ�ʱ��0�ж�             */
    TimerEnable(TIMER0_BASE, TIMER_A);                                  /*  ʹ�ܶ�ʱ��0                 */
    
    /*
     *  ��ʼ����ʱ��1���������Ƶ����ת��
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);                       /*  ʹ�ܶ�ʱ��1ģ��             */
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER);                  /*  ����Ϊ32λ���ڼ���ģʽ      */
    TimerLoadSet(TIMER1_BASE, TIMER_A, __GuiAccelTable[0]);             /*  ���ö�ʱʱ��                */
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);                    /*  ����Ϊ����ж�              */

    IntEnable(INT_TIMER1A);                                             /*  ʹ�ܶ�ʱ��1�ж�             */
    TimerEnable(TIMER1_BASE, TIMER_A);                                  /*  ʹ�ܶ�ʱ��1                 */
}


/*********************************************************************************************************
** Function name:       __keyInit
** Descriptions:        �����Ӱ�����GPIO�ڳ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __keyInit (void)
{
    GPIODirModeSet(GPIO_PORTC_BASE, __KEY, GPIO_DIR_MODE_IN);           /*  ���ð�����Ϊ����            */
}


/*********************************************************************************************************
** Function name:       __sysTickInit
** Descriptions:        ϵͳ���Ķ�ʱ����ʼ����
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __sysTickInit (void)
{
    SysTickPeriodSet(SysCtlClockGet() / 1600);                          /*  ���ö�ʱʱ��Ϊ1ms           */
    SysTickEnable();                                                    /*  ʹ��ϵͳʱ��                */
    SysTickIntEnable();                                                 /*  ʹ��ϵͳʱ���ж�            */
}


/*********************************************************************************************************
** Function name:       __ADCInit
** Descriptions:        �����Ӱ�����GPIO�ڳ�ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void __ADCInit (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);                          /*  ʹ��ADCģ��                 */
    SysCtlADCSpeedSet(SYSCTL_ADCSPEED_125KSPS);                         /*  125KSps������               */

    ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);        /*  ����0Ϊ���������������ȼ�Ϊ0*/
    ADCSequenceStepConfigure(ADC_BASE, 0, 0,
                             ADC_CTL_CH0  | 
                             ADC_CTL_IE   | 
                             ADC_CTL_END);                              /*  ���ò������з������Ĳ���    */
    
    ADCHardwareOversampleConfigure(ADC_BASE, 16);                       /*  ����ADC����ƽ�����ƼĴ���   */
    ADCSequenceEnable(ADC_BASE, 0);                                     /*  ʹ�ܲ�������0               */
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
