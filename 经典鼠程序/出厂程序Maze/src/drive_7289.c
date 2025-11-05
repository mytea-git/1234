

#include "Drive_7289.h"


/*********************************************************************************************************
** Function name:       delayus
** Descriptions:        延时N个微秒
** input parameters:    us: 延时时间
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void delayus (int us)
{
    us = SysCtlClockGet() * us / 2000000;                         /*  根据系统时钟速率确定延时    */
    while (us != 0)us--;
}

/*********************************************************************************************************
** Function name:       SPIWrite_7289
** Descriptions:        向SPI 总线写入1 个字节的数据。
** input parameters:    data：要写入的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void SPIWrite_7289 (char data)
{
    GPIODirModeSet(GPIO_PORTA_BASE, DIO_7289, GPIO_DIR_MODE_OUT);    /*  设置DIO端口为输出模式       */
    /****循环写一个字节的数据  *****/
    for(char cnt = 8;cnt>0;cnt--) 
    {
        if((data & 0x80) == 0x80) {
            GPIOPinWrite(GPIO_PORTA_BASE, DIO_7289, 0xff);
        } else {
            GPIOPinWrite(GPIO_PORTA_BASE, DIO_7289, 0x00);
        }
        data <<= 1;
        GPIOPinWrite(GPIO_PORTA_BASE, CLK_7289, 0xff);
        delayus(5);
        GPIOPinWrite(GPIO_PORTA_BASE, CLK_7289, 0x00);
        delayus(5);
    } 
}


/*********************************************************************************************************
** Function name:       SPIRead_7289
** Descriptions:        从SPI 总线读取1 个字节的数据。
** input parameters:    无
** output parameters:   无
** Returned value:      读取到的数据
*********************************************************************************************************/
char SPIRead_7289 (void)
{
    char data = 0;
    GPIODirModeSet(GPIO_PORTA_BASE, DIO_7289, GPIO_DIR_MODE_IN);     /* 设置DIO端口为输出模式        */
    /*
     *  循环读一个字节的数据
     */
    for(char cnt = 8;cnt>0;cnt--)
    {
        GPIOPinWrite(GPIO_PORTA_BASE, CLK_7289, 0xff);
        delayus(5);
        data <<= 1;
        if (GPIOPinRead(GPIO_PORTA_BASE, DIO_7289)) {
            data++;
        }
        GPIOPinWrite(GPIO_PORTA_BASE, CLK_7289, 0x00);
        delayus(5);
    } 
    return data;
}


/*********************************************************************************************************
** Function name:       Cmd_7289
** Descriptions:        执行ZLG7289 纯指令。
** input parameters:    cmd：命令字
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Cmd_7289 (char  cmd)
{
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289, 0x00);
    delayus(25);
    SPIWrite_7289(cmd);
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289, 0xff);
    delayus(5);
}


/*********************************************************************************************************
** Function name:       CmdDat_7289
** Descriptions:        执行ZLG7289 带数据指令。
** input parameters:    cCmd：命令字
**                      data：数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void CmdDat_7289 (uchar  cmd, char  data)
{
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289, 0x00);
    delayus(25);
    SPIWrite_7289(cmd);
    delayus(15);
    SPIWrite_7289(data);
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289, 0xff);
    delayus(5);
}


/*********************************************************************************************************
** Function name:       Download_7289
** Descriptions:        下载数据。
** input parameters:    mode=0： 下载数据且按方式0 译码
**                      mode=1： 下载数据且按方式1 译码
**                      mode=2： 下载数据但不译码
**                      number：      数码管编号（横坐标），取值0～7
**                      dp=0：   小数点不亮
**                      dp=1：   小数点亮
**                      data：    要显示的数据
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Download_7289 (uchar  mode, char  number, char  dp, char  data)
{
    uchar modeDat[3] = {0x80,0xC8,0x90};
    uchar temp_mode;
    uchar temp_data;
    
    if (mode > 2) {
        mode = 2;
    }
    
    temp_mode  = modeDat[mode];
    number   &= 0x07;
    temp_mode |= number;
    temp_data  = data & 0x7F;
    
    if (dp  == 1) {
        temp_data |= 0x80;
    }
    CmdDat_7289(temp_mode, temp_data);
}


/*********************************************************************************************************
** Function name:       Key_7289
** Descriptions:        执行ZLG7289 键盘命令。
** input parameters:    无
** output parameters:   无
** Returned value:      返回读到的按键值：0～63。如果返回0xFF 则表示没有键按下
*********************************************************************************************************/
char KeyDrive_7289 (void)
{
    char key;
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289, 0x00);
    delayus(25);
    SPIWrite_7289(0x15);
    delayus(15);
    key = SPIRead_7289();
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289, 0xff);
    delayus(5);
    return key;
}


/*********************************************************************************************************
** Function name:       Init_7289
** Descriptions:        ZLG7289 初始化
** input parameters:    无
** output parameters:   无
** Returned value:      无
*********************************************************************************************************/
void Init_7289 (void)
{
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );                              /*  使能GPIO A口外设    */
    
    GPIODirModeSet(GPIO_PORTA_BASE,CS_7289|CLK_7289|DIO_7289,GPIO_DIR_MODE_OUT);/*  设置I/O口为输出模式 */
    
    GPIOPinWrite(GPIO_PORTA_BASE, DIO_7289, 0xff);
    GPIOPinWrite(GPIO_PORTA_BASE, CLK_7289, 0x00);
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289 , 0xff);
    
    Reset_7289();                                                               /*  复位ZLG7289     */
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
