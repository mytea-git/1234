

#include "drive_7289.h"


/*********************************************************************************************************
** Function name:       delayus
** Descriptions:        ��ʱN��΢��
** input parameters:    us: ��ʱʱ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void delayus (int us)
{
    us = SysCtlClockGet() * us / 2000000;                         /*  ����ϵͳʱ������ȷ����ʱ    */
    while (us != 0)us--;
}

/*********************************************************************************************************
** Function name:       SPIWrite_7289
** Descriptions:        ��SPI ����д��1 ���ֽڵ����ݡ�
** input parameters:    data��Ҫд�������
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void SPIWrite_7289 (char data)
{
    GPIODirModeSet(GPIO_PORTA_BASE, DIO_7289, GPIO_DIR_MODE_OUT);    /*  ����DIO�˿�Ϊ���ģʽ       */
    /****ѭ��дһ���ֽڵ�����  *****/
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
** Descriptions:        ��SPI ���߶�ȡ1 ���ֽڵ����ݡ�
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��ȡ��������
*********************************************************************************************************/
char SPIRead_7289 (void)
{
    char data = 0;
    GPIODirModeSet(GPIO_PORTA_BASE, DIO_7289, GPIO_DIR_MODE_IN);     /* ����DIO�˿�Ϊ���ģʽ        */
    /*
     *  ѭ����һ���ֽڵ�����
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
** Descriptions:        ִ��ZLG7289 ��ָ�
** input parameters:    cmd��������
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ִ��ZLG7289 ������ָ�
** input parameters:    cCmd��������
**                      data������
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        �������ݡ�
** input parameters:    mode=0�� ���������Ұ���ʽ0 ����
**                      mode=1�� ���������Ұ���ʽ1 ����
**                      mode=2�� �������ݵ�������
**                      number��      ����ܱ�ţ������꣩��ȡֵ0��7
**                      dp=0��   С���㲻��
**                      dp=1��   С������
**                      data��    Ҫ��ʾ������
** output parameters:   ��
** Returned value:      ��
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
** Descriptions:        ִ��ZLG7289 �������
** input parameters:    ��
** output parameters:   ��
** Returned value:      ���ض����İ���ֵ��0��63���������0xFF ���ʾû�м�����
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
** Descriptions:        ZLG7289 ��ʼ��
** input parameters:    ��
** output parameters:   ��
** Returned value:      ��
*********************************************************************************************************/
void Init_7289 (void)
{
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );                              /*  ʹ��GPIO A������    */
    
    GPIODirModeSet(GPIO_PORTA_BASE,CS_7289|CLK_7289|DIO_7289,GPIO_DIR_MODE_OUT);/*  ����I/O��Ϊ���ģʽ */
    
    GPIOPinWrite(GPIO_PORTA_BASE, DIO_7289, 0xff);
    GPIOPinWrite(GPIO_PORTA_BASE, CLK_7289, 0x00);
    GPIOPinWrite(GPIO_PORTA_BASE, CS_7289 , 0xff);
    
    Reset_7289();                                                               /*  ��λZLG7289     */
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
