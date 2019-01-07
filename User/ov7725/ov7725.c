#include "ov7725.h"
#include <stdlib.h>

typedef enum
{
    SCL = GPIO_Pin_6,
    SDA = GPIO_Pin_7,
}i2cbus;

typedef enum
{
    REG_GAIN = 0x00, 
    REG_BLUE, 
    REG_RED, 
    REG_GREEN, 
    REG_RSVD, 
    REG_BAVG, 
    REG_GAVG, 
    REG_RAVG, 
    REG_AECH, 
    REG_COM2, 
    REG_PID, 
    REG_VER, 
    REG_COM3, 
    REG_COM4, 
    REG_COM5, 
    REG_COM6, 
    REG_AEC, 
    REG_CLKRC, 
    REG_COM7, 
    REG_COM8, 
    REG_COM9, 
    REG_COM10, 
    REG_REG16, 
    REG_HSTART, 
    REG_HSIZE, 
    REG_VSTRT, 
    REG_VSIZE, 
    REG_PSHFT, 
    REG_MIDH, 
    REG_MIDL, 
    REG_LAEC = 0x1F, 
    REG_COM11, 
    REG_BDBase = 0x22, 
    REG_BDMStep, 
    REG_AEW, 
    REG_AEB, 
    REG_VPT, 
    REG_REG28 = 0x28, 
    REG_HOutSize, 
    REG_EXHCH, 
    REG_EXHCL, 
    REG_VOutSize, 
    REG_ADVFL, 
    REG_ADVFH, 
    REG_YAVE, 
    REG_LumHTh, 
    REG_LumLTh, 
    REG_HREF, 
    REG_DM_LNL, 
    REG_DM_LNH, 
    REG_ADoff_B, 
    REG_ADoff_R, 
    REG_ADoff_Gb, 
    REG_ADoff_Gr, 
    REG_Off_B, 
    REG_Off_R, 
    REG_Off_Gb, 
    REG_Off_Gr, 
    REG_COM12, 
    REG_COM13, 
    REG_COM14, 
    REG_COM15, 
    REG_COM16, 
    REG_TGT_B, 
    REG_TGT_R, 
    REG_TGT_Gb, 
    REG_TGT_Gr, 
    REG_LC_CTR, 
    REG_LC_XC, 
    REG_LC_YC, 
    REG_LC_COEF, 
    REG_LC_RADI, 
    REG_LC_COEFB, 
    REG_LC_COEFR, 
    REG_FixGain, 
    REG_AREF0, 
    REG_AREF1, 
    REG_AREF2, 
    REG_AREF3, 
    REG_AREF4, 
    REG_AREF5, 
    REG_AREF6, 
    REG_AREF7, 
    REG_UFix = 0x60, 
    REG_VFix, 
    REG_AWBb_blk, 
    REG_AWB_Ctrl0, 
    REG_DSP_Ctrl1, 
    REG_DSP_Ctrl2, 
    REG_DSP_Ctrl3, 
    REG_DSP_Ctrl4, 
    REG_AWB_bias, 
    REG_AWBCtrl1, 
    REG_AWBCtrl2, 
    REG_AWBCtrl3, 
    REG_AWBCtrl4, 
    REG_AWBCtrl5, 
    REG_AWBCtrl6, 
    REG_AWBCtrl7, 
    REG_AWBCtrl8, 
    REG_AWBCtrl9, 
    REG_AWBCtrl10, 
    REG_AWBCtrl11, 
    REG_AWBCtrl12, 
    REG_AWBCtrl13, 
    REG_AWBCtrl14, 
    REG_AWBCtrl15, 
    REG_AWBCtrl16, 
    REG_AWBCtrl17, 
    REG_AWBCtrl18, 
    REG_AWBCtrl19, 
    REG_AWBCtrl20, 
    REG_AWBCtrl21, 
    REG_GAM1, 
    REG_GAM2, 
    REG_GAM3, 
    REG_GAM4, 
    REG_GAM5, 
    REG_GAM6, 
    REG_GAM7, 
    REG_GAM8, 
    REG_GAM9, 
    REG_GAM10, 
    REG_GAM11, 
    REG_GAM12, 
    REG_GAM13, 
    REG_GAM14, 
    REG_GAM15, 
    REG_SLOP, 
    REG_DNSTh, 
    REG_EDGE0, 
    REG_EDGE1, 
    REG_DNSOff, 
    REG_EDGE2, 
    REG_EDGE3, 
    REG_MTX1, 
    REG_MTX2, 
    REG_MTX3, 
    REG_MTX4, 
    REG_MTX5, 
    REG_MTX6, 
    REG_MTX_Ctrl, 
    REG_BRIGHT, 
    REG_CNST, 
    REG_UVADJ0 = 0x9E, 
    REG_UVADJ1, 
    REG_SCAL0, 
    REG_SCAL1, 
    REG_SCAL2, 
    REG_FIFOdlyM, 
    FIFOdlyA, 
    REG_SDE = 0xA6, 
    REG_USAT, 
    REG_VSAT, 
    REG_HUECOS, 
    REG_HUESIN, 
    REG_SIGN, 
    REG_DSPAuto, 
}ov7725regadd;

typedef struct
{
    ov7725regadd    add;
    uint8_t         val;
}regmap;

regmap regconf[] =
{
    {REG_CLKRC,     0x00},
    {REG_COM7,      0x46},
    {REG_HSTART,    0x3f},
    {REG_HSIZE,     0x50},
    {REG_VSTRT,     0x03},
    {REG_VSIZE,     0x78},
    {REG_HREF,      0x00},
    {REG_HOutSize,  0x50},
    {REG_VOutSize,  0x78},
    {REG_EXHCH,     0x00},
    
    /* DSP control */
    {REG_TGT_B,     0x7f},
    {REG_FixGain,   0x09},
    {REG_AWB_Ctrl0, 0xe0},
    {REG_DSP_Ctrl1, 0xff},
    {REG_DSP_Ctrl2, 0x20},
    {REG_DSP_Ctrl3, 0x00},
    {REG_DSP_Ctrl4, 0x00},

    /* AGC AEC AWB */
    {REG_COM8,      0xf0},
    {REG_COM4,      0x81}, /*Pll AEC CONFIG*/
    {REG_COM6,      0xc5},
    {REG_COM9,      0x21},
    {REG_BDBase,    0xFF},
    {REG_BDMStep,   0x01},
    {REG_AEW,       0x34},
    {REG_AEB,       0x3c},
    {REG_VPT,       0xa1},
    {REG_EXHCL,     0x00},
    {REG_AWBCtrl3,  0xaa},
    {REG_COM8,      0xff},
    {REG_AWBCtrl1,  0x5d},

    {REG_EDGE1,     0x0a},
    {REG_DNSOff,    0x01},
    {REG_EDGE2,     0x01},
    {REG_EDGE3,     0x01},

    {REG_MTX1,      0x5f},
    {REG_MTX2,      0x53},
    {REG_MTX3,      0x11},
    {REG_MTX4,      0x1a},
    {REG_MTX5,      0x3d},
    {REG_MTX6,      0x5a},
    {REG_MTX_Ctrl,  0x1e},

    {REG_BRIGHT,    0x00},
    {REG_CNST,      0x25},
    {REG_USAT,      0x65},
    {REG_VSAT,      0x65},
    {REG_UVADJ0,    0x81},
    {REG_SDE,       0x06},
    
    /*GAMMA config*/
    {REG_GAM1,      0x0c},
    {REG_GAM2,      0x16},
    {REG_GAM3,      0x2a},
    {REG_GAM4,      0x4e},
    {REG_GAM5,      0x61},
    {REG_GAM6,      0x6f},
    {REG_GAM7,      0x7b},
    {REG_GAM8,      0x86},
    {REG_GAM9,      0x8e},
    {REG_GAM10,     0x97},
    {REG_GAM11,     0xa4},
    {REG_GAM12,     0xaf},
    {REG_GAM13,     0xc5},
    {REG_GAM14,     0xd7},
    {REG_GAM15,     0xe8},
    {REG_SLOP,      0x20},

    {REG_HUECOS,    0x80},
    {REG_HUESIN,    0x80},
    {REG_DSPAuto,   0xff},
    {REG_DM_LNL,    0x00},
    {REG_BDBase,    0x99},
    {REG_BDMStep,   0x03},
    {REG_LC_RADI,   0x00},
    {REG_LC_COEF,   0x13},
    {REG_LC_XC,     0x08},
    {REG_LC_COEFB,  0x14},
    {REG_LC_COEFR,  0x17},
    {REG_LC_CTR,    0x05},
    
    {REG_COM3,      0xd0},

    {REG_COM5,      0xf5},
    
    /* Color mode */
    {REG_SDE,       0x06},
    {REG_UFix,      0x80},
    {REG_VFix,      0x80},
    
    /* Light mode */
//    {REG_COM8,      0xff},
//    {REG_COM5,      0x65},
//    {REG_ADVFL,     0x00},
//    {REG_ADVFH,     0x00},
    
    /* Color saturation */
//    {REG_USAT,      0x40},
//    {REG_VSAT,      0x40},
    
    /* Brightness & Contrast */
//    {REG_BRIGHT,    0x08},
//    {REG_SIGN,      0x06},
//    {REG_CNST,      0x30},
    
    /* QVGA */
//    {REG_COM7,      0x46},
};

const size_t  regsize = sizeof(regconf) / sizeof(regmap);
const uint8_t  OV7725_ADDRESS    = 0x42;
const uint8_t  OV7725_ID         = 0x21;

/* SCCB */
void    SCCB_Conf       (void);
bool    SCCB_Start      (void);
void    SCCB_Stop       (void);
void    SCCB_Ack        (void);
int     SCCB_WaitAck    (void);
void    SCCB_NoAck      (void);
void    SCCB_SendByte   (uint8_t byte);
bool    SCCB_WriteByte  (uint16_t addr, uint8_t byte);
uint8_t SCCB_GetByte    (void);
bool    SCCB_ReadByte   (uint8_t* buff, size_t len, ov7725regadd addr);
void    SCCB_SetLevel   (i2cbus bus, bool level);
bool    SCCB_ReadLevel  (i2cbus bus);

void FIFO_Conf       (void);
void VSYNC_Conf      (void);
void OV7725_SetRange (void);

void SCCB_delay(void)
{	
   uint16_t i = 400; 
   while(i) i--;
}

bool OV7725_Init(void)
{
    bool ret = true;
    
    SCCB_Conf();
    FIFO_Conf();
    VSYNC_Conf();
    uint8_t ID;
    do
    {
        if(!SCCB_WriteByte(REG_COM7, 0x80)) break;
        if(!SCCB_ReadByte(&ID, 1, REG_VER)) break;
        if(OV7725_ID != ID) break;
        ret = true;
        
        for(uint8_t index = 0; index < regsize; index++)
        {
            if(!SCCB_WriteByte(regconf[index].add, regconf[index].val))
            {
                ret = false;
                break;
            }
        }
        if(ret) OV7725_SetRange();
    }while(0);
    
    return ret;
}

void OV7725_SetRange(void)
{
    const uint16_t offsetx = 40;
    
	uint8_t regval;

	SCCB_ReadByte (&regval, 1, REG_HSTART);
	SCCB_WriteByte(REG_HSTART, (regval + (offsetx >> 2))); 
	SCCB_WriteByte(REG_HSIZE ,  OV7725_IMAGE_SIZE >> 2);
	SCCB_WriteByte(REG_VSIZE ,  OV7725_IMAGE_SIZE >> 1);
    
	uint8_t low;
	low = (OV7725_IMAGE_SIZE & 0x03) 
       | ((OV7725_IMAGE_SIZE & 0x01) << 2) 
       | ((offsetx           & 0x03) << 4);
	SCCB_ReadByte (&regval, 1, REG_HREF);
	SCCB_WriteByte(REG_HREF, regval | low);
	SCCB_WriteByte(REG_HOutSize, OV7725_IMAGE_SIZE >> 2);
	SCCB_WriteByte(REG_VOutSize, OV7725_IMAGE_SIZE >> 1);

	low = (OV7725_IMAGE_SIZE & 0x03) | ((OV7725_IMAGE_SIZE & 0x01) << 2);	
	SCCB_ReadByte (&regval, 1, REG_EXHCH);	
	SCCB_WriteByte(REG_EXHCH, regval | low);
}

void SCCB_Conf(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SCCB_SetLevel(i2cbus bus, bool level)
{
    if(level) GPIO_SetBits(GPIOC, bus);
    else    GPIO_ResetBits(GPIOC, bus);
    SCCB_delay();
}

bool SCCB_ReadLevel(i2cbus bus)
{
    return GPIO_ReadInputDataBit(GPIOC, bus);
}

void SCCB_SendByte(uint8_t byte) 
{
    for(uint8_t i = 0; i < 8; i++)
    {
        SCCB_SetLevel(SCL, false);
        if(byte & 0x80) SCCB_SetLevel(SDA, true);
        else            SCCB_SetLevel(SDA, false);
        byte <<= 1;
        SCCB_SetLevel(SCL, true);
    }
    SCCB_SetLevel(SCL, false);
}

bool SCCB_WriteByte(uint16_t addr, uint8_t byte)
{		
    bool ret = false;
    do
    {
        if(!SCCB_Start()) break;
        SCCB_SendByte(OV7725_ADDRESS);
        if(!SCCB_WaitAck()) break;
        SCCB_SendByte((uint8_t)(addr & 0x00FF));
        SCCB_WaitAck();    
        SCCB_SendByte(byte);
        SCCB_WaitAck();   
        SCCB_Stop();
        ret = true;
    }while(0);
    SCCB_Stop(); 
    return ret;
}

uint8_t SCCB_GetByte(void)  
{ 
    uint8_t byte = 0;

    SCCB_SetLevel(SDA, true);
    for(uint8_t i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCCB_SetLevel(SCL, false);
        SCCB_SetLevel(SCL, true);
        if(SCCB_ReadLevel(SDA)) byte |= 0x01;
    }
    SCCB_SetLevel(SCL, false);
    return byte;
}

bool SCCB_ReadByte(uint8_t* buff, size_t len, ov7725regadd addr)
{    
    bool ret = false;
    do
    {
        if(!SCCB_Start()) break;
        SCCB_SendByte(OV7725_ADDRESS);
        if(!SCCB_WaitAck()) break;
        
        SCCB_SendByte(addr);
        SCCB_WaitAck();    
        SCCB_Stop();
        
        if(!SCCB_Start()) break;
        
        SCCB_SendByte(OV7725_ADDRESS + 1);
        if(!SCCB_WaitAck()) break;
        
        while(len)
        {
            *(buff++) =  SCCB_GetByte();
            if((len--) == 1) SCCB_NoAck();
            else SCCB_Ack();
        }
        ret = true;
    }while(0);
    SCCB_Stop();
    return ret;
}

static int SCCB_WaitAck(void) 	
{
    bool ret = true;
    SCCB_SetLevel(SCL, false);
    SCCB_SetLevel(SDA, true);
    SCCB_SetLevel(SCL, true);
    if(SCCB_ReadLevel(SDA)) ret = false;
    SCCB_SetLevel(SCL, false);
    return ret;
}

void SCCB_Ack(void)
{    
    SCCB_SetLevel(SCL, false);
    SCCB_SetLevel(SDA, false);
    SCCB_SetLevel(SCL, true);
    SCCB_SetLevel(SCL, false);
}

void SCCB_NoAck(void)
{    
    SCCB_SetLevel(SCL, false);
    SCCB_SetLevel(SDA, true);
    SCCB_SetLevel(SCL, true);
    SCCB_SetLevel(SCL, false);
}

bool SCCB_Start(void)
{
    bool ret = false;
    
    do
    {
        SCCB_SetLevel(SDA, true);
        SCCB_SetLevel(SCL, true);
        if(!SCCB_ReadLevel(SDA)) break;
        
        SCCB_SetLevel(SDA, false);
        if( SCCB_ReadLevel(SDA)) break;
        SCCB_SetLevel(SDA, false);
        
        ret = true;
    }while(0);
    return ret;
}

void SCCB_Stop(void)
{
    SCCB_SetLevel(SCL, false);
    SCCB_SetLevel(SDA, false);
    SCCB_SetLevel(SCL, true);
    SCCB_SetLevel(SDA, true);
}

void VSYNC_Conf(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);     

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource0);
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line    = OV7725_EXTI_Line;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_GenerateSWInterrupt(EXTI_Line0);    

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    
    
    NVIC_Init(&NVIC_InitStructure);
}

#define FIFO_OE_H()   GPIOA->BSRRL = GPIO_Pin_3
#define FIFO_OE_L()   GPIOA->BSRRH = GPIO_Pin_3

#define FIFO_WRST_H() GPIOC->BSRRL = GPIO_Pin_4
#define FIFO_WRST_L() GPIOC->BSRRH = GPIO_Pin_4

#define FIFO_RRST_H() GPIOA->BSRRL = GPIO_Pin_2
#define FIFO_RRST_L() GPIOA->BSRRH = GPIO_Pin_2

#define FIFO_RCLK_H() GPIOC->BSRRL = GPIO_Pin_5
#define FIFO_RCLK_L() GPIOC->BSRRH = GPIO_Pin_5

#define FIFO_WE_H()   GPIOD->BSRRL = GPIO_Pin_3
#define FIFO_WE_L()   GPIOD->BSRRH = GPIO_Pin_3

void FIFO_Conf(void)
{
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
                          | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD ,ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8  | GPIO_Pin_9
                                  | GPIO_Pin_10 | GPIO_Pin_11
                                  | GPIO_Pin_12 | GPIO_Pin_13
                                  | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    FIFO_OE_L();
    FIFO_WE_H();
}

__inline void FIFO_PointReset(void)
{
    FIFO_WRST_L();
    FIFO_WE_H  ();
}

__inline void FIFO_PointMove(void)
{
    FIFO_WE_H  ();
    FIFO_WRST_H();
}

__inline void FIFO_Pause(void)
{
    FIFO_WE_L();
}

__inline void FIFO_Prepare(void)
{
    FIFO_RRST_L();
    FIFO_RCLK_L();
    FIFO_RCLK_H();
    FIFO_RRST_H();
    FIFO_RCLK_L();
    FIFO_RCLK_H();
}

__inline uint16_t FIFO_ReadPixl(void)
{
    uint16_t ret = 0;
    
    FIFO_RCLK_L();
    ret = (GPIOB->IDR) & 0xff00;
    FIFO_RCLK_H();
    
    FIFO_RCLK_L();
    ret |= (GPIOB->IDR >> 8) & 0x00ff;
    FIFO_RCLK_H();
    
    return ret;
}

#undef FIFO_OE_H
#undef FIFO_OE_L

#undef FIFO_WE_H
#undef FIFO_WE_L
 
#undef FIFO_WRST_H
#undef FIFO_WRST_L
 
#undef FIFO_RRST_H
#undef FIFO_RRST_L
 
#undef FIFO_RCLK_H
#undef FIFO_RCLK_L
