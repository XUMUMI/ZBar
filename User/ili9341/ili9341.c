#include "ili9341.h"
#include "stm32f4xx.h"
#include "stm32f4xx_fsmc.h"

uint16_t LCD_X_LENGTH;
uint16_t LCD_Y_LENGTH;
uint8_t  ILI9341_SCAN_MODE;

const uint16_t ILI9341_LESS_PIXEL       = 240;
const uint16_t ILI9341_MORE_PIXEL       = 320;
const uint32_t ILI9341_COMMAND_ADDRESS  = 0x60000000;
const uint32_t ILI9341_DATA_ADDRESS     = 0x60080000;

typedef enum
{
    ILI9341_SetX     = 0x2A,
    ILI9341_SetY     = 0x2B,
    ILI9341_SetPixel = 0x2C
}ILI9341_Cmd;

extern __inline void     ili9341_SendCmd (uint16_t command);
extern __inline void     ili9341_SendData(uint16_t data);
extern __inline uint16_t ili9341_RecData (void);

void ili9341_GPIO_Conf (void);
void ili9341_FSMC_Conf (void);
void ili9341_Reset     (void);
void ili9341_RegConf   (void);

static void delay ( __IO uint32_t cnt)
{
  for (; cnt; cnt--);
}

void ili9341_Init(void)
{
    ili9341_GPIO_Conf();
    ili9341_FSMC_Conf();
    ili9341_BackLedSw(true);
    ili9341_Reset();
    ili9341_RegConf();
    /*
            O - >X
      6 is  |
            v
            Y
    */
    ili9341_SetDirection(0);
}

void ili9341_GPIO_Conf ( void )
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
    GPIO_InitStructure.GPIO_Pin =
      GPIO_Pin_0  | GPIO_Pin_1
    | GPIO_Pin_4  | GPIO_Pin_5
                  | GPIO_Pin_7
    | GPIO_Pin_8  | GPIO_Pin_9
    | GPIO_Pin_10 | GPIO_Pin_13
    | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =
                    GPIO_Pin_7 
    | GPIO_Pin_8  | GPIO_Pin_9 
    | GPIO_Pin_10 | GPIO_Pin_11 
    | GPIO_Pin_12 | GPIO_Pin_13 
    | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_Init(GPIOE, & GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_Init(GPIOB, & GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource4,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,  GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource7,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,  GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15, GPIO_AF_FSMC);
    
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,  GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,  GPIO_AF_FSMC); 
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource15, GPIO_AF_FSMC);
}

void ili9341_FSMC_Conf ( void )
{
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
    	
	
	FSMC_NORSRAMTimingInitTypeDef  readWriteTiming; 
	readWriteTiming.FSMC_AddressSetupTime       = 0x0F;
	readWriteTiming.FSMC_DataSetupTime          = 0x0B;
	readWriteTiming.FSMC_AccessMode             = FSMC_AccessMode_B;	
	readWriteTiming.FSMC_AddressHoldTime        = 0x00;
	readWriteTiming.FSMC_BusTurnAroundDuration  = 0x00;
	readWriteTiming.FSMC_CLKDivision            = 0x00;
	readWriteTiming.FSMC_DataLatency            = 0x00;	
	
	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
	FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_NOR;
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &readWriteTiming; 
	
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
	
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}

void ili9341_BackLedSw(bool sw)
{
    if(sw) GPIO_SetBits   (GPIOB, GPIO_Pin_1);
    else   GPIO_ResetBits (GPIOB, GPIO_Pin_1);
}

void ili9341_Reset(void)
{			
    GPIO_ResetBits(GPIOE, GPIO_Pin_1);
    delay(3000);
    GPIO_SetBits  (GPIOE, GPIO_Pin_1);
    delay(3000);
}

__inline void ili9341_SendCmd(uint16_t command)
{
	*(__IO uint16_t*)(ILI9341_COMMAND_ADDRESS) = command;
}

__inline void ili9341_SendData(uint16_t data)
{
    *(__IO uint16_t*)(ILI9341_DATA_ADDRESS) = data;
}

__inline uint16_t ili9341_RecData(void)
{
    return (*(__IO uint16_t*)(ILI9341_DATA_ADDRESS));
}

void ili9341_RegConf(void)
{
	/* Power control B (CFh) */
	ili9341_SendCmd (0xCF);
	ili9341_SendData(0x00);
	ili9341_SendData(0x81);
	ili9341_SendData(0x30);
	
	/* Power on sequence control (EDh) */
	ili9341_SendCmd (0xED);
	ili9341_SendData(0x64);
	ili9341_SendData(0x03);
	ili9341_SendData(0x12);
	ili9341_SendData(0x81);
	
	/* Driver timing control A (E8h) */
	ili9341_SendCmd (0xE8);
	ili9341_SendData(0x85);
	ili9341_SendData(0x10);
	ili9341_SendData(0x78);
	
	/* Power control A (CBh) */
	ili9341_SendCmd (0xCB);
	ili9341_SendData(0x39);
	ili9341_SendData(0x2C);
	ili9341_SendData(0x00);
	ili9341_SendData(0x34);
	ili9341_SendData(0x02);
	
	/* Pump ratio control (F7h) */
	ili9341_SendCmd (0xF7);
	ili9341_SendData(0x20);
	
	/* Driver timing control B */
	ili9341_SendCmd (0xEA);
	ili9341_SendData(0x00);
	ili9341_SendData(0x00);
	
	/* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
	ili9341_SendCmd (0xB1);
	ili9341_SendData(0x00);
	ili9341_SendData(0x1B);
	
	/* Display Function Control (B6h) */
	ili9341_SendCmd (0xB6);
	ili9341_SendData(0x0A);
	ili9341_SendData(0xA2);
	
	/* Power Control 1 (C0h) */
	ili9341_SendCmd (0xC0);
	ili9341_SendData(0x35);
	
	/* Power Control 2 (C1h) */
	ili9341_SendCmd (0xC1);
	ili9341_SendData(0x11);
	
	/* VCOM Control 1 (C5h) */
	ili9341_SendCmd (0xC5);
	ili9341_SendData(0x45);
	ili9341_SendData(0x45);
	
	/* VCOM Control 2 (C7h) */
	ili9341_SendCmd (0xC7);
	ili9341_SendData(0xA2);
	
	/* Enable 3G (F2h) */
	ili9341_SendCmd (0xF2);
	ili9341_SendData(0x00);
	
	/* Gamma Set (26h) */
	ili9341_SendCmd (0x26);
	ili9341_SendData(0x01);
	
	/* Positive Gamma Correction */
	ili9341_SendCmd (0xE0);
	ili9341_SendData(0x0F);
	ili9341_SendData(0x26);
	ili9341_SendData(0x24);
	ili9341_SendData(0x0B);
	ili9341_SendData(0x0E);
	ili9341_SendData(0x09);
	ili9341_SendData(0x54);
	ili9341_SendData(0xA8);
	ili9341_SendData(0x46);
	ili9341_SendData(0x0C);
	ili9341_SendData(0x17);
	ili9341_SendData(0x09);
	ili9341_SendData(0x0F);
	ili9341_SendData(0x07);
	ili9341_SendData(0x00);
	
	/* Negative Gamma Correction (E1h) */
	ili9341_SendCmd (0XE1);
	ili9341_SendData(0x00);
	ili9341_SendData(0x19);
	ili9341_SendData(0x1B);
	ili9341_SendData(0x04);
	ili9341_SendData(0x10);
	ili9341_SendData(0x07);
	ili9341_SendData(0x2A);
	ili9341_SendData(0x47);
	ili9341_SendData(0x39);
	ili9341_SendData(0x03);
	ili9341_SendData(0x06);
	ili9341_SendData(0x06);
	ili9341_SendData(0x30);
	ili9341_SendData(0x38);
	ili9341_SendData(0x0F);
	
	/* memory access control set */
	ili9341_SendCmd (0x36); 	
	ili9341_SendData(0xC8);
	
	/* column address control set */
	ili9341_SendCmd (ILI9341_SetX); 
	ili9341_SendData(0x00);
	ili9341_SendData(0x00);
	ili9341_SendData(0x00);
	ili9341_SendData(0xEF);
	
	/* page address control set */
	ili9341_SendCmd (ILI9341_SetY); 
	ili9341_SendData(0x00);
	ili9341_SendData(0x00);
	ili9341_SendData(0x01);
	ili9341_SendData(0x3F);
	
	/*  Pixel Format Set (3Ah)  */
	ili9341_SendCmd (0x3a); 
	ili9341_SendData(0x55);
	
	/* Sleep Out (11h)  */
	ili9341_SendCmd (0x11);	
	delay(450);
	
	/* Display ON (29h) */
	ili9341_SendCmd (0x29);
}

void ili9341_SetDirection(uint8_t direction)
{	
	if(direction < 8)
    {
        ILI9341_SCAN_MODE = direction;
        
        if(direction % 2)	
        {
            LCD_X_LENGTH = ILI9341_MORE_PIXEL;
            LCD_Y_LENGTH = ILI9341_LESS_PIXEL; 
        }
        else				
        {
            LCD_X_LENGTH = ILI9341_LESS_PIXEL;
            LCD_Y_LENGTH = ILI9341_MORE_PIXEL;
        }

        ili9341_SendCmd (0x36); 
        ili9341_SendData(0x08 |(direction << 5));
        
        ili9341_SendCmd (ILI9341_SetX); 
        ili9341_SendData(0x00);
        ili9341_SendData(0x00);
        ili9341_SendData(((LCD_X_LENGTH - 1) >> 8) & 0xFF );
        ili9341_SendData ((LCD_X_LENGTH - 1) & 0xFF );

        ili9341_SendCmd (ILI9341_SetY); 
        ili9341_SendData(0x00);
        ili9341_SendData(0x00);
        ili9341_SendData(((LCD_Y_LENGTH - 1) >> 8) & 0xFF);
        ili9341_SendData( (LCD_Y_LENGTH-1)&0xFF );

        /* write gram start */
        ili9341_SendCmd(ILI9341_SetPixel);	
    }
}

void ili9341_SetRange(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{	
    ili9341_SendCmd  (ILI9341_SetX); 
	ili9341_SendData (X1 >> 8);
	ili9341_SendData (X1 & 0xff);
	ili9341_SendData((X1 + X2 - 1) >> 8);
	ili9341_SendData((X1 + X2 - 1) & 0xff);

	ili9341_SendCmd  (ILI9341_SetY);
	ili9341_SendData (Y1 >> 8);
	ili9341_SendData (Y1 & 0xff);
	ili9341_SendData((Y1 + Y2 - 1) >> 8);
	ili9341_SendData((Y1 + Y2 - 1) & 0xff);
}

void ili9341_PrintImage(uint16_t width, uint16_t height, uint16_t (*pixelData)())
{
    ili9341_SendCmd(ILI9341_SetPixel);
    uint16_t tmp;
    for(int h = 0; h < height; h++)
    {
        for(int w = 0; w < width; w++)
        {
            tmp = pixelData();
            if(
                    (h == 46  && w >= 46 && w < 195)
                ||  (w == 46  && h >= 46 && h < 195)
                ||  (h == 195 && w >= 46 && w < 195)
                ||  (w == 195 && h >= 46 && h < 195)
              ) ili9341_SendData(0xFFFF);
            else ili9341_SendData(tmp);
        }
    }
}
