#include "tftlcd.h"
#include "stdlib.h"
#include "font.h"

// ILI9806寄存器定义
#define SLPOUT     0x11
#define DISPON     0x29
#define CASET      0x2A
#define PASET      0x2B
#define RAMWR      0x2C	// 写入RAM
#define MADCTL     0x36
#define COLMOD     0x3A
#define PIXFMT     0x3A

SRAM_HandleTypeDef TFTSRAM_Handler;    //SRAM句柄(用于控制LCD)

TFTLCD_TypeDef TFTLCD = {FSMC_CMD_ADDR, FSMC_DATA_ADDR};
//LCD的画笔颜色和背景色
uint16_t FRONT_COLOR=BLACK;	//画笔颜色
uint16_t BACK_COLOR=WHITE;  //背景色

_tftlcd_data tftlcd_data;


//写寄存器函数
//cmd:寄存器值
void LCD_WriteCmd(uint16_t cmd)
{
	*(TFTLCD.cmd_reg) = cmd;
	__DMB();
}

//写数据
//data:要写入的值
void LCD_WriteData(uint16_t data)
{
	*(TFTLCD.data_reg) = data;
//	__DMB();
}

void LCD_WriteCmdData(uint16_t cmd,uint16_t data)
{
	LCD_WriteCmd(cmd);
	__DMB();
	LCD_WriteData(data);
}

uint32_t LCD_RGBColor_Change(uint16_t color)
{
	uint8_t r,g,b=0;

	r=(color>>11)&0x1f;
	g=(color>>5)&0x3f;
	b=color&0x1f;

	return ((r<<13)|(g<<6)|(b<<1));
}

/**
 * @brief 颜色转换函数
 */
uint16_t LCD_RGBTo565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void LCD_WriteData_Color(uint16_t color)
{
	*(TFTLCD.data_reg) = color;
}

//读数据
//返回值:读到的值
uint16_t LCD_ReadData(void)
{
	return (*TFTLCD.cmd_reg);
}


//设置LCD显示方向
//dir:0,竖屏；1,横屏
void LCD_Display_Dir(uint8_t dir)
{
	tftlcd_data.dir=dir;         //横屏/竖屏
	if(dir==0)  //默认竖屏方向
	{
		LCD_WriteCmdData(0x36,0x00);
		tftlcd_data.height=800;
		tftlcd_data.width=480;
	}
	else
	{
		LCD_WriteCmdData(0x36,0x68);
		tftlcd_data.height=480;
		tftlcd_data.width=800;
	}
}

/**
  * @brief  设置显示窗口
  * @param  x1: 起始X坐标
  * @param  y1: 起始Y坐标
  * @param  x2: 结束X坐标，必须大于x1
  * @param  y2: 结束Y坐标，必须大于y1
  * @retval None
  */
void LCD_SetWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	// 设置列地址
	LCD_WriteCmd(0x2A);
	LCD_WriteData(x1 >> 8);
	LCD_WriteData(x1);
	LCD_WriteData(x2 >> 8);
	LCD_WriteData(x2);
	// 设置行地址
	LCD_WriteCmd(0x2B);
	LCD_WriteData(y1 >> 8);
	LCD_WriteData(y1);
	LCD_WriteData(y2 >> 8);
	LCD_WriteData(y2);
	// 开始写入RAM
	LCD_WriteCmd(RAMWR);
}

//读取个某点的颜色值
//x,y:坐标
//返回值:此点的颜色
uint16_t LCD_ReadPoint(uint16_t x,uint16_t y)
{
 	uint16_t r=0,g=0,b=0;
//	uint16_t r1,r2,r3;
//	uint32_t value;

	if(x>=tftlcd_data.width||y>=tftlcd_data.height)return 0;	//超过了范围,直接返回
	LCD_SetWindow(x, y, x, y);
	LCD_WriteCmd(0X2e);
	r=LCD_ReadData();								//dummy Read
	r=LCD_ReadData();								//实际坐标颜色
	//printf("r=%X\r\n",r);
	b=LCD_ReadData();
	g=LCD_ReadData();
	//printf("g=%X\r\n",g);
	//printf("b=%X\r\n",b);
	return (((r>>11)<<11)|((g>>10)<<5)|(b>>11));
}

//SSD1963 背光设置
//pwm:背光等级,0~100.越大越亮.
void LCD_SSD_BackLightSet(uint8_t pwm)
{
	LCD_WriteCmd(0xBE);	//配置PWM输出
	LCD_WriteData(0x05);	//1设置PWM频率
	LCD_WriteData(pwm*2.55);//2设置PWM占空比
	LCD_WriteData(0x01);	//3设置C
	LCD_WriteData(0xFF);	//4设置D
	LCD_WriteData(0x00);	//5设置E
	LCD_WriteData(0x00);	//6设置F
}

/**
 * @brief LCD初始化函数
 * 初始化ILI9806 TFT LCD显示屏，配置各种参数和寄存器
 */
void LCD_Init(void)
{
	// 上电延时，等待LCD电源稳定
	HAL_Delay(50);

	// 读取LCD ID命令
	LCD_WriteCmd(0xD3);
	// 连续读取4个字节的ID数据
	tftlcd_data.id = *(TFTLCD.data_reg);	// 读取ID字节1
	tftlcd_data.id = *(TFTLCD.data_reg);	// 读取ID字节2
	tftlcd_data.id = *(TFTLCD.data_reg);	// 读取ID字节3
	tftlcd_data.id <<= 8;					// 将前3个字节左移8位
	tftlcd_data.id |= *(TFTLCD.data_reg);	// 读取并合并第4个字节

	// 进入扩展命令模式
	LCD_WriteCmd(0xFF);
	LCD_WriteData(0xFF);
	LCD_WriteData(0x98);
	LCD_WriteData(0x06);

	// 设置SPI读写模式
	LCD_WriteCmd(0xBA);
	LCD_WriteData(0x60);

	// 设置显示相关参数
	LCD_WriteCmd(0xBC);
	LCD_WriteData(0x03);	// 参数1
	LCD_WriteData(0x0E);	// 参数2
	LCD_WriteData(0x03);	// 参数3
	LCD_WriteData(0x63);	// 参数4
	LCD_WriteData(0x01);	// 参数5
	LCD_WriteData(0x01);	// 参数6
	LCD_WriteData(0x1B);	// 参数7
	LCD_WriteData(0x12);	// 参数8
	LCD_WriteData(0x6F);	// 参数9
	LCD_WriteData(0x00);	// 参数10
	LCD_WriteData(0x00);	// 参数11
	LCD_WriteData(0x00);	// 参数12
	LCD_WriteData(0x01);	// 参数13
	LCD_WriteData(0x01);	// 参数14
	LCD_WriteData(0x03);	// 参数15
	LCD_WriteData(0x02);	// 参数16
	LCD_WriteData(0xFF);	// 参数17
	LCD_WriteData(0xF2);	// 参数18
	LCD_WriteData(0x01);	// 参数19
	LCD_WriteData(0x00);	// 参数20
	LCD_WriteData(0xC0);	// 参数21

	// 设置内部时序参数
	LCD_WriteCmd(0xBD);
	LCD_WriteData(0x02);	// 时序参数1
	LCD_WriteData(0x13);	// 时序参数2
	LCD_WriteData(0x45);	// 时序参数3
	LCD_WriteData(0x67);	// 时序参数4
	LCD_WriteData(0x45);	// 时序参数5
	LCD_WriteData(0x67);	// 时序参数6
	LCD_WriteData(0x01);	// 时序参数7
	LCD_WriteData(0x23);	// 时序参数8

	// 设置电源相关参数
	LCD_WriteCmd(0xBE);
	LCD_WriteData(0x01);	// 电源参数1
	LCD_WriteData(0x22);	// 电源参数2
	LCD_WriteData(0x22);	// 电源参数3
	LCD_WriteData(0xDC);	// 电源参数4
	LCD_WriteData(0xBA);	// 电源参数5
	LCD_WriteData(0x67);	// 电源参数6
	LCD_WriteData(0x22);	// 电源参数7
	LCD_WriteData(0x22);	// 电源参数8
	LCD_WriteData(0x22);	// 电源参数9

	// 设置VCOM电压
	LCD_WriteCmd(0xC7);
	LCD_WriteData(0x66);	// VCOM电压值

	// 设置内部电源控制
	LCD_WriteCmd(0xED);
	LCD_WriteData(0x7F);	// 电源控制参数1
	LCD_WriteData(0x0F);	// 电源控制参数2
	LCD_WriteData(0x00);	// 电源控制参数3

	// 设置电源控制1
	LCD_WriteCmd(0xC0);
	LCD_WriteData(0x03);	// BT[1:0], DC0[2:0]
	LCD_WriteData(0x0B);	// DC1[2:0]
	LCD_WriteData(0x00);	// 保留

	// 设置面板驱动时序
	LCD_WriteCmd(0XF5);
	LCD_WriteData(0x20);	// 时序参数1
	LCD_WriteData(0x43);	// 时序参数2
	LCD_WriteData(0x00);	// 时序参数3

	// 设置内部寄存器
	LCD_WriteCmd(0xEE);
	LCD_WriteData(0x0A);	// 内部参数1
	LCD_WriteData(0x1B);	// 内部参数2
	LCD_WriteData(0x5F);	// 内部参数3
	LCD_WriteData(0x40);	// 内部参数4
	LCD_WriteData(0x28);	// 内部参数5
	LCD_WriteData(0x38);	// 内部参数6
	LCD_WriteData(0x02);	// 内部参数7
	LCD_WriteData(0x2B);	// 内部参数8
	LCD_WriteData(0x50);	// 内部参数9
	LCD_WriteData(0x00);	// 内部参数10
	LCD_WriteData(0x80);	// 内部参数11

	// 设置内部振荡器
	LCD_WriteCmd(0xFC);
	LCD_WriteData(0x08);	// 振荡器控制

	// 设置内部参数
	LCD_WriteCmd(0xDF);
	LCD_WriteData(0x00);	// 参数1
	LCD_WriteData(0x00);	// 参数2
	LCD_WriteData(0x00);	// 参数3
	LCD_WriteData(0x00);	// 参数4
	LCD_WriteData(0x00);	// 参数5
	LCD_WriteData(0x20);	// 参数6

	// 设置内部寄存器
	LCD_WriteCmd(0xF3);
	LCD_WriteData(0x74);	// 寄存器值

	// 设置显示反转控制
	LCD_WriteCmd(0xB4);
	LCD_WriteData(0x02);	// 反转参数1
	LCD_WriteData(0x02);	// 反转参数2
	LCD_WriteData(0x02);	// 反转参数3

	// 设置内部参数
	LCD_WriteCmd(0xF7);
	LCD_WriteData(0x82);	// 参数值

	// 设置帧率控制
	LCD_WriteCmd(0xB1);
	LCD_WriteData(0x00);	// 帧率参数1
	LCD_WriteData(0x13);	// 帧率参数2 (RTNA)
	LCD_WriteData(0x13);	// 帧率参数3 (DIVA)

	// 设置显示功能控制
	LCD_WriteCmd(0xF2);
	LCD_WriteData(0x41);	// 功能参数1
	LCD_WriteData(0x04);	// 功能参数2
	LCD_WriteData(0x41);	// 功能参数3
	LCD_WriteData(0x28);	// 功能参数4

	// 设置电源控制2
	LCD_WriteCmd(0xC1);
	LCD_WriteData(0x17);	// SAP[1:0], BT[1:0]
	LCD_WriteData(0x78);	// 电源参数2
	LCD_WriteData(0x7B);	// 电源参数3
	LCD_WriteData(0x20);	// 电源参数4

	// 设置正极性Gamma校正
	LCD_WriteCmd(0xE0);
	LCD_WriteData(0x00);	// Gamma参数1 (VP0)
	LCD_WriteData(0x02);	// Gamma参数2 (VP1)
	LCD_WriteData(0x0C);	// Gamma参数3 (VP2)
	LCD_WriteData(0x0F);	// Gamma参数4 (VP3)
	LCD_WriteData(0x11);	// Gamma参数5 (VP4)
	LCD_WriteData(0x1C);	// Gamma参数6 (VP5)
	LCD_WriteData(0xC8);	// Gamma参数7 (VP6)
	LCD_WriteData(0x07);	// Gamma参数8 (VP7)
	LCD_WriteData(0x03);	// Gamma参数9 (VP8)
	LCD_WriteData(0x08);	// Gamma参数10 (VP9)
	LCD_WriteData(0x03);	// Gamma参数11 (VP10)
	LCD_WriteData(0x0D);	// Gamma参数12 (VP11)
	LCD_WriteData(0x0C);	// Gamma参数13 (VP12)
	LCD_WriteData(0x31);	// Gamma参数14 (VP13)
	LCD_WriteData(0x2C);	// Gamma参数15 (VP14)
	LCD_WriteData(0x00);	// Gamma参数16

	// 设置负极性Gamma校正
	LCD_WriteCmd(0xE1);
	LCD_WriteData(0x00);	// Gamma参数1 (VN0)
	LCD_WriteData(0x02);	// Gamma参数2 (VN1)
	LCD_WriteData(0x08);	// Gamma参数3 (VN2)
	LCD_WriteData(0x0E);	// Gamma参数4 (VN3)
	LCD_WriteData(0x12);	// Gamma参数5 (VN4)
	LCD_WriteData(0x17);	// Gamma参数6 (VN5)
	LCD_WriteData(0x7C);	// Gamma参数7 (VN6)
	LCD_WriteData(0x0A);	// Gamma参数8 (VN7)
	LCD_WriteData(0x03);	// Gamma参数9 (VN8)
	LCD_WriteData(0x09);	// Gamma参数10 (VN9)
	LCD_WriteData(0x06);	// Gamma参数11 (VN10)
	LCD_WriteData(0x0C);	// Gamma参数12 (VN11)
	LCD_WriteData(0x0C);	// Gamma参数13 (VN12)
	LCD_WriteData(0x2E);	// Gamma参数14 (VN13)
	LCD_WriteData(0x2A);	// Gamma参数15 (VN14)
	LCD_WriteData(0x00);	// Gamma参数16

	// 设置撕裂效果输出
	LCD_WriteCmd(0x35);
	LCD_WriteData(0x00);	// 禁用撕裂效果输出

	// 设置内存访问控制
	LCD_WriteCmd(0x36);
	LCD_WriteData(0xC0);	// 控制显示方向、颜色顺序等

	// 设置像素格式
	LCD_WriteCmd(0x3A);
	LCD_WriteData(0x55);	// 16位像素格式 (RGB565)

	// 退出睡眠模式
	LCD_WriteCmd(0x11);
	HAL_Delay(120);			// 等待120ms让LCD稳定

	// 开启显示
	LCD_WriteCmd(0x29);

	// 设置显示方向
	LCD_Display_Dir(TFTLCD_DIR);		// 0：竖屏  1：横屏  默认竖屏

	// 清屏为白色
	LCD_Clear(WHITE);
}

/**
  * @brief  清屏
  * @param  color: 颜色值，要清屏的填充色
  * @retval None
  */
void LCD_Clear(uint16_t color)
{
	uint32_t i;

	LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);

	for(i = 0; i < LCD_PIXEL_COUNT; ++i)
	{
		*(TFTLCD.data_reg) = color;
	}
}


//在指定区域内填充单个颜色
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)
//color:要填充的颜色
void LCD_Fill(uint16_t xState,uint16_t yState,uint16_t xEnd,uint16_t yEnd,uint16_t color)
{
	uint16_t temp;

    if((xState > xEnd) || (yState > yEnd))
    {
        return;
    }
	LCD_SetWindow(xState, yState, xEnd, yEnd);
    xState = xEnd - xState + 1;
	yState = yEnd - yState + 1;

	while(xState--)
	{
	 	temp = yState;
		while (temp--)
	 	{
			LCD_WriteData_Color(color);
		}
	}
}

//在指定区域内填充指定颜色块
//(sx,sy),(ex,ey):填充矩形对角坐标,区域大小为:(ex-sx+1)*(ey-sy+1)
//color:要填充的颜色
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color)
{
	uint16_t height,width;
	uint16_t i,j;
	width=ex-sx+1; 			//得到填充的宽度
	height=ey-sy+1;			//高度

	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_SetWindow(sx+j, sy+i,ex, ey);
			LCD_WriteData_Color(color[i*width+j]);
		}
	}
}
//画点
//x,y:坐标
//FRONT_COLOR:此点的颜色
void LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetWindow(x, y, x, y);  //设置点的位置
	LCD_WriteData_Color(FRONT_COLOR);
}

//快速画点
//x,y:坐标
//color:颜色
void LCD_DrawFRONT_COLOR(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_SetWindow(x, y, x, y);
	LCD_WriteData_Color(color);
}

//画线
//x1,y1:起点坐标
//x2,y2:终点坐标
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向
	else if(delta_x==0)incx=0;//垂直线
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if(delta_y==0)incy=0;//水平线
	else{incy=-1;delta_y=-delta_y;}
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
	else distance=delta_y;
	for(t=0;t<=distance+1;t++ )//画线输出
	{
		LCD_DrawPoint(uRow,uCol);//画点
		xerr+=delta_x ;
		yerr+=delta_y ;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}

void LCD_DrawLine_Color(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量
	delta_y=y2-y1;
	uRow=x1;
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向
	else if(delta_x==0)incx=0;//垂直线
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if(delta_y==0)incy=0;//水平线
	else{incy=-1;delta_y=-delta_y;}
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
	else distance=delta_y;
	for(t=0;t<=distance+1;t++ )//画线输出
	{
		LCD_DrawFRONT_COLOR(uRow,uCol,color);//画点
		xerr+=delta_x ;
		yerr+=delta_y ;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


// 画一个十字的标记
// x：标记的X坐标
// y：标记的Y坐标
// color：标记的颜色
void LCD_DrowSign(uint16_t x, uint16_t y, uint16_t color)
{
    uint8_t i;

    /* 画点 */
    LCD_SetWindow(x-1, y-1, x+1, y+1);
    for(i=0; i<9; i++)
    {
		LCD_WriteData_Color(color);
    }

    /* 画竖 */
    LCD_SetWindow(x-4, y, x+4, y);
    for(i=0; i<9; i++)
    {
		LCD_WriteData_Color(color);
    }

    /* 画横 */
    LCD_SetWindow(x, y-4, x, y+4);
    for(i=0; i<9; i++)
    {
		LCD_WriteData_Color(color);
    }
}

//画矩形
//(x1,y1),(x2,y2):矩形的对角坐标
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	LCD_DrawLine(x1,y1,x2,y1);
	LCD_DrawLine(x1,y1,x1,y2);
	LCD_DrawLine(x1,y2,x2,y2);
	LCD_DrawLine(x2,y1,x2,y2);
}
//在指定位置画一个指定大小的圆
//(x,y):中心点
//r    :半径
void LCD_Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r)
{
	int a,b;
	int di;
	a=0;b=r;
	di=3-(r<<1);             //判断下个点位置的标志
	while(a<=b)
	{
		LCD_DrawPoint(x0+a,y0-b);             //5
 		LCD_DrawPoint(x0+b,y0-a);             //0
		LCD_DrawPoint(x0+b,y0+a);             //4
		LCD_DrawPoint(x0+a,y0+b);             //6
		LCD_DrawPoint(x0-a,y0+b);             //1
 		LCD_DrawPoint(x0-b,y0+a);
		LCD_DrawPoint(x0-a,y0-b);             //2
  		LCD_DrawPoint(x0-b,y0-a);             //7
		a++;
		//使用Bresenham算法画圆
		if(di<0)di +=4*a+6;
		else
		{
			di+=10+4*(a-b);
			b--;
		}
	}
}



//在指定位置显示一个字符
//x,y:起始坐标
//num:要显示的字符:" "--->"~"
//size:字体大小 12/16/24
//mode:叠加方式(1)还是非叠加方式(0)
void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode)
{
    uint8_t temp, t1, t;
	uint16_t y0 = y;
	uint8_t csize =(size / 8 + ((size % 8) ? 1 : 0)) * (size / 2);		//得到字体一个字符对应点阵集所占的字节数
 	num = num - ' '; // 得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
	for(t = 0; t < csize; t++)
	{
		if(size==12)
			temp=ascii_1206[num][t]; 	 	//调用12x06字体
		else if(size==16)
			temp=ascii_1608[num][t];	//调用16x08字体
		else if(size==24)
			temp=ascii_2412[num][t];	//调用24x12字体
		else
			return;								//没有的字库

		for(t1 = 0; t1 < 8; t1++)
		{
			if(temp & 0x80)
				LCD_DrawFRONT_COLOR(x, y, FRONT_COLOR);
			else if(mode == 0)
				LCD_DrawFRONT_COLOR(x, y, BACK_COLOR);
			temp <<= 1;
			y++;
			if(y >= tftlcd_data.height)return;		//超区域了
			if((y - y0) == size)
			{
				y=y0;
				x++;
				if(x>=tftlcd_data.width)return;	//超区域了
				break;
			}
		}
	}
}
//m^n函数
//返回值:m^n次方.
uint32_t LCD_Pow(uint8_t m,uint8_t n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}
//显示数字,高位为0,则不显示
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//color:颜色
//num:数值(0~4294967295);
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,0);
				continue;
			}else enshow=1;

		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,0);
	}
}

//显示数字,高位为0,还是显示
//x,y:起点坐标
//num:数值(0~999999999);
//len:长度(即要显示的位数)
//size:字体大小
//mode:
//[7]:0,不填充;1,填充0.
//[6:1]:保留
//[0]:0,非叠加显示;1,叠加显示.
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode)
{
	uint8_t t,temp;
	uint8_t enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,mode&0X01);
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,mode&0X01);
 				continue;
			}else enshow=1;

		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,mode&0X01);
	}
}
//显示字符串
//x,y:起点坐标
//width,height:区域大小
//size:字体大小
//*p:字符串起始地址
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p)
{
	uint8_t x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//退出
        LCD_ShowChar(x,y,*p,size,0);
        x+=size/2;
        p++;
    }
}

/****************************************************************************
*函数名：LCD_ShowFontHZ
*输  入：x：汉字显示的X坐标
*      * y：汉字显示的Y坐标
*      * cn：要显示的汉字
*      * wordColor：文字的颜色
*      * backColor：背景颜色
*输  出：
*功  能：写二号楷体汉字
****************************************************************************/
#if 0
void LCD_ShowFontHZ(uint16_t x, uint16_t y, uint8_t *cn)
{
	uint8_t i, j, wordNum;
	uint16_t color;

	while (*cn != '\0')
	{
		LCD_SetWindow(x, y, x+31, y+28);
		for (wordNum=0; wordNum<10; wordNum++)
		{	//wordNum扫描字库的字数
			if ((CnChar32x29[wordNum].Index[0]==*cn)
			     &&(CnChar32x29[wordNum].Index[1]==*(cn+1)))
			{
				for(i=0; i<116; i++)
				{	//MSK的位数
					color=CnChar32x29[wordNum].Msk[i];
					for(j=0;j<8;j++)
					{
						if((color&0x80)==0x80)
						{
							LCD_WriteData_Color(FRONT_COLOR);
						}
						else
						{
							LCD_WriteData_Color(BACK_COLOR);
						}
						color<<=1;
					}//for(j=0;j<8;j++)结束
				}
			}
		} //for (wordNum=0; wordNum<20; wordNum++)结束
		cn += 2;
		x += 32;
	}
}
#endif


#if 1
void LCD_ShowFontHZ(uint16_t x, uint16_t y, uint8_t *cn)
{
	uint8_t i, j, wordNum;
	uint16_t color;
	uint16_t x0=x;
	uint16_t y0=y;

	while (*cn != '\0')
	{
		for (wordNum=0; wordNum<10; wordNum++)
		{	//wordNum扫描字库的字数
			if ((CnChar32x29[wordNum].Index[0] == *cn)
			     &&(CnChar32x29[wordNum].Index[1]==*(cn+1))
				 &&(CnChar32x29[wordNum].Index[2]==*(cn+2))
			)
			{
				for(i=0; i<116; i++)
				{	//MSK的位数
					color=CnChar32x29[wordNum].Msk[i];
					for(j=0;j<8;j++)
					{
						if((color&0x80)==0x80)
						{
							LCD_DrawFRONT_COLOR(x,y,FRONT_COLOR);
							__DMB();
						}
						else
						{
							LCD_DrawFRONT_COLOR(x,y,BACK_COLOR);
							__DMB();
						}
						color<<=1;
						x++;
						if((x-x0)==32)
						{
							x=x0;
							y++;
							if((y-y0)==29)
							{
								y=y0;
							}
						}
					}//for(j=0;j<8;j++)结束
				}
			}

		} //for (wordNum=0; wordNum<20; wordNum++)结束
		cn += 3;
		x += 32;
		x0=x;
	}
}
#endif

/**
  * @brief  显示图像
  * @param  x: 起始X坐标
  * @param  y: 起始Y坐标
  * @param  image: Image图像数据指针
  * @retval None
  */
void LCD_ShowPicture(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *pic)
{
	uint32_t i;
	uint32_t total_pixels = width * height;

	if((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) return;
	if((x + width) > LCD_WIDTH)
		width = LCD_WIDTH - x;
	if((y + height) > LCD_HEIGHT)
		height = LCD_HEIGHT - y;

	LCD_SetWindow(x, y, x + width - 1, y + height - 1);

	for(i = 0; i < total_pixels; i++)
	{
		LCD_WriteData(pic[i]);
	}
}

//void LCD_DrawTest()
//{
//	// 测试图案数据
//	const uint16_t test_pattern_16x16[] = {
//	    0xF800, 0xF800, 0xF800, 0xF800, 0x07E0, 0x07E0, 0x07E0, 0x07E0,
//	    0x001F, 0x001F, 0x001F, 0x001F, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
//	    0xF800, 0xF800, 0xF800, 0xF800, 0x07E0, 0x07E0, 0x07E0, 0x07E0,
//	    0x001F, 0x001F, 0x001F, 0x001F, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
//	    0xF800, 0xF800, 0xF800, 0xF800, 0x07E0, 0x07E0, 0x07E0, 0x07E0,
//	    0x001F, 0x001F, 0x001F, 0x001F, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
//	    0xF800, 0xF800, 0xF800, 0xF800, 0x07E0, 0x07E0, 0x07E0, 0x07E0,
//	    0x001F, 0x001F, 0x001F, 0x001F, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
//	};
//	// 将数据指针转换为FSMC数据寄存器指针
//
//	LCD_SetWindow(20, 400, 36-1, 416-1);
//	memcpy(TFTLCD.data_reg, test_pattern_16x16, 8*8*2); // 可行
//	memcpy(TFTLCD.data_reg, test_pattern_16x16, 8*8*2); // 可行
//	memcpy(TFTLCD.data_reg, test_pattern_16x16, 8*8*2); // 可行
//	memcpy(TFTLCD.data_reg, test_pattern_16x16, 8*8*2); // 可行
////	*TFTLCD.data_reg = test_pattern_16x16; // 错误
//
//	__DMB();
//}
/**
  * @brief  使用部分缓冲显示图像
  * @param  x: 起始X坐标
  * @param  y: 起始Y坐标
  * @param  width: 图像宽度
  * @param  height: 图像高度
  * @param  image: 图像数据指针
  * @retval None
  */
//void LCD_DrawImagePartial(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *image)
//{
//    uint16_t chunk_height;
//    uint16_t current_y = y;
//    uint32_t source_offset = 0;
//    uint32_t chunk_size;
//
//    if((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) return;
//    if((x + width) > LCD_WIDTH) width = LCD_WIDTH - x;
//    if((y + height) > LCD_HEIGHT) height = LCD_HEIGHT - y;
//
//    // 分块传输
//    while(height > 0)
//    {
//        chunk_height = (height > PARTIAL_BUFFER_HEIGHT) ? PARTIAL_BUFFER_HEIGHT : height;
//        chunk_size = width * chunk_height;
//
//        // 复制数据到部分缓冲区
//        memcpy(partial_buffer, &image[source_offset], chunk_size * sizeof(uint16_t));
//
//        // 刷新到屏幕
//        LCD_FlushPartialBuffer(x, current_y, width, chunk_height, partial_buffer);
//
//        current_y += chunk_height;
//        source_offset += chunk_size;
//        height -= chunk_height;
//    }
//}

/**
  * @brief  刷新部分缓冲区到屏幕
  * @param  x: 起始X坐标
  * @param  y: 起始Y坐标
  * @param  width: 宽度
  * @param  height: 高度
  * @param  buffer: 缓冲区指针
  * @retval None
  */
//void LCD_FlushPartialBuffer(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *buffer)
//{
//    uint32_t i;
//    uint32_t total_pixels = width * height;
//
//    if((x >= LCD_WIDTH) || (y >= LCD_HEIGHT)) return;
//    if((x + width) > LCD_WIDTH) width = LCD_WIDTH - x;
//    if((y + height) > LCD_HEIGHT) height = LCD_HEIGHT - y;
//
//    LCD_SetWindow(x, y, x + width - 1, y + height - 1);
//
//    for(i = 0; i < total_pixels; i++)
//    {
//        LCD_WriteData(buffer[i]);
//    }
//}
