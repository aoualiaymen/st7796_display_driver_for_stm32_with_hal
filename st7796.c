/*
 * st7796.c
 *
 *  Created on: Aug 27, 2024
 *      Author: ENIGMA
 */
#include "main.h"
#include "st7796.h"
#include <stdbool.h>
#include "stdint.h"
//#include "fonts.h"
//#include "bmp_fonts.h"


uint16_t x_cursor=0;
uint16_t y_cursor=0;
//HAL_GPIO_WritePin(GPIOA, TFT_RESET_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
 // HAL_GPIO_WritePin(TFT_RS_GPIO_Port, TFT_RS_Pin, GPIO_PIN_RESET);
extern SPI_HandleTypeDef hspi1;
#define ST7796_SPI &hspi1
#define ST7796_CS_GPIO GPIOA
#define ST7796_DC_GPIO GPIOC
#define	ST7796_DC TFT_RS_Pin
#define ST7796_CS TFT_CS_Pin
#define ST7796_RES_GPIO GPIOA
#define ST7796_RES TFT_RESET_Pin

void st7796_command(uint8_t cmd){
	HAL_GPIO_WritePin(ST7796_DC_GPIO, ST7796_DC,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_RESET);

	HAL_SPI_Transmit(ST7796_SPI, &cmd,1,100);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_SET);

}
void st7796_write_data(uint8_t* data, uint32_t size){
	HAL_GPIO_WritePin(ST7796_DC_GPIO, ST7796_DC,GPIO_PIN_SET);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_RESET);

	HAL_SPI_Transmit(ST7796_SPI, data,size,100);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_SET);

}
void st7796_read_data(uint8_t* data, uint16_t size){
	HAL_GPIO_WritePin(ST7796_DC_GPIO, ST7796_DC,GPIO_PIN_SET);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_RESET);
	HAL_SPI_Receive(ST7796_SPI, data,size,100);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_SET);
}
void st7796_reset(){
	HAL_GPIO_WritePin(ST7796_RES_GPIO,ST7796_RES,GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ST7796_RES_GPIO,ST7796_RES,GPIO_PIN_SET);

}
void set_window_area(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye){
	uint8_t data[8];
	data[0]=xs>>8;
	data[1]=xs&0xff;
	data[2]=xe>>8;
	data[3]=xe&0xff;
	data[4]=ys>>8;
	data[5]=ys&0xff;
	data[6]=ye>>8;
	data[7]=ye&0xff;
	st7796_command(ST7735_CASET);
	st7796_write_data(data,4);
	st7796_command(ST7735_RASET);
	st7796_write_data(data+4,4);
}

void set_window_color(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye, uint16_t color){
	set_window_area(xs,xe,ys,ye);
	uint8_t data[2];
	data[0]=color>>8;
	data[1]=color&0xff;
	uint32_t area_size=(xe-xs+1)*(ye-ys+1);
	st7796_command(ST7735_RAMWR);
	for(uint32_t i=0;i<area_size;i++){
		st7796_write_data(data,2);


	}
	//st7796_command(ST7735_DISPON);
	HAL_Delay(1);
}
//void print_fonts_char(sFONT font,uint16_t startX,uint16_t startY,char a,uint16_t color,uint16_t bg_color){
//
//	uint8_t char_type= ((int) a)-32;
//
//	uint8_t h=font.Height;
//	uint8_t w=font.Width;
//	uint8_t size=font.Size;
//	const uint8_t *data= &font.table[char_type*(h*size)];
//	set_window_area(startX,startX+w-1,startY,startY+h-1);
//	uint8_t bg[2],ch[2];
//	bg[0] =bg_color>>8;
//	bg[1] =bg_color&0xff;
//	ch[0] =color>>8;
//	ch[1] =color&0xff;
//	int8_t j,i;
//	st7796_command(ST7735_RAMWR);
//	for(j=0;j<h;j++){
//		for(i=0;i<w;i++){
//			if((data[j*size+i/8]>>(7-i%8)) & 1)
//				st7796_write_data(ch,2);
//			else
//				st7796_write_data(bg,2);
//		}}}
//void write_string(sFONT font,char a[],uint16_t startX,uint16_t startY,uint16_t color,uint16_t bg_color){
//	uint16_t char_nbr=strlen(a);
//	x_cursor=startX;
//	y_cursor=startY;
//	uint8_t h=font.Height;
//	uint8_t w=font.Width;
//	uint8_t size=font.Size;
//	uint16_t x_lenght =w+size;
//
//	if((startY+h-1)<=319){
//	for(uint16_t i=0;i<char_nbr;i++){
//		print_fonts_char(font,x_cursor,y_cursor,a[i],color,bg_color);
//		x_cursor +=w;
//        if((x_cursor+x_lenght)>480){
//        	break;
//        }
//		//y_cursor +=h;
//		set_window_color(x_cursor, x_cursor+size-1, y_cursor, y_cursor+h-1, bg_color);
//	}}
//}

void writedata(uint8_t data){
	st7796_write_data(&data,1);

}



void st7796_init(){
	st7796_reset();
	st7796_command(ST7735_SWRESET);
	HAL_Delay(100);
	st7796_command(ST7735_GMCTRP1);
	writedata(0x00);
	writedata(0x03);
	writedata(0x09);
	writedata(0x08);
	writedata(0x16);
	writedata(0x0A);
	writedata(0x3F);
	writedata(0x78);
	writedata(0x4C);
	writedata(0x09);
	writedata(0x0A);
	writedata(0x08);
	writedata(0x16);
	writedata(0x1A);

	st7796_command(ST7735_GMCTRN1);
	writedata(0x00);
	writedata(0x16);
	writedata(0x19);
	writedata(0x03);
	writedata(0x0F);
	writedata(0x05);
	writedata(0x32);
	writedata(0x45);
	writedata(0x46);
	writedata(0x04);
	writedata(0x0E);
	writedata(0x0D);
	writedata(0x35);
	writedata(0x37);

	st7796_command(ST7735_PWCTR1);
	writedata(0x80);
	writedata(0x25);

	st7796_command(ST7735_PWCTR2);
	writedata(0x13);

	st7796_command(ST7735_PWCTR3);
	writedata(0x05);

	st7796_command(ST7735_MADCTL);
	writedata(0xe8);//0x48
	st7796_command(ST7735_COLMOD);
	writedata(0x55);
	st7796_command(ST7796_IFMODE);
	writedata(0x0);

	st7796_command(ST7735_FRMCTR1);
	writedata(0xa0);
	writedata(0x10);

	st7796_command(ST7735_INVCTR);
	writedata(0x02);

	st7796_command(ST7735_DISSET5);
	writedata(0x80);
	writedata(0x02);
	writedata(0x3b);

	st7796_command(ST7796_DOCA);
	writedata(0x40);
	writedata(0x8a);
	writedata(0x0);
	writedata(0x0);
	writedata(0x25);
	writedata(0x0a);
	writedata(0x38);
	writedata(0x33);
	HAL_Delay(200);

	st7796_command(ST7735_SLPOUT);
	HAL_Delay(130);
	st7796_command(ST7735_NORON);
	HAL_Delay(200);
	st7796_command(ST7796_IDMOFF);
	HAL_Delay(200);
	st7796_command(ST7735_DISPON);

}




// printing image and characters and strings
void print_image( uint8_t* image,uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye){
	uint32_t nbr_pixel=(xe-xs+1)*(ye-ys+1);
	set_window_area(xs,xe,ys,ye);
	st7796_command(ST7735_RAMWR);
	for(uint32_t i=0;i<nbr_pixel;i++)
	st7796_write_data(image+(i*2),2);
}
/*void st7796_write_data_DMA(uint8_t* data, uint32_t size){

		HAL_GPIO_WritePin(ST7796_DC_GPIO, ST7796_DC,GPIO_PIN_SET);
		HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_RESET);
		HAL_SPI_Transmit_DMA(ST7796_SPI, data,size);
		//HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_SET);

}*/
void print_image_DMA(uint8_t* image,uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye){
	uint32_t nbr_pixel=(xe-xs+1)*(ye-ys+1);
	set_window_area(xs,xe,ys,ye);
	st7796_command(ST7735_RAMWR);
	HAL_GPIO_WritePin(ST7796_DC_GPIO, ST7796_DC,GPIO_PIN_SET);
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(ST7796_SPI,image,nbr_pixel*2);
	//st7796_write_data_DMA(image,nbr_pixel);
}
void print_dma(uint8_t* image){

	HAL_SPI_Transmit_DMA(ST7796_SPI,image,57600);
}
void cs_clear(){
	HAL_GPIO_WritePin(ST7796_CS_GPIO,ST7796_CS,GPIO_PIN_SET);
}

//void write_char(uint8_t font_size,uint16_t x,uint16_t y, char ch, uint16_t bach_ground_color,uint16_t char_color){
//	uint8_t back_ground[2]={bach_ground_color>>8,bach_ground_color & 0xff};
//	uint8_t character[2]={char_color>>8,char_color & 0xff};
//	uint16_t offset= ((int)ch)-32;
//	uint8_t w=bmp_font50_map[offset].box_w;
//	uint8_t h=bmp_font50_map[offset].box_h;
//	uint32_t nbr_pixel= w*font_size;
//	uint16_t information_area=w*h; // here is the real area of character
//	uint16_t nbr_bytes= ((information_area)/8)+1;
//	//uint8_t bitmap[nbr_bytes];//=&bmp_font8[offset];
//
//	uint32_t i;
//	set_window_area(x,x+w-1,y,y+font_size-1);
//	st7796_command(ST7735_RAMWR);
//	for(i=0;i<nbr_pixel;i++){
//		if(i<nbr_bytes*8){
//			if(bmp_font50[offset+i/8] & (0x80>>(i%8)))
//				st7796_write_data(character,2);
//			else
//				st7796_write_data(back_ground,2);
//
//			}
//		else
//		{	st7796_write_data(back_ground,2);
//
//		}
//	}
//
//	}


void H_line(uint16_t startX,uint16_t startY,uint16_t lenght,uint16_t width,uint16_t color){
	uint16_t l=lenght;
	uint16_t w=width;
	if((startX+l)>480){
		l= 480-startX;
	}
	if((startY-w)>320){
		w= 320-startY;
	}
	set_window_color(startX, startX+l-1, startY, startY+w-1,color);
}

void V_line(uint16_t startX,uint16_t startY,uint16_t lenght,uint16_t width,uint16_t color){
	uint16_t l=lenght;
	uint16_t w=width;
	if((startY+l)>320){
		l= 320-startY;
	}
	if((startX-w)>480){
		w= 480-startX;
	}
	set_window_color(startX, startX+w-1, startY, startY+l-1,color);
}

/*
uint8_t SPI1_TX_completed_flag;

#define DELAY 0x80

// based on Adafruit ST7735 library for Arduino
static const uint8_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    12,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
           //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay://--- here we talk about dic
      1,                   //     No inversion
                                     //                    VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 1      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small                  //     Boost frequency
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0x0,        //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },   //     16-bit color

init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x01, 0x3F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x01, 0xdF ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors

	  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
	      4,                        //  4 commands in list:
	      ST7735_GMCTRP1, 14      , //  1: Gamma Adjustments (pos. polarity), 16 args, no delay:
	        0x02, 0x1c, 0x07, 0x12,
	        0x37, 0x32, 0x29, 0x2d,
	        0x29, 0x25, 0x2B, 0x39,
	        0x00, 0x01,
	      ST7735_GMCTRN1, 14      , //  2: Gamma Adjustments (neg. polarity), 16 args, no delay:
	        0x03, 0x1d, 0x07, 0x06,
	        0x2E, 0x2C, 0x29, 0x2D,
	        0x2E, 0x2E, 0x37, 0x3F,
	        0x00, 0x00,
	      ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
	        10,                     //     10 ms delay
	      ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
	        100 };                  //     100 ms delay

	  static void ST7735_Select() {
	      HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
	  }

	  void ST7735_Unselect() {
	      HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
	  }

	  static void ST7735_Reset() {
	      HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
	      HAL_Delay(5);
	      HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
	  }

	  static void ST7735_WriteCommand(uint8_t cmd) {
	      HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
	      HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
	  }

	  static void ST7735_WriteData(uint8_t* buff, size_t buff_size) {
	      HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
	      HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
	  }
	  void send_command(uint8_t a){
		  ST7735_Select();
		  HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
		  HAL_SPI_Transmit(&ST7735_SPI_PORT, &a,1, HAL_MAX_DELAY);
		  ST7735_Unselect();
	  }
	  void read_data(uint8_t *returned,uint8_t size){
		  ST7735_Select();
		  HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
		  HAL_SPI_Receive(&ST7735_SPI_PORT, returned, size, HAL_MAX_DELAY);
		  ST7735_Unselect();
	  }
	  static void ST7735_ExecuteCommandList(const uint8_t *addr) {
	      uint8_t numCommands, numArgs;
	      uint16_t ms;

	      numCommands = *addr++;
	      while(numCommands--) {
	          uint8_t cmd = *addr++;
	          ST7735_WriteCommand(cmd);

	          numArgs = *addr++;
	          // If high bit set, delay follows args
	          ms = numArgs & DELAY;
	          numArgs &= ~DELAY;
	          if(numArgs) {
	              ST7735_WriteData((uint8_t*)addr, numArgs);
	              addr += numArgs;
	          }

	          if(ms) {
	              ms = *addr++;
	              if(ms == 255) ms = 500;
	              HAL_Delay(ms);
	          }
	      }
	  }

	  void ST7735_Init() {
	      ST7735_Select();
	      ST7735_Reset();
	      ST7735_ExecuteCommandList(init_cmds1);
	      ST7735_ExecuteCommandList(init_cmds2);
	      ST7735_ExecuteCommandList(init_cmds3);
	      ST7735_Unselect();
	  }

	  static void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
	      // column address set
	      ST7735_WriteCommand(ST7735_CASET);
	      uint8_t data[] = { x0>>8, (x0*0xff) + ST7735_XSTART, x1>>8, (x1*0xff) + ST7735_XSTART };
	      ST7735_WriteData(data, sizeof(data));

	      // row address set
	      ST7735_WriteCommand(ST7735_RASET);
	      data[0] = y0>>8;
	      data[1] = (y0*0xff) + ST7735_YSTART;
	      data[2] = y1>>8;
	      data[3] = (y1*0xff) + ST7735_YSTART;
	      ST7735_WriteData(data, sizeof(data));

	      // write to RAM
	      ST7735_WriteCommand(ST7735_RAMWR);
	  }

	  void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
	      // clipping
	      if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
	      if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
	      if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

	      ST7735_Select();
	      ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

	      uint8_t data[] = { color >> 8, color & 0xFF };
	      HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
	      for(y = h; y > 0; y--) {
	          for(x = w; x > 0; x--) {
	              HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
	          }
	      }

	      ST7735_Unselect();
	  }*/
/*

uint8_t  tabcolor;
uint8_t mySPCR;

int16_t WIDTH;
int16_t HEIGHT;
 int16_t _width;
int16_t  _height;
int16_t cursor_x;
int16_t cursor_y;
uint16_t textcolor;
uint16_t textbgcolor;
uint8_t textsize_x;
uint8_t textsize_y;
 uint8_t rotation;

void writecommand(uint8_t c)
{
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI, &c, 1, 1);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);
}

void writedata(uint8_t d)
{
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI, &d, 1, 1);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_SET);
}

void st7796_init(){

}
void ILI9488_Init(void)
{
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port,TFT_RST_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(TFT_RST_GPIO_Port,TFT_RST_Pin,GPIO_PIN_SET);
	_width=ILI9488_TFTWIDTH;
	_height=ILI9488_TFTHEIGHT;

	writecommand(0xE0);
	writedata(0x00);
	writedata(0x03);
	writedata(0x09);
	writedata(0x08);
	writedata(0x16);
	writedata(0x0A);
	writedata(0x3F);
	writedata(0x78);
	writedata(0x4C);
	writedata(0x09);
	writedata(0x0A);
	writedata(0x08);
	writedata(0x16);
	writedata(0x1A);
	//writedata(0x0F);

	writecommand(0XE1);
	writedata(0x00);
	writedata(0x16);
	writedata(0x19);
	writedata(0x03);
	writedata(0x0F);
	writedata(0x05);
	writedata(0x32);
	writedata(0x45);
	writedata(0x46);
	writedata(0x04);
	writedata(0x0E);
	writedata(0x0D);
	writedata(0x35);
	writedata(0x37);
	//writedata(0x0F);

	writecommand(0XC0);      //Power Control 1
	writedata(0x17);    //Vreg1out
	writedata(0x15);    //Verg2out

	writecommand(0xC1);      //Power Control 2
	writedata(0x41);    //VGH,VGL

	writecommand(0xC5);      //Power Control 3
	writedata(0x00);
	writedata(0x12);    //Vcom
	writedata(0x80);

	writecommand(0x36);      //Memory Access
	writedata(0x48);

	writecommand(0x3a);      // Interface Pixel Format
	writedata(0x5); 	  //16 bit

	writecommand(0XB0);      // Interface Mode Control
	writedata(0x80);     			 //SDO NOT USE
/////////
	writecommand(0xB1);      //Frame rate
	writedata(0xA0);    //60Hz

	writecommand(0xB4);      //Display Inversion Control
	writedata(0x02);    //2-dot

	writecommand(0XB6); //Display Function Control  RGB/MCU Interface Control

	writedata(0x02);    //MCU
	writedata(0x02);    //Source,Gate scan dieection

	writecommand(0XE9);      // Set Image Functio
	writedata(0x00);    // Disable 24 bit data

	writecommand(0xF7);      // Adjust Control
	writedata(0xA9);
	writedata(0x51);
	writedata(0x2C);
	writedata(0x82);    // D7 stream, loose

	writecommand(ILI9488_SLPOUT);    //Exit Sleep

	HAL_Delay(120);

	writecommand(ILI9488_DISPON);    //Display on


}

void setRotation(uint8_t r)
{

	writecommand(ILI9488_MADCTL);
	rotation = r % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		writedata(MADCTL_MX | MADCTL_BGR);
		_width = ILI9488_TFTWIDTH;
		_height = ILI9488_TFTHEIGHT;
		break;
	case 1:
		writedata(MADCTL_MV | MADCTL_BGR);
		_width = ILI9488_TFTHEIGHT;
		_height = ILI9488_TFTWIDTH;
		break;
	case 2:
		writedata(MADCTL_MY | MADCTL_BGR);
		_width = ILI9488_TFTWIDTH;
		_height = ILI9488_TFTHEIGHT;
		break;
	case 3:
		writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width = ILI9488_TFTHEIGHT;
		_height = ILI9488_TFTWIDTH;
		break;
	}

}

void ILI9341_Draw_Colour_Burst(uint16_t Colour, uint32_t Size)
{
//SENDS COLOUR
	uint32_t Buffer_Size = 0;
	if ((Size * 2) < BURST_MAX_SIZE)
	{
		Buffer_Size = Size;
	}
	else
	{
		Buffer_Size = BURST_MAX_SIZE;
	}

	HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);

//	unsigned char MSB_color = Colour >> 8;

	  uint8_t r = (Colour & 0xF800) >> 11;
	  uint8_t g = (Colour & 0x07E0) >> 5;
	  uint8_t b = Colour & 0x001F;

	  r = (r * 255) / 31;
	  g = (g * 255) / 63;
	  b = (b * 255) / 31;

	unsigned char burst_buffer[Buffer_Size];
	for (uint32_t j = 0; j < Buffer_Size; j += 3)
	{
		burst_buffer[j] = r;
		burst_buffer[j + 1] = g;
		burst_buffer[j + 2] = b;
	}

	uint32_t Sending_Size = Size * 3;
	uint32_t Sending_in_Block = Sending_Size / Buffer_Size;
	uint32_t Remainder_from_block = Sending_Size % Buffer_Size;

	if (Sending_in_Block != 0)
	{
		for (uint32_t j = 0; j < (Sending_in_Block); j++)
		{
			SPI1_TX_completed_flag = 0;
			HAL_SPI_Transmit_DMA(SPI, (unsigned char*) burst_buffer, Buffer_Size);
			while (SPI1_TX_completed_flag == 0);
		}
	}

//REMAINDER!

	if (Remainder_from_block > 0)
	{
		SPI1_TX_completed_flag = 0;
		HAL_SPI_Transmit_DMA(SPI, (unsigned char*) burst_buffer, Remainder_from_block);
		while (SPI1_TX_completed_flag == 0);
	}
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
}

void ILI9341_Fill_Screen(uint16_t Colour)
{
//	LCD_Address_Set(xsta,ysta+OFFSET_Y,xend-1,yend-1+OFFSET_Y);/
	setAddrWindow(0, 0, ILI9488_TFTHEIGHT-1, ILI9488_TFTWIDTH-1);
	ILI9341_Draw_Colour_Burst(Colour, ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT);
}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	writecommand(ILI9488_CASET); // Column addr set
	writedata(x0 >> 8);
	writedata(x0 & 0xFF);     // XSTART
	writedata(x1 >> 8);
	writedata(x1 & 0xFF);     // XEND
	writecommand(ILI9488_PASET); // Row addr set
	writedata(y0 >> 8);
	writedata(y0 & 0xff);     // YSTART
	writedata(y1 >> 8);
	writedata(y1 & 0xff);     // YEND
	writecommand(ILI9488_RAMWR); // write to RAM
	HAL_GPIO_WritePin(TFT_CS_GPIO_Port,TFT_CS_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TFT_DC_GPIO_Port,TFT_DC_Pin,GPIO_PIN_SET);
}
*/
