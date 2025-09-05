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


