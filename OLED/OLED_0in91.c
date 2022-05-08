/*
 * OLED_0in91.c
 *
 *  Created on: 30 kwi 2022
 *      Author: kowma
 */
#include "main.h"
#include "OLED_0in91.h"

void OLED_0in91_WriteReg(uint8_t val)
{
	LL_I2C_HandleTransfer(I2C1, SSD1306_SLAVE_ADDRESS, LL_I2C_ADDRSLAVE_7BIT ,2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	 while(!LL_I2C_IsActiveFlag_STOP(I2C1))
	 {
		 if(LL_I2C_IsActiveFlag_TXE(I2C1))
		 {
			 LL_I2C_TransmitData8(I2C1, val);
		 }
	}
	//LL_I2C_GenerateStartCondition(I2C1);
}

void OLED_0in91_Init()
{
	OLED_0in91_WriteReg(SSD1306_DISPLAYOFF);
	OLED_0in96_WriteReg(SSD1306_SETDISPLAYCLOCKDIV);
	OLED_0in96_WriteReg(0x80);


	// set mux ratio
	OLED_0in96_WriteReg(SSD1306_SETMULTIPLEX);
	OLED_0in96_WriteReg(0x3F);
	// set display offset
	OLED_0in96_WriteReg(SSD1306_SETDISPLAYOFFSET);
	OLED_0in96_WriteReg(0x0);
	// set display start line
	OLED_0in96_WriteReg(SSD1306_SETSTARTLINE);
	// set segment re-map
	OLED_0in96_WriteReg(SSD1306_SEGREMAP | 0x1);
	// Set COM Output Scan Direction C0h
	OLED_0in96_WriteReg(SSD1306_COMSCANDEC);
	// set COM PINS hardware configuration
	OLED_0in96_WriteReg(SSD1306_SETCOMPINS);
	OLED_0in96_WriteReg(0x12);           // TODO - calculate based on _rawHieght ?
	//set contrast control
	OLED_0in96_WriteReg(SSD1306_SETCONTRAST);
	OLED_0in96_WriteReg(0x7F);
	//set entrie display on
	OLED_0in96_WriteReg(SSD1306_DISPLAYALLON_RESUME);
	// set normal dispay
	OLED_0in96_WriteReg(SSD1306_NORMALDISPLAY);
	// set osc freq
	OLED_0in96_WriteReg(SSD1306_SETDISPLAYCLOCKDIV);
	OLED_0in96_WriteReg(0x80);
	// enable charge pump
	OLED_0in96_WriteReg(SSD1306_CHARGEPUMP);
	OLED_0in96_WriteReg(0x14);
	// Display ON
	OLED_0in96_WriteReg(SSD1306_DISPLAYON);



}


