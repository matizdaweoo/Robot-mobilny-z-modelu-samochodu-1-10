/**
 *  Defines for your entire project at one place
 * 
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

/* Put your global defines for all libraries here used in your project */

/* Change custom SPI for LCD */
#define ILI9341_SPI                 SPI3 //OR something similar
#define ILI9341_SPI_PINS            TM_SPI_PinsPack_2

/* Change custom CS, DC and RESET pins */
/* CS pin */
#define ILI9341_CS_PORT             GPIOD
#define ILI9341_CS_PIN              GPIO_PIN_0
/* WRX or DC pin */
#define ILI9341_WRX_PORT            GPIOD
#define ILI9341_WRX_PIN             GPIO_PIN_2
/* RST pin */
#define ILI9341_RST_PORT            GPIOD
#define ILI9341_RST_PIN             GPIO_PIN_4


#endif
