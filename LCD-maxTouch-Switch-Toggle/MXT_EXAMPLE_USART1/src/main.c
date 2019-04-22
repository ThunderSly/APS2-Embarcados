/**
 * \file
 *
 * \brief Example of usage of the maXTouch component with USART
 *
 * This example shows how to receive touch data from a maXTouch device
 * using the maXTouch component, and display them in a terminal window by using
 * the USART driver.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 *
 * \section intro Introduction
 * This simple example reads data from the maXTouch device and sends it over
 * USART as ASCII formatted text.
 *
 * \section files Main files:
 * - example_usart.c: maXTouch component USART example file
 * - conf_mxt.h: configuration of the maXTouch component
 * - conf_board.h: configuration of board
 * - conf_clock.h: configuration of system clock
 * - conf_example.h: configuration of example
 * - conf_sleepmgr.h: configuration of sleep manager
 * - conf_twim.h: configuration of TWI driver
 * - conf_usart_serial.h: configuration of USART driver
 *
 * \section apiinfo maXTouch low level component API
 * The maXTouch component API can be found \ref mxt_group "here".
 *
 * \section deviceinfo Device Info
 * All UC3 and Xmega devices with a TWI module can be used with this component
 *
 * \section exampledescription Description of the example
 * This example will read data from the connected maXTouch explained board
 * over TWI. This data is then processed and sent over a USART data line
 * to the board controller. The board controller will create a USB CDC class
 * object on the host computer and repeat the incoming USART data from the
 * main controller to the host. On the host this object should appear as a
 * serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
 *
 * Connect a terminal application to the serial port object with the settings
 * Baud: 57600
 * Data bits: 8-bit
 * Stop bits: 1 bit
 * Parity: None
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "conf_board.h"
#include "conf_example.h"

 typedef struct {
	 const uint8_t *data;
	 uint16_t width;
	 uint16_t height;
	 uint8_t dataSize;
 } tImage;

#include "tfont.h"
#include "calibri_36.h"
#include "conf_uart_serial.h"

#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define YEAR        2019
#define MONTH		4
#define DAY         8
#define WEEK        15
#define HOUR        16
#define MINUTE      18
#define SECOND      0

#define BUT_PIO					PIOA
#define BUT_PIO_ID				ID_PIOA
#define BUT_PIO_IDX				11
#define BUT_PIO_IDX_MASK		(1 << BUT_PIO_IDX)
#define BUT_DEBOUNCING_VALUE    79

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 150;
const uint32_t BUTTON_BORDER = 2;
const uint32_t BUTTON_X = ILI9488_LCD_WIDTH/2;
const uint32_t BUTTON_Y = ILI9488_LCD_HEIGHT/2;

volatile uint8_t flag_rtc_ala = 0;
volatile uint8_t flag_rtc_seg = 0;
volatile uint8_t flag_tc = 0;
volatile uint8_t flag_lock = 0;
volatile uint8_t flag_door = 0;
volatile uint8_t flag_play = 0;
volatile uint8_t flag_next = 0;
volatile uint8_t flag_prev = 0;
volatile uint8_t flag_playing = 0;
volatile uint8_t flag_pause = 0;
volatile uint8_t flag_paused = 0;
volatile uint8_t flag_resumed = 0;
volatile uint8_t lock_counter = 0;
volatile uint8_t flag_tela = 0;
volatile uint8_t flag_custom = 0;
volatile uint8_t flag_swap = 0;
volatile uint8_t flag_alter = 0;

volatile uint32_t hour;
volatile uint32_t minute;
volatile uint32_t second;
volatile uint32_t hour_pause;
volatile uint32_t minute_pause;
volatile uint32_t second_pause;
volatile uint32_t hour_start;
volatile uint32_t minute_start;
volatile uint32_t second_start;

volatile uint32_t total_time;
volatile uint32_t seconds_left = 59;
volatile uint32_t time_left;

const int enxague_tempo[4] = {0, 1, 5, 10};
const int enxague_quant[4] = {0, 1, 2, 3};
const int centrifugacao_RPM[4] = {0, 800, 1000, 1200};
const int centrifugacao_tempo[4] = {0, 1, 5, 10};

volatile uint8_t enxague_tempo_counter = 0;
volatile uint8_t enxague_quant_counter = 0;
volatile uint8_t centrifugacao_RPM_counter = 0;
volatile uint8_t centrifugacao_tempo_counter = 0;


 
#include "icones/arrow_left.h"
#include "icones/arrow_right.h"
#include "icones/locked.h"
#include "icones/unlocked.h"
#include "icones/closed_door.h"
#include "icones/opened_door.h"
#include "icones/play_button.h"
#include "icones/pause_button.h"
#include "icones/pesado.h"
#include "icones/rapido.h"
#include "icones/diario.h"
#include "icones/enxague.h"
#include "icones/centrifuga.h"
#include "icones/cancel.h"
#include "icones/custom.h"
#include "icones/config.h"
#include "icones/toggle.h"
#include "icones/confirm.h"

#include "icones/drop0.h"
#include "icones/drop1.h"
#include "icones/drop2.h"
#include "icones/drop3.h"
#include "icones/drop4.h"
#include "icones/drop5.h"
#include "icones/drop6.h"
#include "icones/drop7.h"

#include "maquina1.h"
	
static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);
	

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
}

t_ciclo *initMenuOrder(){
	c_rapido.previous = &c_custom;
	c_rapido.next = &c_diario;

	c_diario.previous = &c_rapido;
	c_diario.next = &c_pesado;

	c_pesado.previous = &c_diario;
	c_pesado.next = &c_enxague;

	c_enxague.previous = &c_pesado;
	c_enxague.next = &c_centrifuga;

	c_centrifuga.previous = &c_enxague;
	c_centrifuga.next = &c_custom;
	
	c_custom.previous = &c_centrifuga;
	c_custom.next = &c_rapido;

	return(&c_diario);
}

void Lock_Handler(void){
	flag_lock = !flag_lock;
	if(flag_lock){
		ili9488_draw_pixmap(32, 406, locked.width, locked.height, locked.data);
	}
	else {
		ili9488_draw_pixmap(32, 406, unlocked.width, unlocked.height, unlocked.data);
	}
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc = 1;
}

void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	lock_counter +=1;
	if (flag_lock){
		if (lock_counter >= 2){
			Lock_Handler();
		}
	}
}

void RTC_Handler(void){
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		flag_rtc_seg = 1;
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		flag_rtc_ala = 1;		
	}
	
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}

void Config_Handler(void){
	flag_tela = !flag_tela;
	flag_swap = 1;
}

void Custom_Handler(int attribute){
	if (attribute == 0){
		enxague_tempo_counter ++;
		if (enxague_tempo_counter > 3){
			enxague_tempo_counter = 0;
		}
		c_custom.enxagueTempo = enxague_tempo[enxague_tempo_counter];
	}
	if (attribute == 1){
		enxague_quant_counter++;
		if (enxague_quant_counter > 3){
			enxague_quant_counter = 0;
		}
		c_custom.enxagueQnt = enxague_quant[enxague_quant_counter];
	}
	if (attribute == 2){
		centrifugacao_RPM_counter ++;
		if (centrifugacao_RPM_counter > 3){
			centrifugacao_RPM_counter = 0;
		}
		c_custom.centrifugacaoRPM = centrifugacao_RPM[centrifugacao_RPM_counter];
	}
	if (attribute == 3){
		centrifugacao_tempo_counter ++;
		if (centrifugacao_tempo_counter > 3){
			centrifugacao_tempo_counter = 0;
		}
		c_custom.centrifugacaoTempo = centrifugacao_tempo[centrifugacao_tempo_counter];
	}
	if (attribute == 4){
		c_custom.heavy = !c_custom.heavy;
	}
	if (attribute == 5){
		c_custom.bubblesOn = !c_custom.bubblesOn;
	}
	flag_alter = 1;
}

void Door_Handler(void){
	if (flag_playing){
		 //PRINTA ALGUMA COISA
		return;
	}
	else{
		flag_door = !flag_door;
		if(flag_door){
			ili9488_draw_pixmap(224, 406, closed_door.width, closed_door.height, closed_door.data);
		}
		else {
			ili9488_draw_pixmap(224, 406, opened_door.width, opened_door.height, opened_door.data);
		}
	}
	
}

void Play_Handler(void){
	if(flag_playing){
		flag_pause = 1;
		flag_paused = 1;
		flag_playing = 0;
	}
	else if (flag_paused){
		flag_paused = 0;
		flag_playing = 1;
		flag_resumed = 1;
	}
	else{
		flag_play = 1;
		flag_playing = 1;
		
	}
}

void Next_Handler(void){
	flag_next = 1;
}

void Prev_Handler(void){
	flag_prev = 1;
}

void Cancel_Handler(void){
	flag_rtc_ala = 1;
}

void RTC_init(void){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MONTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
// 	rtc_enable_interrupt(RTC,  RTC_IER_SECEN);
// 	rtc_enable_interrupt(RTC, RTC_IER_ALREN);

}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	//tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void BUT_init(void){
	
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	pio_enable_interrupt(BUT_PIO, BUT_PIO_IDX_MASK);
	
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_IDX_MASK, PIO_IT_FALL_EDGE, Door_Handler);
	
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
};

static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
			MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	 * the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
			MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	 * value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
			MXT_GEN_COMMANDPROCESSOR_T6, 0)
			+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}

void font_draw_text(tFont *font, const char *text, int x, int y, int spacing) {
	char *p = text;
	while(*p != NULL) {
		char letter = *p;
		int letter_offset = letter - font->start_char;
		if(letter <= font->end_char) {
			tChar *current_char = font->chars + letter_offset;
			ili9488_draw_pixmap(x, y, current_char->image->width, current_char->image->height, current_char->image->data);
			x += current_char->image->width + spacing;
		}
		p++;
	}
}

void draw_screen(void) {
	ili9488_draw_pixmap(128, 10, diario.width, diario.height, diario.data);
	ili9488_draw_pixmap(32, 10, arrow_left.width, arrow_left.height, arrow_left.data);
	ili9488_draw_pixmap(224, 10, arrow_right.width, arrow_right.height, arrow_right.data);
	ili9488_draw_pixmap(32, 406, unlocked.width, unlocked.height, unlocked.data);
	ili9488_draw_pixmap(224, 406, closed_door.width, closed_door.height, closed_door.data);
	ili9488_draw_pixmap(96, 176, play_button.width, play_button.height, play_button.data);
	char modo[32];
	sprintf(modo,"Modo:");
	font_draw_text(&calibri_36, modo, 10, 84, 1);
}

uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}

uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}

void update_screen(uint32_t tx, uint32_t ty) {
	if (flag_tela == 0){
		lock_counter = 0;
		if (!flag_lock){
			if(flag_door){
				if(tx >= 96 && tx <= 96+128){
					if(ty >= 176 && ty <= 176+128){
						Play_Handler();
					}
				}
			}
			if(!flag_playing && !flag_paused){
				if(tx >= 32 && tx <= 32+64){
					if(ty >= 10 && ty <= 10+64){
						Prev_Handler();
					}
				}
			}
			if(!flag_playing && !flag_paused){
				if(tx >= 224 && tx <= 224+64){
					if(ty >= 10 && ty <= 10+64){
						Next_Handler();
					}
				}
			}
			if(tx >= 32 && tx <= 32+64){
				if(ty >= 406 && ty <= 406+64){
					Lock_Handler();
				}
			}
			if(flag_playing || flag_paused){
				if(tx >= 32 && tx <= 32+32){
					if(ty >= 224 && ty <= 224+32){
						Cancel_Handler();
					}
				}
			}
			if(!flag_playing && !flag_paused){
				if(flag_custom){
					if(tx >= 278 && tx <= 278+32){
						if(ty >= 84 && ty <= 84+32){
							Config_Handler();
						}
					}
				}
			}
		}
		else{
			if(tx >= 32 && tx <= 32+64){
				if(ty >= 406 && ty <= 406+64){
					tc_enable_interrupt(TC0, 1, TC_IER_CPCS);
				}
			}
		}
	}
	if (flag_tela == 1){
		if(!flag_lock){
			if(tx >= 128 && tx <= 128+64){
				if(ty >= 342 && ty <= 342+64){
					Config_Handler();
				}
			}
			//BOTAO TOGGLE enxagueTempo, so if tela == 2
			if(tx >= 278 && tx <= 278+32){
				if(ty >= 104 && ty <= 104+32){
					Custom_Handler(0);
				}
			}

			//BOTAO TOGGLE enxagueQnt, so if tela == 2
			if(tx >= 278 && tx <= 278+32){
				if(ty >= 144 && ty <= 144+32){
					Custom_Handler(1);
				}
			}

			//BOTAO TOGGLE centrifugacaoRPM, so if tela == 2
			if(tx >= 278 && tx <= 278+32){
				if(ty >= 184 && ty <= 184+32){
					Custom_Handler(2);
				}
			}

			//BOTAO TOGGLE centrifugacaoTempo, so if tela == 2
			if(tx >= 278 && tx <= 278+32){
				if(ty >= 224 && ty <= 224+32){
					Custom_Handler(3);
				}
			}

			//BOTAO TOGGLE heavy, so if tela == 2
			if(tx >= 278 && tx <= 278+32){
				if(ty >= 264 && ty <= 264+32){
					Custom_Handler(4);
				}
			}

			//BOTAO TOGGLE bubblesOn, so if tela == 2
			if(tx >= 278 && tx <= 278+32){
				if(ty >= 304 && ty <= 304+32){
					Custom_Handler(5);
				}
			}
			if(tx >= 32 && tx <= 32+64){
				if(ty >= 406 && ty <= 406+64){
					Lock_Handler();
				}
			}
		}
		else{
			if(tx >= 32 && tx <= 32+64){
				if(ty >= 406 && ty <= 406+64){
					tc_enable_interrupt(TC0, 1, TC_IER_CPCS);
				}
			}
		}
	}
}

void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	 * maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
	
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		 // eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.y);
		uint32_t conv_y = convert_axis_system_y(touch_event.x);
		
		/* Format a new entry in the data string that will be sent over USART */
		sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status:0x%2x conv X:%3d Y:%3d\n\r",
				touch_event.id, touch_event.x, touch_event.y,
				touch_event.status, conv_x, conv_y);
				
		if(conv_x >= 32 && conv_x <= 32+64){
			if(conv_y >= 406 && conv_y <= 406+64){
				if (touch_event.status == 32){
					tc_disable_interrupt(TC0, 1, TC_IER_CPCS);
				}
			}
		}
		if(touch_event.status == 192){
			update_screen(conv_x, conv_y);
		}
		
		
		

		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		 * if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}

int main(void)
{
	t_ciclo *p_ciclo = initMenuOrder();
	struct mxt_device device;
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	
	configure_lcd();

	
	RTC_init();
	TC_init(TC0,ID_TC0, 0, 15);
	TC_init(TC0,ID_TC1, 1, 1);
	BUT_init();
	mxt_init(&device);
	
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);
	rtc_disable_interrupt(RTC, RTC_IER_SECEN);
	rtc_disable_interrupt(RTC, RTC_IER_ALREN);
	draw_screen();
	char nome[32];
	sprintf(nome,"%s",p_ciclo->nome);
	font_draw_text(&calibri_36, nome, 116, 84, 1);
	
	ili9488_draw_filled_rectangle(0, 314, 316, 354);
	total_time = (p_ciclo->centrifugacaoTempo * (!!p_ciclo->centrifugacaoRPM) + p_ciclo->enxagueTempo * p_ciclo->enxagueQnt);
	if (p_ciclo->heavy){
		total_time *=1.2;
		total_time = (int) total_time;
	}
	time_left = total_time;
	char b3[32];
	sprintf(b3,"%02d : %02d",total_time, 0);
	font_draw_text(&calibri_36, b3, 106, 334, 1);
	const tImage* anim_list[8] = {&drop0, &drop1, &drop2, &drop3, &drop4, &drop5, &drop6, &drop7};
	volatile int icon_counter = 0;
	
		
	while (true) {
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
		if(flag_tela == 0){
			if (flag_tc){
				ili9488_draw_pixmap(256, 224, anim_list[icon_counter]->width, anim_list[icon_counter]->height, anim_list[icon_counter]->data);
				icon_counter++;
				if(icon_counter > 7){
					icon_counter = 0;
				}
				flag_tc = 0;
			}
		}
		if (!flag_lock){
			if (flag_next){
				p_ciclo = p_ciclo->next;
				ili9488_draw_filled_rectangle(116, 84, 316, 124);
				char nome[32];
				sprintf(nome,"%s",p_ciclo->nome);
				font_draw_text(&calibri_36, nome, 116, 84, 1);
				ili9488_draw_pixmap(128, 10, p_ciclo->icon->width, p_ciclo->icon->height, p_ciclo->icon->data);
				if(p_ciclo == &c_custom){
					flag_custom = 1;
					ili9488_draw_pixmap(278, 84, config.width, config.height, config.data);
				}
				else{
					flag_custom = 0;
					ili9488_draw_filled_rectangle(278, 84, 278 + 64, 84 + 64);
				}
				total_time = (p_ciclo->centrifugacaoTempo * (!!p_ciclo->centrifugacaoRPM) + p_ciclo->enxagueTempo * p_ciclo->enxagueQnt);
				if (p_ciclo->heavy){
					total_time *=1.2;
					total_time = (int) total_time;
				}
				time_left = total_time;
				char b3[32];
				sprintf(b3,"%02d : %02d",total_time, 0);
				font_draw_text(&calibri_36, b3, 106, 334, 1);
				flag_next = 0;
			}
			if (flag_prev){
				p_ciclo = p_ciclo->previous;
				ili9488_draw_filled_rectangle(116, 84, 316, 124);
				char nome[32];
				sprintf(nome,"%s",p_ciclo->nome);
				font_draw_text(&calibri_36, nome, 116, 84, 1);
				ili9488_draw_pixmap(128, 10, p_ciclo->icon->width, p_ciclo->icon->height, p_ciclo->icon->data);
				if(p_ciclo == &c_custom){
					flag_custom = 1;
					ili9488_draw_pixmap(278, 84, config.width, config.height, config.data);
				}
				else{
					flag_custom = 0;
					ili9488_draw_filled_rectangle(278, 84, 278 + 32, 84 + 32);
				}
				total_time = (p_ciclo->centrifugacaoTempo * (!!p_ciclo->centrifugacaoRPM) + p_ciclo->enxagueTempo * p_ciclo->enxagueQnt);
				if (p_ciclo->heavy){
					total_time *=1.2;
					total_time = (int) total_time;
				}
				time_left = total_time;
				char b3[32];
				sprintf(b3,"%02d : %02d",total_time, 0);
				font_draw_text(&calibri_36, b3, 106, 334, 1);
				flag_prev = 0;
			}
			if (flag_door){
				if (flag_play){
					if (time_left > 0){
						rtc_enable_interrupt(RTC,  RTC_IER_SECEN);
						rtc_enable_interrupt(RTC, RTC_IER_ALREN);
						rtc_set_time(RTC, HOUR, MINUTE, SECOND);
						rtc_get_time(RTC, &hour_start, &minute_start, &second_start);
						rtc_get_time(RTC,&hour,&minute,&second);
						rtc_set_time_alarm(RTC, 1, hour, 1, minute + total_time, 1, second);
						tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
						time_left -= 1;
						flag_play = 0;
						ili9488_draw_pixmap(96, 176, pause_button.width, pause_button.height, pause_button.data);
						ili9488_draw_pixmap(256, 224, anim_list[icon_counter]->width, anim_list[icon_counter]->height, anim_list[icon_counter]->data);
						ili9488_draw_pixmap(32, 224, cancel.width, cancel.height, cancel.data);
						icon_counter++;
					}
					else{
						flag_play = 0;
						flag_rtc_ala = 1;
					}
				}
			}
			if (flag_pause){
				rtc_get_time(RTC, &hour_pause, &minute_pause, &second_pause);
				rtc_disable_interrupt(RTC, RTC_IER_SECEN);
				rtc_disable_interrupt(RTC, RTC_IER_ALREN);
				time_left = time_left - (minute_pause - minute_start);
				seconds_left = seconds_left - (second_pause - second_start);
				tc_disable_interrupt(TC0, 0, TC_IER_CPCS);
				flag_pause = 0;
				ili9488_draw_pixmap(96, 176, play_button.width, play_button.height, play_button.data);
			}
			if (flag_resumed){
				rtc_enable_interrupt(RTC,  RTC_IER_SECEN);
				rtc_enable_interrupt(RTC, RTC_IER_ALREN);
				tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
				rtc_get_time(RTC, &hour_start, &minute_start, &second_start);
				rtc_set_time(RTC,hour_start, minute_start, 59-seconds_left);
				second_start = 59 - seconds_left;
				rtc_get_time(RTC,&hour,&minute,&second);
				rtc_set_time_alarm(RTC, 1, hour, 1, minute + time_left, 1, second + seconds_left);
				flag_resumed = 0;
				ili9488_draw_pixmap(96, 176, pause_button.width, pause_button.height, pause_button.data);
			}

			
			if (flag_swap){
				if (flag_tela){
					ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, 406);
					char e_tempo[32];
					sprintf(e_tempo,"%d",p_ciclo->enxagueTempo);
					font_draw_text(&calibri_36, e_tempo, 116, 104, 1);
					// enxagueTempo TOGGLE (IMAGEM)
					ili9488_draw_pixmap(278, 104, toggle.width, toggle.height, toggle.data);

					// enxagueQnt TEXTO
					char e_qnt[32];
					sprintf(e_qnt,"%d",p_ciclo->enxagueQnt);
					font_draw_text(&calibri_36, e_qnt, 116, 144, 1);
					// enxagueQnt TOGGLE (IMAGEM)
					ili9488_draw_pixmap(278, 144, toggle.width, toggle.height, toggle.data);

					// centrifugacaoRPM TEXTO
					char c_rpm[32];
					sprintf(c_rpm,"%d",p_ciclo->centrifugacaoRPM);
					font_draw_text(&calibri_36, c_rpm, 116, 184, 1);
					// centrifugacaoRPM TOGGLE (IMAGEM)
					ili9488_draw_pixmap(278, 184, toggle.width, toggle.height, toggle.data);

					// centrifugacaoTempo TEXTO
					char c_tempo[321];
					sprintf(c_tempo,"%d",p_ciclo->centrifugacaoTempo);
					font_draw_text(&calibri_36, c_tempo, 116, 224, 1);
					// centrifugacaoTempo TOGGLE (IMAGEM)
					ili9488_draw_pixmap(278, 224, toggle.width, toggle.height, toggle.data);

					// heavy TEXTO
					char heavy[32];
					sprintf(heavy,"%d",p_ciclo->heavy);
					font_draw_text(&calibri_36, heavy, 116, 264, 1);
					// heavy TOGGLE (IMAGEM)
					ili9488_draw_pixmap(278, 264, toggle.width, toggle.height, toggle.data);

					// bubblesOn TEXTO
					char bubbles[32];
					sprintf(bubbles,"%d",p_ciclo->bubblesOn);
					font_draw_text(&calibri_36, bubbles, 116, 304, 1);
					// bubblesOn TOGGLE (IMAGEM)
					ili9488_draw_pixmap(278, 304, toggle.width, toggle.height, toggle.data);
					ili9488_draw_pixmap(128, 342, confirm.width, confirm.height, confirm.data);
				}
				else{
					ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
					ili9488_draw_pixmap(32, 10, arrow_left.width, arrow_left.height, arrow_left.data);
					ili9488_draw_pixmap(224, 10, arrow_right.width, arrow_right.height, arrow_right.data);
					ili9488_draw_pixmap(128, 10, p_ciclo->icon->width, p_ciclo->icon->height, p_ciclo->icon->data);
					if(!flag_lock){
						ili9488_draw_pixmap(32, 406, unlocked.width, unlocked.height, unlocked.data);
					}
					else{
						ili9488_draw_pixmap(32, 406, locked.width, locked.height, locked.data);
					}
					if(flag_door){
						ili9488_draw_pixmap(224, 406, closed_door.width, closed_door.height, closed_door.data);
					}
					else{
						ili9488_draw_pixmap(32, 406, opened_door.width, opened_door.height, opened_door.data);
					}
					ili9488_draw_pixmap(96, 176, play_button.width, play_button.height, play_button.data);
					char modo[32];
					sprintf(modo,"Modo:");
					font_draw_text(&calibri_36, modo, 10, 84, 1);
					char nome[32];
					sprintf(nome,"%s",p_ciclo->nome);
					font_draw_text(&calibri_36, nome, 116, 84, 1);
					total_time = (p_ciclo->centrifugacaoTempo * (!!p_ciclo->centrifugacaoRPM) + p_ciclo->enxagueTempo * p_ciclo->enxagueQnt);
					if (p_ciclo->heavy){
						total_time *=1.2;
						total_time = (int) total_time;
					}
					time_left = total_time;
					char b3[32];
					sprintf(b3,"%02d : %02d",total_time, 0);
					font_draw_text(&calibri_36, b3, 106, 334, 1);
					ili9488_draw_pixmap(278, 84, config.width, config.height, config.data);
				}
				flag_swap = 0;
			}
			if (flag_alter){
					ili9488_draw_filled_rectangle(116, 104, 250 , 336);
					char e_tempo[32];
					sprintf(e_tempo,"%d",p_ciclo->enxagueTempo);
					font_draw_text(&calibri_36, e_tempo, 116, 104, 1);

					char e_qnt[32];
					sprintf(e_qnt,"%d",p_ciclo->enxagueQnt);
					font_draw_text(&calibri_36, e_qnt, 116, 144, 1);

					char c_rpm[32];
					sprintf(c_rpm,"%d",p_ciclo->centrifugacaoRPM);
					font_draw_text(&calibri_36, c_rpm, 116, 184, 1);

					char c_tempo[321];
					sprintf(c_tempo,"%d",p_ciclo->centrifugacaoTempo);
					font_draw_text(&calibri_36, c_tempo, 116, 224, 1);

					char heavy[32];
					sprintf(heavy,"%d",p_ciclo->heavy);
					font_draw_text(&calibri_36, heavy, 116, 264, 1);

					char bubbles[32];
					sprintf(bubbles,"%d",p_ciclo->bubblesOn);
					font_draw_text(&calibri_36, bubbles, 116, 304, 1);
				flag_alter = 0;
			}
			

		}
		if (flag_rtc_seg){
			rtc_get_time(RTC,&hour,&minute,&second);
			char b3[32];
			sprintf(b3,"%02d : %02d",time_left - (minute - minute_start), seconds_left - (second - second_start));
			font_draw_text(&calibri_36, b3, 106, 334, 1);
			flag_rtc_seg = 0;
		}
		if (flag_rtc_ala){
			rtc_disable_interrupt(RTC, RTC_IER_SECEN);
			rtc_disable_interrupt(RTC, RTC_IER_ALREN);
			tc_disable_interrupt(TC0, 0, TC_IER_CPCS);
			flag_playing = 0;
			flag_paused = 0;
			flag_rtc_ala = 0;
			ili9488_draw_pixmap(96, 176, play_button.width, play_button.height, play_button.data);
			ili9488_draw_filled_rectangle(0, 334, 316, 384);
			total_time = (p_ciclo->centrifugacaoTempo * (!!p_ciclo->centrifugacaoRPM) + p_ciclo->enxagueTempo * p_ciclo->enxagueQnt);
			if (p_ciclo->heavy){
				total_time *=1.2;
				total_time = (int) total_time;
			}
			time_left = total_time;
			seconds_left = 59;
			char b3[32];
			sprintf(b3,"%02d : %02d",total_time, 0);
			font_draw_text(&calibri_36, b3, 106, 334, 1);
			ili9488_draw_filled_rectangle(256, 224, 256+32, 224+32);
			ili9488_draw_filled_rectangle(32, 224, 64+32, 224+32);
		}
				
		
	}

	return 0;
}

//MODO CONFIG