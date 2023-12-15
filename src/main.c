/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "img_fundo.h"
#include "img_logo.h"

#include <asf.h>
#include <string.h>
#include "arm_math.h"
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"

#define PULSO_PIO 		 PIOA
#define PULSO_PIO_ID 	 ID_PIOA
#define PULSO_PIO_IDX	 19
#define PULSO_PIO_IDX_MASK (1u << PULSO_PIO_IDX)


/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

#define LV_HOR_RES_MAX          (240)
#define LV_VER_RES_MAX          (320)

#define LV_FONT_DEFAULT &lv_font_montserrat_24

LV_FONT_DECLARE(dseg30);
LV_FONT_DECLARE(dseg50);

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_RTC_STACK_SIZE				   (1024*6/sizeof(portSTACK_TYPE))
#define TASK_RTC_STACK_PRIORITY			   (tskIDLE_PRIORITY)

#define TASK_SIMULATOR_STACK_SIZE 			(4096 / sizeof(portSTACK_TYPE))
#define TASK_SIMULATOR_STACK_PRIORITY 		(tskIDLE_PRIORITY)

xSemaphoreHandle xSemaphoreHorario;

xQueueHandle xQueuePulso;

SemaphoreHandle_t xMutex;

#define RTT_FREQ 100

#define RAIO 0.508/2
#define VEL_MAX_KMH  5.0f
#define VEL_MIN_KMH  0.5f
//#define RAMP 


/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

static lv_obj_t * screen;
volatile lv_obj_t * labelSetValue;
volatile lv_obj_t * labelVelocidade;
volatile lv_obj_t * labelKm;

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

float kmh_to_hz(float vel, float raio) {
    float f = vel / (2*PI*raio*3.6);
    return(f);
}

void pulso_callback(void) {
	uint32_t value = rtt_read_timer_value(RTT);
	RTT_init(RTT_FREQ, 0, 0);
	xQueueSendFromISR(xQueuePulso, &value, 0);
}

static void event_handler(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}

void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
	
    /* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
		// o código para irq de segundo vem aqui
		xSemaphoreGiveFromISR(xSemaphoreHorario, 0);
    }
	
    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
    	// o código para irq de alame vem aqui
    }

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
	}
}

void lv_ex_btn_1(void) {
	screen = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(screen, lv_color_hex(0x8ecfff), LV_PART_MAIN);
	
	
	lv_obj_t * img2 = lv_img_create(screen);
	lv_img_set_src(img2, &img_logo);
	lv_obj_align(img2, LV_ALIGN_TOP_LEFT, 7, 3);
	lv_img_set_angle(img2, 900);

	labelSetValue = lv_label_create(screen);
	lv_obj_align_to(labelSetValue, img2, LV_ALIGN_RIGHT_MID, 50, 0);
	lv_obj_set_style_text_font(labelSetValue, &dseg30, LV_STATE_DEFAULT);
	lv_obj_set_style_text_color(labelSetValue, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelSetValue, "%02d:%02d:%02d", 0, 0, 0);

	labelVelocidade = lv_label_create(screen);
	lv_obj_align(labelVelocidade, LV_ALIGN_CENTER, -30, -10);
	lv_obj_set_style_text_color(labelVelocidade, lv_color_black(), LV_STATE_DEFAULT);
	lv_obj_set_style_text_font(labelVelocidade, &dseg50, LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelVelocidade, "%02d", 0);

	labelKm = lv_label_create(screen);
	lv_obj_align_to(labelKm, labelVelocidade, LV_ALIGN_BOTTOM_RIGHT, 65, 0);
	lv_obj_set_style_text_color(labelKm, lv_color_black(), LV_STATE_DEFAULT);
	lv_label_set_text_fmt(labelKm, "km/h");
	
	lv_scr_load(screen);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
	int px, py;

	lv_ex_btn_1();

	for (;;)  {
		xSemaphoreTake(xMutex, portMAX_DELAY);
		lv_tick_inc(50);
		lv_task_handler();
		xSemaphoreGive(xMutex);
		vTaskDelay(50);
	}
}
static void task_rtc(void *pvParameters) {
	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);
	
	uint32_t current_hour, current_min, current_sec;

	uint32_t pulso;

	for (;;) {
		if (xSemaphoreTake(xSemaphoreHorario, 0) == pdTRUE) {
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);

			xSemaphoreTake(xMutex, portMAX_DELAY);

			lv_label_set_text_fmt(labelSetValue, "%02d:%02d:%02d", current_hour, current_min, current_sec);

			xSemaphoreGive(xMutex);
		}

		if (xQueueReceive(xQueuePulso, &pulso, 0) == pdTRUE) {
			double dt = ((double) pulso / RTT_FREQ);
			double f = ((double) 1.0 / dt);
			double w = ((double)2 * PI * f);
			int v = (int) (w * RAIO * 3.6);
			
			lv_label_set_text_fmt(labelVelocidade, "%02d", v);
		}
	}
}

static void task_simulador(void *pvParameters) {

	pmc_enable_periph_clk(ID_PIOC);
	pio_set_output(PIOC, PIO_PC31, 1, 0, 0);

	float vel = VEL_MAX_KMH;
	float f;
	int ramp_up = 1;

	while(1){
		pio_clear(PIOC, PIO_PC31);
		delay_ms(1);
		pio_set(PIOC, PIO_PC31);
		#ifdef RAMP
		if (ramp_up) {
			printf("[SIMU] ACELERANDO: %d \n", (int) (10*vel));
			vel += 0.5;
			} else {
			printf("[SIMU] DESACELERANDO: %d \n",  (int) (10*vel));
			vel -= 0.5;
		}

		if (vel >= VEL_MAX_KMH)
		ramp_up = 0;
		else if (vel <= VEL_MIN_KMH)
		ramp_up = 1;
		#else
		vel = 5;
		printf("[SIMU] CONSTANTE: %d \n", (int) (10*vel));
		#endif
		f = kmh_to_hz(vel, RAIO);
		int t = 965*(1.0/f); //UTILIZADO 965 como multiplicador ao inv�s de 1000
		//para compensar o atraso gerado pelo Escalonador do freeRTOS
		delay_ms(t);
	}
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);
	
	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);
	
	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);
	
	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);
	
	rtc_enable_interrupt(rtc, irq_type);

}

void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

void configure_pulso(void) {
	pmc_enable_periph_clk(PULSO_PIO_ID);
	pio_configure(PULSO_PIO, PIO_INPUT, PULSO_PIO_IDX_MASK, 0);
	pio_handler_set(PULSO_PIO, PULSO_PIO_ID, PULSO_PIO_IDX_MASK, PIO_IT_FALL_EDGE, pulso_callback);

	pio_enable_interrupt(PULSO_PIO, PULSO_PIO_IDX_MASK);
	pio_get_interrupt_status(PULSO_PIO);

	NVIC_EnableIRQ(PULSO_PIO_ID);
	NVIC_SetPriority(PULSO_PIO_ID, 5);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = py;
	data->point.y = 320 - px;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	sysclk_init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	ili9341_set_orientation(ILI9341_FLIP_Y | ILI9341_SWITCH_XY);
	configure_touch();
	configure_lvgl();
	configure_pulso();

	xSemaphoreHorario = xSemaphoreCreateBinary();
	if (xSemaphoreHorario == NULL) {
		printf("Failed to create semaphore \n");
	}

	xMutex = xSemaphoreCreateMutex();
	if (xMutex == NULL){
		printf("Failed to create mutex\n");
	}

	xQueuePulso = xQueueCreate(32, sizeof(uint32_t));
	if (xQueuePulso == NULL) {
		printf("Failed to create queue\n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	
	if (xTaskCreate(task_rtc, "RTC", TASK_RTC_STACK_SIZE, NULL, TASK_RTC_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create rtc task\r\n");
	}

    if (xTaskCreate(task_simulador, "SIMUL", TASK_SIMULATOR_STACK_SIZE, NULL, TASK_SIMULATOR_STACK_PRIORITY, NULL) != pdPASS) {
        printf("Failed to create simul task\r\n");
    }
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
