/*
 * ili9341.c
 *
 *  Created on: 11 Apr 2020
 *      Author: ammaree
 */

#include	"ili9341.h"
#include	"hal_spi.h"
#include	"hal_gpio.h"
#include	"FreeRTOS_Support.h"
#include	"fonts.h"

#include	"endpoints.h"

#include	"x_errors_events.h"
#include 	"printfx.h"
#include	"systiming.h"

#include	"esp_err.h"
#include	"esp_system.h"

#include	<string.h>

#define	debugFLAG					0xF000

#define	debugCMDS					(debugFLAG & 0x0001)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### BUILD : CONFIG definitions ###############################

#define	ili9341VERSION				"v0.0.0.1"

#define	ili9341FONT					FONT5X7
#define	ili9341FONT_HEIGHT			CONCAT2(ili9341FONT, _HEIGHT)
#define	ili9341FONT_WIDTH			CONCAT2(ili9341FONT, _WIDTH)

// ####################################### LCD definitions #########################################

#define	LCD_TYPE_320_240			1		// standard

#define	LCD_TYPE					LCD_TYPE_320_240

#if		(LCD_TYPE == LCD_TYPE_320_240)
	#define	LCD_WIDTH					320		// pixels horizontal
	#define	LCD_HEIGHT					240		// pixels vertical
#else
	#error "No/invalid LCD Type selected!!!"
#endif

#define	LCD_COLUMNS					(LCD_WIDTH / ili9341FONT_WIDTH)
#define	LCD_LINES					(LCD_HEIGHT / ili9341FONT_HEIGHT)
#define	LCD_SPARE_PIXELS			(LCD_WIDTH - (LCD_COLUMNS * ili9341FONT_WIDTH))

#define	ANOMALY_OFFSET				32		// XXX CHECK ????

// ############################################ Macros #############################################

// ######################################### Enumerations ##########################################

typedef enum { LCD_TYPE_ILI, LCD_TYPE_ST, LCD_TYPE_MAX } type_lcd_t;

// ########################################## Structures ###########################################

//The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
typedef struct {
	uint8_t cmd;
	uint8_t data[16];
	uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef struct ili9341_s {
	epid_t		epidSPI;
	uint16_t	segment;								// pixel horizontal
	uint16_t	max_seg;
	uint8_t 	page;
	uint8_t 	max_page;
	uint8_t 	mem_mode;
} ili9341_t;

// ################################ private static variables #######################################

static ili9341_t sILI9341 = { .max_seg = LCD_WIDTH, .max_page = LCD_HEIGHT / ili9341FONT_HEIGHT } ;

// ################################# Forward funtion declarations ##################################

void ili9341ToggleDClineCallback(spi_transaction_t *t) ;

// ###################################### Private variables ########################################

spi_device_interface_config_t ili9341_config = {
	.mode			= 0,
#if		(ili9341LCD_OVERCLOCK == 1)
	.clock_speed_hz	= 26 * 1000 * 1000,				// 26 MHz
#else
	.clock_speed_hz	= 10*1000*1000,           		// 10 MHz
#endif
	.spics_io_num	= ili9341GPIO_CS,				// CS pin
	.queue_size 	= 7,           					// queue 7 transactions at a time
	.pre_cb			= ili9341ToggleDClineCallback,	//Specify pre-transfer callback to handle D/C line
};

spi_device_handle_t ili9341handle = 0;
SemaphoreHandle_t ili9341mutex = 0;

//Place data into DRAM. Constant data gets placed into DROM by default, which is not accessible by DMA.

DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
{ 0x36, { (1 << 5) | (1 << 6) }, 1 },	/* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
{ 0x3A, { 0x55 }, 1 },/* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
{ 0xB2, { 0x0c, 0x0c, 0x00, 0x33, 0x33 }, 5 },	/* Porch Setting */
{ 0xB7, { 0x45 }, 1 },	/* Gate Control, Vgh=13.65V, Vgl=-10.43V */
{ 0xBB, { 0x2B }, 1 },	/* VCOM Setting, VCOM=1.175V */
{ 0xC0, { 0x2C }, 1 },	/* LCM Control, XOR: BGR, MX, MH */
{ 0xC2, { 0x01, 0xff }, 2 },	/* VDV and VRH Command Enable, enable=1 */
{ 0xC3, { 0x11 }, 1 },	/* VRH Set, Vap=4.4+... */
{ 0xC4, { 0x20 }, 1 },	/* VDV Set, VDV=0 */
{ 0xC6, { 0x0f }, 1 },	/* Frame Rate Control, 60Hz, inversion=0 */
{ 0xD0, { 0xA4, 0xA1 }, 1 },	/* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
{ 0xE0, { 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19 }, 14 },	/* Positive Voltage Gamma Control */
{ 0xE1, { 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19 }, 14 },	/* Negative Voltage Gamma Control */
{ 0x11, { 0 }, 0x80 },						/* Sleep Out */
{ 0x29, { 0 }, 0x80 }, { 0, { 0 }, 0xff }	/* Display On */
};

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] = {
{ 0xCF, { 0x00, 0x83, 0x30 }, 3 }, /* Power control B, power control = 0, DC_ENA = 1 */
{ 0xED, { 0x64, 0x03, 0x12, 0x81 }, 4 }, /* Power on sequence control, cp1 keeps 1 frame, 1st frame enable
 * vcl = 0, ddvdh=3, vgh=1, vgl=2, DDVDH_ENH=1 */
{ 0xE8, { 0x85, 0x01, 0x79 }, 3 }, /* Driver timing control A, non-overlap=default +1
 * EQ=default - 1, CR=default, pre-charge=default - 1 */
{ 0xCB, { 0x39, 0x2C, 0x00, 0x34, 0x02 }, 5 }, /* Power control A, Vcore=1.6V, DDVDH=5.6V */
{ 0xF7, { 0x20 }, 1 }, /* Pump ratio control, DDVDH=2xVCl */
{ 0xEA, { 0x00, 0x00 }, 2 }, /* Driver timing control, all=0 unit */
{ 0xC0, { 0x26 }, 1 }, /* Power control 1, GVDD=4.75V */
{ 0xC1, { 0x11 }, 1 }, /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
{ 0xC5, { 0x35, 0x3E }, 2 }, /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
{ 0xC7, { 0xBE }, 1 }, /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
{ 0x36, { 0x28 }, 1 }, /* Memory access control, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
{ 0x3A, { 0x55 }, 1 }, /* Pixel format, 16bits/pixel for RGB/MCU interface */
{ 0xB1, { 0x00, 0x1B }, 2 }, /* Frame rate control, f=fosc, 70Hz fps */
{ 0xF2, { 0x08 }, 1 }, /* Enable 3G, disabled */
{ 0x26, { 0x01 }, 1 }, /* Gamma set, curve 1 */
{ 0xE0, { 0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0x87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00 }, 15 }, /* + gamma correction */
{ 0XE1, { 0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F }, 15 }, /* - gamma correction */
{ 0x2A, { 0x00, 0x00, 0x00, 0xEF }, 4 }, /* Column address set, SC=0, EC=0xEF */
{ 0x2B, { 0x00, 0x00, 0x01, 0x3f }, 4 }, /* Page address set, SP=0, EP=0x013F */
{ 0x2C, { 0 }, 0 }, /* Memory write */
{ 0xB7, { 0x07 }, 1 }, /* Entry mode set, Low vol detect disabled, normal display */
{ 0xB6, { 0x0A, 0x82, 0x27, 0x00 }, 4 }, /* Display function control */
{ 0x11, { 0 }, 0x80 }, /* Sleep out */
{ 0x29, { 0 }, 0x80 }, /* Display on */
{ 0, { 0 }, 0xff }, };

//Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
//function is finished because the SPI driver needs access to it even while we're already calculating the next line.
#define	ili9341NUM_TRANS			6
spi_transaction_t trans[ili9341NUM_TRANS] = { 0 };

int32_t CurFrame = 0, SentBuf, CalcBuf = 0;
uint16_t *LinesBuf[2], CurCol = 0;

// ####################################### private functions #######################################

/* Send a command to the LCD, waits until the transfer is complete.
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete. */
void ili9341SendCommand(const uint8_t cmd) {
	spi_transaction_t t = { 0 };					// D/C needs to be set to 0
	t.length = BITS_IN_BYTE;
	t.tx_buffer = &cmd;							// The data is the cmd itself
	ESP_ERROR_CHECK(spi_device_polling_transmit(ili9341handle, &t)) ;
}

/* Send data to the LCD, waits until the transfer is complete.
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete. */
void ili9341SendData(const uint8_t * data, int len) {
	if (len) {
		spi_transaction_t t = { 0 };
		t.length = len * BITS_IN_BYTE;// Len is in bytes, transaction length is in bits.
		t.tx_buffer = data;								// Data
		t.user = (void*) 1;							// D/C needs to be set to 1
		ESP_ERROR_CHECK(spi_device_polling_transmit(ili9341handle, &t)) ;  //Transmit!
	}
}

int32_t ili9341GetID(void) {
	ili9341SendCommand(0x04) ;
	spi_transaction_t t = { 0 } ;
	t.length = BITS_IN_BYTE * 3 ;
	t.flags = SPI_TRANS_USE_RXDATA ;
	t.user = (void *) 1 ;
	ESP_ERROR_CHECK(spi_device_polling_transmit(ili9341handle, &t)) ;
	return *(int32_t *) t.rx_data ;
}

void ili9341SetColumnAddress(uint16_t CAstart, uint16_t CAend) {
	ili9341SendCommand(SET_COL_AD);
	uint8_t cBuf[4];
	cBuf[0] = CAstart >> 8;
	cBuf[1] = CAstart & 0xFF;
	cBuf[2] = CAend >> 8;
	cBuf[3] = CAend & 0xFF;
	ili9341SendData(cBuf, sizeof(cBuf));
}

void ili9341SetPageAddress(uint16_t PAstart, uint16_t PAend) {
	ili9341SendCommand(SET_PAG_AD);
	uint8_t cBuf[4];
	cBuf[0] = PAstart >> 8;
	cBuf[1] = PAstart & 0xFF;
	cBuf[2] = PAend >> 8;
	cBuf[3] = PAend & 0xFF;
	ili9341SendData(cBuf, sizeof(cBuf));
}

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and in
 * the mean while the lines for next transactions can get calculated.
 */
void ili9341SendMultiLines(int ypos, uint16_t *linedata) {
	myASSERT(trans[0].tx_data[0] == SET_COL_AD) ;
	myASSERT(trans[2].tx_data[0] == SET_PAG_AD) ;
	myASSERT(trans[4].tx_data[0] == WR_MEM) ;

	trans[3].tx_data[0] = ypos >> 8;        			// Start page high
	trans[3].tx_data[1] = ypos & 0xff;      			// start page low
	trans[3].tx_data[2] = (ypos + ili9341LINES_PARALLEL) >> 8;	//end page high
	trans[3].tx_data[3] = (ypos + ili9341LINES_PARALLEL) & 0xff;//end page low

	trans[5].tx_buffer = linedata;        		// finally send the line data

	for (int x = 0; x < ili9341NUM_TRANS; ++x) {	// Queue all transactions.
		ESP_ERROR_CHECK(spi_device_queue_trans(ili9341handle, &trans[x], portMAX_DELAY)) ;
	}
	//When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
	//mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
	//finish because we may as well spend the time calculating the next line. When that is done, we can call
	//send_line_finish, which will wait for the transfers to be done and check their status.
}

/*
 * Wait for all transactions to be done and get back the results.
 */
void ili9341CheckMultiLinesSend(void) {
	spi_transaction_t *rtrans;
	for (int x = 0; x < ili9341NUM_TRANS; ++x)
		ESP_ERROR_CHECK(spi_device_get_trans_result(ili9341handle, &rtrans, portMAX_DELAY)) ;
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void ili9341ToggleDClineCallback(spi_transaction_t *t) {
	int dc = (int) t->user ;
	gpio_set_level(ili9341GPIO_D_C_X, dc) ;
}

// ####################################### Public functions ########################################

int ili9341Init(void) {
	ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &ili9341_config, &ili9341handle)) ;	//Attach the LCD to the SPI bus

	//Initialize non-SPI GPIOs
	gpio_set_direction(ili9341GPIO_D_C_X, GPIO_MODE_OUTPUT);
	gpio_set_direction(ili9341GPIO_RESET, GPIO_MODE_OUTPUT);
	gpio_set_direction(ili9341GPIO_LIGHT, GPIO_MODE_OUTPUT);

	//Reset the display
	gpio_set_level(ili9341GPIO_RESET, 0) ;
	vTaskDelay(pdMS_TO_TICKS(100)) ;
	gpio_set_level(ili9341GPIO_RESET, 1) ;
	vTaskDelay(pdMS_TO_TICKS(100)) ;
	IF_SYSTIMER_INIT(debugTIMING, stILI9341a, stMICROS, "ili9341A", 1, 1000);
	IF_SYSTIMER_INIT(debugTIMING, stILI9341b, stMICROS, "ili9341B", 1, 1000);
	return erSUCCESS;
}

int ili9341DeInit(void) {
	gpio_reset_pin(ili9341GPIO_LIGHT);
	gpio_reset_pin(ili9341GPIO_RESET);
	gpio_reset_pin(ili9341GPIO_D_C_X);
	ESP_ERROR_CHECK(spi_bus_remove_device(ili9341handle));
	ESP_ERROR_CHECK(spi_bus_free(HSPI_HOST));
	IF_TRACK(debugTRACK, "DeInit ILI9341/ST7789V device");
	return erSUCCESS;
}

int ili9341Identify(void) { return ili9341GetID() ; }

int ili9341Config(int32_t DevType) {
	const lcd_init_cmd_t *lcd_init_cmds ;
	if (DevType == LCD_TYPE_ILI)
		lcd_init_cmds = ili_init_cmds ;
	else if (DevType == LCD_TYPE_ST)
		lcd_init_cmds = st_init_cmds ;
	else
		return erFAILURE ;
	IF_TRACK(debugTRACK, "Configured device type '%s'", DevType ? "ST7789V" : "ILI9341" ) ;
	int cmd = 0 ;
	while (lcd_init_cmds[cmd].databytes != 0xff) {		// Send all the commands
		ili9341SendCommand(lcd_init_cmds[cmd].cmd) ;
		ili9341SendData(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F) ;
		if (lcd_init_cmds[cmd].databytes & 0x80)
			vTaskDelay(pdMS_TO_TICKS(100)) ;
		++cmd ;
	}
	gpio_set_level(ili9341GPIO_LIGHT, 0);				// Enable backlight
	sILI9341.epidSPI.val = DEFN_EPID(devILI9341, subDSP320X240, URI_UNKNOWN, UNIT_PIXEL) ;
	return erSUCCESS ;
}

// ############################## Text Character & String write functions ##########################

/**
 * @brief Write a single character at the current cursor position
 *
 * Using vertical addressing mode, we can have the hardware
 * automatically move to the next column after writing 7 pixels
 * in each page (rows).
 */
int ili9341PutChar(int cChr) {
	if (sILI9341.epidSPI.devclass != devILI9341 || sILI9341.epidSPI.subclass != subDSP320X240)
		return cChr;

	const char * pFont = &font5X7[cChr * (ili9341FONT_WIDTH - 1)] ;
	IF_PRINT(debugCMDS, "%c : %02x-%02x-%02x-%02x-%02x\n", cChr, *pFont, *(pFont + 1), *(pFont + 2), *(pFont + 3), *(pFont + 4));
	IF_EXEC_1(debugTIMING, xSysTimerStart, stILI9341b) ;
	ili9341SendCommand(WR_MEM) ;
	uint8_t cBuf[ili9341FONT_WIDTH] ;
	int i ;
	for (i = 0; i < (ili9341FONT_WIDTH - 1); cBuf[i++] = *pFont++) ;
	cBuf[i] = 0x00 ;
	ili9341SendData(cBuf, sizeof(cBuf)) ;

	sILI9341.segment += ili9341FONT_WIDTH;
	if (sILI9341.segment >= (sILI9341.max_seg - LCD_SPARE_PIXELS)) {
		++sILI9341.page;								// update the cursor location
		if (sILI9341.page == sILI9341.max_page)
			sILI9341.page = 0 ;
//		ili9341SetPageAddr(sILI9341.page) ;
//		ili9341SetSegmentAddr(0) ;
	}
	IF_EXEC_1(debugTIMING, xSysTimerStop, stILI9341b) ;
	return cChr ;
}

void ili9341PutString(const char *pString) { while (*pString) ili9341PutChar(*pString++) ; }

void ili9341_set_brightness(uint8_t level) { }

// ################################## Diagnostic support functions #################################

void ili9341TestInit(void) {
	for (int i = 0; i < 2; ++i) {		// Allocate memory for the pixel buffers
		LinesBuf[i] = heap_caps_malloc(320 * ili9341LINES_PARALLEL * sizeof(uint16_t), MALLOC_CAP_DMA);
		assert(LinesBuf[i] != NULL);
		IF_PRINT(debugTRACK, "Allocating Buf #%d = %d bytes", i, 320 * ili9341LINES_PARALLEL * sizeof(uint16_t));
	}
	for (int x = 0; x < ili9341NUM_TRANS; ++x) {
		if ((x & 1) == 0) {            					// Even transfers are commands
			trans[x].length = BITS_IN_BYTE ;
		} else {		            					// Odd transfers are data
			trans[x].length = BITS_IN_BYTE * 4 ;
			trans[x].user = (void *) 1 ;
		}
		trans[x].flags = SPI_TRANS_USE_TXDATA;
	}
	trans[0].tx_data[0] = SET_COL_AD;    				// Column Address Set

	trans[1].tx_data[0] = 0;              				// Start Col High
	trans[1].tx_data[1] = 0;              				// Start Col Low
	trans[1].tx_data[2] = (320) >> 8;       			// End Col High
	trans[1].tx_data[3] = (320) & 0xff;     			// End Col Low

	trans[2].tx_data[0] = SET_PAG_AD;    				// Page address set
	trans[4].tx_data[0] = WR_MEM;       				// memory write
	trans[5].length		= 320 * 2 * BITS_IN_BYTE * ili9341LINES_PARALLEL; // Data length, in bits
	trans[5].flags 		= 0;							// undo SPI_TRANS_USE_TXDATA flag
	// ensure ili9341TestUpdate() start correctly
	SentBuf = -1 ;
}

void ili9341TestFillBuffer(uint16_t *dest, int line) {
	for (int y = line; y < (line + ili9341LINES_PARALLEL); ++y) {
		for (int x = 0; x < 320; ++x) *dest++ = CurCol;
	}
}

void ili9341TestUpdate(void) {
	CurCol += 0x0821 ;
	for (int y = 0; y < 240; y += ili9341LINES_PARALLEL) {
		ili9341TestFillBuffer(LinesBuf[CalcBuf], y) ;
		if (SentBuf != -1) ili9341CheckMultiLinesSend();// Finish sending previous lines
		IF_EXEC_1(debugTIMING, xSysTimerStart, stILI9341a) ;
		ili9341SendMultiLines(y, LinesBuf[CalcBuf]) ;	// Send the line buffer we just calculated.
		IF_EXEC_1(debugTIMING, xSysTimerStop, stILI9341a) ;

		SentBuf = CalcBuf ;								// save buffer just calc(+sent) as sent
		CalcBuf = CalcBuf ? 0 : 1 ;						// toggle index to select next buffer to calc
	}
	IF_PRINT(debugTRACK, "Frame=%d\r", CurFrame) ;
	++CurFrame;
}
