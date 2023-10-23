// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>


#include "system.h"
#include "io.h"

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "FreeRTOS/timers.h"

#include <altera_avalon_pio_regs.h>

// Keyboard includes
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"

// VGA includes
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

// LCD defines
#define ESC 27
#define CLEAR_LCD_STRING "[2J"

// Definition of Task Stacks
#define   TASK_STACKSIZE       2048

// Definition of Task Priorities
#define LEDcontroller_priority 3
#define switchPolling_priority 3
#define PRVGADraw_Task_P      4
#define keyboard_task_P		  3
//#define stabilityCheck_task_P	   4
#define maintenance_Task_P	   3
#define manageLoad_Task_P	   3


// Definition of Queues
static QueueHandle_t keyboardData; 		  // Queue for changing frequency threshold
static QueueHandle_t raw_freq_data;       // Queue for receiving data

// State enum declaration
typedef enum{
	DEFAULT,
	SHEDDING,
	MONITORING,
	LOADING,
	MAINTENANCE,
	NORMAL
}state;

// flags declarations
int PREVstable;		// Monitors if stability changes in monitoring state
int stable;			// Stability bool
int timeout_500 = 0; // Flag for 500ms timer finish
int timing = 1;			// Flag for timing reaction time
int allConnected = 0;  // Flag for if relay has reonnected all loads

//int load_status[5];		// Load array controlled by relay
int switch_status[5];		// Switch array controlled only by switches
int shed_status[5];		// Tracks which loads have been shed


int measurements[5];		// Previous 5 reaction times
double average;				// Average reaction time
int minimum = 1000;				// Minimum reaction time
int maximum = 0;				// Maximum reaction time
int tick3;
int tick4;
int first_led;
int reaction_time[5];

#define CHECK_BIT(var,pos) ((var) & (1<<(pos))) // macro checks if a specific bit is set

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

// Timer and task handles
TimerHandle_t timer500;
TimerHandle_t timer200;
TaskHandle_t Timer_Reset;
TaskHandle_t xHandle;
TaskHandle_t PRVGADraw;
int timeout_200;


// Definition of Semaphores
xSemaphoreHandle thresholdSemaphore;
xSemaphoreHandle loadStatusSemaphore;
xSemaphoreHandle systemStatusSemaphore;
xSemaphoreHandle measurementSemaphore;
xSemaphoreHandle stableSemaphore;
xSemaphoreHandle switchSemaphore;
xSemaphoreHandle shedSemaphore;
xSemaphoreHandle allOnSemaphore;
xSemaphoreHandle freqRelaySemaphore;


//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.2		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 			//minimum frequency to draw


// GLOBAL VARIABLES
static volatile int keyboard_toggle = 0; // Keyboard debounce
static volatile int thresholdFreq = 50;
static volatile int thresholdRoc = 20;
state currentState = DEFAULT;

state operationState = NORMAL;

unsigned volatile int reactionStart = 0; //
unsigned volatile int reactionTotal = 0;  //
unsigned volatile int statCount = 0;  //

unsigned volatile int time500 = 0;       // Variable used for 500ms timer for loading/unloading
unsigned volatile int totalTime = 0;     // Total system uptime
unsigned volatile int dummy_value = 0;   // Passed into push button isr but not used as isr only toggles
unsigned volatile int button_value = 0;  // To switch between modes (0 = normal)
unsigned int switch_value = 0;           // Value of switches
char char_test[100];

static volatile double currentFreq;
static volatile int updatedSwitch;

static volatile int load_state[5] = {1, 1, 1, 1, 1};
int manuallyShed[5];
int all_on = 1;
int all_off = 0;
int first_load_shed_flag = 2;
int shed;
int freq_relay_start;


int redLED_state;
int redLED[5];
int redLED0;
int redLED1;
int redLED2;
int redLED3;
int redLED4;

int greenLED_state;
int greenLED[5];
int greenLED0;
int greenLED1;
int greenLED2;
int greenLED3;
int greenLED4;


// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);
int initISRs(void);


double freq[100], RoC[100];
//int i = 0;

//int i = 99,
int j = 0;
Line line_freq, line_roc;
double frequency;
#define SAMPLING_FREQ 16000.0


void PRVGADraw_Task(void *pvParameters ){


	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);



	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

	alt_up_char_buffer_string(char_buf, "System Stability: ", 7, 43);
	alt_up_char_buffer_string(char_buf, "Total System Uptime: ", 7, 45);
	alt_up_char_buffer_string(char_buf, "Minimum Reaction time: ", 7, 47);
	alt_up_char_buffer_string(char_buf, "Maximum Reaction time: ", 7, 49);
	alt_up_char_buffer_string(char_buf, "Average Reaction time: ", 7, 51);
	alt_up_char_buffer_string(char_buf, "Frequency Threshold Values: ", 7, 53);
	alt_up_char_buffer_string(char_buf, "RoC Threshold Values: ", 7, 55);

	alt_up_char_buffer_string(char_buf, "Time 1: ", 50, 43);
	alt_up_char_buffer_string(char_buf, "Time 2: ", 50, 45);
	alt_up_char_buffer_string(char_buf, "Time 3: ", 50, 47);
	alt_up_char_buffer_string(char_buf, "Time 4: ", 50, 49);
	alt_up_char_buffer_string(char_buf, "Time 5: ", 50, 51);


	int i = 99, j = 0;
	Line line_freq, line_roc;

	double newSampling;
	double prevSampling;
	double newFreq;
	double prevFreq;
	double avgSampling;

	while(1){
		totalTime = xTaskGetTickCount();

		//receive frequency data from queue
		while(uxQueueMessagesWaiting( raw_freq_data ) != 0){

			if (xQueueReceive(raw_freq_data, &newSampling, ( TickType_t ) 10 ) == pdPASS){
				newFreq = SAMPLING_FREQ / newSampling;
				freq[i] = newFreq; // the number is saved from the 99th index of the array

				if (prevFreq == 0) {
					prevFreq = newFreq;// reduce error
				}

				if (prevSampling == 0) {
					prevSampling = newSampling;
				}

				avgSampling = (prevSampling + newSampling) / 2;
			}



			//calculate frequency RoC

			if(i==0){
				RoC[0] = fabs((freq[0]-freq[99]) * SAMPLING_FREQ / avgSampling);
			}
			else{
				RoC[i] = fabs((freq[i]-freq[i-1]) * SAMPLING_FREQ / avgSampling);
			}

			if (RoC[i] > 100.0){
				RoC[i] = 100.0;
			}


			i =	++i%100; //point to the next data (oldest) to be overwritten

			// Comparing against thresholds to check stability of system
			xSemaphoreTake(stableSemaphore, portMAX_DELAY);


			//checking system's stable
			if ((freq[i] < thresholdFreq) || (RoC[i] > thresholdRoc)) {
				stable = 0;

				if (all_on == 1) {//Normal state
					xSemaphoreTake(allOnSemaphore, portMAX_DELAY);
					all_on = 0;
					xSemaphoreGive(allOnSemaphore);
					first_load_shed_flag = 1;//freq_realy state
				}
			} else {
				stable = 1;
			}

			PREVstable = stable;
			xSemaphoreGive(stableSemaphore);

		}

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * RoC[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * RoC[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		// System stability
		alt_up_char_buffer_string(char_buf, "        ",25,43);
		xSemaphoreTake(stableSemaphore, portMAX_DELAY);
		if(stable == 1){
			alt_up_char_buffer_string(char_buf,"Stable",25,43);
		}else{
			alt_up_char_buffer_string(char_buf,"Unstable",25,43);
		}
		xSemaphoreGive(stableSemaphore);

		// Reaction time on Screen
		alt_up_char_buffer_string(char_buf, "             ",28,45);
		totalTime = xTaskGetTickCount()/1000;
		sprintf(char_test,"%1d (in second)",totalTime);
		alt_up_char_buffer_string(char_buf, char_test, 28,45);

		// Measurement time
		alt_up_char_buffer_string(char_buf, "     ", 30, 47); // Minimum
		alt_up_char_buffer_string(char_buf, "     ", 30, 49); // Maximum
		alt_up_char_buffer_string(char_buf, "     ", 30, 51); // Average

		sprintf(char_test,"%d (in ms)", minimum);
		alt_up_char_buffer_string(char_buf, char_test, 30, 47); // Minimum
		sprintf(char_test,"%d (in ms)", maximum);
		alt_up_char_buffer_string(char_buf, char_test, 30, 49); // Maximum
		sprintf(char_test,"%.1f (in ms)", average);
		alt_up_char_buffer_string(char_buf, char_test, 30, 51); // Average





		// threshold value
		alt_up_char_buffer_string(char_buf,"       ",34,53);
		alt_up_char_buffer_string(char_buf,"       ",34,55);

		sprintf(char_test,"%d (in Hz)", thresholdFreq);
		alt_up_char_buffer_string(char_buf, char_test, 34, 53); // frequency threshold
		sprintf(char_test,"%d (in Hz/s)", thresholdRoc);
		alt_up_char_buffer_string(char_buf, char_test, 34, 55); // frequency threshold

		// Measurements
		alt_up_char_buffer_string(char_buf, "     ", 58, 43); // Time1
		alt_up_char_buffer_string(char_buf, "     ", 58, 45); // Time2
		alt_up_char_buffer_string(char_buf, "     ", 58, 47); // Time3
		alt_up_char_buffer_string(char_buf, "     ", 58, 49); // Time4
		alt_up_char_buffer_string(char_buf, "     ", 58, 51); // Time5

		sprintf(char_test,"%d (in ms)", reaction_time[0]);
		alt_up_char_buffer_string(char_buf, char_test, 58, 43); // time
		sprintf(char_test,"%d (in ms)", reaction_time[1]);
		alt_up_char_buffer_string(char_buf, char_test, 58, 45); // time
		sprintf(char_test,"%d (in ms)", reaction_time[2]);
		alt_up_char_buffer_string(char_buf, char_test, 58, 47); // time
		sprintf(char_test,"%d (in ms)", reaction_time[3]);
		alt_up_char_buffer_string(char_buf, char_test, 58, 49); // time
		sprintf(char_test,"%d (in ms)", reaction_time[4]);
		alt_up_char_buffer_string(char_buf, char_test, 58, 51); // time

		vTaskDelay(10);

	}
}

void maintenance_Task(void *pvParamters){
	int a;

	while(1){


		if(operationState == MAINTENANCE) {
			for(a = 0; a < 5; a++) {
				manuallyShed[a] = 0;
			}//clear all manually shed load
			if( loadStatusSemaphore != NULL )
			{
				/* See if we can obtain the semaphore.  If the semaphore is not
			        available wait 10 ticks to see if it becomes free. */
				if( xSemaphoreTake( loadStatusSemaphore, ( TickType_t ) 10 ) == pdTRUE )
				{
					for (a = 0; a < 5; a++) {
						if (updatedSwitch & (1<<a)){
							load_state[a] = 1;
						} else {
							load_state[a] = 0;
						}
					}


					xSemaphoreGive( loadStatusSemaphore );
				}

			}

			int counter = 0;
			for (a = 0; a < 5; a++) {
				if (load_state[a] == 1) {
					counter++;
				}
			}

			if (counter == 5) {
				xSemaphoreTake(allOnSemaphore, portMAX_DELAY);
				all_on = 1;
				xSemaphoreGive(allOnSemaphore);

				counter = 0;
			} else {
				counter = 0;
			}
		}
	}

}

void manageLoad_Task(void *pvParamters) {
	int a;
	int b = 0;

		while(1){

			if( loadStatusSemaphore != NULL )
			{
				/* See if we can obtain the semaphore.  If the semaphore is not
						        available wait 10 ticks to see if it becomes free. */

				if (stable == 0) {
					if (first_load_shed_flag == 1) { // shed first load, calculae time taking
							if( xSemaphoreTake( loadStatusSemaphore, ( TickType_t ) 10 ) == pdTRUE ){
								load_state[0] = 0;// 0 means shed
								xSemaphoreGive( loadStatusSemaphore );
								first_led = 1;
								freq_relay_start = 1;
								tick4 =  xTaskGetTickCount();

								reaction_time[b] = tick4 - tick3;
								if(reaction_time[b] < minimum){
									minimum = reaction_time[b];
								}
								if(reaction_time[b] > maximum){
									maximum = reaction_time[b];
								}

								b++;
								if (b == 5) {
									b = 0;
								}
							}
							first_load_shed_flag = 0;
					}
				}

				if (freq_relay_start == 1) {
					xTimerReset(timer500, 10);//start counting 500ms when enter freq_relay
					timeout_500 = 0;

					while(timeout_500 == 0) {
						//dectecting maunually shed
						for (a = 0; a < 5; a++) {
							if (updatedSwitch & (1<<a)){
								//load_state[a] = 1;
							} else {
								xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
								load_state[a] = 0;
								manuallyShed[a] = 1;
								xSemaphoreGive(loadStatusSemaphore);
							}
						}
						if (stable != PREVstable) {
							//reset timer
							xTimerReset(timer500, 10);//status changes
							timeout_500 = 0;
						}
					}

					if (timeout_500 == 1) {
						//stable or unstable for 500ms, shed or turn on the load
						if (stable == 0) {//shed load
							for (a = 0; a < 5; a++) {
								if (load_state[a] == 1) {
									xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
									load_state[a] = 0;
									xSemaphoreGive(loadStatusSemaphore);
									break;
								}
							}
						} else if (stable == 1) {//unshed load
							for (a = 4; a > -1; a--) {
								if (load_state[a] == 0 && manuallyShed[a] != 1) {
									xSemaphoreTake(loadStatusSemaphore, portMAX_DELAY);
									load_state[a] = 1;
									xSemaphoreGive(loadStatusSemaphore);

									if (a == 0) {
										freq_relay_start = 0;
										xSemaphoreTake(allOnSemaphore, portMAX_DELAY);
										all_on = 1;
										xSemaphoreGive(allOnSemaphore);
									}
									break;
								}
							}
						}

					}
				}

			}
		}

}

// Button ISR toggles a variable whenever button is pressed to
// enter and exit the maintenance state
void buttonISR(void* context, alt_u32 id){

	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	if(button_value == 1){
		button_value = 0;
		operationState = NORMAL;
		//printf("enter normal");
	}else{
		button_value = 1;
		operationState = MAINTENANCE;
		//printf("enter maintenance");
	}

}


void keyboardISR(void* context, alt_u32 id){
	char ascii;
	int status = 0;
	unsigned char key = 0;

	KB_CODE_TYPE decode_mode;
	status = decode_scancode (context, &decode_mode , &key , &ascii) ;

	IOWR(SEVEN_SEG_BASE,0 ,key);

	if(keyboard_toggle == 3){ // 42 0 42
		if ( status == 0 ){
			xQueueSendFromISR(keyboardData, &key, pdFALSE);
			keyboard_toggle = 0;
		}
	}else{
		keyboard_toggle += 1;
	}
	//printf("keyboard_toggle: %d\n", keyboard_toggle);
}


void keyboard_task(void *pvParameters){
	unsigned char key;
	while(1){
		xQueueReceive(keyboardData, &key, portMAX_DELAY);
		//printf("keyboardData: %x\n", key);
		xSemaphoreTake(thresholdSemaphore, portMAX_DELAY);
		if (key == 0x16) { // 1
			thresholdFreq+= 1;
			printf("increased thresholdFreq: %d\n", thresholdFreq);
		}
		else if (key == 0x1e) { // 2
			thresholdFreq -= 1;
			printf("decreased thresholdFreq: %d\n", thresholdFreq);
		}
		else if (key == 0x46) { // 9
			thresholdRoc += 1;
			printf("increased thresholdRoc: %d\n", thresholdRoc);
		}
		else if (key == 0x45) { // 0
			thresholdRoc -= 1;
			printf("decreased thresholdRoc: %d\n", thresholdRoc);
		}
		xSemaphoreGive(thresholdSemaphore);

	}
}

// Receive frequency data from board and send into queue
void freq_relay(){ // this is the frequency relay isr then needs to be setup in the isr setup session

	double sampling = (double)IORD(FREQUENCY_ANALYSER_BASE, 0); // raw frequency
	xQueueSendToBackFromISR(raw_freq_data, &sampling, pdFALSE ); // save the frequency into the queue
	tick3 = xTaskGetTickCount();

	return;
}

// Green represents loads being switched off (load shedding)
// Red represent loads that are switched on
// In maintenance mode, no loads are shed and all the Green light are off
// Loads can be controlled only by switch
void LEDcontroller_task(void *pvParameters){
	int sum = 0;
	int c = 0;

    while(1){

        	redLED_state = 0x00000;

        	redLED0 = load_state[0] << 0;
        	redLED1 = load_state[1] << 1;
        	redLED2 = load_state[2] << 2;
        	redLED3 = load_state[3] << 3;
        	redLED4 = load_state[4] << 4;

        	redLED_state = redLED_state | redLED0;
        	redLED_state = redLED_state | redLED1;
        	redLED_state = redLED_state | redLED2;
        	redLED_state = redLED_state | redLED3;
        	redLED_state = redLED_state | redLED4;

        	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redLED_state);

        	if (first_led == 1) {

        		first_led = 0;

        		for(c=0; c<5;c++){
        			if(reaction_time[c] != 0){
        				sum = sum + reaction_time[c];
        				average = sum/(c+1);
        			}
        		}
        		sum = 0;
        	}

        	if (operationState == MAINTENANCE) {
        		greenLED_state = 0x00000;
        		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLED_state);
        	}

        	if (operationState != MAINTENANCE) {
        		greenLED_state = 0x00000;

        		if (manuallyShed[0] != 1) {
        			greenLED0 = !load_state[0] << 0;
        		}
        		if (manuallyShed[1] != 1) {
        			greenLED1 = !load_state[1] << 1;
        		}
        		if (manuallyShed[2] != 1) {
        			greenLED2 = !load_state[2] << 2;
        		}
        		if (manuallyShed[3] != 1) {
        			greenLED3 = !load_state[3] << 3;
        		}
        		if (manuallyShed[4] != 1) {
        			greenLED4 = !load_state[4] << 4;
        		}


        		greenLED_state = greenLED_state | greenLED0;
        		greenLED_state = greenLED_state | greenLED1;
        		greenLED_state = greenLED_state | greenLED2;
        		greenLED_state = greenLED_state | greenLED3;
        		greenLED_state = greenLED_state | greenLED4;

        		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLED_state);
        	}

		vTaskDelay(5);
	}
}


// Task for polling the switches, sets switch statuses and load statuses
void switchPolling_task(void *pvParameters){

	while(1){
		switch_value = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		if( switchSemaphore != NULL )
		    {
		        /* See if we can obtain the semaphore.  If the semaphore is not
		        available wait 10 ticks to see if it becomes free. */
		        if( xSemaphoreTake( switchSemaphore, ( TickType_t ) 10 ) == pdTRUE )
		        {
		        	updatedSwitch = switch_value & 0x1F;
		            xSemaphoreGive( switchSemaphore );
		        }

		    }

		vTaskDelay(5);
	}
}

void vTimer500Callback(xTimerHandle t_timer500){ // as the stability timer
	timeout_500 = 1;
}

void vTimer200Callback(xTimerHandle t_timer200){ // as the stability timer
	timeout_200 = 1;
}

// Initialise queues and semaphores
int initOSDataStructs(void)
{
	thresholdSemaphore = xSemaphoreCreateMutex();  // mutex for threshold values for freq and RoC
	loadStatusSemaphore = xSemaphoreCreateMutex(); // mutex for load status and shed status array
	measurementSemaphore = xSemaphoreCreateMutex(); // mutex for various measurements displayed
	systemStatusSemaphore = xSemaphoreCreateMutex();
	stableSemaphore = xSemaphoreCreateMutex();
	switchSemaphore = xSemaphoreCreateMutex();
	shedSemaphore = xSemaphoreCreateMutex();
	allOnSemaphore = xSemaphoreCreateMutex();
	freqRelaySemaphore = xSemaphoreCreateMutex();

	keyboardData = xQueueCreate(100, sizeof(unsigned char));
	raw_freq_data = xQueueCreate( 100, sizeof(double) );
	timer500 = xTimerCreate("500ms timer", 1500, pdTRUE, NULL, vTimer500Callback);
	timer200 = xTimerCreate("200ms timer", 200, pdTRUE, NULL, vTimer200Callback);

	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(LEDcontroller_task, "LEDcontroller_task", TASK_STACKSIZE, NULL, LEDcontroller_priority, NULL);
	xTaskCreate(switchPolling_task, "switchPolling_task", TASK_STACKSIZE, NULL, switchPolling_priority, NULL);
	xTaskCreate(PRVGADraw_Task, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, PRVGADraw_Task_P, &PRVGADraw );
	xTaskCreate(keyboard_task, "keyboard_task", TASK_STACKSIZE, NULL, keyboard_task_P, NULL);
	xTaskCreate(maintenance_Task, "maintenance_task", TASK_STACKSIZE, NULL, maintenance_Task_P, NULL);
	xTaskCreate(manageLoad_Task, "manageLoad_task", TASK_STACKSIZE, NULL, manageLoad_Task_P, NULL);
	return 0;
}

// Initialise IRs
int initISRs(void){
	// SETUP FOR PUSH BUTTON ISR
    // clears the edge capture register
    IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0);
    // enable interrupts for all buttons
    IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
    // register the buttons ISR
    alt_irq_register(PUSH_BUTTON_IRQ, (void*)&dummy_value, buttonISR);

    // SETUP FOR KEYBOARD ISR
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	if(ps2_device == NULL){
		printf("can't find PS/2 device\n");
	}
	alt_up_ps2_clear_fifo (ps2_device);
    alt_irq_register(PS2_IRQ, ps2_device, keyboardISR);
    IOWR_8DIRECT(PS2_BASE,4,1);

    // SETUP FOR FREQUENCY RELAY ISR
    alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

    return 0;

}

int main(int argc, char* argv[], char* envp[])
{
	initOSDataStructs();
	initCreateTasks();
	initISRs();
	vTaskStartScheduler();
	for (;;);
	return 0;
}
