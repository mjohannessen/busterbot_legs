#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

extern"C"{
#include "pico/stdio/driver.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"
#include "pico_uart_transports.h"
#include "pico/stdlib.h"
}

#include "FreeRTOS.h"
#include "task.h"

#include "BlinkAgent.h"
#include "MotorsAgent.h"
#include "uRosBridge.h"
#include "Busterbot.h"

#define LED_PIN 25
//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

//Left Motor
#define LEFT_DIR_1		2
#define LEFT_DIR_2	    3
#define LEFT_ROTENC_A 	12
#define LEFT_ROTENV_B	13
#define LEFT_PWM		18

//Right Motor
#define RIGHT_DIR_1	    4
#define RIGHT_DIR_2	    5
#define RIGHT_ROTENC_A 	10
#define RIGHT_ROTENV_B	11
#define RIGHT_PWM		19

//PID
#define KP	0.55
#define KI	0.019
#define KD	0.24

char ROBOT_NAME[]="busterbot";


////rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

/***
 * Main task to blink external LED
 * @param params - unused
 */
void mainTask(void *params){

    BlinkAgent blink(LED_PIN);
    //blink.start("Blink", TASK_PRIORITY);

    MotorsAgent motors;
	motors.addMotor(0, LEFT_DIR_1, LEFT_DIR_2, LEFT_ROTENC_A, LEFT_ROTENV_B, LEFT_PWM);
	motors.addMotor(1, RIGHT_DIR_1, RIGHT_DIR_2, RIGHT_ROTENC_A, RIGHT_ROTENV_B, RIGHT_PWM);
	motors.configAllPID(KP, KI, KD);
	motors.start("Motors", TASK_PRIORITY);

	//Busterbot
	Busterbot busterbot;
	busterbot.setMotorsAgent(&motors);
	busterbot.start("Busterbot", TASK_PRIORITY);

	//Start up a uROS Bridge
	uRosBridge *bridge = uRosBridge::getInstance();
	bridge->setuRosEntities(&busterbot);
	bridge->setLed(LED_PIN);
	bridge->start("Bridge",  TASK_PRIORITY+2);

   for(;;){
		vTaskDelay(10000);
	}
}

void vLaunch( void) {

	//Start blink task
    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main()
{
    stdio_init_all();
    stdio_filter_driver(&stdio_uart);
    sleep_ms(2000);
    printf("GO\n");

     //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
    
    return 0;
}

