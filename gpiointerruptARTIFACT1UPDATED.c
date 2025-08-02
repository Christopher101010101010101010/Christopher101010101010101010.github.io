/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/* Project header files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>
#include <stdio.h>

// UART Global Variables
char output[150];
int bytesToSend;

// Driver Handles - Global variables
UART2_Handle uart;

#define DISPLAY(x) UART2_write(uart, &output, sizeof(output), 0);

void initUART(void)
{
    UART2_Params uartParams;

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_CALLBACK;
    uartParams.readMode = UART2_Mode_CALLBACK;

    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// Driver Handles - Global variables
Timer_Handle timer0;
int periodCondition = 0;

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    if(periodCondition < 10)
        periodCondition++;

    else
        periodCondition = 0;
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY( snprintf(output, 150, "Initializing I2C Driver - "));

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY( snprintf(output, 150, "Failed\n\r"));
        while (1);
    }

    DISPLAY( snprintf(output, 64, "Passed\n\r"));

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY( snprintf(output, 150, "Is this %s? ", sensors[i].id));
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY( snprintf(output, 150, "Found\n\r"));
            found = true;
            break;
        }
        DISPLAY( snprintf(output, 150, "Not Found at sensor index\n\r"));
    }

    if(found)
    {
        DISPLAY( snprintf(output, 150, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    }
    else
    {
        DISPLAY( snprintf(output, 150, "Temperature sensor not found, contact professor\n\r"));
    }
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY( snprintf(output, 150, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY( snprintf(output, 150, "Please power cycle your board by unplugging/plugging back in.\n\r"));
    }
    return temperature;
}

//global temperature variables

uint16_t setPoint = 0;              //TI CC3220S board's button interface controlled variable to set user defined temperature preference
uint16_t roomTemp = 0;              //TI CC3220S board's thermopile(temperature sensor) defined variable for degree F reading local to device
uint16_t systemMode = 0;            //0, off. 1, heating. 2, cooling.
uint16_t activationStatus = 0;      //0, off. 1, on

uint16_t button0Activation = 0;     //0, inactive. 1, active.
uint16_t button1Activation = 0;     //0, inactive. 1, active.

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    //increase set point temperature value by 1 degree
    setPoint++;
    button0Activation = 1;              // set button activation status to on
    usleep(200000);                     // only check for altered button state every 200 ms

}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    //increase set point temperature value by 1 degree
    setPoint--;
    button1Activation = 1;              // set button activation status to on
    usleep(200000);                     // only check for altered button state every 200 ms

}

void buttonStatusCh(){
    //check for both buttons being active
    //if both buttons are active cycle to the next system mode setting
    //0, off. 1, heating mode. 2, cooling mode.
    if(button0Activation == 1 && button1Activation == 1){
        if(systemMode < 2 && systemMode > -1){
            systemMode++;
        }
        else if(systemMode == 2){
            systemMode = 0;
        }
        else{
            DISPLAY( snprintf(output, 150, "Error: systemMode setting out of bounds\nDEACTIVATING SYSTEM."));
            systemMode = 0;
        }
    }
}

void checkTempStatus(){
    if(roomTemp  < setPoint && systemMode == 1){                            //if room temp is less than set point and the system is in heating mode then activate system
         GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
         activationStatus = 1;
    }
    else if(roomTemp > setPoint && systemMode == 2){                        //else-if room temp is greater than set point and the system is in cooling mode then activate system
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        activationStatus = 1;
    }
    else if(roomTemp >= setPoint && systemMode == 1){                       //else-if room temp is greater than or equal to set point and system is in heating mode then deactivate system
         GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
         activationStatus = 0;
    }
    else if(roomTemp <= setPoint && systemMode == 2){                       //else-if room temp is less than or equal to set point and system is in cooling mode then deactivate system
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        activationStatus = 0;
    }
}

void buttonReset(){                                                         //Reset button activation status to avoid trigger unwanted system mode cycling
    button0Activation = 0;
    button1Activation = 0;
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initTimer();
    initI2C();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // initialize room-temp to current temperature read by device.
    // initialize set point to the room temperature
    roomTemp = readTemp();
    setPoint = roomTemp;
    uint32_t serverDisplayTicks = 10000000; //1sec is 10000000 us
    uint16_t totalTime = 0;

    // output board data to command prompt
    DISPLAY( snprintf(output, 150, "<Room Temperature: %02d, Set Temperature: %02d, Activation Status(0-OFF, 1-ON): %d, Mode(0-OFF, 1-HEATING, 2-COOLING): %d, Total Time Since Execution: %04d>\n\r",
                      roomTemp, setPoint, activationStatus, systemMode, totalTime));

    while(1){

        // Each case represents a 100 millisecond period of activation, up to 1 second inwhich the period condition value will reset it's value back to 0, or the initial period index
        switch(periodCondition){

            case 0:                                                                     // every 200 ms update the heating/cooling LED and activation status to ON/OFF

                buttonStatusCh();                                                       // if both buttons have been activated then cycle to the next system mode option
                checkTempStatus();                                                      // compare set point temperature to the room temperature reading and activate/deactivate the system appropriately

                buttonReset();                                                          // reset both button 0 and 1 activation status to off, 0.
                break;

            case 1:
                break;

            case 2:                                                                    // every 200 ms update the heating/cooling LED and activation status to ON/OFF

                buttonStatusCh();                                                      // if both buttons have been activated then cycle to the next system mode option
                checkTempStatus();                                                     // compare set point temperature to the room temperature reading and activate/deactivate the system appropriately

                buttonReset();                                                          // reset both button 0 and 1 activation status to off, 0.
                break;

            case 3:
                break;

            case 4:                                                                     // every 200 ms update the heating/cooling LED and activation status to ON/OFF
                roomTemp = readTemp();                                                  // every 500 ms update temperature sensor reading in roomTemp variable

                buttonStatusCh();                                                       // if both buttons have been activated then cycle to the next system mode option
                checkTempStatus();                                                      // compare set point temperature to the room temperature reading and activate/deactivate the system appropriately

                buttonReset();                                                          // reset both button 0 and 1 activation status to off, 0.
                break;

            case 5:                                                                     // every 500 ms update roomTemp to current room temperature measurement
                if(!(roomTemp > -1 && roomTemp < 120)){                                 // if room temperature reading is out-of-bounds set room-temp to 0
                    //error code                                                        // and turn off heatStatus/LED
                    DISPLAY( snprintf(output, 150, "error reading room temperature(temp. sensor fault)"));
                    roomTemp = 0;
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                    activationStatus = 0;
                    systemMode = 0;
                }
                break;

            case 6:                                                                     // every 200 ms update the heating/cooling LED and activation status to ON/OFF

                buttonStatusCh();                                                       // if both buttons have been activated then cycle to the next system mode option
                checkTempStatus();                                                      // compare set point temperature to the room temperature reading and activate/deactivate the system appropriately

                buttonReset();                                                          // reset both button 0 and 1 activation status to off, 0.
                break;

            case 7:
                break;

            case 8:                                                                     // every 200 ms update the heating/cooling LED and activation status to ON/OFF

                buttonStatusCh();                                                       // if both buttons have been activated then cycle to the next system mode option
                checkTempStatus();                                                      // compare set point temperature to the room temperature reading and activate/deactivate the system appropriately

                buttonReset();                                                          // reset both button 0 and 1 activation status to off, 0.
                break;

            case 9:                                                                    // every 1 sec update server with thermostats current data
                roomTemp = readTemp();                                                  // every 500ms update temperature sensor reading stored in roomTemp variable

                totalTime = Timer_getCount(timer0) / serverDisplayTicks;                // output board data to command prompt
                DISPLAY( snprintf(output, 150, "<Room Temperature: %02d, Set Temperature: %02d, Mode(0-OFF, 1-HEATING, 2-COOLING): %d, Total Time Since Execution: %04d>\n\r",
                                 roomTemp, setPoint, systemMode, totalTime));
                break;

            default:
                break;

        }



    }

    return;
}
