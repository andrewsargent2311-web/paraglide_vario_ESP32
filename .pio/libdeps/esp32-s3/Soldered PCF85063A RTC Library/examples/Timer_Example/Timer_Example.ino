/**
 **************************************************
 *
 * @file        Timer_Example.ino
 * @brief       Example for using timer on the PCF85063A
 *
 *              Product: www.solde.red/333051
 *
 * @authors     @ Zvonimir Haramustek for soldered.com.com
 ***************************************************/

#include "PCF85063A-SOLDERED.h"

PCF85063A rtc;

uint8_t countdown_time = 5; // timer countdown in seconds

void setup()
{
    Serial.begin(115200); //Start serial communication with PC using 115200 baudrate
    rtc.begin();  //Initialize RTC module

    //  setTime(hour, minute, sec);
    rtc.setTime(6, 54, 00); // 24H mode, ex. 6:54:00
    //  setDate(weekday, day, month, yr);
    rtc.setDate(6, 16, 5, 2020); // 0 for Sunday, ex. Saturday, 16.5.2020.

    Serial.print("Now is:");
}

void loop()
{
    printCurrentTime(); //Call funtion printCurrentTime()
    Serial.print("Setting timer countdown, waking up in ");
    Serial.print(countdown_time);
    Serial.println(" seconds.");
    // while(!Serial.available());

    /*   source_clock
     *       PCF85063A::TIMER_CLOCK_4096HZ     -> clk = 4096Hz -> min timer = 244uS -> MAX timer = 62.256mS
     *       PCF85063A::TIMER_CLOCK_64HZ       -> clk = 64Hz   -> min timer = 15.625mS -> MAX timer = 3.984s
     *       PCF85063A::TIMER_CLOCK_1HZ        -> clk = 1Hz    -> min timer = 1s -> MAX timer = 255s
     *       PCF85063A::TIMER_CLOCK_1PER60HZ   -> clk = 1/60Hz -> min timer = 60s -> MAX timer = 4h15min
     *   value
     *       coundowntime in seconds
     *   int_enable
     *       true = enable interrupt; false = disable interrupt
     *   int_pulse
     *       true = interrupt generate a pulse; false = interrupt follows timer flag
     */
    rtc.timerSet(PCF85063A::TIMER_CLOCK_1HZ, countdown_time, false, false);

    Serial.print("Waiting for a countdown");
    while (!rtc.checkTimerFlag()) // Check if timer finished countdown
    {
        Serial.print(".");
        delay(1000);
    }

    Serial.print("\nInterrupt triggered on: ");
}

void printCurrentTime()
{
    switch (rtc.getWeekday()) // Get weekday, 0 is Sunday
                              // and decode to string
    {
    case 0:                   
        Serial.print("Sunday , ");
        break;
    case 1:
        Serial.print("Monday , ");
        break;
    case 2:
        Serial.print("Tuesday , ");
        break;
    case 3:
        Serial.print("Wednesday , ");
        break;
    case 4:
        Serial.print("Thursday , ");
        break;
    case 5:
        Serial.print("Friday , ");
        break;
    case 6:
        Serial.print("Saturday , ");
        break;
    }

    Serial.print(rtc.getDay()); //Function for getting day in month
    Serial.print(".");
    Serial.print(rtc.getMonth()); //Function for getting month
    Serial.print(".");
    Serial.print(rtc.getYear()); //Function for getting year
    Serial.print(". ");
    Serial.print(rtc.getHour()); //Function for getting hours
    Serial.print(":");
    Serial.print(rtc.getMinute()); //Function for getting minutes
    Serial.print(":");
    Serial.println(rtc.getSecond()); //Function for getting seconds
}
