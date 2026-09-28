/**
 **************************************************
 *
 * @file        Basic_Example.ino
 * @brief       Example for using PCF85063A Library
 *
 *              Product: www.solde.red/333051
 *
 * @authors     @ Zvonimir Haramustek for soldered.com.com
 ***************************************************/

#include "PCF85063A-SOLDERED.h"

PCF85063A rtc; 


void setup()
{
    Serial.begin(115200); //Start serial communication with PC using 115200 baudrate
    rtc.begin();  //Initialize RTC module

    //  setTime(hour, minute, sec);
    rtc.setTime(6, 54, 00); // 24H mode, ex. 6:54:00
    //  setDate(weekday, day, month, yr);
    rtc.setDate(6, 16, 5, 2020); // 0 for Sunday, ex. Saturday, 16.5.2020.
}

void loop()
{
    printCurrentTime(); //Call funtion printCurrentTime()
    delay(1000);
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
