/*
 Name:    Loetkolben_selbstbau.ino
 Created: 09/01/2020 10:12:25 AM
 Author:  Thomas von Loh
 Version: 1.0
*/


#include "Arduino.h"
#include <gfxfont.h>
#include <Adafruit_SPITFT_Macros.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_GFX.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "TFT_ILI9163C.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include <math.h>

#define BAUDRATE    250000

#define IN_SW_STBY    2
#define OUT_HEAT    3
#define IN_SW_DOWN    4
#define OUT_HEAT_LED    5
#define IN_SW_UP    6
#define IN_SW_T3    7
#define IN_SW_T2    8
#define IN_SW_T1    9
#define OUT_TFT_CS      10
//#define POWER     12 //use MISO PULLUP as switch
//#define OUT_LED_STATUS 13
#define IN_TEMP_SENSE A0
//#define IN_STBY_NO    A1
#define IN_BAT_C3   A2
#define IN_BAT_C2   A3
#define IN_BAT_C1   A4
#define OUT_TFT_DS  A5
#define KP        5     // Momentane Differenz
#define KI        0.1     // Zeitliche Abweichng (Signaländerung)
#define KD        0     // Beschleunigender Anteil kurzzeitige Verstärkerung des P-Anteils
#define PID_MIN_VAL   0
#define PID_MAX_VAL   250
#define PID_SAMPLE_TIME 200
#define PID_DIRECTION DIRECT  //DIRECT (like a car) or REVERSE (like a refrigerator)
#define ADJUST_TEMP_CONST 52.2298
#define ADJUST_TEMP_LINEAR  0.458863
#define ADJUST_TEMP_QUBIC -0.000057
#define EEADRESS_T1     0
#define EEADRESS_T2     8
#define EEADRESS_T3     16
#define TFT_ROTATION  135

#ifdef USE_TFT_RESET
TFT_ILI9163C tft = TFT_ILI9163C(OUT_TFT_CS, OUT_TFT_DC, IN_STBY_NO);
#else
TFT_ILI9163C tft = TFT_ILI9163C(OUT_TFT_CS, OUT_TFT_DS);
#endif
#define BLACK   0x0000
#define BLUE    0xF800
#define RED     0x11FF //0x001F
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF
#define GRAY    0xF79E  
#define OLIVE   0x7BE0

#define RGB_DISP 0x0
#define BGR_DISP 0x2

// workhorse var.
//-------------------------------------
byte status_sw_stby = 0;
byte status_sw_T1 = 0;
byte status_sw_T2 = 0;
byte status_sw_T3 = 0;

double pid_Kp = 5;
double pid_Ki = 0.1;
double pid_Kd = 0;

double temp = 0;
double temp_t1 = 200;
double temp_t2 = 280;
double temp_t3 = 310;
double timer = millis();

String serial_data_in_string_array [3];

// set Variable for PID
//--------------------------------------
double pid_setpoint, pid_input, pid_output;
double Kp = KP, Ki = KI, Kd = KD;

PID myPID(&pid_input, &pid_output, &pid_setpoint, Kp, Ki, Kd, PID_DIRECTION);


//---------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------- funktions
//---------------------------------------------------------------------------------------------------------------------------------------------------
double get_temp(double temp_sens)
{
  double temp_f = 0;
  temp_f = ADJUST_TEMP_CONST + ADJUST_TEMP_LINEAR * temp_sens + ADJUST_TEMP_QUBIC * square(temp_sens);
  return temp_f;
}

void PID()
{
  pid_input = temp;
  myPID.Compute();
  analogWrite(OUT_HEAT, pid_output);
  analogWrite(OUT_HEAT_LED, pid_output);
}

void tft_print(byte x, byte y, byte size_text, int collor_01, int collor_02, String text_01, String text_02)
{
  tft.setCursor(x, y);
  tft.setTextSize(size_text);
  tft.setTextColor(collor_01, collor_02);
  tft.print(text_01); tft.print(text_02);
}

void tft_draw_power(int x, int y, int collor)
{
  tft.drawCircle(x, y, 5, collor);
  tft.drawCircle(x, y, 6, collor);
  tft.drawFastVLine(x, y-7, 6, collor);
  tft.drawFastVLine(x-1, y-7, 6, BLACK);
  tft.drawFastVLine(x+1, y-7, 6, BLACK);
}

void serial_com()
{
  char serial_data_in_char =' ';
  String serial_data_in_string = "";
  byte i=0;
  while (Serial.available() > 0)
  {
    serial_data_in_char = Serial.read();
    if(serial_data_in_char == ',')
    {
      serial_data_in_string = "";
      i++;
    }
    else
    {
      serial_data_in_string += serial_data_in_char;
      serial_data_in_string[i] = serial_data_in_string;
    }
  }
  pid_Kp = serial_data_in_string_array[0].toDouble();
  pid_Ki = serial_data_in_string_array[1].toDouble();
  pid_Kd = serial_data_in_string_array[2].toDouble();
  Serial.print(pid_Kp); Serial.print(pid_Ki); Serial.println(pid_Kd);

  Serial.flush();
}

//---------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------- setup funktion
//---------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  pinMode(IN_SW_STBY, INPUT_PULLUP);
  pinMode(IN_SW_DOWN, INPUT_PULLUP);
  pinMode(IN_SW_UP, INPUT_PULLUP);
  pinMode(IN_SW_T1, INPUT_PULLUP);
  pinMode(IN_SW_T2, INPUT_PULLUP);
  pinMode(IN_SW_T3, INPUT_PULLUP);
  pinMode(IN_TEMP_SENSE, INPUT);
  pinMode(OUT_HEAT, OUTPUT);
  pinMode(OUT_HEAT_LED, OUTPUT);
  pinMode(OUT_TFT_CS, OUTPUT);

  timer = millis();
  pid_input = analogRead(temp);
  pid_setpoint = 0;

  analogWrite(OUT_HEAT, 0);
  analogWrite(OUT_HEAT_LED, 0);
  digitalWrite(OUT_TFT_CS, HIGH);

  Serial.begin(BAUDRATE);
  Serial.println("Hello, im redy for rumble");
  Serial.print("Baudrate: "); Serial.println(BAUDRATE);

  EEPROM.get(EEADRESS_T1, temp_t1);
  EEPROM.get(EEADRESS_T2, temp_t2);
  EEPROM.get(EEADRESS_T3, temp_t3);

  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(PID_MIN_VAL, PID_MAX_VAL);
  myPID.SetSampleTime(PID_SAMPLE_TIME);

  tft.begin();
  tft.setRotation(TFT_ROTATION);
  tft.clearScreen();
  tft.fillTriangle(148, 65, 158, 65, 153, 50, WHITE);
  tft.fillTriangle(148, 90, 158, 90, 153, 105, WHITE);
  tft_draw_power(11,98,GREEN);
  tft_print(1, 1, 2, WHITE, BLACK, "Temp: ", String((int)temp));
  tft_print(1, 20, 2, WHITE, BLACK, "pid: ", String((int)pid_setpoint));
  tft_print(1, 40, 2, GRAY, BLACK, " T1: ", String((int)temp_t1));
  tft_print(1, 55, 2, GRAY, BLACK, " T2: ", String((int)temp_t2));
  tft_print(1, 70, 2, GRAY, BLACK, " T3: ", String((int)temp_t3));
  tft_print(25, 90, 2, GREEN, BLACK, "stop ", "");
  tft_print(136, 1, 1, OLIVE, BLACK, "V1.0", "");
}

//---------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------- main funktion
//---------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  Serial.println(temp);
  //serial_com();
  temp = get_temp(analogRead(IN_TEMP_SENSE));
  PID();
  tft_print(1, 1, 2, WHITE, BLACK, "Temp: ", String((int)temp));
  tft_print(1, 20, 2, WHITE, BLACK, "pid: ", String((int)pid_setpoint) + "  ");

  if (!digitalRead(IN_SW_STBY) || !digitalRead(IN_SW_T1) || !digitalRead(IN_SW_T2) || !digitalRead(IN_SW_T3) || !digitalRead(IN_SW_UP) || !digitalRead(IN_SW_DOWN))
  {
    timer = millis();
    while (!digitalRead(IN_SW_STBY) || !digitalRead(IN_SW_T1) || !digitalRead(IN_SW_T2) || !digitalRead(IN_SW_T3))
    {
      if (timer + 1000 < millis())
      {
        analogWrite(OUT_HEAT_LED, 250);
        (digitalRead(IN_SW_STBY) == 0) ? (status_sw_stby = 2) : (status_sw_stby = 0);
        (digitalRead(IN_SW_T1) == 0) ? (status_sw_T1 = 2) : (status_sw_T1 = 0);
        (digitalRead(IN_SW_T2) == 0) ? (status_sw_T2 = 2) : (status_sw_T2 = 0);
        (digitalRead(IN_SW_T3) == 0) ? (status_sw_T3 = 2) : (status_sw_T3 = 0);
        Serial.print(status_sw_stby); Serial.print(", "); Serial.print(status_sw_T1); Serial.print(", "); Serial.print(status_sw_T2); Serial.print(", "); Serial.println(status_sw_T3);
      }
      else
      {
        (digitalRead(IN_SW_STBY) == 0) ? (status_sw_stby = 1) : (status_sw_stby = 0);
        (digitalRead(IN_SW_T1) == 0) ? (status_sw_T1 = 1) : (status_sw_T1 = 0);
        (digitalRead(IN_SW_T2) == 0) ? (status_sw_T2 = 1) : (status_sw_T2 = 0);
        (digitalRead(IN_SW_T3) == 0) ? (status_sw_T3 = 1) : (status_sw_T3 = 0);
        Serial.print(status_sw_stby); Serial.print(", "); Serial.print(status_sw_T1); Serial.print(", "); Serial.print(status_sw_T2); Serial.print(", "); Serial.println(status_sw_T3);
      }
    }

    while (!digitalRead(IN_SW_UP))
    {
      pid_setpoint++;
      tft_print(1, 20, 2, WHITE, BLACK, "pid: ", String((int)pid_setpoint) + "  ");
      delay(50);
    }

    while (!digitalRead(IN_SW_DOWN))
    {
      pid_setpoint--;
      tft_print(1, 20, 2, WHITE, BLACK, "pid: ", String((int)pid_setpoint) + "  ");
      delay(50);
    }
  }

  switch (status_sw_stby)
  {
  case 1:
    analogWrite(OUT_HEAT_LED, 0);
    myPID.SetMode(MANUAL);
    pid_output = 0;
    tft_print(25, 90, 2, GREEN, BLACK, "stop ", "");
    tft_draw_power(11,98,GREEN);
    status_sw_stby = 0;
    break;
  case 2:
    myPID.SetMode(AUTOMATIC);
    tft_print(25, 90, 2, RED, BLACK, "start", "");
    tft_draw_power(11, 98, RED);

    status_sw_stby = 0;
    break;
  default:
    break;
  }

  switch (status_sw_T1)
  {
  case 1:
    pid_setpoint = temp_t1;
    status_sw_T1 = 0;
    break;
  case 2:
    temp_t1 = pid_setpoint;
    EEPROM.put(EEADRESS_T1, pid_setpoint);
    tft_print(1, 40, 2, GRAY, BLACK, " T1: ", String((int)temp_t1) + "  ");
    status_sw_T1 = 0;
    break;
  default:
    break;
  }

  switch (status_sw_T2)
  {
  case 1:
    pid_setpoint = temp_t2;
    status_sw_T2 = 0;
    break;
  case 2:
    temp_t2 = pid_setpoint;
    EEPROM.put(EEADRESS_T2, pid_setpoint);
    tft_print(1, 55, 2, GRAY, BLACK, " T2: ", String((int)temp_t2) + "  ");
    status_sw_T2 = 0;
    break;
  default:
    break;
  }

  switch (status_sw_T3)
  {
  case 1:
    pid_setpoint = temp_t3;
    status_sw_T3 = 0;
    break;
  case 2:
    temp_t3 = pid_setpoint;
    EEPROM.put(EEADRESS_T3, pid_setpoint);
    tft_print(1, 70, 2, GRAY, BLACK, " T3: ", String((int)temp_t3) + "  ");
    status_sw_T3 = 0;
    break;
  default:
    break;
  }
}
