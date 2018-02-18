/** Sensors PinOut
 * 
 * Soil moisture pin A0......DONE
 * Air DHT Sensor pin 3......DONE
 * Soil DHT Sensor ......... pin 2 DONE
 * EZ Conductivity Sensor  A1.......DONE
 * EZ Temp pin 8.......DONE
 * Pressure Sensor ......... I2C......DONE
 * 2 gase : Hydrogen   pin A2   mq8
 * methane   Pin A3    mq3
 * 
 * LCD
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 7
 * LCD D5 pin to digital pin 6
 * LCD D6 pin to digital pin 5
 * LCD D7 pin to digital pin 4
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * Vo ----> Ground
 * 
 **/

#include <ros.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <std_msgs/String.h>

String output;
std_msgs::String sensors_msg;
ros::NodeHandle nh;
ros::Publisher pub("sensors", &sensors_msg);

Adafruit_BMP085 bmp;
LiquidCrystal lcd(12, 10, 7, 6, 5, 4);

#define StartConvert 0
#define ReadTemperature 1
const byte numReadings = 20;
byte ECsensorPin = A1;
byte DS18B20_Pin = 13;
unsigned int AnalogSampleInterval = 25, printInterval = 700, tempSampleInterval = 850;
unsigned int readings[numReadings];
byte index = 0;
unsigned long AnalogValueTotal = 0;
unsigned int AnalogAverage = 0, averageVoltage = 0;
unsigned long AnalogSampleTime, printTime, tempSampleTime;
float temperature, ECcurrent;
//Temperature chip i/o
OneWire ds(DS18B20_Pin);

float hydrogen_raw, methane_raw;
float hydrogen, methane;
int soilMoisturePin = A0;
int soilMoistureValue_raw = 0;
int soilMoistureValue = 0; // Percentage 0% ---> 100%

//Constants
#define DHTPIN 9
#define DHTPIN_SOIL 2

#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
DHT dht_soil(DHTPIN_SOIL, DHTTYPE);

//Variables
int chk;
int chk_soil;
float hum;
float hum_soil;
float temp; //Stores temperature value
float temp_soil;

float TempProcess(bool ch);

float temp_bmp, pressure_bmp, altitude_bmp;

byte degree[8] = {
    0b00111,
    0b00101,
    0b00111,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
    0b00000};

byte space[8] = {
    0b10001,
    0b01010,
    0b00100,
    0b10001,
    0b01010,
    0b00100,
    0b10001,
    0b01010};

byte four[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00001,
    0b00011,
    0b00101,
    0b01111,
    0b00001};

byte two[8] = {
    0b00000,
    0b00000,
    0b00000,
    0b00110,
    0b01001,
    0b00010,
    0b00100,
    0b01111};

byte conductivity[8] = {
    0b00000,
    0b00000,
    0b00001,
    0b00110,
    0b01000,
    0b10100,
    0b01000,
    0b00000};

void setup()
{
  pinMode(soilMoisturePin, INPUT);
  // dht.begin();
  // dht_soil.begin();
  // lcd.begin(16, 4);
  // lcd.createChar(1, degree);
  // lcd.createChar(0, space);
  // lcd.createChar(4, four);
  // lcd.createChar(2, two);
  // lcd.createChar(7, conductivity);

  for (byte thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  TempProcess(StartConvert); //let the DS18B20 start the convert
  AnalogSampleTime = millis();
  printTime = millis();
  tempSampleTime = millis();
  // //bmp.begin();
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  nh.initNode();
  nh.advertise(pub);
  nh.loginfo("DONE");
  //Serial.begin(9600);
}

void loop()
{
  soilMoistureValue_raw = analogRead(soilMoisturePin);
  soilMoistureValue = -18.08 * log(soilMoistureValue_raw) + 125.09;
  // hum = dht.readHumidity();
  // temp = dht.readTemperature();
  // hum_soil = dht_soil.readHumidity();
  // temp_soil = dht_soil.readTemperature();

  if (millis() - AnalogSampleTime >= AnalogSampleInterval)
  {
    AnalogSampleTime = millis();
    AnalogValueTotal = AnalogValueTotal - readings[index];
    readings[index] = analogRead(ECsensorPin);
    AnalogValueTotal = AnalogValueTotal + readings[index];
    index = index + 1;
    if (index >= numReadings)
      index = 0;

    AnalogAverage = AnalogValueTotal / numReadings;
  }
  if (millis() - tempSampleTime >= tempSampleInterval)
  {
    tempSampleTime = millis();
    temperature = TempProcess(ReadTemperature); // read the current temperature from the  DS18B20
    TempProcess(StartConvert);                  //after the reading,start the convert for next reading
  }

  if (millis() - printTime >= printInterval)
  {
    printTime = millis();
    averageVoltage = AnalogAverage * (float)5000 / 1024;

    float TempCoefficient = 1.0 + 0.0185 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge = (float)averageVoltage / TempCoefficient;
    if (CoefficientVolatge < 150)
      ; //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    else if (CoefficientVolatge > 3300)
      ; //>20ms/cm,out of the range
    else
    {
      if (CoefficientVolatge <= 448)
        ECcurrent = 6.84 * CoefficientVolatge - 64.32; //1ms/cm<EC<=3ms/cm
      else if (CoefficientVolatge <= 1457)
        ECcurrent = 6.98 * CoefficientVolatge - 127; //3ms/cm<EC<=10ms/cm
      else
        ECcurrent = 5.3 * CoefficientVolatge + 2278; //10ms/cm<EC<20ms/cm
      ECcurrent /= 1000;
    }
  }
  // temp_bmp = bmp.readTemperature();
  // pressure_bmp = bmp.readPressure();
  // altitude_bmp = bmp.readAltitude();
  hydrogen_raw = analogRead(A2);
  methane_raw = analogRead(A3);
  hydrogen = hydrogen_raw / 80;
  methane = 20 / 50; //some sort of calibration or whatever
  output = "";
  output += "$SENSORS,";
  output += soilMoistureValue;
  output += ",";
  output += (int)(hum * 1000); //air
  output += ",";
  output += (int)(temp * 1000); //air
  output += ",";
  output += (int)(hum_soil * 1000); //soil....won't be used, use Soil Moisture instead
  output += ", ";
  output += (int)(temp_soil * 1000); //soil
  output += ", ";
  output += (int)(ECcurrent * 1000); //Conductivity
  output += ", ";
  output += (int)(temp_bmp * 1000);
  output += ", ";
  output += (int)(pressure_bmp * 1000);
  output += ", ";
  output += (int)(altitude_bmp * 1000);
  output += ",";
  output += (int)(hydrogen * 1000);
  output += ",";
  output += (int)(methane * 1000);
  output += ",*";
  nh.loginfo(&output[0]);

  sensors_msg.data = &output[0];
  pub.publish(&sensors_msg);

  nh.spinOnce();

  // lcd.setCursor(0, 0);
  // lcd.print("P=");
  // lcd.print(pressure_bmp / 101300, 2);
  // lcd.print("atm");
  // lcd.print((char)0);
  // lcd.print("T=");
  // lcd.print((int)temp_bmp);
  // lcd.write(1);
  // lcd.print("C");
  // lcd.setCursor(0, 1);
  // lcd.write(7); // Conductivity
  // lcd.print("=00ppm");
  // lcd.setCursor(9, 1);
  // lcd.print((char)0);
  // lcd.print("H");
  // lcd.write(2);
  // lcd.print("O=");
  // lcd.print(hum);
  // lcd.setCursor(0, 2);
  // lcd.print("H");
  // lcd.write(2);
  // lcd.print("=");
  // lcd.print(hydrogen);
  // lcd.print("ppm");
  // lcd.setCursor(9, 2);
  // lcd.print((char)0);
  // lcd.print("CH"); //methane
  // lcd.write(4);
  // lcd.print("=");
  // lcd.print((int)methane);
  // lcd.setCursor(0, 3);
  // lcd.print("ST=");
  // lcd.print(temp_soil, 1);
  // lcd.write(1);
  // lcd.print("C");
  // lcd.setCursor(9, 3);
  // lcd.print((char)0);
  // lcd.print("SM=");
  // lcd.print((int)soilMoistureValue);
  // lcd.print("%");

  delay(1500);
}
float TempProcess(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if (!ch)
  {
    if (!ds.search(addr))
    {
      ds.reset_search();
      return 0;
    }
    if (OneWire::crc8(addr, 7) != addr[7])
    {

      return 0;
    }
    if (addr[0] != 0x10 && addr[0] != 0x28)
    {
      return 0;
    }
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end
  }
  else
  {
    byte present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad
    for (int i = 0; i < 9; i++)
    { // we need 9 bytes
      data[i] = ds.read();
    }
    ds.reset_search();
    byte MSB = data[1];
    byte LSB = data[0];
    float tempRead = ((MSB << 8) | LSB); //using two's compliment
    TemperatureSum = tempRead / 16;
  }
  return TemperatureSum;
}
