#include <SHT1x.h>
#define dataPin 10
#define clockPin 11
SHT1x sht1x(dataPin, clockPin);

#include <Arduino.h>
#include <SPI.h>

#include <WiFi101.h>

#include "IoTMakers.h"
#include "Shield_Wrapper.h"

/*
Dht11 sensor
*/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN            5         // Pin which is connected to the DHT sensor.
// Uncomment the type of sensor in use:
#define DHTTYPE           DHT11     // DHT 11 
//#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)
// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

/*liquid ph sensor*/
// 센서 스트림 태크  - [사용자환경에 맞게 변경 필요]
char streamTag_pHValue[]  = "pHValue"; 
#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset -0.09            //deviation compensate
#define LED 13
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;   


DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

/*
Arduino Shield
*/

Shield_Wrapper	g_shield;

#define SDCARD_CS	4
void sdcard_deselect()
{
	pinMode(SDCARD_CS, OUTPUT);
	digitalWrite(SDCARD_CS, HIGH); //Deselect the SD card
}
void init_shield()
{
	sdcard_deselect();

  const char* WIFI_SSID = "xxxxx";
  const char* WIFI_PASS = "xxxxx";
  
	g_shield.begin(WIFI_SSID, WIFI_PASS);

	g_shield.print();
}


/*
IoTMakers
*/
IoTMakers g_im;

const char deviceID[]   = "xxxx";
const char authnRqtNo[] = "xxxxx";
const char extrSysID[]  = "xxxxx";

void init_iotmakers()
{
	Client* client = g_shield.getClient();
	if ( client == NULL )	{
		Serial.println(F("No client from shield."));
		while(1);
	}

	g_im.init(deviceID, authnRqtNo, extrSysID, *client);
	g_im.set_numdata_handler(mycb_numdata_handler);
	g_im.set_strdata_handler(mycb_strdata_handler);
	g_im.set_dataresp_handler(mycb_resp_handler);

	// IoTMakers 서버 연결
	//Serial.println(F("connect()..."));
	while ( g_im.connect() < 0 ){
	//	Serial.println(F("retrying."));
		delay(3000);
	}

	// Auth

	//Serial.println(F("auth."));
	while ( g_im.auth_device() < 0 ) {
	//	Serial.println(F("fail"));
		while(1);
	}

//	Serial.print(F("FreeRAM="));Serial.println(g_im.getFreeRAM());

}

#define PIN_LED		13

void setup() 
{
//	Serial.begin(115200);
//  	while ( !Serial )  {
//	  ;
//	}
  // DHT11 Initialize device.
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);

  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, HIGH);

	init_shield();
	
	init_iotmakers();

  digitalWrite(PIN_LED, LOW);
 
}

void loop()
{
	static unsigned long tick = millis();

	// 3초 주기로 센서 정보 송신
	if ( ( millis() - tick) > 3000 )
	{
		digitalWrite(PIN_LED, HIGH);
		send_temperature2();
   // send_soil_temperature2();
    send_humidity();
		send_light();
    send_liquid_ph();
		digitalWrite(PIN_LED, LOW);

		tick = millis();
 	}
  
	// IoTMakers 서버 수신처리 및 keepalive 송신
	g_im.loop();
}


int send_temperature()
{
	int tmpVal = analogRead(A0);
	float voltage = (tmpVal/1024.0) * 5.0;
	float temperature = (voltage - .5) * 100;

	//Serial.print(F("Temperature (c): ")); Serial.println(temperature);
	if ( g_im.send_numdata("temperature", (double)temperature) < 0 ) {
  	//	Serial.println(F("fail"));  
		return -1;
	}
	return 0;   
}


int send_liquid_ph()
{
   byte result;
  static unsigned long timer_collect = millis();
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      pHArray[pHArrayIndex++]=analogRead(SensorPin);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
      pHValue = 3.5*voltage+Offset;
      samplingTime=millis();
  }

  
  if ( g_im.send_numdata("liquidph", (double)pHValue) < 0 ) {
    //  Serial.println(F("fail"));  
    return -1;
  }
  return 0;   
}//end send ph

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

int send_temperature2()
{
  
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
   int t = event.temperature;

 //Serial.print(F("Temperature (c): ")); Serial.println(t);
 if ( g_im.send_numdata("temperature", (double)t) < 0 ) {
    //  Serial.println(F("fail"));  
    return -1;
  }
  return 0;   
}



int send_humidity()
{
  
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.humidity().getEvent(&event);
   int h = event.relative_humidity;

 //Serial.print(F("humidity (%): ")); Serial.println(h);
 if ( g_im.send_numdata("humidity", (double)h) < 0 ) {
    //  Serial.println(F("fail"));  
    return -1;
  }
  return 0;   
}

int send_light()
{
	int tmpVal = analogRead(A1);
	int light = tmpVal/4;

	Serial.print(F("Light : ")); Serial.println(light);
	if ( g_im.send_numdata("light", (double)tmpVal) < 0 ){
  		Serial.println(F("fail"));  
		return -1;
	}
	return 0;   
}



void mycb_numdata_handler(char *tagid, double numval)
{
	// !!! USER CODE HERE
	//Serial.print(tagid);Serial.print(F("="));Serial.println(numval);
}
void mycb_strdata_handler(char *tagid, char *strval)
{
	// !!! USER CODE HERE
	//Serial.print(tagid);Serial.print(F("="));Serial.println(strval);
  
	if ( strcmp(tagid, "led")==0 && strcmp(strval, "on")==0 )  	
		digitalWrite(PIN_LED, HIGH);
	else if ( strcmp(tagid, "led")==0 && strcmp(strval, "off")==0 )  	
		digitalWrite(PIN_LED, LOW);
}
void mycb_resp_handler(long long trxid, char *respCode)
{
	if ( strcmp(respCode, "100")==0 ){
		//Serial.println("resp:OK");
	}else{
		//Serial.println("resp:Not OK");
	}
}
