#include <Arduino.h>
#include <SPI.h>

#include <WiFi101.h>

#include "IoTMakers.h"
#include "Shield_Wrapper.h"


/*
DHT22 sensor 
*/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 5     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

/*
식물용 LED 핀번호 정의
*/

#define PlightPin   7

/*모터 핀번호*/
#define MotorPin    2  //팬 모터핀
#define MotorPin2   3  //팬 모터핀


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
	
	const char* WIFI_SSID = "WGF_2G";
	const char* WIFI_PASS = "minimo9595!";
  
	g_shield.begin(WIFI_SSID, WIFI_PASS);

	g_shield.print();
}


/*
IoTMakers
*/
IoTMakers g_im;

const char deviceID[]   = "";
const char authnRqtNo[] = "9";
const char extrSysID[]  = "9";

void init_iotmakers()
{
	Client* client = g_shield.getClient();
	if ( client == NULL )	{
		//Serial.println(F("No client from shield."));
		while(1);
	}

	g_im.init(deviceID, authnRqtNo, extrSysID, *client);
	g_im.set_numdata_handler(mycb_numdata_handler);
	g_im.set_strdata_handler(mycb_strdata_handler); //모터제어 핸들러로 사용
	g_im.set_dataresp_handler(mycb_resp_handler);
 

	// IoTMakers 서버 연결
	Serial.println(F("connect()..."));
	while ( g_im.connect() < 0 ){
		Serial.println(F("retrying."));
		delay(3000);
	}

	// Auth

	Serial.println(F("auth."));
	while ( g_im.auth_device() < 0 ) {
		Serial.println(F("fail"));
		while(1);
	}

	//Serial.print(F("FreeRAM="));Serial.println(g_im.getFreeRAM());
}

#define PIN_LED		6  

void setup() 
{  
	Serial.begin(115200);
  	while ( !Serial )  {
	  ;
	}
  
  pinMode(MotorPin, OUTPUT); //환기팬모터
	pinMode(PIN_LED, OUTPUT);
  pinMode(PlightPin, OUTPUT);//식물LED
	digitalWrite(PIN_LED, HIGH);

	init_shield();
	
	init_iotmakers();

	digitalWrite(PIN_LED, LOW);
 
 /*DHT22 start */
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
 // Set delay between sensor readings based on sensor details.
 // delayMS = sensor.min_delay / 1000;
  /*DHT22 init end */

}

void loop()
{
	static unsigned long tick = millis();

	// 3초 주기로 센서 정보 송신
	if ( ( millis() - tick) > 3000 )
	{
		digitalWrite(PIN_LED, HIGH);
    
    send_temperature(); //온도
    send_humidity(); //습도
		send_light();
   
		digitalWrite(PIN_LED, LOW);

		tick = millis();
 	}
  
	// IoTMakers 서버 수신처리 및 keepalive 송신
	g_im.loop();
}

/*온도 함수*/
int send_temperature()
{
  //int tmpVal = analogRead(A0);
  //float voltage = (tmpVal/1024.0) * 5.0;
  //float temperature = (voltage - .5) * 100;

 // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);

  Serial.print(F("Temperature (c): ")); Serial.println(event.temperature);
  if ( g_im.send_numdata("temperature", (double)event.temperature) < 0 ) {
      Serial.println(F("fail"));  
    return -1;
  }
  return 0;   
}

/*습도 함수*/
int send_humidity()
{
  //int tmpVal = analogRead(A0);
  //float voltage = (tmpVal/1024.0) * 5.0;
  //float temperature = (voltage - .5) * 100;

 // Get temperature event and print its value.
  sensors_event_t event;
  dht.humidity().getEvent(&event);

  Serial.print(F("Humidity (c): ")); Serial.println(event.relative_humidity);
  if ( g_im.send_numdata("humidity", (double)event.relative_humidity) < 0 ) {
      Serial.println(F("fail"));  
    return -1;
  }
  return 0;   
}

int send_temperature_old()
{
	int tmpVal = analogRead(A0);
	float voltage = (tmpVal/1024.0) * 5.0;
	float temperature = (voltage - .5) * 100;

	//Serial.print(F("Temperature (c): ")); Serial.println(temperature);
	if ( g_im.send_numdata("temperature", (double)temperature) < 0 ) {
  		//Serial.println(F("fail"));  
		return -1;
	}
	return 0;   
}

int send_light()
{
	int tmpVal = analogRead(A1);
	int light = tmpVal/4;

	Serial.print(F("Light : ")); Serial.println(light);
	if ( g_im.send_numdata("light", (double)light) < 0 ){
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
	Serial.print(tagid);Serial.print(F("="));Serial.println(strval);
  
	if ( strcmp(tagid, "airfan")==0 && strcmp(strval, "on")==0 ) {
     	analogWrite(MotorPin, 100); analogWrite(MotorPin2, 0);
	}else if ( strcmp(tagid, "airfan")==0 && strcmp(strval, "off")==0 ){
      	analogWrite(MotorPin, 0);  analogWrite(MotorPin2, 0);
	}
	    
  //식물용 LED
  //if ( strcmp(tagid, "plight")==0 && strcmp(strval, "on")==0 )   
  //      digitalWrite(PlightPin, HIGH);  // turn off relay with voltage LOW
  //else if ( strcmp(tagid, "plight")==0 && strcmp(strval, "off")==0 )   
  //     digitalWrite(PlightPin, LOW);  // turn off relay with voltage LOW

}

void mycb_resp_handler(long long trxid, char *respCode)
{
	if ( strcmp(respCode, "100")==0 )
		Serial.println("resp:OK");
	else
		Serial.println("resp:Not OK"); 
}
