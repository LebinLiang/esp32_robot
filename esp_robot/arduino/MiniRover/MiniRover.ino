#include <WiFi.h>
#include <Wire.h>
#include "Pins.h"
#include "Camera.h"
#include "Motor.h"
#include "Servo_Motor.h"

const char* ssid = "test1";
const char* password = "12345678";

WiFiServer server(81);

//Motor motor_L, motor_R;

Servo_Motor motor_L, motor_R;
// OV2640 camera
Camera ov2640;

// MPU6050 accelerator and gyroscope

void setup()
{
  motor_L.initialize(2,5);
  motor_R.initialize(4,6);
  //motor_L.initialize(2, 5, 4, 6);
  //motor_R.initialize(15, 7, 13, 8);
  motor_L.setPwmDuty(-1);
  motor_R.setPwmDuty(-1);

	Serial.begin(115200);

	ov2640.initialize();

	int n = WiFi.scanNetworks();
	Serial.println("scan done");
	if (n == 0)
	{
		Serial.println("no networks found");
	}
	else
	{
		Serial.print(n);
		Serial.println(" networks found");
		for (int i = 0; i < n; ++i)
		{
			// Print SSID and RSSI for each network found
			Serial.print(i + 1);
			Serial.print(": ");
			Serial.print(WiFi.SSID(i));
			Serial.print(" (");
			Serial.print(WiFi.RSSI(i));
			Serial.print(")");
			Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
			delay(10);
		}
	}

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED)
	{
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected");

	ov2640.startCameraServer();

	Serial.print("Camera Ready! Use 'http://");
	Serial.print(WiFi.localIP());
	Serial.println("' to connect");

	server.begin();
}


float i = -1;
long heart_beat = 0;
void loop()
{

	WiFiClient client = server.available();   // listen for incoming clients
  
  if (client)
  {
    // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected())
    {            // loop while the client's connected

      
      if (client.available())
      {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n')
        {
          String angle_val = getValue(currentLine, ':', 0);
          String power_val = getValue(currentLine, ':', 1);
          int angle = atoi(angle_val.c_str());
          int power = atoi(power_val.c_str());

          Serial.print(angle);
          Serial.print(",");
          Serial.println(power);

          currentLine = "";
          client.println("qwertty");
        }
        else if (c != '\r')
        {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
  }

}


String getValue(String data, char separator, int index)
{
	int found = 0;
	int strIndex[] = { 0, -1 };
	int maxIndex = data.length() - 1;

	for (int i = 0; i <= maxIndex && found <= index; i++)
	{
		if (data.charAt(i) == separator || i == maxIndex)
		{
			found++;
			strIndex[0] = strIndex[1] + 1;
			strIndex[1] = (i == maxIndex) ? i + 1 : i;
		}
	}
	return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
