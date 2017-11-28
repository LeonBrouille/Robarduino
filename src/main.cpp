/*
 WiFi Web Server LED Blink

 A simple web server that lets you blink an LED via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to turn on and off the LED on pin 5.

 If the IP address of your shield is yourAddress:
 http://yourAddress/H turns the LED on
 http://yourAddress/L turns it off

 This example is written for a network using WPA encryption. For
 WEP or WPA, change the Wifi.begin() call accordingly.

 Circuit:
 * WiFi shield attached
 * LED attached to pin 5

 created for arduino 25 Nov 2012
 by Tom Igoe

ported for sparkfun esp32


 */
#include <Arduino.h>
#include <WiFi.h>

const char* ssid     = "PEPINIERE";
const char* password = "jw3qv429";

WiFiServer server(80);

#define PIN_MOTOR_RIGHT_UP 26//12**********************************
#define PIN_MOTOR_RIGHT_DN 35 //4
#define PIN_MOTOR_RIGHT_SPEED 34//3

/* left motor control pins */
#define PIN_MOTOR_LEFT_UP 25 //5
#define PIN_MOTOR_LEFT_DN 33//7
#define PIN_MOTOR_LEFT_SPEED 32//6*

unsigned char RightMotor[3] = {PIN_MOTOR_RIGHT_UP, PIN_MOTOR_RIGHT_DN, PIN_MOTOR_RIGHT_SPEED};
unsigned char LeftMotor[3] = {PIN_MOTOR_LEFT_UP, PIN_MOTOR_LEFT_DN, PIN_MOTOR_LEFT_SPEED};

void analogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {

  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * std::min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void Wheel (unsigned char * motor, int v)
{
  uint8_t i;

  if (v>100) v=100;
  if (v<-100) v=-100;
  if (v>0) {
    digitalWrite(motor[0], HIGH);
    digitalWrite(motor[1], LOW);
    for (i=1; i<=v; i++) {
      analogWrite(motor[2], i*2.55);
    }

  } else if (v<0) {
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], HIGH);
    for (i=1; i<=v; i++) {
      analogWrite(motor[2], (-i)*2.55);
    }

  } else {
    digitalWrite(motor[0], LOW);
    digitalWrite(motor[1], LOW);
    analogWrite(motor[2], 0);
  }
}



void setup()
{
  pinMode (PIN_MOTOR_RIGHT_UP, OUTPUT);
  pinMode (PIN_MOTOR_RIGHT_DN, OUTPUT);
  pinMode (PIN_MOTOR_LEFT_UP, OUTPUT);
  pinMode (PIN_MOTOR_LEFT_DN, OUTPUT);//*

    Serial.begin(115200);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();
}


void loop(){

 WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<a href=\"/R\">Reculer</a> <br>");
            client.print("<a href=\"/A\">Avancer</a> <br>");
            client.print("<a href=\"/G\">Tourner &agrave; gauche </a> <br>");
            client.print("<a href=\"/D\">Tourner &agrave droite </a> <br>");
            client.print("<a href=\"/S\">Stop</a> <br>");
            client.print("Youpi !<br>");
            client.print("Sign&eacute; L&eacuteon.<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /R")) {
          Wheel (RightMotor, -100);
          Wheel (LeftMotor, -100);
          delay(1000);
          Wheel (RightMotor, 0);
          Wheel (LeftMotor, 0);
        }
        if (currentLine.endsWith("GET /A")) {
          Wheel (RightMotor, 100);
          Wheel (LeftMotor, 100);
          delay(1000);
          Wheel (RightMotor, 0);
          Wheel (LeftMotor, 0);
        }
        if (currentLine.endsWith("GET /G")) {
          Wheel (RightMotor, 100);
          Wheel (LeftMotor, -100);
          delay(1000);
          Wheel (RightMotor, 0);
          Wheel (LeftMotor, 0);
        }
        if (currentLine.endsWith("GET /D")) {
          Wheel (RightMotor, -100);
          Wheel (LeftMotor, 100);
          delay(1000);
          Wheel (RightMotor, 0);
          Wheel (LeftMotor, 0);
        }
        if (currentLine.endsWith("GET /S")) {
          Wheel (RightMotor, 0);
          Wheel (LeftMotor, 0);
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
