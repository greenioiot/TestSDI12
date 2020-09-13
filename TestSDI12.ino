/**
   @file f_basic_data_request.ino
   @copyright (c) 2013-2020 Stroud Water Research Center (SWRC)
                            and the EnviroDIY Development Team
              This example is published under the BSD-3 license.
   @author Ruben Kertesz <github@emnet.net> or @rinnamon on twitter
   @date 2/10/2016

   @brief Example F: Basic Data Request to a Single Sensor

   This is a very basic (stripped down) example where the user initiates a measurement
   and receives the results to a terminal window without typing numerous commands into
   the terminal.

   Edited by Ruben Kertesz for ISCO Nile 502 2/10/2016
*/

#include "BluetoothSerial.h"
#define _TASK_TIMECRITICAL

#include <TaskScheduler.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "HardwareSerial_NB_BC95.h"

#include <SDI12.h>

#define SERIAL_BAUD 115200 /*!< The baud rate for the output serial port */
#define DATA_PIN 13         /*!< The pin of the SDI-12 data bus */
#define POWER_PIN -1       /*!< The sensor power pin (or -1 if not switching power) */
#define SENSOR_ADDRESS 0
#define DRY_CONTACT_PIN  25

String deviceToken = "TvF7UQ52NR9o2LM3gXRA";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "9956"; // Your Server Port;
String json = "";
String udpData = "";
boolean stateGetWaterLevel = 0;
int prevStage = 0;
int rainGate = 0;

String waterLevel = "";

int reading;                // ค่าที่อ่านได้จากปุ่มกด (รวม bounce)
int counter = 0;            // จำนวน iteration ที่เราเห็นการกด หรือปล่อย
int current_state = LOW;    // ค่าที่ได้หลังการทำ debounce (คือการกดหรือปล่อยจริงๆ)
long timeLoop = 0;              // เวลาล่าสุดที่มีการเก็บค่า
int debounce_count = 2;    // จำนวนมิลลิวินาที/รอบการวนลูป ที่ต่ำสุดที่เชื่อได้ว่ามีการกด หรือปล่อยจริงๆ
int rainCount = 0;       // ไว้แสดงจำนวนการกด

HardwareSerial_NB_BC95 AISnb;
signal meta ;

BluetoothSerial SerialBT;
/** Define the SDI-12 bus */
SDI12 mySDI12(DATA_PIN);

String sdiResponse = "";
String myCommand   = "";
String model = "";
String result = "";
String _minusSign = "-";

const long interval = 300000;  //millisecond
unsigned long previousMillis = 0;


const long intervalDrycontact = 1000;  //millisecond
unsigned long previousMillisDrycontact = 0;
void t1CallgetWaterLevel();

void t3CallgetRain();

Scheduler runner;
//TASK
Task t1(60000, TASK_FOREVER, &t1CallgetWaterLevel);

Task t3(50, TASK_FOREVER, &t3CallgetRain);
//



void t1CallgetWaterLevel() {
  getWaterLevel();
}



void t3CallgetRain() {
  checkRainGate();
}

void getModel() {
  Serial.println("start query..");
  myCommand = String(SENSOR_ADDRESS) + "I!";


  Serial.print("cmd:");
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(1000);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }

  if (sdiResponse.length() > 1) {
    Serial.println(sdiResponse);  // write the response to the screen

    model = sdiResponse;

  }
  mySDI12.clearBuffer();


  delay(200);       // delay between taking reading and requesting data


  // next command to request data from last measurement
  myCommand = String(SENSOR_ADDRESS) + "D0!";
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(300);  // wait a while for a response

  while (mySDI12.available()) {  // build string from response
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  if (sdiResponse.length() > 1) {
    Serial.println(sdiResponse);  // write the response to the screen
    model = sdiResponse;
  }
  mySDI12.clearBuffer();


}




void setup() {
  Serial.begin(SERIAL_BAUD);
  SerialBT.begin("hyquest001"); //Bluetooth device name
  SerialBT.println("Hello:hyquest001");
  while (!Serial)
    ;

  Serial.println("Opening SDI-12 bus...");
  mySDI12.begin();
  delay(1000);  // allow things to settle

  // Power the sensors;
  if (POWER_PIN > 0) {
    Serial.println("Powering up sensors...");
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    delay(200);
  }
  Serial.print("devie:");
  Serial.println(SENSOR_ADDRESS);
  SerialBT.print("devie:");
  SerialBT.println(SENSOR_ADDRESS);
  getModel();

  AISnb.debug = true;



  AISnb.setupDevice(serverPort);

  String ip1 = AISnb.getDeviceIP();
  delay(1000);

  pingRESP pingR = AISnb.pingIP(serverIP);
  previousMillis = millis();
  Serial.println(model);
  pinMode(DRY_CONTACT_PIN, INPUT_PULLUP);
  String nccid = AISnb.getNCCID();
  //  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");

  runner.addTask(t3);
  Serial.println("added t3");
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");

  t3.enable();  Serial.println("Enabled t3");

  Serial.print("nccid:");
  Serial.println(nccid);

  SerialBT.println(nccid);
  SerialBT.println("Start..");
}

boolean getResponse() {

}


void getWaterLevel() {
  sdiResponse = "";  // clear the response string

  Serial.println("start query..");
  myCommand = String(SENSOR_ADDRESS) + "M!";


  Serial.print("cmd:");
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(12000);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  Serial.print("  sdiResponse:");
  Serial.println(sdiResponse);  // write the response to the screen

  if (! sdiResponse.indexOf("121") > 0 ) {

    int whereis_ = sdiResponse.indexOf("+");
    Serial.println(whereis_);

    if (whereis_ > 0) {
      Serial.print("  +:");
      waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
      waterLevel = string2float(waterLevel);
      Serial.println(waterLevel);


    } else {
      whereis_ = sdiResponse.indexOf("-");
      if (whereis_ > 0 ) { // check for - value
        Serial.print("  -:");
        waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
        waterLevel = string2float(waterLevel);
        Serial.println(waterLevel);

      }
    }

    mySDI12.clearBuffer();

  } else {


    delay(200);       // delay between taking reading and requesting data
    sdiResponse = "";  // clear the response string


    // next command to request data from last measurement
    myCommand = String(SENSOR_ADDRESS) + "D0!";
    Serial.print("cmd:");
    Serial.println(myCommand);  // echo command to terminal

    mySDI12.sendCommand(myCommand);
    delay(1200);  // wait a while for a response
    //    for (int i = 0; i < 1200000; i++);
    while (mySDI12.available()) {  // build string from response
      char c = mySDI12.read();
      if ((c != '\n') && (c != '\r')) {
        sdiResponse += c;
        delay(5);
      }
    }
    Serial.print("  sdiResponse:");
    Serial.println(sdiResponse);  // write the response to the screen
    if (sdiResponse.length() > 1) {


      int whereis_ = sdiResponse.indexOf("+");
      Serial.println(whereis_);
      if (whereis_ > 0) {
        Serial.print("  +:");
        waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
        waterLevel = string2float(waterLevel);
        Serial.println(waterLevel);



      } else {
        whereis_ = sdiResponse.indexOf("-");
        if (whereis_ > 0 ) { // check for - value
          Serial.print("  -:");
          waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
          waterLevel =  string2float(waterLevel);
          Serial.println(waterLevel);
        }
      }
    }
    mySDI12.clearBuffer();


  }

}


void checkRainGate()
{

  // เช็คทุกมิลลิวินาที
  if (millis() != timeLoop)
  {
    //อ่านค่าปุ่มกด
    reading = digitalRead(DRY_CONTACT_PIN);

    //ถ้าค่าปุ่มกดเท่ากับค่าเดิม ให้ลดการนับคะแนน (อาจเกิดการ bounce หรือไม่มีactionอะไรเกิดขึ้น)
    if (reading == current_state && counter > 0)
    {
      counter--;
    }
    //ถ้าค่าที่อ่านได้ไม่เท่ากับค่าเดิม ให้เพิ่มคะแนนไปเรื่อยๆ
    if (reading != current_state)
    {
      counter++;
    }

    //ถ้าคะแนนมากกว่าค่าที่ตั้งไว้ ก็ให้รีเซ็ตคะแนนและเปลี่ยนสถานะว่ามีการกดหรือปล่อยจริงๆ พร้อมแสดงออกไปยัง LED
    if (counter >= debounce_count)
    {
      //ถ้าเป็นการกดจะแสดงผลจำนวนที่กดไปทาง serial monitor
      if (reading == LOW) {
        rainCount++;
      }

      counter = 0;
      current_state = reading;
      Serial.print(rainCount);
      Serial.print(":");
      Serial.println(current_state);
      SerialBT.print(rainCount);
      SerialBT.print(":");
      SerialBT.println(current_state);

    }
    timeLoop = millis();
  }


}

String string2float(String v) {
  String result = "";
  //+0145.715
  //+0105.715
  Serial.print("    :");
  Serial.println(v);

  if(v.indexOf("-")==0)
    result = _minusSign;

  if (!(  v.startsWith("+0000", 0) == 0  && v.startsWith("-0000", 0)  == 0) ) {
    Serial.println("  5");
    result += "0" + v.substring(5, v.length());
    return result;
  } else if (! ( v.startsWith("+000", 0) == 0  && v.startsWith("-000", 0) == 0 ) ) {
    Serial.println("  4");
    result +=   v.substring(4, v.length());
    return result;
  } else if  (!( v.startsWith("+00", 0) == 0  && v.startsWith("-00", 0) == 0 ) ) {
    Serial.println("  3");
    result += v.substring(3, v.length());
    Serial.print("result:");
    Serial.println(result);
    return result;
  } else if  (! ( v.startsWith("+0", 0) == 0  && v.startsWith("-0", 0) == 0 ) ) {
    Serial.println("  2");
    result += v.substring(2, v.length());
    return result;
  } else if  (! ( v.startsWith("+", 0) == 0 && v.startsWith("-", 0) == 0 ) ) {
    Serial.println("  1");
    result += v.substring(1, v.length());
    return result;
  } else {
    Serial.println("  0");
    result += v.substring(0, v.length());
    return result;
  }


}
void loop() {

  unsigned long currentMillis = millis();


  runner.execute();


  if (currentMillis - previousMillis >= interval) {



    meta = AISnb.getSignal();
    udpData = "{\"Tn\":\"";
    udpData.concat(deviceToken);
    udpData.concat("\",\"level\":");

    udpData.concat(waterLevel);
    udpData.concat(",\"rssi\":");
    udpData.concat(meta.rssi);
    udpData.concat(",\"rain\":");
    udpData.concat(rainCount);

    udpData.concat("}");
    Serial.println(udpData);

    SerialBT.print("send:");
    SerialBT.println(udpData);
    //    if (d.toFloat() != 0) {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, udpData);

    previousMillis = currentMillis;

    Serial.print("udp.status:");
    Serial.println(udp.status);


    if (udp.status) {
      Serial.println("reset counter rain");
      rainCount = 0;
    }

    //    }

    previousMillis = currentMillis;
  }


}
