/*******************************************************************
 *  Trafic notifier by Midas Zegers at Technical University Eidnhoven
 *  1-12-2023
 *  This codes uses an example of usisng the distance-matrix api to get            
 *  travel time (with traffic) between two locations. It was written by Written by Brian Lough                                         
 *  and can be found here: https://github.com/witnessmenow/arduino-google-maps-api
 *******************************************************************/

#include <TMCStepper.h>
#include <AccelStepper.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <GoogleMapsApi.h>

int Array[] = {500000, 20000, 100000};
int Arraydelay1[] = {5000, 10000, 15000};  // Delay < 1 minute
int Arraydelay2[] = {1000, 2000, 3000};    // 1 < Delay < 15
int Arraydelay3[] = {100, 200, 400};       // 15 < Delay

// TMC2209
#define EN_PIN_1 13    // Enable
#define DIR_PIN_1 14   // Direction
#define STEP_PIN_1 12  // Step
#define PUMP 33        // Pump

#define SERIAL_PORT Serial   // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f  // Match to your driver
#define MICROSTEPS 16
#define STEPS_PER_REVOLUTION 200 * MICROSTEPS

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);

char ssid[] = "iotroam";       // your network SSID (name)
char password[] = "midas123";  // your network key
#define API_KEY "AIzaSyAeIhq-99NvY1k03LZbRssbulhemGVncWs"  // your google apps API Token

WiFiClientSecure client;
GoogleMapsApi api(API_KEY, client);
unsigned long previousGoogleMapsCheckTime = 0;
const unsigned long googleMapsCheckInterval = 30 d000;  // Interval for checking Google Maps (in milliseconds)

int delaytime = 0;
int randomdelay = 0;
int randomdelay2 = 0;
int durationInTrafficInSeconds;

String origin = "51.4392064,5.4951936";
String destination = "51.9333891,4.4666079";
String departureTime = "now";               // This can also be a timestamp, needs to be in the future for traffic info
String trafficModel = "best_guess";         // defaults to this anyways. see https://developers.google.com/maps/documentation/distance-matrix/intro#DistanceMatrixRequests for more info

void setup() {
  pinMode(EN_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(PUMP, OUTPUT);
  digitalWrite(EN_PIN_1, LOW);

  SERIAL_PORT.begin(115200);

  // Set WiFi to station mode and disconnect from an AP if it was Previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Attempt to connect to Wifi network:
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    client.setInsecure();
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);

  //stepper
  driver.begin();
  driver.toff(5);
  driver.rms_current(500);
  driver.microsteps(MICROSTEPS);
  driver.pwm_autoscale(true);

  stepper1.setMaxSpeed(30000);
  stepper1.setAcceleration(10000);
  stepper1.setPinsInverted(false, false, true);
}

bool checkGoogleMaps() {
//  const int capacity = JSON_OBJECT_SIZE(2) * 3  +  JSON_OBJECT_SIZE(1) * 4 + JSON_OBJECT_SIZE(4) *2;
  Serial.println("Getting traffic for " + origin + " to " + destination);
  String responseString = api.distanceMatrix(origin, destination, departureTime, trafficModel);
//  DynamicJsonDocument jsonBuffer(capacity);
 StaticJsonDocument <1024> jsonBuffer;

//  Serial.println("Received JSON response:");
//  Serial.println(responseString);

  
  DeserializationError response = deserializeJson(jsonBuffer, responseString);
  JsonObject root = jsonBuffer.as<JsonObject>();
    Serial.println(response.c_str());

  if (response == DeserializationError::Ok) {
    String status = root["status"];
    Serial.println("Status : " + status); 
    if (status == "OK") {
      Serial.println("Status : " + status);
      
      // Extract the duration_in_traffic value directly as an integer
      durationInTrafficInSeconds = root["rows"][0]["elements"][0]["duration_in_traffic"]["value"];
      
      // ... (rest of your code)
      
      return true;
    } else {
      Serial.println("Got an error status");
      return false;
    }
  } else {
    Serial.println("Failed to parse Json. Response:");
    Serial.println(responseString);

    if (response == DeserializationError::Ok) {
      Serial.println("No issues with serialization");
    } else {
      Serial.print("Error: ");
      Serial.println(response.c_str());
    }

    return false;
  }
  return false;
}

void loop() {
  int randomIndex = random(0, 3);
  int randomIndex2 = random(0, 3);



  int randomdelayIndex = random(0, 3);
  int randomdelayIndex2 = random(0, 3);

 
  if (stepper1.distanceToGo() == 0) {
      int randomspeed = Array[randomIndex];
  int randomspeed2 = Array[randomIndex2];

   if (durationInTrafficInSeconds < 5100) {
    randomdelay = Arraydelay1[randomdelayIndex];
    randomdelay2 = Arraydelay1[randomdelayIndex2];
  }

  if (durationInTrafficInSeconds < 5100 && delaytime >= 5700) {
    randomdelay = Arraydelay1[randomdelayIndex];
    randomdelay2 = Arraydelay1[randomdelayIndex2];
  }

  if (durationInTrafficInSeconds >= 5700) {
    randomdelay = Arraydelay1[randomdelayIndex];
      randomdelay2 = Arraydelay1[randomdelayIndex2];
  }

    // setting to turn over
    static bool direction = false;

    if (direction) {
      stepper1.setMaxSpeed(randomspeed);
      stepper1.move(3 * STEPS_PER_REVOLUTION);
      delay(randomdelay);
    } else {
      stepper1.setMaxSpeed(randomspeed2);
      stepper1.move(-3 * STEPS_PER_REVOLUTION);
      delay(randomdelay2);
    }
    direction = !direction;
  }

  stepper1.run();
  checkGoogleMapsNonBlocking();
}

void checkGoogleMapsNonBlocking() {
  // Get the current time
  unsigned long currentMillis = millis();
  // Check if the specified interval has passed since the last check
  if (currentMillis - previousGoogleMapsCheckTime >= googleMapsCheckInterval) {
    // Perform Google Maps check
    checkGoogleMaps();
    // Update the last check time
    previousGoogleMapsCheckTime = currentMillis;
    Serial.println("--------------------------");
    Serial.println(durationInTrafficInSeconds);
    Serial.println("--------------------------");
  }
}
