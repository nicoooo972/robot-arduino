#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#ifndef STASSID
#define STASSID "dcrobot" // nom de votre réseau
#define STAPSK "dcrobot24" // Mdp de votre réseau
#endif

const char* ssid = STASSID;
const char* password = STAPSK;
const char* host = "10.43.122.49"; 
uint16_t port = 8080; 

float coefG = 125;
float coefD = 120;
int distance = 0;
long startTime = 0;
long duree = 0;
bool obstacle = false;
bool movementActive = false;
char cmd[7];


WiFiClient client;

// Sensor
#define PIN_TRIG 4 // D2
#define PIN_ECHO 2 // D4

ESP8266WebServer server(80);
WiFiServer tcpServer(port);

const int  rightBack = 14; // D5
const int rightFoward = 16; // D0
const int leftFoward = 13; // D7
const int leftBack = 5; // D1
const int led = 12; //D6

void avancer(int coef);
void reculer(int coef);
void gauche(int coef);
void droite(int coef);
void demiTour();
void on();
void off();
void puissancePlusGauche();
void puissancePlusDroit();
void puissanceMoinsGauche();
void puissanceMoinsDroit();
void stop();
void updateCoefG();
void updateCoefD();
void sendAcknowledge(WiFiClient& client, const String& command);
void sendMovementFinished(WiFiClient& client);
void testCollision();
void timeToStop();
bool isObstacleDetected();

void setup() {
  pinMode(leftFoward, OUTPUT);
  pinMode(rightFoward, OUTPUT);
  pinMode(rightBack, OUTPUT);
  pinMode(leftBack, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  digitalWrite(led, LOW); // Set default to OFF
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  Serial.begin(115200);
  delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  server.begin();
  tcpServer.begin();
  Serial.println("Server started");
}

void loop() {
  if (client.connected()) {
    if (client.available() >= 6) {
      for (int i = 0; i < 6; i++) {
        cmd[i] = client.read();
      }
      cmd[6] = '\0';

      String command = String(cmd);
      command.trim();
      Serial.print("Received command: ");
      Serial.println(command);
      client.print(command);
      processCommand(command);
    }
  } else {
    if (client.connect(host, port)) {
      Serial.println("Connected");
      digitalWrite(LED_BUILTIN, LOW); 
    }
  }

  timeToStop();

  delay(10);
}

void processCommand(const String& command) {
  int param = command.length() > 4 ? command.substring(4).toInt() : 0;
  Serial.print(param);
  Serial.println("");
  String baseCommand = command.substring(0, 3);
  if (baseCommand == "AVA") {
    avancer(param + 1);

  } else if (baseCommand == "REC") {
    reculer(param + 1);

  } else if (baseCommand == "GAU") {
    gauche(param);

  } else if (baseCommand == "DRO") {
    droite(param);

  } else if (baseCommand == "MVT") {
    on();

  } else if (baseCommand == "STO") {
    off();

  } else if (baseCommand == "DEM") {
    demiTour();

  }
}

  void testCollision() {
    if (isObstacleDetected()) {
      obstacle = true;
    }
  }

void timeToStop() {
  if (duree > 0)
  {
    if ((millis() - startTime) >= duree) {
      stop();
      movementActive = false;
      duree = 0;
      client.print("FIN\n");
    }
  }
  obstacle = false;
}

void on() {
  Serial.println("ON");
  digitalWrite(led, HIGH);
}

void off() {
  Serial.println("OFF");
  digitalWrite(led, LOW);
}

bool isObstacleDetected() {
    // Start a new measurement:
  digitalWrite(PIN_TRIG, HIGH); // Envoie une impulsion haute de 10 microsecondes
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW); // Arrête l'impulsion

  // Read the result:
  int duration = pulseIn(PIN_ECHO, HIGH); // Mesure la durée pendant laquelle la broche echo reste haute
  distance = duration / 58;
  Serial.print("Distance en CM: ");
  Serial.println(distance);

  //if (distance < 15) {
 //   return true;
 // }
  return false;
}

void avancer(int coef) {
  Serial.println("AVANCER: coefG = " + String(coefG) + ", coefD = " + String(coefD) + ", coef = " + String(coef));
  
  analogWrite(leftFoward, coefG);
  analogWrite(rightFoward, coefD);
  digitalWrite(rightBack, LOW);
  digitalWrite(leftBack, LOW);

  duree = coef * 300;
  startTime = millis();
  Serial.println("J'avance pour " + String(duree) + " ms");
}

void reculer(int coef) {
  Serial.println("RECULER: coefG = " + String(coefG) + ", coefD = " + String(coefD));

  analogWrite(leftFoward, 0);
  analogWrite(rightFoward, 0);
  analogWrite(rightBack, coefD);
  analogWrite(leftBack, coefG);

  duree = coef * 300;
  startTime = millis();
}

void gauche(int coef) {
  Serial.println("DROITE: coefG = " + String(coefG) + ", coefD = " + String(coefD));

  if (coef == 1 || coef > 2) {
    analogWrite(leftFoward, 0);
    analogWrite(rightFoward, coefD);
    analogWrite(rightBack, 0);
    analogWrite(leftBack, coefG);

      duree = 300;
      Serial.print(duree);
      startTime = millis();

    } else if (coef == 2) {
      demiTour();
  } 
} 

void droite(int coef) {
    Serial.println("GAUCHE: coefG = " + String(coefG) + ", coefD = " + String(coefD));


  if (coef == 1 || coef > 2) {
    analogWrite(leftFoward, coefG);
    analogWrite(rightFoward, 0);
    analogWrite(rightBack, coefD);
    analogWrite(leftBack, 0);

    duree = 750;
    startTime = millis();
  } else if (coef == 2) {
    demiTour();
  }
  
}

void demiTour() {
  Serial.println("DEMI TOUR");
  Serial.print(coefG);
  Serial.println("");
  Serial.print(coefD);

  analogWrite(leftFoward, coefG);
  analogWrite(rightFoward, 0);
  analogWrite(rightBack, coefD);
  analogWrite(leftBack, 0);

  duree = 1200;
  startTime = millis();
}

void puissancePlusGauche() {
  if (coefG < 255 ) {
    coefG += 63.75;
    if (coefG > 255) coefG = 255;
  }
}

void puissanceMoinsGauche() {
  if (coefG >= 63.75) {
    coefG -= 63.75;
    if (coefG < 0) coefG = 63.75;
  }
}

void puissancePlusDroit() {
  if (coefD < 255 ) {
    coefD += 63.75;
    if (coefD > 255) coefD = 255;
  }
}

void puissanceMoinsDroit() {
  if (coefD >= 63.75) {
    coefD -= 63.75;
    if (coefD < 0) coefD = 63.75;
  }
}

void stop() {
  digitalWrite(leftFoward, LOW);
  digitalWrite(rightFoward, LOW);
  digitalWrite(rightBack, LOW);
  digitalWrite(leftBack, LOW);
}

void updateCoefG() {
  if (server.hasArg("value")) {
    coefG = server.arg("value").toFloat();
    Serial.print("Updated coefG: ");
    Serial.println(coefG);
  }
}

void updateCoefD() {
  if (server.hasArg("value")) {
    coefD = server.arg("value").toFloat();
    Serial.print("Updated coefD: ");
    Serial.println(coefD);
  }
}
