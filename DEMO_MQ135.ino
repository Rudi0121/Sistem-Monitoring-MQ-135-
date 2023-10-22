
#include <MQUnifiedsensor.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ThingSpeak.h>
#include <LiquidCrystal_I2C.h>

//Definitions
#define placa "ESP8266"
#define Voltage_Resolution 5
#define pin A0 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 10 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//#define calibration_button 13 //Pin to calibrate your sensor

#define BLYNK_PRINT Serial
#define sensor A0
#define buzzer D0
#define ledRed D0
#define ledBlue D3 
#define BLYNK_TEMPLATE_ID "TMPL6FeyJTtJP"
#define BLYNK_TEMPLATE_NAME "Sistem Monitoring"
#define BLYNK_AUTH_TOKEN "_NJuO08qaYITQkbWdOloKjnVaPXYmK0l"

WiFiClient client;
long myChannelNumber = 2210106;
const char myWriteAPIKey[] = "YRBO23Q5GPOKJWRO";

char auth[] = "_NJuO08qaYITQkbWdOloKjnVaPXYmK0l";
char ssid[] = "AndroidAp";
char pass[] = "qwerty12";

LiquidCrystal_I2C lcd(0x27,16,2);

byte smiley[8] = {
  0b00000,
  0b01010,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};

byte sad[8] = {
  0b00000,
  0b01010,
  0b01010,
  0b00000,
  0b00000,
  0b01110,
  0b10001,
  0b00000
};

BlynkTimer timer;

//Declare Sensor
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

void setup() {
  //Init the serial port communication - to debug the library
  Serial.begin(115200); //Init serial port
  WiFi.begin(ssid, pass);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(500L, sendSensor);
  while(WiFi.status() != WL_CONNECTED)

  pinMode(buzzer, OUTPUT);
  pinMode(pin, INPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  lcd.init(); 
  lcd.backlight();
  lcd.begin(16, 2); 
  lcd.setCursor (0,0);                   
  lcd.print("                ");
  lcd.setCursor (0,1);
  lcd.print("                ");
  lcd.setCursor (0,0);
  lcd.print("  CO2 Meter   ");
  lcd.setCursor (0,0);
  delay(1000);

  lcd.print("Please wait");
  for(int i =0;i<=100;i++)
  {
    lcd.setCursor(12,0);
    if (i<100) lcd.print(" ");
    if (i<10) lcd.print(" ");
    lcd.print(i);
    lcd.print("%");
    delay(500);
  }

  lcd.createChar(0, smiley);
  lcd.createChar(1, sad);
  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ135.init(); 
  MQ135.setRL(10);
  /* 
    //If the RL value is different from 10K please assign your RL value with the following method:
    MQ135.setRL(10);
  */
 /*****************************  MQ CAlibration ********************************************/ 
  // Explanation: 
  // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
  // and on clean air (Calibration conditions), setting up R0 value.
  // We recomend executing this routine only on setup in laboratory conditions.
  // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  Serial.println("  done!.");
  Serial.println("NodeMCU is connected!");
  Serial.println(WiFi.localIP());
  ThingSpeak.begin(client);
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  Serial.println("** Values from MQ-135 ****");
  Serial.println("   CO2  ");  
}

void sendSensor() {
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin 
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  ThingSpeak.writeField(myChannelNumber, 1, CO2+418, myWriteAPIKey);

  Serial.print("|   "); Serial.print(CO2+418); 
  // Note: 400 Offset for CO2 source: https://github.com/miguel5612/MQSensorsLib/issues/29
  Serial.print("   |   "); Serial.print(CO2); 
  Serial.println("   |"); 

  lcd.setCursor(0,0);
  lcd.print("RL :            ");
  lcd.setCursor(4,0);
  lcd.print(CO2);
  lcd.setCursor(0,1);
  lcd.print("Co2: ");
  lcd.setCursor(4,1);
  lcd.print(CO2+418); 
  lcd.setCursor(10,1);
  lcd.print(" ppm ");
  Blynk.virtualWrite(V2, CO2+418);

  if ((CO2+418 >=0)&&(CO2+418 <=1024))
 {
   lcd.write(byte(0));
   delay(1000);
   digitalWrite(buzzer,LOW);
   digitalWrite(ledBlue, HIGH);
   delay(500);
   digitalWrite(ledBlue, LOW);
   delay(1000);
   noTone(buzzer);
 } 

 else if ((CO2+418 >=1025)&&(CO2+418 <=2024))
 {
   Blynk.logEvent("alert_bad"); //"Warning! CO2 level is Bad!");
   lcd.write(byte(1));
   delay(1000);
   digitalWrite(ledBlue, LOW);
   digitalWrite(buzzer,HIGH);
   delay(1000);
   digitalWrite(buzzer,LOW);
   delay(1000);
   digitalWrite(ledRed, HIGH);
   delay(1000);
   digitalWrite(ledRed, LOW);
   delay(1000);
 }

 else 
 {
   Blynk.logEvent("alert_danger"); //"Warning! CO2 level is Danger!");
   lcd.setCursor(0,1);
   lcd.print("Diluar Jangkauan ");
   digitalWrite(ledBlue, LOW);
   digitalWrite(buzzer,HIGH);
   delay(1000);
   digitalWrite(buzzer,LOW);
   delay(1000);
   digitalWrite(ledRed, HIGH);
   delay(1000);
   digitalWrite(ledRed, LOW);
   delay(1000);
  }

    /*Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Toluen  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Aceton  | 34.668 | -3.369
  */
  
  //Sampling frequency
}

void loop() {
  Blynk.run();
  timer.run();
  delay(1000);
}
