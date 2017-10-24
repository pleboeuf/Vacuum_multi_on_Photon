/*
 * Project Vacuum_multi_on_Photon
 * Description: Test du circuit imprimé pour le capteur de vide multilignes
 * Author: P. Leboeuf
 * Date: 22 oct. 2017
 */

#include "Particle.h"
#include "math.h"
#define FirmwareVersion "0.0.1"     // Version du firmware du capteur.
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time; //Date et heure de compilation UTC

#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define myEventName "dev2_Data"    // Usage data
#define TimeBoundaryOffset 0        // Wake at time boundary plus some seconds
#define sleepTimeInMinutes 1        // Duration of sleep for test

// Definition for vacuum transducer
#define R1 16900UL                  // pressure xducer scaling resistor 1
#define R2 32400UL                  // Pressure xducer scaling resistor 2
#define Vref 3.3                    // Analog input reference voltage
#define K_fact 0.007652             // Vacuum xducer K factor
#define Vs 5.0                      // Vacuum xducer supply voltage
#define Vcc 3.347                   //Analog system reference voltage
#define ResistorScaling (R1 + R2) / R2 // To scale output of 5.0V xducer to 3.3V Electron input

// Definition for thermistor
#define TEMPERATURENOMINAL 25       // Ref temperature for thermistor
#define BCOEFFICIENT 3270           // The beta coefficient at 0 degrees of the thermistor (nominal is 3435 (25/85))
#define SERIESRESISTOR 10000UL      // the value of the resistor in serie with the thermistor
#define THERMISTORNOMINAL 10000UL   // thermistor resistance at 25 degrees C

// Définition pour le capteur de lumière
#define LOADRESISTOR 51000UL        // Resistor used to convert current to voltage on the light sensor

#define NUMSAMPLES 5                // Number of readings to average to reduce the noise

float Vacuum_0_raw = 0;
float Vacuum_1_raw = 0;
float Vacuum_2_raw = 0;
float Vacuum_3_raw = 0;

int Vacuum_0_Pin = A0;
int Vacuum_1_Pin = A1;
int Vacuum_2_Pin = A2;
int Vacuum_3_Pin = A3;

int lightSensorPowerPin = D0;
int lightSensorInputPin = A5;
float lightRawValue = 0;

int thermistorPowerPin = D1;
int thermistorInputPin = A4;
float thermistorRawValue = 0;

int fiveVoltsEnablePin = D5;

int LedPin = D7;
bool LedState = false;
unsigned long loopTime;
unsigned long now;

SYSTEM_MODE(AUTOMATIC);

// setup() runs once, when the device is first turned on.
void setup() {
  pinMode(fiveVoltsEnablePin, OUTPUT);
  pinMode(lightSensorPowerPin, OUTPUT);
  pinMode(thermistorPowerPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(115200); // Pour débug
  now = micros();
  loopTime = now;
  /*setADCSampleTime(ADC_SampleTime_480Cycles);*/
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // First test the Five volts system by toggling it for 5 sec ON and OFF
  while (true)
  {
      LedState = !LedState; // Toggle LED to show activity
      digitalWrite(LedPin, LedState);

      /* Turn ON the Thermistor, light sensor and vacuum transducers,  */
      digitalWrite(thermistorPowerPin, true);
      digitalWrite(lightSensorPowerPin, true);
      digitalWrite(fiveVoltsEnablePin, true);

      /* Read the temperature */
      thermistorRawValue = AverageReadings(thermistorInputPin, NUMSAMPLES, 0);
      digitalWrite(thermistorPowerPin, false); // Turn OFF the Thermistor

      /* Read the ambieant light intensity */
      lightRawValue = AverageReadings(lightSensorInputPin, NUMSAMPLES, 0);
      digitalWrite(lightSensorPowerPin, false); // Turn OFF the light sensor

      /*Serial.printf(", thermistorRawValue = %.0f", thermistorRawValue);*/
      /* Wait for the vacuum sensors to stabilize */
      delay(20UL);
      Vacuum_0_raw = AverageReadings(Vacuum_0_Pin, NUMSAMPLES, 0);
      Vacuum_1_raw = AverageReadings(Vacuum_1_Pin, NUMSAMPLES, 0);
      Vacuum_2_raw = AverageReadings(Vacuum_2_Pin, NUMSAMPLES, 0);
      Vacuum_3_raw = AverageReadings(Vacuum_3_Pin, NUMSAMPLES, 0);
      digitalWrite(fiveVoltsEnablePin, false); // Turn OFF the pressure transducers

      /*Serial.printf(" Vacuum_0 = %.0f", Vacuum_0_raw);*/
      /*Serial.printf(", Vacuum_1 = %.0f", Vacuum_1_raw);*/
      /*Serial.printf(", Vacuum_2 = %.0f", Vacuum_2_raw);*/
      /*Serial.printf(", Vacuum_3_raw = %.0f", Vacuum_3_raw);*/

      Serial.printf(", Temperature = %.1f", Temperature(thermistorRawValue));
      Serial.printf(", lightRawValue = %.0f", lightRawValue);
      Serial.printf(", Lux = %.0f", LightRaw2Lux(lightRawValue));
      Serial.printf(", Vacuum_0 = %.1f inHg", VacRaw2kPa(Vacuum_0_raw));
      Serial.printf(", Vacuum_1 = %.1f inHg", VacRaw2kPa(Vacuum_1_raw));
      Serial.printf(", Vacuum_2 = %.1f inHg", VacRaw2kPa(Vacuum_2_raw));
      Serial.printf(", Vacuum_3 = %.1f inHg", VacRaw2kPa(Vacuum_3_raw));
      Serial.printlnf(", Loop time = %d", micros() - loopTime);
      delay(2000UL);
      loopTime = micros();
  }
}

/* Fonction de conversion valeur numérique vers pression (vide) en Kpa
 * D'aprés le datasheet du MPXV6115V6U
 * Vout = Vs * (K_fact * Vac + 0.92)
 * ou K_fact = 0.007652
 * soit : Vac_kpa = (Vout / Vs * K_fact) - (0,92 / K_fact)
 * avec Vout = (Vref*Vraw/4095UL)*(r1+r2)/r2
 * pour convertir de kpa to Hg (inch of mercury) il faut multiplier par 0.295301
 */
double VacRaw2kPa(float raw) {
  double Vout = (Vref * raw / 4095.0f) * (R1 + R2) / R2;      // Vout = Vref*Vraw*(r1_+r2_)/(4096*r2_)
  double Vac_kpa = (Vout/(K_fact*Vs)) - 0.92/K_fact;  // Vac_kpa = (Vout-(Vs-0,92))/(Vs*k)
  double Vac_inHg = Vac_kpa * 0.2953001;
  /*Serial.printlnf("Vout= %f, Vac_kpa= %f, Vac_inHg= %f", Vout, Vac_kpa, Vac_inHg);*/
  return Vac_inHg;                              // multiplie par 0.295301 pour avoir la valeur en Hg
}

/* Fonction de conversion resistance vers temperature
 * Utilise l'équation de Steinhart simplifié
*/
float Temperature(float RawADC) {
  float resistance;
  /* convert the value to resistance */
  resistance = SERIESRESISTOR / ((4095.0f / RawADC) - 1);
  Serial.printf("Rtc = %.0f", resistance);

  /* Conversion en degrés Celcius */
  double steinhart;
  steinhart = resistance / THERMISTORNOMINAL;       // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to deg. C;
  return steinhart - 0.7;                           // Return the temperature
}

/* Fonction d'acquisition des entrées analogues avec moyenne */
float AverageReadings (int anInputPinNo, int NumberSamples, int interval) {
  float accumulator = 0;
  for (int i=0; i< NumberSamples; i++){
    delay(interval);
    accumulator += analogRead(anInputPinNo);
  }
  return accumulator / NumberSamples;
}

/* Fonction de conversion valeur numérique vers intensité lumineuse en Lux */

double LightRaw2Lux (float raw){
  double Iout = (Vcc * raw / 4095.0f) / LOADRESISTOR;
  Serial.printf(", Iout = %.6f A", Iout);
  double Lux = pow( 10.0f, Iout / 0.00001f);
  return Lux;
}
