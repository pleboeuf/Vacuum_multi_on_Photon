/*
 * Project Vacuum_multi_on_Photon
 * Description: Test du circuit imprimé pour le capteur de vide multilignes
 * Author: P. Leboeuf
 * Date: 24 oct. 2017
 */

#include "Particle.h"
#include "math.h"
#define FirmwareVersion "0.0.3"     // Version of this firmware.
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time; //compilation date and time (UTC)

// General definitions
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define myEventName "dev2_Data"     // Name of the event to be sent to the cloud
#define TimeBoundaryOffset 0        // Wake at time boundary plus some seconds
#define sleepTimeInMinutes 1        // Duration of sleep for test
#define NUMSAMPLES 5                // Number of readings to average to reduce the noise

// Thermistor parameters and variable definition
#define TEMPERATURENOMINAL 25       // Ref temperature for thermistor
#define BCOEFFICIENT 3270           // The beta coefficient at 0 degrees of the thermistor (nominal is 3435 (25/85))
#define SERIESRESISTOR 10000UL      // the value of the resistor in serie with the thermistor
#define THERMISTORNOMINAL 10000UL   // thermistor resistance at 25 degrees C
int thermistorPowerPin = D1;
int thermistorInputPin = A4;
float thermistorRawValue = 0;
float tempDegreeC = 0;

// Light sensor parameters and variable definition
#define LOADRESISTOR 51000UL        // Resistor used to convert current to voltage on the light sensor
int lightSensorPowerPin = D0;
int lightSensorInputPin = A5;
float lightRawValue = 0;
float lightIntensityLux = 0;

// Vacuum transducer parameters and variable definition
#define R1 16900UL                  // pressure xducer scaling resistor 1
#define R2 32400UL                  // Pressure xducer scaling resistor 2
#define Vref 3.3                    // Analog input reference voltage
#define K_fact 0.007652             // Vacuum xducer K factor
#define Vs 5.0                      // Vacuum xducer supply voltage
#define Vcc 3.347                   //Analog system reference voltage
#define ResistorScaling (R1 + R2) / R2 // To scale output of 5.0V xducer to 3.3V Electron input
int vacuum5VoltsEnablePin = D5; // Vacuums sensors operate at 5V
int VacuumPins[]      = {A0, A1, A2, A3};
float VacuumRawData[] = {0, 0, 0, 0};
float VacuumInHg[]    = {0, 0, 0, 0};

// Publish string definition
char publishStr[45];

// Declare general variables
int LedPin = D7;
bool LedState = false;
unsigned long loopTime;
unsigned long now;

/* Define a log handler for log messages */
SerialLogHandler logHandler(LOG_LEVEL_WARN, { // Logging level for non-application messages
    { "app", LOG_LEVEL_INFO }                 // Logging level for application messages
});

SYSTEM_MODE(AUTOMATIC);

// setup() runs once, when the device is first turned on.
void setup() {
  pinMode(vacuum5VoltsEnablePin, OUTPUT);        //Put control pins in output mode
  pinMode(lightSensorPowerPin, OUTPUT);
  pinMode(thermistorPowerPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  now = micros();
  loopTime = now;                             // Initialize loop time
}

void loop() {
  //
  LedState = !LedState;                   // Toggle LED to show activity
  digitalWrite(LedPin, LedState);

  /* Turn ON the Thermistor, light sensor and vacuum transducers,  */
  digitalWrite(thermistorPowerPin, true);
  digitalWrite(lightSensorPowerPin, true);
  digitalWrite(vacuum5VoltsEnablePin, true);

  /* Read and log the temperature */
  thermistorRawValue = AverageReadings(thermistorInputPin, NUMSAMPLES, 0);
  tempDegreeC = rawTemp2DegreesC(thermistorRawValue);
  Log.info("Temperature = %.1f°C", tempDegreeC);
    digitalWrite(thermistorPowerPin, false); // Turn OFF the Thermistor

  /* Read and log the ambieant light intensity */
  lightRawValue = AverageReadings(lightSensorInputPin, NUMSAMPLES, 0);
  lightIntensityLux = LightRaw2Lux(lightRawValue);
  Log.trace("lightRawValue = %.0f", lightRawValue);
  Log.info("Light int. = %.0f Lux", lightIntensityLux);
  digitalWrite(lightSensorPowerPin, false); // Turn OFF the light sensor

  delay(20UL); // Wait 20 ms for the vacuum sensors to stabilize
  /* Read and log the vacuum values */
  for (int i = 0 ; i < 4; i++){
    VacuumRawData[i] = AverageReadings(VacuumPins[i], NUMSAMPLES, 0);
    VacuumInHg[i] = VacRaw2kPa(VacuumRawData[i]);
    Log.trace("Vacuum_%d_raw = %.0f", i, VacuumRawData[i] );
    Log.info("Vacuum_%d = %.1f inHg", i, VacuumInHg[i]);
  }
  digitalWrite(vacuum5VoltsEnablePin, false); // Turn OFF the pressure transducers

  // Log the loop time at the warning level (.warn)
  Log.warn("Loop time = %d us\n", micros() - loopTime);
  delay(2000UL);
  loopTime = micros();
}

void publishData() {
  /*
  FuelGauge fuel;
  // Publish voltage and SOC, RSSI, QUALITY. plus Tx & Rx count
  sprintf(publishStr, "%d, %02.4f, %03.3f, %d, %d, %d, %d, %03.3f",
          cycleNumber - 1, fuel.getVCell(), fuel.getSoC(), signalRSSI, signalQuality, deltaTx, deltaRx, aw_time);
  sprintf(publishStr, "%.1f, %.0f, %.1f, %.1f, %.1f, %.1f, %02.4f, %03.3f, %d, %d", tempDegreeC, lightIntensityLux, V
          acuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3], fuel.getVCell(), fuel.getSoC(), signalRSSI, signalQuality);
  Log.info(publishStr);
  Particle.publish(myEventName, publishStr, PRIVATE, NO_ACK);
  start = millis();
  while (millis() - start < 500UL) {
      Particle.process(); // Wait a second to received the time.
  }
  */
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

/* Convert thermistor resistance to °C
 * Use the simplified Steinhart equation
*/
float rawTemp2DegreesC(float RawADC) {
  float resistance;
  /* convert the value to resistance */
  resistance = SERIESRESISTOR / ((4095.0f / RawADC) - 1);
  Log.trace("Rtm = %.0f ohms", resistance);         // Log (trace) intermediate results for debugging
  /* Converst to degrees Celcius */
  double steinhart;
  steinhart = resistance / THERMISTORNOMINAL;       // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to deg. C;
  return steinhart - 0.7;                           // Return the temperature
}

/* Acquire N readings of an analog input and compute the average */
float AverageReadings (int anInputPinNo, int NumberSamples, int interval) {
  float accumulator = 0;
  for (int i=0; i< NumberSamples; i++){
    delay(interval);                                // In case a delay is required between successives readings
    accumulator += analogRead(anInputPinNo);        // Read data from selected analog input and accumulate the readings
  }
  return accumulator / NumberSamples;               // Divide the accumulator by the number of readings
}

/* Convert ADC numerical value to light intensity in Lux for the APDS-9007 chip
 * The resistor LOADRESISTOR is used to convert (Iout) the output current to a voltage
 * that is measured by the analog to digital converter. This chip is logarithmic (base 10).
*/
double LightRaw2Lux (float raw){
  double Iout = (Vcc * raw / 4095.0f) / LOADRESISTOR; // Calculate the chip output current from raw data
  Log.trace("Iout = %.6f A", Iout);                   // Log (trace) intermediate results for debugging
  double Lux = pow( 10.0f, Iout / 0.00001f);          // Compute the value in LUX
  return Lux;
}
