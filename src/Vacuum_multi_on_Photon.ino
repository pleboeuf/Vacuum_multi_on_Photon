/*
* Project Vacuum_Multilignes_Electron
* Description: Code running on an Electron on the "Capteur de vide multilignes"
* Author: Pierre Leboeuf
* Date: 26 octobre 2017
*/

/*
*** Notes about sleep ***
Sleep mode: Stop mode + SLEEP_NETWORK_STANDBY, use SETUP BUTTON (20) to wake
Publish: NO_ACK
Delay loop: 500 ms for publish and print
Sleep duration: See #define SLEEPTIMEinMINUTES
*/

//Dev name      No de lignes
// VA1-4     4  Lignes A1 à A4
// VA5B1-2   3  Lignes A5, B1 et B2
// VC1-3     3  Lignes C1 à C3
// VC4-6     3  Lignes C4 à C6
// VC7-8     2  Lignes C7 et C8
// VD1A-2B   4  Lignes D1A, D1B, D2A et D2B
// VE1-3     3  Lignes E1 à E3
// VE4-6     3  Lignes E4 à E6
// VE7-9     3  Lignes E7 à E9
// VE10-12   3  Lignes E10 à E12
// VF1-3     3  Lignes F1 à F3
// VF4-6     3  Lignes F4 à F6
// VF7-9     3  Lignes F7 à F9
// VF10-12   3  Lignes F10 à F12
// VF13-16   4  Lignes F13 à F16
// VG1-2-H14 3  Lignes G1, G2 et H14
// VG3-5     3  Lignes G3 à G5
// VG6-8     3  Lignes G6 à G8
// VG9-12    4  Lignes G9 à G12
// VH2-4     3  Lignes H2 à H4
// VH5-7     3  Lignes H5 à H7
// VH8-10    3  Lignes H8 à H10
// VH11-13   3  Lignes H11 à H13

#include "Particle.h"
#include "math.h"
#include "photon-thermistor.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// General definitions
#define FirmwareVersion "0.1.0"               // Version of this firmware.
String thisDevice = "PL-Ph-A";

String myEventName = "PL-Ph-A_Test_Multilignes";   // Name of the event to be sent to the cloud
String F_Date  = __DATE__;
String F_Time = __TIME__;
String FirmwareDate = F_Date + " " + F_Time;  //compilation date and time (UTC)

#define SLEEPTIMEinMINUTES 1                  // *** Duration of sleep for test ***
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define TimeBoundaryOffset 0                  // Wake at time boundary plus some seconds
#define NUMSAMPLES 5                          // Number of readings to average to reduce the noise
#define SAMPLEsINTERVAL 20UL                  // Interval of time between samples in ms
#define VacuumPublishLimits 1                 // Minimum vacuum required to permit publish( 1 always publish, -1: publish only if vacuum)#define BLUE_LED  D7                          // Blue led awake activity indicator
#define BLUE_LED  D7                          // Blue led awake activity indicator

// wakeupPin definition
#define wakeupPin  D2

// Thermistor parameters and variable definitions
#define TEMPERATURENOMINAL 25                 // Ref temperature for thermistor
#define BCOEFFICIENT 3470                     // The beta coefficient at 0 degrees of the thermistor (nominal is 3435 (25/85))
#define SERIESRESISTOR 10000UL                // the value of the resistor in serie with the thermistor
#define THERMISTORNOMINAL 10000UL             // thermistor resistance at 25 degrees C

Thermistor *thermistor;
int thermistorPowerPin = D1;
int thermistorInputPin = A4;
float tempDegreeC = 0;
int minPublishTemp = 5;

// Light sensor parameters and variable definitions
#define LOADRESISTOR 51000UL                  // Resistor used to convert current to voltage on the light sensor
int lightSensorPowerPin = D0;
int lightSensorInputPin = A5;
float lightIntensityLux = 0;

// Vacuum transducer parameters and variable definitions
#define R1 16900UL                            // pressure xducer scaling resistor 1
#define R2 32400UL                            // Pressure xducer scaling resistor 2
#define Vref 3.3                              // Analog input reference voltage
#define K_fact 0.007652                       // Vacuum xducer K factor
#define Vs 5.0                                // Vacuum xducer supply voltage
#define Vcc 3.3                               //Analog system reference voltage
#define ResistorScaling (R1 + R2) / R2        // To scale output of 5.0V xducer to 3.3V Electron input

#define NVac 4                                // Number of vacuum sensors
int vacuum5VoltsEnablePin = D6;               // Vacuums sensors operate at 5V
int VacuumPins[] = {A0, A1, A2, A3};
float minActVac = 100;
static float VacuumInHg[] = {0, 0, 0, 0};         // Vacuum scaled data arra

// Cellular signal and data variables definitions
int signalRSSI;
int signalQuality;
int      txPrec   = 0;                        // Previous tx data count
int      rxPrec   = 0;                        // Previous rx data count
int      deltaTx  = 0;                        // Difference tx data count
int      deltaRx  = 0;                        // Difference rx data count
uint32_t start    = 0;
uint32_t w_time   = 0;                        // Wakeup time in ms
uint32_t s_time   = 0;                        // Go to sleep time in ms
uint32_t aw_time  = 0;                        // Awake time in sec
retained int lastDay     = 0;

unsigned long lastSync = millis();
char publishStr[80];
retained uint32_t noSerie;                             // Le numéro de série est généré automatiquement
time_t newGenTimestamp = 0;

// PMIC pmic;
/* Define a log handler on Serial for log messages */
SerialLogHandler logHandler(LOG_LEVEL_WARN, {   // Logging level for non-application messages
  { "app", LOG_LEVEL_TRACE }                            // Logging level for application messages
});
/* Define a log handler on Serial1 for log messages */
Serial1LogHandler log1Handler(115200, LOG_LEVEL_WARN, {   // Logging level for non-application messages
  { "app", LOG_LEVEL_TRACE }                            // Logging level for application messages
});


void setup() {
  Time.zone(-4);
  pinMode(vacuum5VoltsEnablePin, OUTPUT);              // Put all control pins in output mode
  pinMode(lightSensorPowerPin, OUTPUT);
  pinMode(thermistorPowerPin, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(wakeupPin, INPUT_PULLUP);
  pinMode(D5, INPUT);   // Only required for old PCB design
  // pmic.setChargeVoltage(4112);                         // Set charge to more than 80%
  thermistor = new Thermistor(thermistorInputPin, SERIESRESISTOR, 4095, THERMISTORNOMINAL, TEMPERATURENOMINAL, BCOEFFICIENT, NUMSAMPLES, 20UL);
  w_time = millis();
  Particle.connect();                                  // This imply Cellular.on() and Cellular.connect()
  waitUntil(Particle.connected);
  Particle.variable("PubMinTemp", minPublishTemp);
  Particle.function("set", remoteSet);
  Log.trace("Cloud connedted!");
  // readCellularData("Initial data \t", TRUE);
  newGenTimestamp = Time.now();
  Log.trace("Setup Completed");
}


void loop() {
  if (Time.day() != lastDay || Time.year() < 2000){    // a new day calls for a sync
     Log.trace("Sync time");
     if(waitFor(Particle.connected, 1 * 60000UL)){
       Particle.syncTime();
       start = millis();
       while (millis() - start < 1000UL) {
         delay (10);
         Particle.process(); // Wait a second to received the time.
       }
       lastDay = Time.day();
       Log.trace("Sync time completed");
     }
  }
  tempDegreeC = readThermistor(NUMSAMPLES, 1);            // First check the temperature

  if (tempDegreeC >= (float)minPublishTemp){
    lightIntensityLux = readLightIntensitySensor();       // Then light intensity
    readVacuums();                                        // Finally red the 4 vacuum transducers
    // readCellularData("Before publish \t", TRUE);          // Read amount of data sent during previous cycle
    // checkSignal();                                        // Read cellular signal strength and quality
    s_time = millis();                                    // Sleep time is now
    aw_time = s_time - w_time;                            // Time the system is awake

    if (minActVac <= VacuumPublishLimits){
      publishData();                                        // Publish a message indicating battery status
    } else {
      Log.trace("Nothing to publish!");
    }
    Log.trace("Going to sleep at: %d\n", s_time);
  }

  digitalWrite(BLUE_LED, false);                         // Turn off blue activity indicator before sleep
  // sleeps duration corrected to next time boundary + TimeBoundaryOffset seconds
  // wake at next time boundary + TimeBoundaryOffset seconds
  uint32_t dt = (SLEEPTIMEinMINUTES - Time.minute() % SLEEPTIMEinMINUTES) * 60 - Time.second() + TimeBoundaryOffset;
  // System.sleep(wakeupPin, FALLING, dt, SLEEP_NETWORK_STANDBY); // Press wakup BUTTON to awake
  delay (SLEEPTIMEinMINUTES * 60000UL);
  digitalWrite(BLUE_LED, true);                           // Turn on blue activity indicator on wakup
  w_time = millis();
  Log.trace("Wake up at: %d", w_time);                   // Log wakup time
}

void publishData() {
  // FuelGauge fuel;
  // Format à utiliser en prod.
  Particle.publish(myEventName,
    makeJSON(noSerie, newGenTimestamp, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3], tempDegreeC, lightIntensityLux, 80.0, 3.7432),
    PRIVATE, NO_ACK);
    Log.trace("incrementing noSerie now");
    noSerie++;
    for(int i=0; i<200; i++) {
      Particle.process();
      delay(10);
    }
}

/* Read and log the temperature */
float readThermistor(int NSamples, int interval){
  // float thermistorRawValue;
  digitalWrite(thermistorPowerPin, true);     // Turn ON thermistor Power
  delay(5UL);                                 // Wait for voltage to stabilize
  float sum = 0;
  for (int i=0; i< NSamples; i++){
    delay(interval);                          // Delay between successives readings
    sum += thermistor->readTempC();           // Read temperature and accumulate the readings
  }
  digitalWrite(thermistorPowerPin, false);    // Turn OFF thermistor
  float temp = sum / NSamples;                // Average the readings
  Log.info("Temperature = %.1f°C", temp);     // Log final value at info level
  return temp;
}

// Read and average vacuum readings
float readVacuums(){
  float VacuumRawData[] = {0, 0, 0, 0};                                     // Vacuum raw data array
  digitalWrite(vacuum5VoltsEnablePin, true);                                // Turn ON the vacuum trasnducer
  minActVac = 100;                                                          // Reset min to a high value initially
  delay(100UL);                                                              // Wait 25 ms for the vacuum sensors to stabilize
  /* Read and log the vacuum values for the four (4) sensors */
  for (int i = 0 ; i < NVac; i++){
    VacuumRawData[i] = AverageReadings(VacuumPins[i], NUMSAMPLES, 0);       // Average multiple raw readings to reduce noise
    Log.trace("Vacuum_%d_raw = %.0f", i, VacuumRawData[i]);                 // Log raw data at trace level for debugging
    VacuumInHg[i] = VacRaw2inHg(VacuumRawData[i]);                          // Convert to inHg
    if (VacuumInHg[i] < -30) {VacuumInHg[i] = 0;};
    Log.info("Vacuum_%d = %.1f inHg", i, VacuumInHg[i]);                    // Log final value at info level
    minActVac = min(minActVac, VacuumInHg[i]);                              // Find the minimum readings of all sensors
  }
  Log.info("Min Vacuum readings %.1f inHg", minActVac);                    // Log final value at info level
  digitalWrite(vacuum5VoltsEnablePin, false);                               // Turn OFF the pressure transducers
}

/* Convert ADC raw value to Kpa or inHg
* From MPXV6115V6U datasheet
* Vout = Vs * (K_fact * Vac + 0.92)
* where K_fact = 0.007652
* thus : Vac_kpa = (Vout / Vs * K_fact) - (0,92 / K_fact)
* and Vout = (Vref*Vraw/4095UL)*(r1+r2)/r2
* To convert kpa to Hg (inch of mercury) multiply by 0.295301
*/
double VacRaw2inHg(float raw) {
  double Vout = (Vref * raw / 4095.0f) * (R1 + R2) / R2;      // Vout = Vref*Vraw*(r1_+r2_)/(4096*r2_)
  double Vac_kpa = (Vout/(K_fact*Vs)) - 0.92/K_fact;  // Vac_kpa = (Vout-(Vs-0,92))/(Vs*k)
  double Vac_inHg = Vac_kpa * 0.2953001;
  return Vac_inHg;                              // multiplie par 0.295301 pour avoir la valeur en Hg
}

/* Read and log the ambieant light intensity */
float readLightIntensitySensor(){
  float lightRawValue;
  float lightIntensity;
  digitalWrite(lightSensorPowerPin, true);                                  // Turn ON the light sensor Power
  lightRawValue = AverageReadings(lightSensorInputPin, NUMSAMPLES, 10);     // Average multiple raw readings to reduce noise
  Log.trace("lightRawValue = %.0f", lightRawValue);                         // Log raw data at trace level for debugging
  lightIntensity = LightRaw2Lux(lightRawValue);                             // Convert to Lux
  Log.info("Light int. = %.0f Lux", lightIntensity);                        // Log final value at info level
  digitalWrite(lightSensorPowerPin, false);                                 // Turn OFF the light sensor
  return lightIntensity;
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

/* Acquire N readings of an analog input and compute the average */
float AverageReadings (int anInputPinNo, int NumberSamples, int interval) {
  float accumulator = 0;
  for (int i=0; i< NumberSamples; i++){
    delay(interval);                                // In case a delay is required between successives readings
    accumulator += analogRead(anInputPinNo);        // Read data from selected analog input and accumulate the readings
  }
  return accumulator / NumberSamples;               // Divide the accumulator by the number of readings
}

// Formattage standard pour les données sous forme JSON
String makeJSON(uint32_t numSerie, uint32_t timeStamp, float va, float vb, float vc, float vd, float temp, float li, float soc, float volt){
    char publishString[135];
    sprintf(publishString,"{\"noSerie\": %lu,\"generation\": %lu,\"va\":%.1f,\"vb\":%.1f,\"vc\":%.1f,\"vd\":%.1f,\"temp\":%.1f,\"li\":%.0f,\"soc\":%.1f,\"volt\":%.3f}",
                              numSerie, newGenTimestamp, VacuumInHg[0], VacuumInHg[1], VacuumInHg[2], VacuumInHg[3], tempDegreeC, lightIntensityLux, soc, volt);
    Log.trace("makeJSON: %s", publishString);
    return publishString;
}

// Read cellular data and substract from the previous cycle
// void readCellularData(String s, bool prtFlag) {
//   CellularData data;
//   if (!Cellular.getDataUsage(data)) {
//          //Log.warn("Error! Not able to get Cellular data.");
//   }
//   else {
//        deltaTx = data.tx_session - txPrec;
//        deltaRx = data.rx_session - rxPrec;
//        if (prtFlag) {
//        }
//        txPrec = data.tx_session;
//        rxPrec = data.rx_session;
//   }
// }

// Read cellular signal strength and quality
// void checkSignal() {
//   CellularSignal sig = Cellular.RSSI();
//   signalRSSI = sig.rssi;
//   signalQuality = sig.qual;
//   String s = "RSSI.QUALITY: \t" + String(signalRSSI) + "\t" + String(signalQuality) + "\t";
// }

// Pour modifier la température de publication par défault
int remoteSet(String command){
  String token;
  String data;
  int sep = command.indexOf(",");
  if (sep > 0){
    token = command.substring(0, sep);
    data = command.substring(sep + 1);
  } else {
    return -1; //Fail
  }

  if (token == "MinPublishTemp"){
    minPublishTemp = data.toInt();
    Log.info("Now publishing at %d deg C", minPublishTemp);
    return 0;
  } else {
    return -1;
  }
}
