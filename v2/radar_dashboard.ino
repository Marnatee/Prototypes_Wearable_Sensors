#include <Arduino_BMI270_BMM150.h> // Rev2 IMU & Magnetometer Combo
#include <Arduino_HS300X.h>        // Rev2 Temp/Humidity
#include <Arduino_LPS22HB.h>       // Rev2 Barometric Pressure
#include <Arduino_APDS9960.h>      // Rev2 IR/Proximity/Color
#include <PDM.h>                    // Onboard Digital Microphone
#include <DFRobot_HumanDetection.h> // C1001 Radar Driver

DFRobot_HumanDetection_Detailed radar(&Serial1);

// Hardware IO Pin Mapping
const int FALL_PIN = 2;         // Connected to C01
const int PRESENCE_PIN = 3;     // Connected to C02

// Audio Tracking Buffer
short sampleBuffer[256];
volatile int samplesRead = 0;

void setup() {
  Serial.begin(115200);   // To PC via USB Cable
  Serial1.begin(115200);  // To C1001 Radar Pins D0(RX)/D1(TX)
  
  while (!Serial);

  // Initialize Onboard GPIO Fast Switches
  pinMode(FALL_PIN, INPUT);
  pinMode(PRESENCE_PIN, INPUT);

  // Initialize Rev2 Core Sensors
  if (!IMU.begin()) while(1);
  if (!HS300x.begin()) while(1);
  if (!BARO.begin()) while(1);
  if (!APDS.begin()) while(1);

  // Configure Infrared/Proximity Engines
  APDS.enableProximity();
  APDS.enableColor();

  // Initialize PDM Audio Driver
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) while(1);

  // Initialize Radar over Serial1
  while (!radar.begin()) { delay(1000); }
  radar.configWorkMode(DFRobot_HumanDetection_Detailed::eFallMode);
}

void loop() {
  // 1. Grab IMU Accelerometer Forces
  float ax = 0, ay = 0, az = 0;
  if (IMU.accelerationAvailable()) { IMU.readAcceleration(ax, ay, az); }

  // 2. Grab Magnetometer vector coordinates
  float mx = 0, my = 0, mz = 0;
  if (IMU.magneticFieldAvailable()) { IMU.readMagneticField(mx, my, mz); }

  // 3. Climate Calculations
  float temp = HS300x.readTemperature();
  float hum  = HS300x.readHumidity();
  float pres = BARO.readPressure(); // kPa

  // 4. IR proximity values (0-255 scale) & Color Red Intensity
  int proximity = APDS.readProximity();
  int r = 0, g = 0, b = 0;
  if (APDS.colorAvailable()) { APDS.readColor(r, g, b); }

  // 5. Process PDM sound amplitude spikes
  int soundAmp = 0;
  if (samplesRead > 0) {
    for (int i = 0; i < samplesRead; i++) {
      if (abs(sampleBuffer[i]) > soundAmp) soundAmp = abs(sampleBuffer[i]);
    }
    samplesRead = 0;
  }

  // 6. Poll Soft-Radar API Metrics
  radar.track();
  int radarPresence = radar.getExistState();
  int radarMotion   = radar.getMotionState();

  // 7. Read Physical Hardware IO Status Pins Directly
  int pinFall = digitalRead(FALL_PIN);
  int pinPresence = digitalRead(PRESENCE_PIN);

  // 8. Construct Updated 16-Value Data Frame to USB Out
  // Format: AX,AY,AZ,MX,MY,MZ,SOUND,TEMP,HUM,PRES,PROX,RED,RADAR_P,RADAR_M,PIN_FALL,PIN_PRES
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(mx); Serial.print(",");
  Serial.print(my); Serial.print(",");
  Serial.print(mz); Serial.print(",");
  Serial.print(soundAmp); Serial.print(",");
  Serial.print(temp); Serial.print(",");
  Serial.print(hum); Serial.print(",");
  Serial.print(pres); Serial.print(",");
  Serial.print(proximity); Serial.print(",");
  Serial.print(r); Serial.print(",");
  Serial.print(radarPresence); Serial.print(",");
  Serial.print(radarMotion); Serial.print(",");
  Serial.print(pinFall); Serial.print(",");
  Serial.println(pinPresence);

  delay(50); // Steady 20Hz refresh interval
}

// Microsecond sound collection interrupt
void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}
