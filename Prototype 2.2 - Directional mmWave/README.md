version 2, [dfRobot C1001] + [Arduino Nano 33 IoT sense Rev2] + [Breadboard sleeve] + [Breadboard stand]

Still missing a PDF explaining the goals of this prototype.
Additionally, the code files are very rough and have a lot of junk code in them because I wanted to read from the sensors in the fancy new Arduino Nano BLE Sense rev2.

Future prototype changes:
***The dfRobot C1001 chip does not allow low-level access to its radar readings so this prototype version is a failure as it stands.
-Replace Arduino BLE with a cheaper Arduino ESP32 unit and transition this prototype version into a sleep-study monitor
-Make new prototype version with a different mmWave chip that allows low-level access to radar for gait detection.
