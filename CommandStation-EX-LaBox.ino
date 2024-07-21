////////////////////////////////////////////////////////////////////////////////////
// Upgrade RailCom juillet 2024
// Dernière modif: 20-07-24
// lebelge2@yahoo.fr
//
// DCC-EX CommandStation-EX   Please see https://DCC-EX.com
//
// This file is the main sketch for the Command Station.
//
// CONFIGURATION:
// Configuration is normally performed by editing a file called config.h.
// This file is NOT shipped with the code so that if you pull a later version
// of the code, your configuration will not be overwritten.
//
// If you used the automatic installer program, config.h will have been created automatically.
//
// To obtain a starting copy of config.h please copy the file config.example.h which is
// shipped with the code and may be updated as new features are added.
//
// If config.h is not found, config.example.h will be used with all defaults.
////////////////////////////////////////////////////////////////////////////////////

 
#if __has_include ( "config.h")
#include "config.h"
//#ifndef LABOX_MAIN_MOTOR_SHIELD
#ifndef LABOX_MOTOR_SHIELD                            // * Nouvelle configuration Pin
#error Your config.h must include a LABOX_MAIN_MOTOR_SHIELD and a LABOX_PROG_MOTOR_SHIELD definition. If you see this warning in spite not having a config.h, you have a buggy preprocessor and must copy config.example.h to config.h
#endif
#else
#warning config.h not found. Using defaults from config.example.h
#include "config.example.h"
#endif

/*
    © 2021 Neil McKechnie
    © 2020-2021 Chris Harlow, Harald Barth, David Cutting,
   			Fred Decker, Gregor Baues, Anthony W - Dayton
    © 2023 Thierry Paris for Locoduino.
    All rights reserved.

    This file is part of CommandStation-EX-Labox

    This is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    It is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "DCCEX.h"
#include "EEPROM.h"
#include "version_labox.h"

#ifdef CPU_TYPE_ERROR
#error CANNOT COMPILE - DCC++ EX ONLY WORKS WITH THE ARCHITECTURES LISTED IN defines.h
#endif

#ifdef WIFI_WARNING
#warning You have defined that you want WiFi but your hardware has not enough memory to do that, so WiFi DISABLED
#endif
#ifdef ETHERNET_WARNING
#warning You have defined that you want Ethernet but your hardware has not enough memory to do that, so Ethernet DISABLED
#endif
#ifdef EXRAIL_WARNING
#warning You have myAutomation.h but your hardware has not enough memory to do that, so EX-RAIL DISABLED
#endif

//--------------------------- HMI client -------------------------------------
#ifdef USE_HMI
#include "hmi.h"
hmi boxHMI(&Wire);
#endif
//----------------------------------------------------------------------------
hw_timer_t * TimerCutOutMain = NULL;                                          // *  Timer
hw_timer_t * TimerCutOutProg = NULL;                                          // * Timer durée CutOut
extern int canalMain;                                                         // * Variable
extern int canalProg;                                                         // * Variable
int p, q;

void IRAM_ATTR timer_isr_CutOutMain() {
  p++;
  switch (p) {
    case 1:
      break;
    case 2:
      gpio_matrix_out(GPIO_NUM_14, 0x100, false, false);                         // * Déconnecte la pin 14 du module RMT
      gpio_set_level(GPIO_NUM_14, 1);                                            // * Pin 14 à l'état haut
      break;
    case 3:
      gpio_matrix_out(GPIO_NUM_14, RMT_SIG_OUT0_IDX + canalMain, true, false);    // * Reconnecte la pin 14 au module RMT en inverse
      gpio_matrix_out(GPIO_NUM_12, RMT_SIG_OUT0_IDX + canalMain, true, false);    // * Reconnecte la pin 12 au module RMT en inverse
      break;
    default:
      gpio_set_level(GPIO_NUM_12, 1);                                             // * Pin 14 à l'état haut
      gpio_matrix_out(GPIO_NUM_12, RMT_SIG_OUT0_IDX + canalMain, false, false);   // * Reconnecte la pin 12 au module RMT
      timerAlarmWrite(TimerCutOutMain, 160, false);                               // * Arrêt Timer
      break;
  }
}
void IRAM_ATTR timer_isr_CutOutProg() {
  q++;
  switch (q) {
    case 1:
      break;
    case 2:
      gpio_matrix_out(GPIO_NUM_27, 0x100, false, false);                          // * Déconnecte la pin 27 du module RMT
      gpio_set_level(GPIO_NUM_27, 1);                                             // * Pin 27 à l'état haut
      break;
    case 3:
      gpio_matrix_out(GPIO_NUM_27, RMT_SIG_OUT0_IDX + canalProg, true, false);    // * Reconnecte la pin 27 au module RMT en inverse
      gpio_matrix_out(GPIO_NUM_33, RMT_SIG_OUT0_IDX + canalProg, true, false);    // * Reconnecte la pin 33 au module RMT en inverse
      break;
    default:
      gpio_set_level(GPIO_NUM_33, 1);                                             // * Pin 27 à l'état haut
      gpio_matrix_out(GPIO_NUM_33, RMT_SIG_OUT0_IDX + canalProg, false, false);   // * Reconnecte la pin 33 au module RMT
      timerAlarmWrite(TimerCutOutProg, 160, false);                               // * Arrêt Timer
      break;
  }
}
void setup()
{
  // The main sketch has responsibilities during setup()

  // Responsibility 1: Start the usb connection for diagnostics
  // This is normally Serial but uses SerialUSB on a SAMD processor
  SerialManager::init();

  DIAG(F("License GPLv3 fsf.org (c) dcc-ex.com"));

  // Initialise HAL layer before reading EEprom or setting up MotorDrivers
  IODevice::begin();

  // As the setup of a motor shield may require a read of the current sense input from the ADC,
  // let's make sure to initialise the ADCee class!
  ADCee::begin();

  DIAG(F("License GPLv3 fsf.org (c) Locoduino.org"));
  DIAG(F("LaBox : %s"), VERSION_LABOX);

  DISPLAY_START (
    // This block is still executed for DIAGS if display not in use
    LCD(0, F("CommandStation-EX v%S"), F(VERSION));
    LCD(1, F("Lic GPLv3"));
  );

#ifdef USE_HMI
  EEPROM.begin(512);
  byte mode = EEPROM.read(hmi::EEPROMModeProgAddress);

  hmi::progMode = false;
  hmi::silentBootMode = false;
  if (mode == 'P') hmi::progMode = true;
  if (mode == 'B') hmi::silentBootMode = true;

  DIAG(F("Mode %s"), hmi::progMode ? "Prog" : "Main");

  if (hmi::progMode) {
    // Reset to Main mode for next reboot.
    EEPROM.writeByte(hmi::EEPROMModeProgAddress, 'M');
    EEPROM.commit();
  }

  // must be done before Wifi setup
  boxHMI.begin();
#endif

  // Responsibility 2: Start all the communications before the DCC engine
  // Start the WiFi interface on a MEGA, Uno cannot currently handle WiFi
  // Start Ethernet if it exists
#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::setup(WIFI_SERIAL_LINK_SPEED, F(WIFI_SSID), F(WIFI_PASSWORD), F(WIFI_HOSTNAME), IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif // WIFI_ON
#else
  // ESP32 needs wifi on always
  WifiESP::setup(WIFI_SSID, WIFI_PASSWORD, WIFI_HOSTNAME, IP_PORT, WIFI_CHANNEL, WIFI_FORCE_AP);
#endif // ARDUINO_ARCH_ESP32

#if ETHERNET_ON
  EthernetInterface::setup();
#endif // ETHERNET_ON

  // Initialise HAL layer before reading EEprom or setting up MotorDrivers
  IODevice::begin();

  // Responsibility 3: Start the DCC engine.
  // Note: this provides DCC with two motor drivers, main and prog, which handle the motor shield(s)
  // Standard supported devices have pre-configured macros but custome hardware installations require
  //  detailed pin mappings and may also require modified subclasses of the MotorDriver to implement specialist logic.
  // STANDARD_MOTOR_SHIELD, POLOLU_MOTOR_SHIELD, FIREBOX_MK1, FIREBOX_MK1S are pre defined in MotorShields.hrr
  /*
    #ifdef USE_HMI
    // Set up MotorDrivers early to initialize all pins
    if (hmi::progMode) {
      DIAG(F("LaBox Prog mode."));
      TrackManager::Setup(LABOX_PROG_MOTOR_SHIELD);
    }
    else {
      DIAG(F("LaBox Main mode."));
      TrackManager::Setup(LABOX_MAIN_MOTOR_SHIELD);
    }
    #endif
  */
  TrackManager::Setup(LABOX_MOTOR_SHIELD);

  // Responsibility 3: Start the DCC engine.
  DCC::begin();

  // Start RMFT aka EX-RAIL (ignored if no automnation)
  RMFT::begin();


  // Invoke any DCC++EX commands in the form "SETUP("xxxx");"" found in optional file mySetup.h.
  //  This can be used to create turnouts, outputs, sensors etc. through the normal text commands.
#if __has_include ( "mySetup.h")
#define SETUP(cmd) DCCEXParser::parse(F(cmd))
#include "mySetup.h"
#undef SETUP
#endif

#if defined(LCN_SERIAL)
  LCN_SERIAL.begin(115200);
  LCN::init(LCN_SERIAL);
#endif
  LCD(3, F("Ready"));
  CommandDistributor::broadcastPower();

#ifdef USE_HMI
  if (hmi::progMode) {
    // must be done after all other setups.
    boxHMI.setProgMode();
  }
  if (hmi::silentBootMode) {
    // Reset to Main mode for next reboot.
    EEPROM.writeByte(hmi::EEPROMModeProgAddress, 'M');
    EEPROM.commit();
  }

#endif
  Serial.print("Version IDF: ");
  Serial.println(esp_get_idf_version());                                       // * Affiche version IDF
  TimerCutOutMain = timerBegin(2, 80, true);                                   //* Timer 2
  timerAttachInterrupt(TimerCutOutMain, &timer_isr_CutOutMain, true);
  timerAlarmEnable(TimerCutOutMain);
  TimerCutOutProg = timerBegin(3, 80, true);                                   //* Timer 3
  timerAttachInterrupt(TimerCutOutProg, &timer_isr_CutOutProg, true);
  timerAlarmEnable(TimerCutOutProg);
}

void StarTimerCutOutMain() {                                                   // Ajout RaiCom
  timerAlarmWrite(TimerCutOutMain, 160, true);                                 // * En continu
  timerAlarmEnable(TimerCutOutMain);
  p = 0;
}
void StarTimerCutOutProg() {                                                   // Ajout RaiCom
  timerAlarmWrite(TimerCutOutProg, 160, true);                                 // * En continu
  timerAlarmEnable(TimerCutOutProg);
  q = 0;
}

void loop()
{
  // The main sketch has responsibilities during loop()

  // Responsibility 1: Handle DCC background processes
  //                   (loco reminders and power checks)
  DCC::loop();

  // Responsibility 2: handle any incoming commands on USB connection
  SerialManager::loop();

  // Responsibility 3: Optionally handle any incoming WiFi traffic
#ifndef ARDUINO_ARCH_ESP32
#if WIFI_ON
  WifiInterface::loop();
#endif //WIFI_ON
#else  //ARDUINO_ARCH_ESP32
#ifndef WIFI_TASK_ON_CORE0
  WifiESP::loop();
#endif
#endif //ARDUINO_ARCH_ESP32
#if ETHERNET_ON
  EthernetInterface::loop();
#endif

  RMFT::loop();  // ignored if no automation

#if defined(LCN_SERIAL)
  LCN::loop();
#endif

  // Display refresh
  DisplayInterface::loop();

  // Handle/update IO devices.
  IODevice::loop();

  Sensor::checkAll(); // Update and print changes

  // Report any decrease in memory (will automatically trigger on first call)
  static int ramLowWatermark = __INT_MAX__; // replaced on first loop

  int freeNow = DCCTimer::getMinimumFreeMemory();
  if (freeNow < ramLowWatermark) {
    ramLowWatermark = freeNow;
    LCD(3, F("Free RAM=%5db"), ramLowWatermark);
  }
#ifdef USE_HMI
  boxHMI.update();
#endif
}
