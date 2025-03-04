/**
 * @file main.cpp
 * @brief This is the system monitor code for SV Gone With The Wind
 *
 * Author: K. Youngblood <keithyoungblood@protonmail.com>
 * https://svgonewiththewind.com
 *
 */

#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/moving_average.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>

#include "displays/SSD1306Display.h"

#include "interpreters/FreshWaterTankLevelInterpreter.h"
#include "interpreters/BatteryMonitorInterpreters.h"
#include "interpreters/RTDTemperatureTransmitterInterpreters.h"
#include "interpreters/EngineDataInterpreters.h"

using namespace sensesp;
using namespace sensesp::onewire;

// I2C config for SH-ESP32
#define SDA_PIN 16
#define SCL_PIN 17

// One-wire data line is connected to pin 4 on the SH-ESP32
#define ONEWIRE_PIN 4

// Opto-isolated input pin for bilge pump status
#define BILGE_OPTO_IN_PIN 35

// Serial port pins used for the CAN bus on the SH-ESP32. 
// When disabled, can be used by other hardware such as the Wit IMU module
#define RXD1 34
#define TXD1 32 // Tx to the WitMotion HWT901B-TTL IMU

// SH-ESP32 on-board, blue LED (GPIO2)
#define STATUS_LED 2

// Declare the two ADS1115 IC's
Adafruit_ADS1115 ads0;
Adafruit_ADS1115 ads1;

/*=========================================================================
    I2C ADDRESS/BITS 
      Tie ADDR pin to:
      GND -- 1001 000 = 0x48
      VDD -- 1001 001 = 0x49
      SDA -- 1001 010 = 0x4a // Future expansion
      SCL -- 1001 011 = 0x4b // Future expansion
    -----------------------------------------------------------------------*/
#define ADS1115_0_ADDR (0x48) // ADDR -> GND
#define ADS1115_1_ADDR (0x49) // ADDR -> VCC
/*=========================================================================*/

// ADS1115_0 channels
#define ENGINE_OIL_PRESSURE_CHANNEL 0
#define ENGINE_COOLANT_TEMPERATURE_CHANNEL 1
#define RAW_WATER_TEMPERATURE_CHANNEL 2
#define DIESEL_TANK_FUEL_LEVEL_CHANNEL 3

// ADS1115_1 channels
#define BATTERY_CURRENT_CHANNEL 0
#define BATTERY_VOLTAGE_CHANNEL 1
#define FRESH_WATER_TANK_LEVEL_CHANNEL 2
#define RESERVED_CHANNEL 3

// I2C bus
TwoWire* i2c;

// Temperature, pressure and humidity sensor
Adafruit_MS8607 ms8607;
// Adafruit_Sensor *pressure_sensor; // = ms8607.getPressureSensor();
// Adafruit_Sensor *temp_sensor; // = ms8607.getTemperatureSensor();
// Adafruit_Sensor *humidity_sensor; // = ms8607.getHumiditySensor();

// Tiny 128x64 pixel, 0.96" OLED display
Adafruit_SSD1306* display;
#define SSD1306_I2C_ADDR 0x3C


void I2C_Scanner (TwoWire *i2c)
{
  ESP_LOGD(__FILENAME__, "I2C scanner. Scanning ...");

  byte count = 0;

  // Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    i2c->beginTransmission (i);          // Begin I2C transmission Address (i)
    if (i2c->endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      // Serial.print ("Found address: ");
      // Serial.print (i, DEC);
      // Serial.print (" (0x");
      // Serial.print (i, HEX);     // PCF8574 7 bit address
      ESP_LOGD(__FILENAME__, "0x%02X\n", (uint16_t)i);
      // Serial.println (")");
      count++;
    }
  }
  // Serial.print ("Found ");      
  // Serial.print (count, DEC);        // numbers of devices
  // Serial.println (" device(s).");
}

//////////////////////////////////////////////////////////////////////
// Define callbacks for the ADS1115 ADC boards
//////////////////////////////////////////////////////////////////////
// First ADS1115 board
int16_t oil_pressure_read_callback() { 
  return ads0.readADC_SingleEnded(ENGINE_OIL_PRESSURE_CHANNEL); }

int16_t engine_temp_read_callback() { 
  return ads0.readADC_SingleEnded(ENGINE_COOLANT_TEMPERATURE_CHANNEL); }

int16_t raw_water_temp_read_callback() { 
  return ads0.readADC_SingleEnded(RAW_WATER_TEMPERATURE_CHANNEL); }

int16_t diesel_tank_level_read_callback() { 
  return ads0.readADC_SingleEnded(DIESEL_TANK_FUEL_LEVEL_CHANNEL); }

// Second ADS1115 board
int16_t fresh_water_level_read_callback() { 
  return ads1.readADC_SingleEnded(FRESH_WATER_TANK_LEVEL_CHANNEL); }

int16_t battery_voltage_read_callback() { 
  return ads1.readADC_SingleEnded(BATTERY_VOLTAGE_CHANNEL); }

int16_t battery_current_read_callback() { 
  return ads1.readADC_SingleEnded(BATTERY_CURRENT_CHANNEL); }

int16_t reserved_read_callback() { 
  return ads1.readADC_SingleEnded(RESERVED_CHANNEL); }

int16_t debug_read_callback() { 
  // Return a value representing a center reading (12mA in the 4-20mA range)
  return 15840; 
}

//////////////////////////////////////////////////////////////////////
// Define callbacks for PHT sensor
//////////////////////////////////////////////////////////////////////
float pressure_read_callback() { 
  sensors_event_t pressure;

  Adafruit_Sensor *pressure_sensor = ms8607.getPressureSensor();

  pressure_sensor->getEvent(&pressure);

  // Return barometric pressure in Pascals
  return pressure.pressure * 100.0f; // hPa to Pa (1 hPa == 1 mBar)
}

float humidity_read_callback() { 
  sensors_event_t humidity;

  Adafruit_Sensor *humidity_sensor = ms8607.getHumiditySensor();

  humidity_sensor->getEvent(&humidity);

  // Return cabin relative humidity in percent
  return humidity.relative_humidity; // Percent
}

float temperature_read_callback() { 
  // sensors_event_t temp;

  // Adafruit_Sensor *temp_sensor = ms8607.getTemperatureSensor();

  // temp_sensor->getEvent(&temp);

  sensors_event_t temp, pressure, humidity;
  ms8607.getEvent(&pressure, &temp, &humidity);

  // Return cabin temperature in Kelvin
  return temp.temperature + 273.15f; // Degrees C to K
}

////////////////////////////////////////////////////////////////////////
///////////////////////// SETUP SETUP SETUP ////////////////////////////
////////////////////////////////////////////////////////////////////////

// The setup function performs one-time application initialization.
void setup() {
  SetupLogging();

  // Create the global SensESPApp() object.
  SensESPAppBuilder builder;
  // sensesp_app = builder.set_hostname("gwtw-sysmon")
  sensesp_app = builder.set_hostname("sensESP")
                    // ->set_sk_server("192.168.0.152", 3000)
                    // ->set_wifi_client("Franklin T10 6907", "supersecretpassword")
                    ->get_app();

  // Set up Dallas one-wire bus
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(SDA_PIN, SCL_PIN);

  // I2C_Scanner(i2c);
  // while (1);

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1115
  //                                                                -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

  // Set the ADC gains to GAIN_ONE for a 0.0V to 4.096V range (3.3V Vcc, 165 Ohm, 4-20mA resistor)
  // This limits values read to between codes: 0 and 26400 (0 to 3.3V)
  ads0.setGain(GAIN_ONE);
  ads1.setGain(GAIN_ONE);

  if (!ads0.begin(ADS1115_0_ADDR, i2c)) {
    Serial.println("Failed to initialize ads0.");
    //while (1);
  }

  if (!ads1.begin(ADS1115_1_ADDR, i2c)) {
    Serial.println("Failed to initialize ads1.");
    //while (1);
  }

  /////////////////////////////////////////////////////////////
  // ms8607 temperature, barometric pressure and humidity 
  // sensor

  if (!ms8607.begin(i2c)) {
    // Serial.println("Failed to find MS8607 chip");
    ESP_LOGW(
      __FILENAME__,
      "Failed to find MS8607 chip");
    // while (1) { delay(10); }
  } else {
    Serial.println("MS8607 PHT Found!");
    ESP_LOGD(
      __FILENAME__,
      "MS8607 PHT Found!");

    // pressure_sensor = ms8607.getPressureSensor();
    // temp_sensor = ms8607.getTemperatureSensor();
    // humidity_sensor = ms8607.getHumiditySensor();
  }

  // Override defaults if needed
  // ms8607.setHumidityResolution(MS8607_HUMIDITY_RESOLUTION_OSR_8b); // Default: MS8607_HUMIDITY_RESOLUTION_OSR_12b
  // ms8607.setPressureResolution(MS8607_PRESSURE_RESOLUTION_OSR_4096); // Default: MS8607_PRESSURE_RESOLUTION_OSR_4096

  /////////////////////////////////////////////////////////////
  // Set up the little OLED display. SSD1306 display with a 
  // 0.96" diagonal viewing area and 128x64 pixels, monochrome
  // 
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDR)) {
    Serial.println(F("SSD1306 allocation failed")); // Now what?!?
  }

  // Wait for display to be ready
  delay(100);

  display->setRotation(DISPLAY_ROTATION);
  display->clearDisplay();
  display->setTextSize(1); // Text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
  display->setTextColor(SSD1306_WHITE);
  display->setCursor(0, 0);
  display->printf("Hostname: %s\n", sensesp_app->get_hostname().c_str());
  display->display();

  // Let's read the sensor(s) every 1000 ms.
  unsigned int read_interval = 1000;

  //////////////////////////////////////////////////////////////////////
  // One-wire temperature sensors
  // ----------------------------
  /////////////////////////////////////////////////////////////////////
  // SK path: /environment/outside/temperature (K) [one wire]
  // Measure outdoor temperature every 5 seconds
  auto outdoor_temp =
      new OneWireTemperature(dts, 5000, "/outdoorTemperature/oneWire");

  ConfigItem(outdoor_temp)
      ->set_title("Outdoor Temperature")
      ->set_description("Temperature of the outdoors")
      ->set_sort_order(100);

  outdoor_temp
    // Smooth it out by averaging over a minute with 12 readings (unity scaling at 1.0)
    // ->connect_to(new MovingAverage(12, 1.0, "/outdoor/temp/average"))
    ->connect_to(new SKOutputFloat("environment.outside.temperature", "/outdoor/temp/sk"))
    ->connect_to(new SSD1306Display(display, DATA_ROW_7, DATA_COL_2, "K"));

  /////////////////////////////////////////////////////////////////////
  // SK path: /environment/inside/[A-Za-z0-9]+/temperature (K) [various, one wire]

  // TODO: add the other 1-wire temperature sensors here...


  //////////////////////////////////////////////////////////////////////
  // 4-20mA sensor channels on the ADS1115 ADC's
  // ---------------------------------------------
  // 
  //////////////////////////////////////////////////////////////////////
  // SK Path: /propulsion/<main>/temperature (K) [RTD]
  auto* engine_water_temp_input =
      new RepeatSensor<int16_t>(5000, engine_temp_read_callback); //engine_temp_read_callback (debug_read_callback = 50C)

  engine_water_temp_input
      ->connect_to(new RTD_Minus50To150C_TempInterpreter("/engine/temp/curve"))
      // Smooth it out by averaging 10 readings (scaling by 1.0)
      // ->connect_to(new MovingAverage(10, 1.0, "/engine/temp/average"))
      ->connect_to(new Linear(1.0, 0.0, "/engine/temp/calibrate"))
      ->connect_to(new SKOutputFloat("propulsion.main.temperature", "/engine/temp/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_5, DATA_COL_1, "K"));

  /////////////////////////////////////////////////////////
  // SK Path: /propulsion/<main>/oilPressure (Pa)
  // Oil pressure sender (0-100 PSI = 0-689475.729 Pa) Read every 1000ms???
    auto* oil_pressure_input =
      new RepeatSensor<int16_t>(read_interval, oil_pressure_read_callback);

    oil_pressure_input
      ->connect_to(new OilPressureInterpreter("/engine/oilpressure/curve"))
      // Smooth it out by averaging 10 readings (scaling by 1.0)
      // ->connect_to(new MovingAverage(10, 1.0, "/engine/oilpressure/average"))
      ->connect_to(new Linear(1.0, 0.0, "/engine/oilpressure/calibrate"))
      ->connect_to(new SKOutputFloat("propulsion.main.oilPressure", "/engine/oilpressure/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_4, DATA_COL_1, "Pa"));

  /////////////////////////////////////////////////////////
  // SK Path: /tanks/fuel/<mainDiesel>/currentLevel (ratio)
  auto* diesel_tank_level_input =
      new RepeatSensor<int16_t>(30000, diesel_tank_level_read_callback);

  diesel_tank_level_input
      ->connect_to(new DieselTankLevelInterpreter("/diesel/tank/curve"))
      // Smooth it out by averaging 10 readings over 5 mins (scaling by 1.0)
      ->connect_to(new MovingAverage(10, 1.0, "/diesel/tank/average"))
      ->connect_to(new Linear(1.0, 0.0, "/diesel/tank/calibrate"))
      ->connect_to(new SKOutputFloat("tanks.fuel.main.currentLevel", "/diesel/tank/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_3, DATA_COL_1, "%"));
  
  /////////////////////////////////////////////////////////
  // SK Path: /environment/water/temperature (K) [RTD]
  // Raw water intake temperature, taken at the seacock flange nearest the hull
  auto* raw_water_temp_input =
      new RepeatSensor<int16_t>(5000, raw_water_temp_read_callback);

  raw_water_temp_input
      ->connect_to(new RTD_ZeroTo100C_TempInterpreter("/rawwater/temp/curve"))
      // Smooth it out by averaging 10 readings (scaling by 1.0)
      // ->connect_to(new MovingAverage(10, 1.0, "/rawwater/temp/average"))
      ->connect_to(new Linear(1.0, 0.0, "/rawwater/temp/calibrate"))
      ->connect_to(new SKOutputFloat("environment.water.temperature", "/rawwater/temp/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_2, DATA_COL_0, "K"));
  
  /////////////////////////////////////////////////////////
  // SK Path: /electrical/batteries/<house>/voltage (V)
  // 0 to 20 volts full scale, 5280 to 26400
  auto* battery_voltage_input =
      new RepeatSensor<int16_t>(1000, battery_voltage_read_callback);

  battery_voltage_input 
      // Can we call the display twice with different levels of transforms? 
      // Useful for debugging and calibration.
      ->connect_to(new SSD1306Display(display, DATA_ROW_0, DATA_COL_0)) // Display raw ADC value
      ->connect_to(new BatteryVoltageInterpreter("/battery/voltage/curve"))
      ->connect_to(new Linear(1.0, 0.0, "/battery/voltage/calibrate"))
      ->connect_to(new SKOutputFloat("electrical.batteries.houseBattery.voltage", "/battery/voltage/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_0, DATA_COL_2, "V")); // Show value sent to SK
  
  /////////////////////////////////////////////////////////
  // SK Path: /electrical/batteries/<house>/current (I)
  // +300 to -300 ampere range, 5280 to 26400
  auto* battery_current_input =
      new RepeatSensor<int16_t>(1000, battery_current_read_callback);

  battery_current_input 
      ->connect_to(new SSD1306Display(display, DATA_ROW_1, DATA_COL_0)) // Raw ADC code value
      ->connect_to(new BatteryCurrentInterpreter("/battery/current/curve"))
      ->connect_to(new Linear(1.0, 0.0, "/battery/current/calibrate"))
      ->connect_to(new SKOutputFloat("electrical.batteries.houseBattery.current", "/battery/current/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_1, DATA_COL_2, "A"));

  /////////////////////////////////////////////////////////
  // SK Path: /tanks/freshWater/<main>/currentLevel (ratio)
  // Freshwater tank level reading, read every 30,000 ms(30 secs)
  // /tanks/freshWater/<main>/capacity (m3)
  // 48 gallons = 0.1817 cubic meters
  // /tanks/freshWater/<main>/currentVolume (m3)
  auto* fresh_water_level_input =
      new RepeatSensor<int16_t>(30000, fresh_water_level_read_callback);

  const char* sk_freshwater_level_path = "tanks.freshWater.main.currentLevel";

  // TODO: Do we need to send custom metadata to describe standard SK schema paths???
  // SKMetadata* fw_lvl_metadata = new SKMetadata();
  // fw_lvl_metadata->description_ = "Freshwater tank level";
  // fw_lvl_metadata->display_name_ = "Freshwater Tank Level";
  // fw_lvl_metadata->short_name_ = "Water Tank";
  // fw_lvl_metadata->units_ = "ratio";

  fresh_water_level_input
      ->connect_to(new SSD1306Display(display, DATA_ROW_6, DATA_COL_0)) // Raw ADC code value for interpolation calibration process
      ->connect_to(new WaterTankLevelInterpreter("/freshwater/tank/curve"))
      // Smooth it out by averaging 10 readings over 5 mins (scaling by 1.0)
      ->connect_to(new MovingAverage(10, 1.0, "/freshwater/tank/average"))
      ->connect_to(new Linear(1.0, 0.0, "/freshwater/tank/calibrate"))
      ->connect_to(new SKOutputFloat(sk_freshwater_level_path, "/freshwater/tank/sk"))
      ->connect_to(new SSD1306Display(display, DATA_ROW_6, DATA_COL_2, "%"));


  //////////////////////////////////////////////////////////////
  // Read values from the pressure, humidity and temperature
  // sensors

  /////////////////////////////////////////////////////////
  // SK Path: /environment/inside/cabin/temperature (K)
  // 
  auto* cabin_temp_input =
    new RepeatSensor<float>(5000, temperature_read_callback);

  cabin_temp_input
    // Smooth it out by averaging 10 readings (scaling by 1.0)
    // ->connect_to(new MovingAverage(10, 1.0, "/cabin/temp/average"))
    ->connect_to(new Linear(1.0, 0.0, "/cabin/temp/calibrate"))
    ->connect_to(new SKOutputFloat("environment.inside.cabin.temperature", "/cabin/temp/sk"));
    // ->connect_to(new SSD1306Display(display, DATA_ROW_5, DATA_COL_1, "K"));

  /////////////////////////////////////////////////////////
  // SK Path: /environment/inside/cabin/relativeHumidity (ratio)
  // 
  auto* cabin_humidity_input =
    new RepeatSensor<float>(10000, humidity_read_callback);

  cabin_humidity_input
    // Smooth it out by averaging 10 readings (scaling by 1.0)
    // ->connect_to(new MovingAverage(10, 1.0, "/cabin/humidity/average"))
    ->connect_to(new Linear(1.0, 0.0, "/cabin/humidity/calibrate"))
    ->connect_to(new SKOutputFloat("environment.inside.cabin.relativeHumidity", "/cabin/humidity/sk"));
    // ->connect_to(new SSD1306Display(display, DATA_ROW_5, DATA_COL_1, "%"));

  /////////////////////////////////////////////////////////
  // SK Path: /environment/inside/cabin/pressure (Pa)
  // 
  auto* barometric_pressure_input =
    new RepeatSensor<float>(30000, pressure_read_callback);

    barometric_pressure_input
    // Smooth it out by averaging 10 readings (scaling by 1.0)
    // ->connect_to(new MovingAverage(10, 1.0, "/cabin/pressure/average"))
    ->connect_to(new Linear(1.0, 0.0, "/cabin/pressure/calibrate"))
    ->connect_to(new SKOutputFloat("environment.inside.cabin.pressure", "/cabin/pressure/sk"));
    // ->connect_to(new SSD1306Display(display, DATA_ROW_5, DATA_COL_1, "Pa"));

  ///////////////////////////////////////////////////////////
  // Send bilge status to SignalK server, 
  // TODO: need to use SK custom metadata??
  DigitalInputState* bilge_pump_state_input = new DigitalInputState(BILGE_OPTO_IN_PIN, INPUT, 500, "/keelbilgepump/status/raw");
  // bilge_pump_state_input->connect_to(new SKOutputBool("path",));

  auto int_to_string_function = [](int input) ->String {
     if (input == 1) {
       return "Bilge pump ON";
     } 
     else { // input == 0
       return "Bilge pump OFF";
     }
  };

  auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

  bilge_pump_state_input
        ->connect_to(new SSD1306Display(display, DATA_ROW_7, DATA_COL_0, "<-BLG"))
        ->connect_to(int_to_string_transform)
        ->connect_to(new SKOutputString("sensors.bilgePump.status"));

  // bilge_pump_state_input->connect_to(new SKOutputString("propulsion.engine.bilge.raw"));


  // Setup a second serial port to read WitMotion IMU device at 9600 baud
  // Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);

  // char buf[2048];
  // Serial1.read(buf, 1024); // Test read

  // Spit it out the debug serial? This will likely print garbage...
  // Need to decode the binary data. Hex dump?
  // Serial.println(buf, HEX);
  // Serial.println(buf);

  /////////////////////////////////////////////////////////////////////
  // /navigation/attitude (object: roll(rad),pitch(rad),yaw(rad))
  // /navigation/headingMagnetic (rad)


  // Monitor engine status? Engine hour tracking?
  // SK Path: /propulsion/<RegExp>/state -- Description: The current state of the engine
  // DigitalInputState* 
    // engine_state_input = new DigitalInputState(ENGINE_RUNNING_OPTO_IN_PIN, INPUT, 1000);
}

void loop() {
  // We're storing the event loop in a static variable so that it's only
  // acquired once. Saves a few function calls per loop iteration.
  static auto event_loop = SensESPBaseApp::get_event_loop();
  event_loop->tick();
}
