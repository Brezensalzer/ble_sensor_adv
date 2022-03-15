//-------------------------------------------------------------------------------
// ble_sensor_adv.ino
//-------------------------------------------------------------------------------
#include <bluefruit.h>
#include <ble_gap.h>

// save power without serial interface...
//#define DEBUG

struct ATTR_PACKED
{
  uint16_t manufacturer;
  uint16_t temperature;
  uint16_t humidity;
  uint16_t pressure;
  uint16_t battery;
} sensor_data;

//--- MS8607 temperature, humidity and pressure sensor ---
#include <Wire.h>
#include <Adafruit_MS8607.h>
#include <Adafruit_Sensor.h>
#define VCC_I2C 10    // define the GPIO pin used as i2c vcc

//---- ADC Battery Monitoring ----------------------------
const int adcPin = A5;

//------------------------------------------------------------------------------
float batteryVoltage()
//------------------------------------------------------------------------------
{
  analogReference(AR_INTERNAL); // reference voltage 0..3.6V
  analogReadResolution(12);     // analog resolution 12bit
  int adcValue = analogRead(adcPin);
  adcValue = analogRead(adcPin);
  // 4095 = 3.6V, 2.7V cut off voltage for LiFePO4 batteries
  float value = adcValue * 0.87890625;
  #ifdef DEBUG
    Serial.print("Battery: ");
    Serial.println(value);
  #endif
  return value;  
}

//---------------------------------------------------------------------------
void sensorData()
//---------------------------------------------------------------------------
{  
  //--- power on i2c vcc --------------------------------------------
  pinMode(VCC_I2C, OUTPUT);
  digitalWrite(VCC_I2C, HIGH);
  #ifdef DEBUG
    Serial.println("setting pin VCC_I2C to HIGH");
  #endif
  delay(100); // sensor boot time

  // Initialize sensor --------------------------
  Wire.begin();
  Adafruit_MS8607 ms8607;
  
  if (!ms8607.begin()) {
    #ifdef DEBUG
      Serial.println("Failed to find MS8607 chip");
    #endif
    while (1) { delay(10); }
  }
  #ifdef DEBUG
    Serial.println("MS8607 Found!");
  #endif
  
  // read sensor data
  sensors_event_t temp, pressure, humidity;
  Adafruit_Sensor *pressure_sensor = ms8607.getPressureSensor();
  Adafruit_Sensor *temp_sensor = ms8607.getTemperatureSensor();
  Adafruit_Sensor *humidity_sensor = ms8607.getHumiditySensor();

  temp_sensor->getEvent(&temp);
  pressure_sensor->getEvent(&pressure);
  humidity_sensor->getEvent(&humidity);

  #ifdef DEBUG
    Serial.print("Temperatur: ");
    Serial.println(temp.temperature);
    Serial.print("Luftfeuchte: ");
    Serial.println(humidity.relative_humidity);
    Serial.print("Luftdruck: ");
    Serial.println(pressure.pressure);
  #endif

  sensor_data.manufacturer = 0x0822;
  sensor_data.temperature = int(temp.temperature * 100.0);
  sensor_data.humidity = int(humidity.relative_humidity * 100.0);
  sensor_data.pressure = int(pressure.pressure * 10.0); 
  sensor_data.battery = int(batteryVoltage());

  Bluefruit.Advertising.addData(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &sensor_data, sizeof(sensor_data));

  // sensor sleep 
  Wire.end();
  
  //--- power off i2c vcc --------------------------------------------
  digitalWrite(VCC_I2C, LOW);
  pinMode(VCC_I2C, INPUT);
  #ifdef DEBUG
    Serial.println("setting pin VCC_I2C to LOW");
  #endif  
}

//---------------------------------------------------------------------------
void startAdvertising()
//---------------------------------------------------------------------------
{   
  #ifdef DEBUG
    Serial.println("Starting to Advertise");
  #endif

  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.setName("Brezensalzer");
  Bluefruit.ScanResponse.addName();
  Bluefruit.setTxPower(4);
  
  sensorData();

  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(250, 250);
  Bluefruit.Advertising.setFastTimeout(1);
  Bluefruit.Advertising.start(1);
}

//---------------------------------------------------------------------------
void stopAdvertising()
//---------------------------------------------------------------------------
{
  #ifdef DEBUG
    Serial.println("Clearing Advertisement Data");
    Serial.println("----------------------------------");
  #endif
  Bluefruit.Advertising.clearData();
  Bluefruit.Advertising.stop();
}

//---------------------------------------------------------------------------
void setup() 
//---------------------------------------------------------------------------
{
  #ifdef DEBUG
    Serial.begin(115200);
    int i = 0;
    while ( !Serial ) 
    {
      i++;
      if (i > 20) break;
      delay(100);   // for nrf52840 with native usb
    }
    Serial.println("ItsyBitsy nRF52840 Environmental Sensor");
    Serial.println("---------------------------------------");
  #endif
  #ifndef DEBUG
    Serial.end();   // power saving
  #endif
  
  if (!Bluefruit.begin())
  {
    #ifdef DEBUG
      Serial.println("Unable to init Bluefruit");
    #endif
    while(1)
    {
      digitalToggle(LED_RED);
      delay(100);
    }
  }
}

//---------------------------------------------------------------------------
void loop() 
//---------------------------------------------------------------------------
{
  // losing 2.8 µA
  delay(59000);
  startAdvertising();
  delay(500);
  stopAdvertising();
  //--- brute force soft reset ----------------
  // otherwise we lose 274 µA
  NVIC_SystemReset();
}
