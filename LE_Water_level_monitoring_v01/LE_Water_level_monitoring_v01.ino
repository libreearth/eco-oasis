/**
 * Libre earth solutions SL
 * 2021
 */
#include <Arduino.h>

#include <LoRaWan-RAK4630.h> //Click here to get the library: http://librarymanager/All#SX126x

#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680
#include "ClosedCube_SHT31D.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS.h>        //http://librarymanager/All#TinyGPS

//Real OTAA keys written down in secret.h
#include "secret.h"
//uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x04, 0x7B, 0x78}; 
//uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//uint8_t nodeAppKey[16] = {0x5C, 0x05, 0x97, 0xCF, 0x52, 0xEC, 0x9D, 0x31, 0x88, 0xE6, 0x76, 0x42, 0x2C, 0x54, 0xDD, 0x78};

// Check if the board has an LED port defined
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

#define VBAT_MV_PER_LSB (2.9296875F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */

#define LORAWAN_DATERATE DR_0
#define LORAWAN_TX_POWER TX_POWER_0
#define JOINREQ_NBTRIALS 3 /**< Number of trials for the join request. */

DeviceClass_t g_CurrentClass = CLASS_A;
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_CONFIRMED_MSG;
uint8_t g_AppPort = LORAWAN_APP_PORT;
int depths = 0;
float flat, flon;
int sats;
unsigned long age;

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {
  LORAWAN_ADR_ON,
  LORAWAN_DATERATE,
  LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS,
  LORAWAN_TX_POWER,
  LORAWAN_DUTYCYCLE_OFF
};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {
  BoardGetBatteryLevel,
  BoardGetUniqueId,
  BoardGetRandomSeed,
  lorawan_rx_handler,
  lorawan_has_joined_handler,
  lorawan_confirm_class_handler,
  lorawan_join_failed_handler
};


// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t g_m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];        //< Lora user application data buffer.
static lmh_app_data_t g_m_lora_app_data = {g_m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t g_appTimer;
static uint32_t timers_init(void);

static uint32_t g_count = 0;
static uint32_t g_count_fail = 0;
static boolean blocked_not_join = false;

// BME680
Adafruit_BME680 bme;

// Might need adjustments
#define SEALEVELPRESSURE_HPA (1010.0)

ClosedCube_SHT31D sht3xd;
SHT31D sht31d_measurement;

// dallas temperature sensor
OneWire oneWireObjeto(WB_IO1);
DallasTemperature sensorDS18B20(&oneWireObjeto);

//gps
TinyGPS gps;

//vbat
float vbat;

void water_temp_init()
{
  sht3xd.begin(0x44); // I2C address: 0x44 or 0x45
  Serial.print("Serial #");
  Serial.println(sht3xd.readSerialNumber());
  if (sht3xd.periodicStart(SHT3XD_REPEATABILITY_HIGH, SHT3XD_FREQUENCY_10HZ) != SHT3XD_NO_ERROR) 
    Serial.println("[ERROR] Cannot start periodic mode");
}

void bme680_init()
{
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void gps_init(){
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);
  
  Serial1.begin(9600);
  while (!Serial1);
  Serial.println("gps uart init ok!");
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void)
{
  Serial.println("OTAA Mode, Network Joined!");
  digitalWrite(LED_BUILTIN2, LOW);

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&g_appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&g_appTimer);
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}

/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  g_m_lora_app_data.buffsize = 0;
  g_m_lora_app_data.port = g_AppPort;
  lmh_send(&g_m_lora_app_data, g_CurrentConfirm);
}

/**
 * @brief Get RAW Battery Voltage
 */
float readVBAT(void)
{
    int raw;

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(WB_A0);
    Serial.printf("%d\n", raw);

    return raw * REAL_VBAT_MV_PER_LSB;
}

int get_depths(void)
{
  int i;

  int sensor_pin = WB_A1;   // select the input pin for the potentiometer
  int mcu_ain_raw = 0; // variable to store the value coming from the sensor

  int depths; // variable to store the value of oil depths
  int average_raw;
  float voltage_ain, factor;

  for (i = 0; i < 5; i++)
  {
    mcu_ain_raw += analogRead(sensor_pin);
  }
  average_raw = mcu_ain_raw / i;
  voltage_ain = average_raw * (3.0/1024.0);
 

  factor = (voltage_ain-0.5)/2.0;
  depths = factor*SENSOR_DEPTH*1000;
 
  Serial.printf("read value = %d = %f v = %d mm\n" , average_raw, voltage_ain, depths);
 
  return depths;
}

void read_gps_data(){
  // For one second we parse GPS data and report some key values
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      if (gps.encode(c))// Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);
    sats = gps.satellites();
  }
}

void float2Bytes(byte bytes_temp[4], float float_variable){ 
  byte buf[4] = {0x0, 0x0, 0x0, 0x0};
  memcpy(buf, (unsigned char*) (&float_variable), 4);
  bytes_temp[0] = buf[3];
  bytes_temp[1] = buf[2];
  bytes_temp[2] = buf[1];
  bytes_temp[3] = buf[0];
}

void send_lora_frame(void)
{

  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;

  g_m_lora_app_data.port = g_AppPort;
  //Depth sensor
  if (HAS_DEPTH_SENSOR){
    g_m_lora_app_data.buffer[i++] = 0x01;
    g_m_lora_app_data.buffer[i++] = 0x00;
    g_m_lora_app_data.buffer[i++] = 0x00;
    g_m_lora_app_data.buffer[i++] = (depths >> 8) & 0xFF;
    g_m_lora_app_data.buffer[i++] = depths & 0xFF;
  }

  if (HAS_BME680) {
    //Ambient temperature sensor
    g_m_lora_app_data.buffer[i++] = 0x02;
    float2Bytes(g_m_lora_app_data.buffer + i, bme.temperature);
    i+=4;
    
     //Ambient humidity sensor
    g_m_lora_app_data.buffer[i++] = 0x03;
    float2Bytes(g_m_lora_app_data.buffer + i, bme.humidity);
    i+=4;
    
    //Ambient pressure sensor
    g_m_lora_app_data.buffer[i++] = 0x04;
    g_m_lora_app_data.buffer[i++] = (bme.pressure >> 24) & 0xFF;
    g_m_lora_app_data.buffer[i++] = (bme.pressure >> 16) & 0xFF;
    g_m_lora_app_data.buffer[i++] = (bme.pressure >> 8) & 0xFF;
    g_m_lora_app_data.buffer[i++] = bme.pressure & 0xFF;
  }

  //Water temperature sensor
  if (HAS_WATER_TEMP){
    g_m_lora_app_data.buffer[i++] = 0x05;
    float2Bytes(g_m_lora_app_data.buffer + i, sht31d_measurement.t);
    i+=4;
  }

  if (HAS_DS18B20) {
    g_m_lora_app_data.buffer[i++] = 0x06;
    float2Bytes(g_m_lora_app_data.buffer + i, sensorDS18B20.getTempCByIndex(0));
    i+=4;
  }

  if (HAS_GPS) {
    g_m_lora_app_data.buffer[i++] = 0x07;
    float2Bytes(g_m_lora_app_data.buffer + i, flat);
    i+=4;
    
    g_m_lora_app_data.buffer[i++] = 0x08;
    float2Bytes(g_m_lora_app_data.buffer + i, flon);
    i+=4;

    g_m_lora_app_data.buffer[i++] = 0x09;
    g_m_lora_app_data.buffer[i++] = 0x00;
    g_m_lora_app_data.buffer[i++] = 0x00;
    g_m_lora_app_data.buffer[i++] = (sats >> 8) & 0xFF;
    g_m_lora_app_data.buffer[i++] = sats & 0xFF;
  }

  if (HAS_BATTERY) {
    g_m_lora_app_data.buffer[i++] = 0x0A;
    float2Bytes(g_m_lora_app_data.buffer + i, vbat);
    i+=4;
  }
    
  g_m_lora_app_data.buffsize = i;
  
  lmh_error_status error = lmh_send(&g_m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    g_count++;
    digitalWrite(LED_BUILTIN2, HIGH);
    Serial.printf("lmh_send ok count %d\n", g_count);
    delay(100);
    digitalWrite(LED_BUILTIN2, LOW);
  }
  else
  {
    g_count_fail++;
    Serial.printf("lmh_send fail count %d\n", g_count_fail);
  }
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void)
{
  TimerSetValue(&g_appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&g_appTimer);
  Serial.println("Sending frame now...");
  send_lora_frame();
}

/**@brief Function for the Timer initialization.

   @details Initializes the timer module. This creates and starts application timers.
*/
uint32_t timers_init(void)
{
  TimerInit(&g_appTimer, tx_lora_periodic_handler);
  return 0;
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LED_BUILTIN2, OUTPUT);
  digitalWrite(LED_BUILTIN2, HIGH);
  

  /*
     WisBLOCK 5811 Power On
  */
  /*pinMode(WB_IO1, OUTPUT);
  digitalWrite(WB_IO1, HIGH);*/

  if (HAS_DEPTH_SENSOR){
    pinMode(WB_A1, INPUT_PULLDOWN);
    analogReference(AR_INTERNAL_3_0);
    analogOversampling(128);
  }

  //if (HAS_BATTERY){
  //  pinMode(WB_A0, INPUT_PULLDOWN);
  //}

  // Initialize LoRa chip.
  lora_rak4630_init();

  // Initialize Serial for debug output
  Serial.begin(115200);
  time_t serial_timeout = millis();
  while (!Serial)
  {
    if ((millis() - serial_timeout) < 5000)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else
    {
      break;
    }
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  Serial.println("=====================================");
  //creat a user timer to send data to server period
  uint32_t err_code;
  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    digitalWrite(LED_BUILTIN2, HIGH);
    return;
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);
  
  // Initialize LoRaWan
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    digitalWrite(LED_BUILTIN2, HIGH);
    return;
  }

  // Start Join procedure
  lmh_join();

  // IC2 sensors
  Wire.begin();
  if (HAS_BME680)
    bme680_init();
  if (HAS_WATER_TEMP)
    water_temp_init();
  if (HAS_DS18B20)
    sensorDS18B20.begin(); 
  if (HAS_GPS)
    gps_init();
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  noInterrupts();
  if (HAS_DEPTH_SENSOR)
    depths = get_depths();
  if (HAS_BATTERY)
    vbat = readVBAT();
  if (HAS_BME680)
    bme.performReading();
  interrupts();
  if (HAS_GPS)
    read_gps_data();
  if (HAS_DS18B20)
    sensorDS18B20.requestTemperatures();
  if (HAS_WATER_TEMP)
    sht31d_measurement = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
  if (HAS_DEPTH_SENSOR)
    Serial.printf("Sensor 1 water depth %d\n", depths);
  if (HAS_BME680){
    Serial.printf("Sensor 2 temperature %f\n", bme.temperature);
    Serial.printf("Sensor 3 humidity %f\n", bme.humidity);
    Serial.printf("Sensor 4 pressure %d\n", bme.pressure);
  }
  if (HAS_WATER_TEMP)
    Serial.printf("Sensor 5 water temp %f\n", sht31d_measurement.t);
  if (HAS_DS18B20)
    Serial.printf("Sensor 6 DS18B20 water temp %f\n", sensorDS18B20.getTempCByIndex(0));
  if (HAS_GPS){
    Serial.printf("Sensor 7 latitude %f\n", flat);
    Serial.printf("Sensor 8 longitude %f\n", flon);
    Serial.printf("Sensor 9 satellites %d\n", sats);
  }
  if (HAS_BATTERY)
    Serial.printf("Sensor 10 Battery %f mv\n", vbat);

  digitalWrite(LED_BUILTIN, LOW);
  delay(15000);
}
