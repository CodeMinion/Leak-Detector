#include <bluefruit.h>

#define STATUS_LED (19)
/* Home Environment Sensing Service Definitions
   Home Environment Service:  0x28FF
   Leak Sensing Char: 0x2A38
*/

const char* DEVICENAME = "Leak HP%X%X";
const char* DEVICE_MODEL = "HomePi Motion";
const char* DEVICE_MANUFACTURER = "Rounin Labs";

/** 
 *  Home Environmental Sensing Service.
 * This service exposes measurement data from an home sensor intended for home automation applications. 
 * A wide range of environmental parameters is supported.
 **/
const int UUID16_SVC_HOME_ENV_SENSE = 0x28FF;
const int UUID16_CHR_LEAK_SENSE_MEASUREMENT = 0x4A37;

const int VAL_LEAK_TOP = 0x01; // Leak detected by top probe
const int VAL_LEAK_BOTTOM = 0x02; // Leak detected by bottom probe

volatile int leakDetectedValue = 0x00;

BLEService        hess = BLEService(UUID16_SVC_HOME_ENV_SENSE);
BLECharacteristic lsc = BLECharacteristic(UUID16_CHR_LEAK_SENSE_MEASUREMENT);

BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance


const int BATTERY_INFO_PIN = A7;
// Level at which to signal low battery
const int LOW_BATT = 25;
int lastBattLevel = 0;
// Interval at which the battery is reported in MS
const int BATT_REPORTING_INTERVAL = 1000; 

const int iLeakTopInterruptPin = A2;
const int iLeakBottomInterruptPin = A3;

/**
 * Notification task informaiton
 */
static TaskHandle_t _notifyLeakValueHandle;
uint32_t _notifyLeakValueStackSize = 512;
void TaskNotifyLeak(void * pvParameters);


void setup() 
{
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb

  Serial.println("Setting up Leak Sensor");
  Serial.println("-----------------------\n"); 

  pinMode(STATUS_LED, OUTPUT);
 
  // Initialise the Bluefruit module
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // We'll control the LED so we can save some power.
  //Bluefruit.autoConnLed(false);

  // Set the advertised device name (keep it short!)
  Serial.print("Setting Device Name to ");
  uint8_t address [6];
  Bluefruit.Gap.getAddr(address);
  char nameBuff[50] = "";
  sprintf(nameBuff, DEVICENAME, address[1],address[0]);
  Serial.println(nameBuff);
  Bluefruit.setName(nameBuff);

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer(DEVICE_MANUFACTURER);
  bledis.setModel(DEVICE_MODEL);
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Leak Sensor Service");
  setupLS();

   // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();
  Serial.println("\nAdvertising");

  //Setup Motion Sensor.
  setupLeakSensor();
  setupLeakDectectionInterrupts();

  // Setup FreeRTOS notification tasks
  Serial.println("Setting up FreeRTOS notification task(s)");
  setupNotificationTasks();

  Serial.println("\nSetup Complete!");
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
  // Disable the BT connection LED to save battery.
  digitalWrite(STATUS_LED, LOW);
}

/**
   Helper function to notify the battery level.
*/
void notifyBatteryLevel(int level)
{
  blebas.notify(level);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
   https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/cores/nRF5/nordic/softdevice/s140_nrf52_6.1.1_API/include/ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");

  // Consider ligthing LED when it is disconnected.
}

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
  // Display the raw request packet
  Serial.print("CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  Serial.print(cccd_value);
  Serial.println("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  if (chr.uuid == lsc.uuid) 
  {
    if (chr.notifyEnabled()) 
    {
      Serial.println("Leak Sensing Measurement 'Notify' enabled");
    } 
    else 
    {
      Serial.println("Leak Sensing Measurement 'Notify' disabled");
    }
  }
}

void setupLS(void)
{
  // Configure the Motion Sensing service
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Leak Sense Measurement     0x2A37  Mandatory   Notify
  hess.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  lsc.setProperties(CHR_PROPS_NOTIFY);
  lsc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  lsc.setFixedLen(2);
  lsc.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  lsc.setUserDescriptor("Leak Sense Measurement. A value of greater than 0 means detected.");
  lsc.begin();
  uint8_t lsdata[1] = { 0 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  lsc.notify(lsdata, 1);                   // Use .notify instead of .write!

}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Motion Sensor Service UUID
  Bluefruit.Advertising.addService(hess);

  // Include Name
  Bluefruit.Advertising.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

/**
   Set up to read from the Motion Sensor ouput pin.
*/
void setupLeakSensor()
{
  //pinMode(iLeakTopInterruptPin, INPUT);
  pinMode(iLeakBottomInterruptPin, INPUT);
}

void setupLeakDectectionInterrupts()
{
  attachInterrupt(digitalPinToInterrupt(iLeakTopInterruptPin), leakChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(iLeakBottomInterruptPin), leakChanged, CHANGE);
}

/**
 * Interrupt function called when 
 * there is a change on the pins detecting a leak.
 */
void leakChanged()
{
  Serial.println("Leak Change");

  /*
  BaseType_t yieldRequired;

  yieldRequired = xTaskResumeFromISR(_notifyLeakValueHandle);

  if(yieldRequired == pdTRUE)
  {
    taskYIELD();
  }*/

    BaseType_t xHigherPriorityTaskWoken;
    /* Clear the interrupt. */
    //prvClearInterruptSource();

    /* xHigherPriorityTaskWoken must be initialised to pdFALSE.
    If calling vTaskNotifyGiveFromISR() unblocks the handling
    task, and the priority of the handling task is higher than
    the priority of the currently running task, then
    xHigherPriorityTaskWoken will be automatically set to pdTRUE. */
    xHigherPriorityTaskWoken = pdFALSE;

    /* Unblock the handling task so the task can perform any processing
    necessitated by the interrupt.  xHandlingTask is the task's handle, which was
    obtained when the task was created.  vTaskNotifyGiveFromISR() also increments
    the receiving task's notification value. */
    vTaskNotifyGiveFromISR( _notifyLeakValueHandle, &xHigherPriorityTaskWoken );

    /* Force a context switch if xHigherPriorityTaskWoken is now set to pdTRUE.
    The macro used to do this is dependent on the port and may be called
    portEND_SWITCHING_ISR. */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void setupNotificationTasks()
{
  xTaskCreate(
    TaskNotifyLeak
    ,(const portCHAR *)"NotifyLeak" // A name just for humans
    , _notifyLeakValueStackSize // Stack Size
    , NULL // Parameters, should be address to variable on heap not stack.
    , TASK_PRIO_LOW // priority
    , &_notifyLeakValueHandle
    );
}

void TaskNotifyLeak(void * pvParameters)
{
  (void)pvParameters;
  BaseType_t xEvent;
  uint32_t ulNotifiedValue;
  for (;;)
  {

    /* Block indefinitely (without a timeout, so no need to check the function's
        return value) to wait for a notification.  Here the RTOS task notification
        is being used as a binary semaphore, so the notification value is cleared
        to zero on exit.  NOTE!  Real applications should not block indefinitely,
        but instead time out occasionally in order to handle error conditions
        that may prevent the interrupt from sending any more notifications. */
       ulNotifiedValue = ulTaskNotifyTake( pdTRUE,          /* Clear the notification value before
                                           exiting. */
                          portMAX_DELAY ); /* Block indefinitely. */

      /* ulNotifiedValue holds a count of the number of outstanding
      interrupts.  Process each in turn. */
      while( ulNotifiedValue > 0 )
      {
          
        ulNotifiedValue--;
        if(Bluefruit.connected())
        {
          Serial.println("Notifying Leak Data");
          // Use this chance to update the battery value as well.
          int batteryLevel = readBatteryLevel();
          notifyBatteryLevel(batteryLevel);   
    
          if(batteryLevel < LOW_BATT)
          {
            digitalToggle(LED_RED);
          }
    
          int topReading = digitalRead(iLeakTopInterruptPin) != HIGH ? 0x01 : 0x00;
          int bottomReading = digitalRead(iLeakBottomInterruptPin) != HIGH ? 0x02 : 0x00;
    
          Serial.print("Top: "); Serial.println(topReading);
          Serial.print("Bottom: "); Serial.println(bottomReading);
          
          int leakDetectionValue = topReading | bottomReading;
          notifyLeakDetectionValue(leakDetectionValue);
        }
        else
        {
          digitalToggle(STATUS_LED);
        }  
  
      }
        
    // The task suspends itself after notifying.
    //vTaskSuspend( NULL );
  }
  
}

void notifyLeakDetectionValue(int value)
{
   uint8_t leakData[1] = {value};
   if(lsc.notify(leakData, sizeof(leakData)))
   {
      Serial.print("Leak Value Measurement updated to: ");
      Serial.println(leakData[0]);
   }
   else
   {
      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
   }
   
}

void loop() 
{
  if(Bluefruit.connected() )
  {
    int batteryLevel = readBatteryLevel();

    // Notify the battery level only if it has changed.
    if(batteryLevel != lastBattLevel)
    {
      notifyBatteryLevel(batteryLevel);
      lastBattLevel = batteryLevel;
    }

  }

  // Only send update once per second
  delay(BATT_REPORTING_INTERVAL);

}



/**
   Reads the battery level from the feather pin.
   Note: Updated from: https://learn.adafruit.com/adafruit-feather-32u4-basic-proto/power-management
*/

/**
 * Excerpt From https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/examples/Hardware/adc_vbat/adc_vbat.ino
 */
#define VBAT_PIN          (A7)
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.71275837F)   // 2M + 0.806M voltage divider on VBAT = (2M / (0.806M + 2M))
#define VBAT_DIVIDER_COMP (1.403F)        // Compensation factor for the VBAT divider

int readVBAT(void) 
{
  int raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(VBAT_PIN);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  return raw;
}

uint8_t mvToPercent(float mvolts) {
    uint8_t battery_level;

    if (mvolts >= 3000)
    {
        battery_level = 100;
    }
    else if (mvolts > 2900)
    {
        battery_level = 100 - ((3000 - mvolts) * 58) / 100;
    }
    else if (mvolts > 2740)
    {
        battery_level = 42 - ((2900 - mvolts) * 24) / 160;
    }
    else if (mvolts > 2440)
    {
        battery_level = 18 - ((2740 - mvolts) * 12) / 300;
    }
    else if (mvolts > 2100)
    {
        battery_level = 6 - ((2440 - mvolts) * 6) / 340;
    }
    else
    {
        battery_level = 0;
    }

    return battery_level;
}

int readBatteryLevel()
{
  /*
  float measuredvbat = analogRead(BATTERY_INFO_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);
  Serial.print("CBat: " ); Serial.println(map(measuredvbat, 3.0, 4.2, 0, 100));
  */
   int vbat_raw = readVBAT();

  // Convert from raw mv to percentage (based on LIPO chemistry)
  uint8_t vbat_per = mvToPercent(vbat_raw * VBAT_MV_PER_LSB);
  Serial.print("VBat: " ); Serial.print(vbat_per); Serial.println("%");
  return vbat_per;
}
