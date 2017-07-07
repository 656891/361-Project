#include "Arduino_FreeRTOS.h"
#include <avr/power.h>
#include <avr/sleep.h>
#include <CapacitiveSensor.h>


CapacitiveSensor   cs_4_2 = CapacitiveSensor(11, 12);       // 10M resistor between pins 11 & 12, pin 12 is sensor pin, add a wire and or foil if desired
int sensitivity = 700;


/* The task function. */
void vTaskFunction1( void *pvParameters );
void vTaskFunction2( void *pvParameters );
void vTaskFunction3( void *pvParameters );
void vTaskFunction4( void *pvParameters );

/* Define the strings that will be passed in as the task parameters. These are
  defined const and off the stack to ensure they remain valid when the tasks are
  executing. */
const char *pcTextForTask1 = "Task 1 is running\r\n";
const char *pcTextForTask2 = "Task 2 is running\t\n";
const char *pcTextForTask3 = "Task 3 is running\t\n";
const char *pcTextForTask4 = "Task 4 is running\t\n";
/*-----------------------------------------------------------*/

void setup( void )
{
  cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate for the water level sensor
  //The rest of setup disables unneeded features, to save power
  // Digital Input Disable on Analogue Pins
  // When this bit is written logic one, the digital input buffer on the corresponding ADC pin is disabled.
  // The corresponding PIN Register bit will always read as zero when this bit is set. When an
  // analogue signal is applied to the ADC7..0 pin and the digital input from this pin is not needed, this
  // bit should be written logic one to reduce power consumption in the digital input buffer.

  DIDR0 = 0x3F;

  // Analogue Comparator Disable
  // When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
  // This bit can be set at any time to turn off the Analogue Comparator.
  // This will reduce power consumption in Active and Idle mode.
  // When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
  // Otherwise an interrupt can occur when the ACD bit is changed.

  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);
  // Disable the Analog to Digital Converter module.
  power_adc_disable();

  // Disable the Serial Peripheral Interface module.
  power_spi_disable();

  // Disable the Two Wire Interface or I2C module.
  power_twi_disable();

  // Disable the Timer 0 module. millis() will stop working.
  power_timer0_disable();

  // Disable the Timer 1 module.
  power_timer1_disable();

  // Disable the Timer 2 module. Used for RTC in Goldilocks 1284p devices.
  power_timer2_disable();

  //#define portUSE_WDTO WDTO_500MS

  //Serial.begin(9600);
  pinMode(7, OUTPUT); //To pump control transistor
  pinMode(10, OUTPUT); //To Vcc of the sensor

  pinMode(2, OUTPUT); //Rotary switch position testing pins
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  digitalWrite(3, HIGH); //Pullup Resistors for the rotary read pins
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);

  digitalWrite(7, LOW); //Initializing other control pins to be turned off
  digitalWrite(10, LOW);
  digitalWrite(13, LOW);

  /* Create the first task at priority 1... */
  xTaskCreate( vTaskFunction1, "Measure, pump", 200, (void*)pcTextForTask1, 3, NULL );
  xTaskCreate( vTaskFunction2, "Water level check", 200, (void*)pcTextForTask2, 1, NULL );


  /* Start the scheduler so our tasks start executing. */
  vTaskStartScheduler();

  for ( ;; );
  // return 0;
}
/*-----------------------------------------------------------*/

void vTaskFunction1( void *pvParameters )
{
  char *pcTaskName;

  /* The string to print out is passed in via the parameter. Cast this to a
    character pointer. */
  pcTaskName = ( char * ) pvParameters;

  /* As per most tasks, this task is implemented in an infinite loop. */
  for ( ;; )
  {

    power_adc_enable();

    digitalWrite(10, HIGH); //Turn on the moisture sensor
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    vTaskSuspendAll(); //Prevent context switches while reading

    int sensorValue = analogRead(A0); //Take the moisture reading
    digitalWrite(10, LOW); //Turn off the moisture sensor
    power_adc_disable();
    //Serial.println(sensorValue);

    xTaskResumeAll();

    digitalWrite(2, LOW); //Turn on ouput to test which pin gets put low by it

    if (!digitalRead(3)) //Read the switch position, and set sensitivity accordingly
      sensitivity = 1000;
    else if (!digitalRead(4))
      sensitivity = 900;
    else if (!digitalRead(5))
      sensitivity = 800;
    else if (!digitalRead(6))
      sensitivity = 700;

    digitalWrite(2, HIGH); //Done determining switch position, return to high to save power

    if (sensorValue > sensitivity)
    {
      //Serial.println("pumping...");
      digitalWrite(7, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(7, LOW);
      //Serial.println("done");
    }
    digitalWrite(7, LOW); //Redudant pump turn off, in case noise issues prevented the pin from turning off before
    /* Delay for a period. This time we use a call to vTaskDelay() which
      puts the task into the Blocked state until the delay period has expired.
      The delay period is specified in 'ticks'. */
    vTaskDelay( 1800000 / portTICK_PERIOD_MS ); //30 min
  }
}

void vTaskFunction2( void *pvParameters )
{
  char *pcTaskName;

  /* The string to print out is passed in via the parameter. Cast this to a
    character pointer. */
  pcTaskName = ( char * ) pvParameters;

  /* As per most tasks, this task is implemented in an infinite loop. */
  for ( ;; )
  {
    vTaskSuspendAll();
    long waterLev =  cs_4_2.capacitiveSensorRaw(30); //Take a capacitive measurement
    //Serial.println(waterLev);
    if (waterLev < 5000) //High numbers indicate higher capacitance (the wire is in contact with water)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
    xTaskResumeAll();

    vTaskDelay(1800000 / portTICK_PERIOD_MS); //Block for 30 minutes
  }
}



//------------------------------------------------------------------------------
void loop() { //The loops is the idle task, consisting of power-saving measures
  set_sleep_mode( SLEEP_MODE_PWR_DOWN );

  portENTER_CRITICAL();

  sleep_enable();

  // Only if there is support to disable the brown-out detection.
  // If the brown-out is not set, it doesn't cost much to check.
#if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
#endif

  portEXIT_CRITICAL();

  sleep_cpu(); // Good night.

  // Ugh. Yawn... I've been woken up. Better disable sleep mode.
  // Reset the sleep_mode() faster than sleep_disable();
  sleep_reset();
}
