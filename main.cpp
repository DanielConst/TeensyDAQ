#include <Arduino.h>
#include <ADC.h>
#include <usb_serial.h>

#define line_offset 44
#define BUFFER_SIZE 2024
#define RESOLUTION 512


//Variable Declarations

volatile unsigned int pixel_counter = 0;
volatile unsigned int line_counter = 0;
volatile unsigned int sample_counter = 0;
volatile unsigned int cum_sensorvalue = 0;
char serialBuffer[BUFFER_SIZE];
volatile int bufferIndex = 0;
int mean_pixel_value;
unsigned int IMAGE_END = RESOLUTION+line_offset-1;

//Interrupt Service Routines
void pixel_ISR(){
  mean_pixel_value = cum_sensorvalue/sample_counter;
  //mean_pixel_value = pixel_counter+line_counter;
  if (pixel_counter >= line_offset && pixel_counter <= IMAGE_END){//only send actual pixels. (new line signal comes 44 pixels before 1st actual pixel (overall 88 after end of line))
    int written = snprintf(serialBuffer + bufferIndex, BUFFER_SIZE - bufferIndex, "<%u,%u,%u>\n", pixel_counter - line_offset, line_counter, mean_pixel_value);
      bufferIndex += written;
  }
  __disable_irq();
  cum_sensorvalue=0;
  sample_counter=0; 
  pixel_counter++;
__enable_irq();
  // Falls der Buffer zu 75% voll ist, senden
  if (bufferIndex >= BUFFER_SIZE * 2 / 4) {
    Serial.write(serialBuffer, bufferIndex);
    bufferIndex = 0;
  }
}

void line_ISR(){
  // Serial.println("l");
  __disable_irq();
  pixel_counter=0;
  line_counter++;
  __enable_irq();
}

void frame_ISR(){
  // Serial.println("f");
  int written = snprintf(serialBuffer + bufferIndex, BUFFER_SIZE - bufferIndex, "<f>\n");
  __disable_irq();
  line_counter=0;
  pixel_counter=2;
  __enable_irq();
  bufferIndex += written;
}

ADC *adc = new ADC(); // adc object
#define analogReadPin 24

void setup() {
  // put your setup code here, to run once:  

  //Enable DWT and CYCCNT for ARM Clock Counters
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;  

  //open Serial port (teensy automatically uses USB serial)
  Serial.begin(921600);
  while (!Serial) {} //"what are you doing, Lain? Serial Experiments."
  //Digital Input Pins
  pinMode(5, INPUT);//Pin DI05
  pinMode(4, INPUT);//Pin DI06
  pinMode(3, INPUT);//Pin DI07
  pinMode(2, INPUT);//Pin DI08

  //Analog Input Pin
  pinMode(analogReadPin, INPUT);//Pins 24-27

  //setup hardware interrupts on rising edge of pixelclock signals
  NVIC_SET_PRIORITY(IRQ_GPIO6789, 32);
  attachInterrupt(5,pixel_ISR,RISING);//Pin DI05
  attachInterrupt(4,line_ISR,RISING);//Pin DI06
  attachInterrupt(3,frame_ISR,RISING);//Pin DI07
  //attachInterrupt(2,stack_ISR,RISING);//Pin DI08

  //ADC settings
  adc->adc0->setAveraging(32); // set number of Hardware averages
  adc->adc0->setResolution(12); // change the resolution to 16 bit
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  adc->adc0->startContinuous(analogReadPin);//start continous measurements
  //adc->adc0->recalibrate();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (true){
    //sample sensor value
    int sensorValue = adc->adc0->analogReadContinuous();
    __disable_irq();
    cum_sensorvalue+=sensorValue;
    sample_counter++;
    __enable_irq();
    delayNanoseconds(200);
  }
}
