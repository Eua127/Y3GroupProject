#include "mbed.h"
#include "stm32l476xx.h"
#include "BufferedSerial.h"




#define BUFFER_SIZE 128

AnalogIn HeartBeatSensor(PA_1);
PwmOut Buzzer(A0);

BufferedSerial serial(USBTX, USBRX);
BufferedSerial bluetooth(ARDUINO_UNO_D1, ARDUINO_UNO_D0);

//RGB LED Class System
//===========================================================================================
class RGBLED
{
public:
    RGBLED(PinName redLpin, PinName greenLpin, PinName blueLpin);
    void write(float red,float green, float blue);
private:
    PwmOut _redLpin;
    PwmOut _greenLpin;
    PwmOut _blueLpin;
};

RGBLED::RGBLED (PinName redLpin, PinName greenLpin, PinName blueLpin)
    : _redLpin(redLpin), _greenLpin(greenLpin), _blueLpin(blueLpin)
{
    //2000Hz 
    _redLpin.period(0.0005);
}

void RGBLED::write(float red,float green, float blue)
{
    _redLpin = red;
    _greenLpin = green;
    _blueLpin = blue;
}

//Setup RGB led using PWM pins and class
RGBLED myRGBLED(D11,D10,D9); //RGB PWM pins
//===============================================================================================

volatile uint8_t buffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile uint8_t isConnected = 0;

void SendData(char data);
char ReceiveData(void);
void SendString(char* str);

EventQueue queue;

void SendData(char data)
{
    while (!(USART1->ISR & USART_ISR_TXE));

    USART1->TDR = data;
}

char ReceiveData(void)
{
    while (!(USART1->ISR & USART_ISR_RXNE))
        ;

    char data = USART1->RDR;

    return data;
}

void SendString(char* str)
{
    while (*str)
    {
        SendData(*str++);
    }
}

void onSerialRx(void)
{
    while (serial.readable())
    {
        serial.read((void*)&buffer[bufferIndex], 1);


        if (++bufferIndex >= BUFFER_SIZE)
        {
            bufferIndex = 0;
        }
    }
}



void onSerialError(void)
{
    serial.read((void*)buffer, BUFFER_SIZE);
    bufferIndex = 0;
    isConnected = 0;
}

void onBluetoothRx(void)
{
    while (bluetooth.readable())
    {
          char c;
        bluetooth.read(&c, 1);
        serial.write(&c, 1);
    }
}

void onBluetoothError(void)
{
    while (bluetooth.readable())
    {
         char data;
        bluetooth.read(&data, 1);
        serial.write(&data, 1);
    }
}

void serialRxCallback(void)
{
   while (serial.readable()) {
        serial.read((void*)&buffer[bufferIndex], 1);
        bufferIndex++;
        if (bufferIndex >= BUFFER_SIZE) {
            bufferIndex = 0;
        }
    }
}

void bluetoothRxCallback(void)
{
    while (bluetooth.readable()) {
        bluetooth.read((void*)&buffer[bufferIndex], 1);
        bufferIndex++;
        if (bufferIndex >= BUFFER_SIZE) {
            bufferIndex = 0;
        }
    }
}

void Siren(void)
{
    int i;
    for (i = 0; i < 32; i = i + 2)
    {
        Buzzer.period(1.0 / 969.0);
        Buzzer = float(i) / 50.0;
        ThisThread::sleep_for(500ms);
        Buzzer.period(1.0 / 800.0);
        ThisThread::sleep_for(500ms);
    }
}

int main()
{
    serial.set_baud(9600);
    bluetooth.set_baud(9600);

     serial.sigio(queue.event(serialRxCallback));
     bluetooth.sigio(queue.event(bluetoothRxCallback));


    while (1)
    {
        char data = ReceiveData();

        if (!bluetooth.readable() && isConnected)
        {
            //disconnection
            isConnected = 0;
            bufferIndex = 0;
            myRGBLED.write(1.0,0.0,0.0); //Red LED
            Siren();
        }
        else if (bluetooth.readable() && !isConnected)
        {
            //connection
            isConnected = 1;
            myRGBLED.write(0.0,1.0,0.0); //Green LED
        }
    }
}
