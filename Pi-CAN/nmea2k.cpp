// Application using NMEA2000 libray from Timo Lappalainen (ttlappalainen)
// https://github.com/ttlappalainen/NMEA2000 to read Airmar DST-810 CAN triducer
// and convert can speed to 12VDC PWM output that can be read by Mastercraft
// Indmar MEFI5 ECU to replace failed proprietary triducer
// Hardware is Pi Pico W, Adafruit CAN PiCowbell Hat, Airmar DST-810
// There is also an LCD 2x16 LCD screen for visual that the converter is
// reading values from the CAN bus and is operational.


#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "NMEA2000_CAN.h"
#include "N2kMsg.h"
#include "NMEA2000.h"
#include "N2kMessages.h"
#include "hardware/pwm.h"
//#include "N2kMessagesEnumToStr.h"
#include "lcd_1602_i2c.h"

LCD16X2 lcdScreen;
double currentSpeed;
double currentTemp;

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void PaddleWheel(const tN2kMsg &N2kMsg);
void WaterTemp(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {128259L, &PaddleWheel},
  {130316L , &WaterTemp},
  {0,0}
};

void SetPWMFreq(uint slice_num, uint freq_hz) {
  if (freq_hz == 0) {
      pwm_set_enabled(slice_num, false); // Disable PWM at 0 Hz
      return;
  }

  uint32_t clock = 125000000; // Default PWM clock frequency (125 MHz)
  uint32_t divider = clock / (freq_hz * 65535);

  if (divider < 1) divider = 1;
  if (divider > 255) divider = 255;

  pwm_set_clkdiv(slice_num, divider);
  pwm_set_wrap(slice_num, 65535);
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 65535 / 2); // 50% duty
  pwm_set_enabled(slice_num, true);
}

void PaddleWheel(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double Speed = 0;
  double Ground = 0;
  tN2kSpeedWaterReferenceType speedWR;

  if (ParseN2kBoatSpeed(N2kMsg, SID, Speed, Ground, speedWR)) {
    double temp;
      // DST810 units for speed are m/s so convert to MPH
      temp = msToMPH(Speed);
      currentSpeed = temp;
  }
}

void WaterTemp(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char TempInstance;
  tN2kTempSource TempSource;
  double ActualTemperature;
  double SetTemperature;
  
  if (ParseN2kTemperatureExt(N2kMsg, SID, TempInstance, TempSource, ActualTemperature, SetTemperature)) {
    // DST810 units are Kelvin so convert to Fahrenheit
    currentTemp = KelvinToF(ActualTemperature);
  } 
}

//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}

int main() {

    int freq = 0;

    char str[16];  // Ensure this array is large enough to hold the resulting string
    char str2[16];  // Ensure this array is large enough to hold the resulting string

    stdio_init_all(); // this is needed for the pico to start a serial port in windows
    lcdScreen.lcd_init();

    const int PWM_OUTPUT_PIN = 28;
    gpio_set_function(PWM_OUTPUT_PIN, GPIO_FUNC_PWM);  // Set GPIO 28 to PWM mode
  
    uint slice_num = pwm_gpio_to_slice_num(PWM_OUTPUT_PIN);
    SetPWMFreq(slice_num, freq);

    NMEA2000.Open();

    NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(100);
    //NMEA2000.SetForwardStream((N2kStream*)&stdout); // PC output on due native port
    //NMEA2000.EnableForward(false);                  // Disable all msg forwarding to USB (=Serial)
    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);    // Show in clear text
    NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

    while (true) {

         NMEA2000.ParseMessages();

        // Convert double to string
        sprintf(str, "Speed %f", currentSpeed);
        sprintf(str2, "%.1f%c", currentTemp, 223); // %c - 223 is degree symbol

        lcdScreen.lcd_print_top(str);
        lcdScreen.lcd_print_bottom(str2);

        SetPWMFreq(slice_num, currentSpeed * 5.6);

        sleep_ms(1);
    }

    return 0;
}