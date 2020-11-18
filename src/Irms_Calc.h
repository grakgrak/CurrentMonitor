//http://www.bizkit.ru/2019/08/01/14312/
#ifndef ACS712_Irms_h
#define ACS712_Irms_h

#include "Arduino.h"

enum scaleFactor
{
    ACS712_5A = 185,
    ACS712_20A = 100,
    ACS712_30A = 66
};

class ACS712_Irms
{
private:
    uint32_t start_time = 0;
    int maxValue = 0;
    int minValue = 0;

public:
    const scaleFactor mVperAmp = scaleFactor::ACS712_20A; // use 100 for 20A Module and 66 for 30A Module and 185 for 5A Module

    const double ADCSamples = 991.0; //1024 samples
    const double maxADCVolt = 3.3;   // Volts
    const double ZeroCorrection = 0.01; //Calibration coefficient
    const double VoltsPerSample = maxADCVolt / ADCSamples;

    double Process()
    {
        maxValue = 0;
        minValue = ADCSamples;
        start_time = millis();
        while (millis() - start_time < 333)
        {
            int readValue = analogRead(A0);
            maxValue = (readValue > maxValue) ? readValue : maxValue;
            minValue = (readValue < minValue) ? readValue : minValue;
        }

        double Vpp = (maxValue - minValue) * VoltsPerSample; //Volts Peak to Peak
        double Vpeak = Vpp / 2.0;                            //Volts peak/Amplitude
        double VRMS = Vpeak * 0.707;
        double AmpsRMS = (VRMS * 1000.0) / mVperAmp - ZeroCorrection;

        //Serial.println("Amps RMS: " + String(AmpsRMS));

        return AmpsRMS;
    }
};

#endif
