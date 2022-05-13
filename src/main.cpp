#include <Arduino.h>
#include <SmartLeds.h>
#include "RBControl.hpp" // for encoders 
#include "roboruka.h"
using namespace rb;

// nastaveni paze roboruky: https://roboticsbrno.github.io/RB3201-RBControl-Roboruka-library/group__arm.html

const int LED_COUNT = 15;
const int DATA_PIN = 32;
const int CHANNEL = 0;

// SmartLed -> RMT driver (WS2812/WS2812B/SK6812/WS2813)
SmartLed leds(LED_WS2812, LED_COUNT, DATA_PIN, CHANNEL, DoubleBuffer);

void SetLedAll(uint8_t R, uint8_t G, uint8_t B)
{
    for (int i = 0; i < LED_COUNT; i++)
        leds[i] = Rgb{R, G, B};
    leds.wait();
    leds.show();
}

void setup() {
    SetLedAll(64, 0, 0);
    rkConfig cfg;
    cfg.arm_bone_trims[0] = 25;
    cfg.arm_bone_trims[1] = 30;
    cfg.owner = "MirekBurda"; // Ujistěte se, že v aplikace RBcontrol máte nastavené stejné
    cfg.name = "Capricorn";

    cfg.motor_enable_failsafe = false;
    cfg.rbcontroller_app_enable = true;
    cfg.motor_max_power_pct = 100;
    cfg.motor_polarity_switch_left = true;
    cfg.motor_polarity_switch_right = true;
    rkSetup(cfg);
    auto& man = Manager::get();

    fmt::print("{}'s roboruka '{}' started!\n", cfg.owner, cfg.name);
    fmt::print("Battery at {}%, {}mV\n", rkBatteryPercent(), rkBatteryVoltageMv());

    rkArmSetGrabbing(false);
    fmt::print("open: {}\n", rkArmGetServo(2) );
    delay(1000);
    rkArmSetGrabbing(true);
    fmt::print("close: {}\n", rkArmGetServo(2) );

    int k = 0; 
    while(true) {
        delay(100);
        
        if (rkButtonIsPressed(1)) {
            fmt::print("tlacitko\n");
            fmt::print("{}\n", k); k++;
            // //rkMotorsSetPower(50, 50);
            // man.motor(MotorId::M1).drive(-500*110/100, 50);	// right motor 
            // man.motor(MotorId::M2).drive(-500, 50);	// left motor
            // delay(100);
            // //man.motor(MotorId::M1).drive(0, 0);	// nezastavi, dokud nedojede minuly drive  
            // //man.motor(MotorId::M2).drive(0, 0);
            // rkMotorsSetPower(0, 0); // zastavi ihned, i kdyz probiha drive  	
            // int32_t enR = man.motor(MotorId::M1).enc()->value();  // reading encoder
            // int32_t enL = man.motor(MotorId::M2).enc()->value();
            // fmt::print("{}: {},  {}\n", k, enL, enR);
            rkArmSetServo(2, 120+k);  // 124 deg - rovnoběžná klepeta 
            delay(1000);
            fmt::print("vision: {}, position: {}\n", 120+k, rkArmGetServo(2) ); // pozice klepeta -  124 deg - 
        }
    }
}

            //rkArmMoveTo(145, -45);   // je uprostred v prostoru
            //rkArmSetGrabbing(false); // open: 87 deg
 
            //rkArmMoveTo(145, 65);   // je tesne nad zemi 
            //rkArmSetGrabbing(true); // close: 160 deg 