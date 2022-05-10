#include <Arduino.h>
#include "RBControl.hpp" // for encoders 
#include "roboruka.h"
using namespace rb;

// #define GRIDUI_LAYOUT_DEFINITION
// #include "layout.h"

void setup() {
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

    int k = 0; 
    while(true) {
        delay(100);
        //fmt::print("{}\n", k); k++;
        if (rkButtonIsPressed(1)) {
            fmt::print("tlacitko\n");
            //rkMotorsSetPower(50, 50);
            man.motor(MotorId::M1).drive(-500*110/100, 50);	// right motor 
            man.motor(MotorId::M2).drive(-500, 50);	// left motor
            delay(100);
            //man.motor(MotorId::M1).drive(0, 0);	// nezastavi, dokud nedojede minuly drive  
            //man.motor(MotorId::M2).drive(0, 0);
            rkMotorsSetPower(0, 0); // zastavi ihned, i kdyz probiha drive  	
            int32_t enR = man.motor(MotorId::M1).enc()->value();  // reading encoder
            int32_t enL = man.motor(MotorId::M2).enc()->value();
            fmt::print("{}: {},  {}\n", k, enL, enR);
        }
    }
}

            //rkArmMoveTo(145, -45);
            //rkArmSetGrabbing(false); // open
 
            //rkArmMoveTo(145, 65);
            //rkArmSetGrabbing(true); // close 