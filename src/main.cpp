#include <Arduino.h>

#include "roboruka.h"

#define GRIDUI_LAYOUT_DEFINITION
#include "layout.h"

void setup() {
    rkConfig cfg;
    cfg.arm_bone_trims[0] = 25;
    cfg.arm_bone_trims[1] = 30;
    cfg.owner = "MirekBurda"; // Ujistěte se, že v aplikace RBcontrol máte nastavené stejné
    cfg.name = "Capricorn";

    // Ve výchozím stavu lze WiFi na robotovi nastavit pomocí Android aplikace
    // RBControl (verze 1.0 nebo novější) přes Bluetooth.
    // Robot si toto nastavení pamatuje, a znovu ho použije při dalším zapnutí.

    // Můžete WiFi i přímo nastavit zde v kódu, v takovém případě se nastavení
    // nedá z aplikace změnit.

    // Zde v kódu můžete BUĎTO připojit robota na WiFi...
    //cfg.wifi_name = "RukoKraj";
    //cfg.wifi_password = "PlnoRukou";

    // A NEBO vytvořit vlastní WiFi (odkomentovat další dva řádky)
    cfg.wifi_default_ap = true;
    cfg.wifi_ap_password = "12345678";

    cfg.motor_enable_failsafe = false;
    cfg.rbcontroller_app_enable = true;
    cfg.motor_max_power_pct = 100;
    cfg.motor_polarity_switch_left = true;
    cfg.motor_polarity_switch_right = true;
    rkSetup(cfg);

    auto builder = Layout.begin();
    builder.Arm1
        .onPositionChanged([&](Arm& arm) {
            rkArmMoveTo(arm.x(), arm.y());
        })
        .onGrab([&](Arm&) {
            rkArmSetGrabbing(!rkArmIsGrabbing());
        });

    builder.Joystick1
        .onPositionChanged([&](Joystick& joy) {
            rkMotorsJoystick(-joy.x(), -joy.y());
        });

    builder.commit();

    fmt::print("{}'s roboruka '{}' started!\n", cfg.owner, cfg.name);
    fmt::print("Battery at {}%, {}mV\n", rkBatteryPercent(), rkBatteryVoltageMv());
    //rkArmMoveTo(145, -45);
    //rkArmSetGrabbing(false); // open
    rkMotorsSetPower(50, 50); 	
    delay(1000);
    //rkArmMoveTo(145, 65);
    //rkArmSetGrabbing(true); // close 
    rkMotorsSetPower(0, 0); 	
    for (int i = 0; i<10; ++i) {
        // Send text to the android application
        rkControllerSendLog(fmt::format("Tick #{}, battery at {}%, {}mv\n",
            i, rkBatteryPercent(), rkBatteryVoltageMv()));
        // fmt::print("{}\n", i);
        int32_t enL = man.motor(0).enc()->value();
        int32_t enR = man.motor(1).enc()->value();
        fmt::print("{}: {},  {}\n", i, enL, enR);
        delay(500);
    }
}
