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

    cfg.motor_enable_failsafe = true;
    cfg.rbcontroller_app_enable = true;
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
    rkArmMoveTo(145, -45);
    delay(2000);
    for (int i = 0; i<10; ++i) {
        // Send text to the android application
        rkControllerSendLog(fmt::format("Tick #{}, battery at {}%, {}mv\n",
            i, rkBatteryPercent(), rkBatteryVoltageMv()));
        fmt::print("{}\n", i);
        rkArmMoveTo(i, i);
        delay(1000);
    }
}
