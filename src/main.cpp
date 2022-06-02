#include <Arduino.h> // from Roboruka
#include <SmartLeds.h>
#include "RBControl.hpp" // for encoders 
#include "roboruka.h"
using namespace rb;

#include <driver/spi_master.h> // from pixy2
#include <driver/uart.h>
#include "pixy2/pixy2.hpp"
#include "pixy2/spi.hpp"
#include "pixy2/i2c.hpp"
using namespace pixy2;

// nastaveni paze roboruky: https://roboticsbrno.github.io/RB3201-RBControl-Roboruka-library/group__arm.html
//todo nastaveni konstant - maximální hodnoty poloh pro serva 
//todo dokončení komunikace mezi nano a esp32 - hlavička 1 bajt, zprávu pošlu 3x za sebou a vyhodnotím, if přišla pokaždé stejně 
// ukládám si minulou hodnotu, if nová hodnota nepřišla a minulá hodnota dost velká - v pohodě, if ne, brzdím až stojím  
//todo musí zastavit za nastavený čas - max. 126 sekund (a rozsvítí všechny LED vzadu) 
//todo přesné řízení 
//todo iLED pásek, který indikuje, co měří ultrazvuky - zelená/bílá - volno, červená -stát 
//todo předstartovní taneček - po stisku  tl 1 
//todo start na vytažení lanka -> rozpojení tl. 3 
// dotaz: na jakém napětí běží data pro LX16A? 

// kalibrace pixy2: s klepetem na zemi a SetLedAll(127, 127, 127); 
// potom pro klepeto ve výšce cca 10 cm a víc nastavit SetLedAll(255, 255, 255);
// příjezd do prostoru kostek, kameru více nahoru, pootočit vpravo a vlevo, najít nejblžší kostku, 
// poodjet, aby se dalo klepeto položit na zem - nesmí být jiná kostka těsně kolem klepeta 
// dojet ke kostce, vzít ji, odvést 

bool LORRIS = false; // varianty debugu:  true - data UART0 jdou do Lorris, false - data jdou do Serial monitoru 

const int LED_COUNT = 16; // zacatek nastaveni iLED  -------------------
const int DATA_PIN = 32;
const int CHANNEL = 0;
// SmartLed -> RMT driver (WS2812/WS2812B/SK6812/WS2813)
SmartLed leds(LED_WS2812, LED_COUNT, DATA_PIN, CHANNEL, DoubleBuffer);

void SetLedAll(uint8_t R, uint8_t G, uint8_t B) // iLED pro přisvícení kamery 
{
    for (int i = 0; i < LED_COUNT; i++)
        leds[i] = Rgb{R, G, B};
    leds.wait();
    leds.show();
}       // konec nastaveni iLED ---------------------------------------

void setup() {
    // SetLedAll(0, 0, 0);  
    // SetLedAll(127, 127, 127);  
     SetLedAll(255, 255, 255);

    rkConfig cfg;
    cfg.arm_bone_trims[0] = 25;  // nastaveni posunu serva robopaze vuci nulove poloze
    cfg.arm_bone_trims[1] = 30;

    cfg.motor_enable_failsafe = false;
    cfg.rbcontroller_app_enable = true;
    cfg.motor_max_power_pct = 100;
    cfg.motor_polarity_switch_left = true;
    cfg.motor_polarity_switch_right = true;
    auto &man = Manager::get();
    man.initSmartServoBus(5, (gpio_num_t)cfg.pins.arm_servos); // nastaveni poctu serv na roboruce 
    rkSetup(cfg);
    
    uart_config_t uart_config = {  
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));

    spi_bus_config_t busCfg;        // zacatek nastaveni kamery +++++++++++++++++++++++++++++++++
    busCfg.mosi_io_num = GPIO_NUM_25; 
    busCfg.miso_io_num = GPIO_NUM_26; 
    busCfg.sclk_io_num = GPIO_NUM_27; 
    busCfg.quadwp_io_num = -1;
    busCfg.quadhd_io_num = -1;
    busCfg.flags = SPICOMMON_BUSFLAG_MASTER;
    busCfg.intr_flags = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &busCfg, 0)); // HSPI_HOST je typ enum, nepsat číslo (1), ale tu danou konstantu z výběru  (HSPI_HOST)

    auto linkRes = LinkSpi::addSpiDevice(HSPI_HOST, 2000000); // nastavení rychlost komunikace SPI na běžných pinech, aby to stíhaly RBC i zařízení - nutno vyzkoušet 
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    LinkSpi link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkSpi>(std::move(link));

    ESP_ERROR_CHECK(pixy.waitForStartup());
    GetBlocksContext blocksCtx;     // konec nastaveni kamery +++++++++++++++++++++++++++++++++++++

    

    Serial2.begin(115200, SERIAL_8N1, 4, 14); // Rx = 4 Tx = 14

    fmt::print("{}'s roboruka '{}' started!\n", cfg.owner, cfg.name);
    fmt::print("Battery at {}%, {}mV\n", rkBatteryPercent(), rkBatteryVoltageMv());

    // rkArmSetServo(3, 60); // parkovaci pozice 
    int k = 120; 
    int IDservo = 4;

    while(true) {     
        if (Serial2.available() > 0) { 
            byte readData[10]= { 1 }; //The character array is used as buffer to read into.
            int x = Serial2.readBytes(readData, 8); //It require two things, variable name to read into, number of bytes to read.
            Serial.println(x); //display number of character received in readData variable.
            for(int i = 0; i<10; i++) {
                if(!LORRIS) { 
                    printf("i: %i, ", readData[i]); 
                }
            }               
        }
        
        // // zacatek nastavovani serv ****************************************************************************************
        // if (rkButtonIsPressed(1)) {
        //     k += 10;
        //     rkArmSetServo(IDservo, k);
        // }
        // if (rkButtonIsPressed(2)) { 
        //     k-=10;
        //     rkArmSetServo(IDservo, k);      
        // }
        // if (rkButtonIsPressed(3)) {
        //     k += 1;
        //     rkArmSetServo(IDservo, k);
        // }
        // if(!LORRIS) {
        //     fmt::print("vision: {}, position: {}\n", k, rkArmGetServo(IDservo) ); //konec nastavovani serv *****************
        // }

        auto err = pixy.getColorBlocks(1|2|4|8, 8, blocksCtx); // cervena | modra | zelena | cerna
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error1: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if(LORRIS) {
            const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
            uart_write_bytes(UART_NUM_0, (const char*)lorrisHeader, sizeof(lorrisHeader));
            if(blocksCtx.blocks.size() > 0) {
                uart_write_bytes(UART_NUM_0, (const char*)blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
            }
        
            vTaskDelay(1000); // aby Lorris nepadala, musí tady být pauza 

            uint8_t orderTest[] = { 0xFF,  0x02, 0x02, 0x03, 15  };  // test exportu jiných dat než z kamery do Lorris 
            uart_write_bytes(UART_NUM_0, (const char*)orderTest, sizeof(orderTest));
        }
        else  {
            // int poziceX = blocksCtx.blocks[0]->x;
            // int poziceY = blocksCtx.blocks[0]->y;
            // int poziceX2 = blocksCtx.blocks[1]->x;
            // int poziceY2 = blocksCtx.blocks[1]->y;

           // printf("i: %i, %i, %i, %i\n", poziceX, poziceY, poziceX2, poziceY2);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));  
    }
}


