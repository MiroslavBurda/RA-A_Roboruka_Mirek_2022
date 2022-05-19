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
//todo nastaveni konstant
const int LED_COUNT = 16;
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
    // SetLedAll(0, 0, 0); 
    SetLedAll(255, 255, 255);

    gpio_num_t SerialRx1 = GPIO_NUM_14;
    gpio_num_t SerialTx1 = GPIO_NUM_4;
    pinMode(SerialRx1, OUTPUT);
    pinMode(SerialTx1, OUTPUT);
    digitalWrite(SerialRx1, HIGH);
    digitalWrite(SerialTx1, HIGH);
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
    auto &man = Manager::get();
    man.initSmartServoBus(5, (gpio_num_t)cfg.pins.arm_servos); // nastaveni poctu serv na roboruce 
    rkSetup(cfg);

    uart_config_t uart_config = {  // zacatek nastaveni kamery 
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));

    spi_bus_config_t busCfg;
    busCfg.mosi_io_num = GPIO_NUM_25; //13 
    busCfg.miso_io_num = GPIO_NUM_26; //12
    busCfg.sclk_io_num = GPIO_NUM_27; //14
    busCfg.quadwp_io_num = -1;
    busCfg.quadhd_io_num = -1;
    busCfg.flags = SPICOMMON_BUSFLAG_MASTER;
    busCfg.intr_flags = 0;

    // spi_host_device_t SPI2_HOST = (spi_host_device_t)1; 
    // spi_host_device_t SPI2_HOST = 1;
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &busCfg, 0)); 

    auto linkRes = LinkSpi::addSpiDevice(HSPI_HOST, 2000000);
    ESP_ERROR_CHECK(std::get<1>(linkRes));

    LinkSpi link = std::move(std::get<0>(linkRes));
    auto pixy = Pixy2<LinkSpi>(std::move(link));

    ESP_ERROR_CHECK(pixy.waitForStartup());
    GetBlocksContext blocksCtx;     // konec nastaveni kamery

    int k = 120; 
    int IDservo = 4;
    // rkArmSetServo(3, 60); // parkovaci pozice 

    fmt::print("{}'s roboruka '{}' started!\n", cfg.owner, cfg.name);
    fmt::print("Battery at {}%, {}mV\n", rkBatteryPercent(), rkBatteryVoltageMv());

    while(true) {          
        // nastavovani serv 
        if (rkButtonIsPressed(1)) {
            k += 10;
            rkArmSetServo(IDservo, k);
        }
        if (rkButtonIsPressed(2)) { 
            k-=10;
            rkArmSetServo(IDservo, k);      
        }
        if (rkButtonIsPressed(3)) {
            k += 1;
            rkArmSetServo(IDservo, k);
        }
        //fmt::print("vision: {}, position: {}\n", k, rkArmGetServo(IDservo) ); //konec testovani serv 
        //delay(300);

        auto err = pixy.getColorBlocks(1|2|4|8, 4, blocksCtx); // cervena | modra | zelena | cerna
        if(err == pixy.ERR_PIXY_BUSY) {
            vTaskDelay(1);
            continue;
        } else if(err != ESP_OK) {
            printf("Error1: %d\n", err);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
 
        const uint8_t lorrisHeader[] = { 0xFF, 0x01, (uint8_t)(blocksCtx.blocks.size() * sizeof(ColorBlock)) };
        uart_write_bytes(UART_NUM_0, (const char*)lorrisHeader, sizeof(lorrisHeader));
        if(blocksCtx.blocks.size() > 0) {
            uart_write_bytes(UART_NUM_0, (const char*)blocksCtx.blocks.data(), sizeof(ColorBlock)*blocksCtx.blocks.size());
        }
        // vTaskDelay(10);
        // uint8_t orderTest[] = { 0xFF,  0x01, 0x02, 0x03, 15  };
        // uart_write_bytes(UART_NUM_0, (const char*)orderTest, sizeof(orderTest));

    
        // int poziceX = blocksCtx.blocks[0]->x;
        // int poziceY = blocksCtx.blocks[0]->y;
        // int poziceX2 = blocksCtx.blocks[1]->x;
        // int poziceY2 = blocksCtx.blocks[1]->y;

        //    printf("i: %i, %i, %i, %i\n", poziceX, poziceY, poziceX2, poziceY2);
        // vTaskDelay(pdMS_TO_TICKS(1000));   // konec testovani kamery 
    }
}


