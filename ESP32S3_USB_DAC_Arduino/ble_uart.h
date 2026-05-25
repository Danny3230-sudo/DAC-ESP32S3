/**
 * ble_uart.h — BLE UART (Nordic UART Service) for ESP32-S3
 * ─────────────────────────────────────────────────────────────
 * Provides bidirectional serial communication between the
 * ESP32-S3 and an Android phone over BLE.
 * Uses Nordic UART Service (NUS) UUIDs — supported by the PWA's
 * Web Bluetooth API and most BLE serial applications.
 *
 * DATA FLOW:
 *   Android → write RX char → onBleWrite() → processATCommand()
 *   ESP32   → notify TX char → Android displays response / status JSON
 *
 * MTU NOTE:
 *   Default BLE MTU is 23 bytes. After negotiation it reaches up
 *   to 247 usable bytes. ble_send() splits automatically.
 *
 * Author: Danny Liu  |  v2.0.0
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ─── Forward declarations (defined in at_handler.h and main .ino)
void processATCommand(const String& cmd);
void debugPrint(const String& msg);

// ─── Global BLE state ─────────────────────────────────────────
bool               g_ble_connected = false;
BLEServer*         g_ble_server    = nullptr;
BLECharacteristic* g_ble_tx_char  = nullptr;

// ─── Server callbacks ─────────────────────────────────────────
class BleServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        g_ble_connected = true;
        BLEDevice::stopAdvertising();
        debugPrint("[BLE] Client connected");
    }
    void onDisconnect(BLEServer* s) override {
        g_ble_connected = false;
        debugPrint("[BLE] Client disconnected — restarting advertising");
        delay(200);
        s->startAdvertising();
    }
};

// ─── RX write callback: phone → ESP32 ────────────────────────
class BleRxCB : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        std::string val = c->getValue();
        if (!val.empty()) {
            String cmd = String(val.c_str());
            cmd.trim();
            processATCommand(cmd);
        }
    }
};

// ─── Initialise BLE ───────────────────────────────────────────
inline void ble_init(const String& deviceName) {
    BLEDevice::init(deviceName.c_str());
    BLEDevice::setPower(ESP_PWR_LVL_P9);  // max TX power

    g_ble_server = BLEDevice::createServer();
    g_ble_server->setCallbacks(new BleServerCB());

    BLEService* svc = g_ble_server->createService(BLE_NUS_SERVICE_UUID);

    // TX: ESP32 → App (notify)
    g_ble_tx_char = svc->createCharacteristic(
        BLE_NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    g_ble_tx_char->addDescriptor(new BLE2902());

    // RX: App → ESP32 (write)
    BLECharacteristic* rxChar = svc->createCharacteristic(
        BLE_NUS_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR);
    rxChar->setCallbacks(new BleRxCB());

    svc->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_NUS_SERVICE_UUID);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06); // 7.5 ms min connection interval
    adv->setMaxPreferred(0x18); // 30 ms max
    BLEDevice::startAdvertising();
}

// ─── Send string to phone (chunks if necessary) ───────────────
inline void ble_send(const String& msg) {
    if (!g_ble_connected || !g_ble_tx_char) return;
    const size_t CHUNK = 200;
    size_t len = msg.length(), start = 0;
    while (start < len) {
        size_t end = min(start + CHUNK, len);
        String chunk = msg.substring(start, end);
        g_ble_tx_char->setValue(chunk.c_str());
        g_ble_tx_char->notify();
        start = end;
        delay(10);
    }
}
