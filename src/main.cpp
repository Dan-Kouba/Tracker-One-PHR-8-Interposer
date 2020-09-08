/*
 * Copyright (c) 2020 Particle Industries, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Particle.h"

#include "tracker_config.h"
#include "tracker.h"

#include "SparkFun_Qwiic_Rfid.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

PRODUCT_ID(TRACKER_PRODUCT_ID);
PRODUCT_VERSION(TRACKER_PRODUCT_VERSION);

#define RFID_ADDR 0x7D // Default I2C address 
Qwiic_Rfid myRfid(RFID_ADDR);
const auto RfidInt = D3;

Logger RfidLog("app.RFID");

// This variable holds the number representing the lightning or non-lightning
// event issued by the lightning detector. 
int intVal = 0;
int noise = 2; // Value between 1-7 
int disturber = 2; // Value between 1-10

typedef struct {
    String tag;
    unsigned long time;
    bool processed;
} last_event_t;

last_event_t le;

volatile bool interrupt = false;

SerialLogHandler logHandler(115200, LOG_LEVEL_TRACE, {
    { "app.gps.nmea",   LOG_LEVEL_INFO },
    { "app.gps.ubx",    LOG_LEVEL_INFO },
    { "ncp.at",         LOG_LEVEL_INFO },
    { "net.ppp.client", LOG_LEVEL_INFO },
    { "app.RFID",       LOG_LEVEL_ALL  },
});

void locationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context);
void rfid_isr();

void setup()
{
    Tracker::instance().init();
    Tracker::instance().location.regLocGenCallback(locationGenerationCallback);

    // Turn on 3V3 power to the interposer board
    pinMode(CAN_PWR, OUTPUT);
    digitalWrite(CAN_PWR, HIGH);

    pinMode(RfidInt, INPUT_PULLUP); 

    Particle.connect();

    while (!Serial.isConnected()) {
        delay(500);
        Particle.process();
    }

    Wire3.begin(); // Begin Wire before lightning sensor. 

    if(myRfid.begin(Wire3)) {
        RfidLog.info("Sparkfun RFID Demo: Ready to scan some tags!"); 
        attachInterrupt(RfidInt, rfid_isr, FALLING);
        le.processed = true;
    } else {
        RfidLog.error("Sparkfun RFID Demo: Could not communicate with the Qwiic RFID Reader!!!"); 
    }
}

void loop()
{
    // Trigger a location publish on interrupt
    if (interrupt) {
        RfidLog.info("RFID interrupt detected!");
        le.tag = myRfid.getTag();
        le.time = millis();
        TrackerLocation::instance().triggerLocPub(Trigger::IMMEDIATE, "RFID");
        
        RfidLog.info("Tag read: %s @ %lu", le.tag.c_str(), le.time);

        // Reset our state
        interrupt = false;
        le.processed = false;
    }

    Tracker::instance().loop();
}

void locationGenerationCallback(JSONWriter &writer, LocationPoint &point, const void *context)
{
    // We only want this data added to our immediate publishes, so check if we have
    // an unprocessed tag read.  Hacky, but it should work.
    if (!le.processed) {
        writer.name("tag").value(le.tag);
        writer.name("time").value((int)le.time);
        le.processed = true;
    }
}

void rfid_isr() {
    interrupt = true;
    RfidLog.log("ISR");
}