/*
 * live_receiver.h  —  ESP32-kant van de LIVE-stream van de STM32
 *
 * De STM32 stuurt vanaf het inschakelen elke 50 ms één frame over Serial1 (115200 8N1):
 *
 *   0xA5 0x5A | type=0x04 | len=41 | seq(2) count(1)=2 | 2 x record(19) | crc16(2)
 *   record = sensor(1) timestamp_ms(4) temp(4, 0,01 °C) pres(4, Pa)
 *            acc_x(2) acc_y(2) acc_z(2)  (0,01 m/s²)       — alles little-endian
 *   crc16  = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len en payload
 *
 * Zelfde frame- en recordformaat als de dump (DUMP_DATA 0x02), maar ZONDER ACK:
 * de flash op de STM32 blijft de betrouwbare kopie.
 *
 * Gebruik (in de bestaande sketch):
 *   #include "live_receiver.h"
 *   setup():  na het mounten van de SD-kaart:   live_begin(SD_MMC);   // of SD
 *   loop():   live_poll();                      // schrijft elke seconde weg naar de kaart
 *
 *   Elke byte die van Serial1 binnenkomt ook doorgeven:   live_feed(b);
 *   (Of: heeft de sketch al een frame-parser voor de dump, roep dan bij
 *    type 0x04 met geldige CRC rechtstreeks live_handle(payload, len) aan.)
 *
 * Resultaat op de SD-kaart: /live_<n>_sensor1.csv en /live_<n>_sensor2.csv,
 * met dezelfde kolommen als de dump-CSV. <n> telt op bij elke herstart.
 */
#pragma once
#include <Arduino.h>
#include <FS.h>

namespace live_rx {

static fs::FS  *g_fs = nullptr;
static File     g_file[2];
static uint32_t g_last_flush = 0;
static uint16_t g_last_seq = 0;
static bool     g_have_seq = false;
static uint32_t g_lost_frames = 0;     // ter info: gemiste frames (seq-sprongen)

static uint16_t crc16(const uint8_t *d, uint16_t n, uint16_t crc = 0xFFFF)
{
    for (uint16_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint32_t u32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t u16(const uint8_t *p) { return p[0] | (p[1] << 8); }

} // namespace live_rx

// Opent nieuwe CSV-bestanden met een vrij volgnummer.
inline void live_begin(fs::FS &fs)
{
    using namespace live_rx;
    g_fs = &fs;
    int n = 0;
    char name[32];
    while (true) {
        snprintf(name, sizeof(name), "/live_%d_sensor1.csv", n);
        if (!fs.exists(name)) break;
        n++;
    }
    for (int s = 0; s < 2; s++) {
        snprintf(name, sizeof(name), "/live_%d_sensor%d.csv", n, s + 1);
        g_file[s] = fs.open(name, FILE_WRITE);
        if (g_file[s])
            g_file[s].println("timestamp_ms,temp_C,pres_Pa,acc_x_ms2,acc_y_ms2,acc_z_ms2");
    }
    g_last_flush = millis();
}

// Verwerkt de payload van een geldig LIVE-frame (type 0x04, CRC al gecontroleerd).
inline void live_handle(const uint8_t *pl, uint8_t len)
{
    using namespace live_rx;
    if (len < 3) return;
    uint16_t seq = u16(&pl[0]);
    if (g_have_seq && (uint16_t)(seq - g_last_seq) != 1)
        g_lost_frames += (uint16_t)(seq - g_last_seq - 1);
    g_last_seq = seq;
    g_have_seq = true;

    uint8_t count = pl[2];
    if (len < 3 + count * 19) return;
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t *r = &pl[3 + i * 19];
        uint8_t sensor = r[0];
        if (sensor < 1 || sensor > 2 || !g_file[sensor - 1]) continue;
        uint32_t t    = u32(&r[1]);
        int32_t  temp = (int32_t)u32(&r[5]);
        int32_t  pres = (int32_t)u32(&r[9]);
        int16_t  ax   = (int16_t)u16(&r[13]);
        int16_t  ay   = (int16_t)u16(&r[15]);
        int16_t  az   = (int16_t)u16(&r[17]);
        g_file[sensor - 1].printf("%lu,%.2f,%ld,%.2f,%.2f,%.2f\n",
                                  (unsigned long)t, temp / 100.0, (long)pres,
                                  ax / 100.0, ay / 100.0, az / 100.0);
    }
}

// Byte-voor-byte parser voor als de sketch zelf geen frame-parser heeft.
inline void live_feed(uint8_t b)
{
    using namespace live_rx;
    static uint8_t buf[4 + 255 + 2];
    static uint16_t n = 0;

    if (n == 0 && b != 0xA5) return;
    if (n == 1 && b != 0x5A) { n = (b == 0xA5) ? 1 : 0; return; }
    buf[n++] = b;
    if (n < 4) return;
    uint16_t need = 4 + buf[3] + 2;
    if (n < need) return;
    n = 0;
    if (buf[2] != 0x04) return;                               // alleen LIVE-frames
    uint16_t crc = crc16(&buf[2], (uint16_t)(2 + buf[3]));
    if ((buf[4 + buf[3]] | (buf[5 + buf[3]] << 8)) != crc) return;
    live_handle(&buf[4], buf[3]);
}

// Regelmatig aanroepen: schrijft de buffers elke seconde naar de kaart,
// zodat bij stroomverlies hooguit ~1 s CSV-data verloren gaat.
inline void live_poll()
{
    using namespace live_rx;
    if (millis() - g_last_flush < 1000) return;
    g_last_flush = millis();
    for (int s = 0; s < 2; s++)
        if (g_file[s]) g_file[s].flush();
}
