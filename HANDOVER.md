# Overdracht payload-firmware (stand 14 oktober 2026)

Dit document vat samen wat er gebouwd en getest is, welke beslissingen genomen zijn en wat er nog moet gebeuren. Wie verder werkt (met of zonder Claude) kan hiermee meteen starten.

---

## 1. Hardwarestatus

| Onderdeel | Status |
|---|---|
| MCU | STM32C092KCT6, STM32CubeIDE 2.2 + STM32CubeMX 6.18 |
| BNO055 op de PCB | **Verwijderd.** Verstoorde de volledige I2C-bus (spook-ACK's, busfouten), vermoedelijk verkeerd georiënteerd na herhaald herplaatsen. |
| BNO055 vervanger | **Adafruit BNO055-breakout** via kabel op de header (3,3 V / SDA / SCL / GND) van bus 1 |
| MS5607 en SHT31 op de PCB | **Verwijderd.** Beschadigd door hete lucht bij het verwijderen van de BNO055; de MS5607 trok SDA/SCL naar ~2 V. Worden niet gebruikt. |
| Tankdruksensoren | 2 × MS5837, één op I2C1 en één op I2C2 (vast adres 0x76, dus één per bus) |
| Camera | Seeed XIAO ESP32-S3, gevoed vanuit het payloadboard, aangestuurd via UART |
| Flash | **Winbond W25Q256JV**, 32 MB SPI-NOR (dus **niet** de MT29F1G01 uit eerdere plannen). JEDEC-ID `EF 40 19` bevestigd. |

### MS5837: let op het type
De sensoren geven versiecode `0` in PROM-woord C0 (wat op een 02BA zou wijzen), maar met de 02BA-formule kwam de druk **exact een factor 2 te laag** uit. Met de **30BA-formule** klopt de luchtdruk (~1017–1025 hPa). De code forceert daarom `is_02ba = 0`. Kijk op de verpakking of factuur welk type echt besteld werd.

Gevolg: een 30BA is grover (bij OSR 1024 ongeveer 0,5 mbar resolutie, ≈ enkele mm vloeistofkolom). Overweeg een hogere OSR als dat te grof is voor het experiment.

---

## 2. Pinout STM32C092

| Pin | Functie | Aangesloten op |
|---|---|---|
| PB7 / PB8 | I2C1 SDA / SCL | Adafruit BNO055 (0x28), MS5837 #1 (0x76) |
| PA10 / PA11 | I2C2 SDA / SCL | MS5837 #2 (0x76) |
| PA0 / PA1 | USART1 TX / RX, 115200 8N1 | XIAO ESP32-S3 (RX/TX gekruist, gemeenschappelijke GND) |
| PA5 / PA6 / PA7 | SPI1 SCK / MISO / MOSI, modus 0, 6 Mbit/s | W25Q256JV |
| PA4 | GPIO, CS (actief laag, initieel hoog) | W25Q256JV /CS |
| PA8 | GPIO, initieel hoog | W25Q256JV /RESET |
| PB0 / PB1 / PB2 | GPIO | LED0 / LED1 / LED2 |
| PB3 / PB4 / PB5 | GPIO | Waren voor de verwijderde BNO055 op de PCB; niet meer in gebruik |
| PA13 / PA14 | SWD | ST-Link. JP1 op PA14 (BOOT0) open laten. |

---

## 3. Wat werkt (getest op de hardware)

- **I2C1 en I2C2** op 100 kHz, propere bus (SDA/SCL in rust ~3,1 V).
- **BNO055:** chip-ID `0xA0`, modus `ACCONLY`, bereik **±16 g** (`ACC_Config` = `0x0F` op page 1). In de fusiemodi ligt het bereik vast op 4 g, wat bij de lancering zou afkappen.
- **MS5837 × 2:** reset, PROM inlezen met CRC-4-controle, druk en temperatuur (OSR 1024, eerste en tweede orde compensatie). Uitvoer in Pa en 0,01 °C.
- **Lanceerdetectie:** $|a| > 3\,g$ gedurende **100 ms**, op tijd gebaseerd met `HAL_GetTick()` en dus onafhankelijk van de lusduur. Vergelijking op het kwadraat (geen `sqrt`).
- **Camerastart:** `START\n` via UART, wacht op `ACK\n`, maximaal 5 pogingen. De XIAO draait nu een **testsketch** die enkel een LED aanzet.
- **Lustiming:** versnelling elke 10 ms, druk elke 50 ms, geen `HAL_Delay` in de hoofdlus.
- **LED's:** opstartanimatie; LED0 = hartslag; LED1 = sensoren OK (snel knipperen = fout); LED2 = lancering + camera-ACK (snel knipperen = geen ACK).
- **SPI1 + flash:** JEDEC-ID correct gelezen.

---

## 4. Wat nog moet (in volgorde van belang)

1. **Flashdriver W25Q256JV**
   - Write Enable `0x06`; status lezen `0x05` (bit 0 = BUSY) en wachten tot die 0 is.
   - **4-byte-commando's** gebruiken (32 MB > 16 MB): sector wissen 4 KB `0x21`, pagina schrijven `0x12` (max 256 bytes, **niet over een paginagrens**), lezen `0x13`.
   - Eerst testen: laatste sector wissen, testpatroon schrijven, terug lezen en vergelijken.
2. **Logging naar de flash**
   - Vanaf het opstarten loggen (nulmeting op het platform).
   - **Nooit automatisch wissen bij opstarten.** Bij het opstarten de eerste lege positie zoeken (`0xFF`) en daar verder schrijven, zodat een herstart na de landing de vluchtdata niet vernietigt.
   - Capaciteit: 32 MB. Kies recordformaat en meetfrequentie zodat de wachttijd op het platform er ruim in past.
3. **Uitleesmodus:** data via UART naar een pc dumpen na de vlucht.
4. **Camerafirmware op de XIAO:** video naar SD starten op `START`, `ACK` terugsturen. Eigenaar nog niet toegewezen.
5. **Robuustheid**
   - Watchdog (IWDG) zodat een vastgelopen I2C-transfer de STM32 herstart. De camera blijft opnemen, want een ontvangen `START` blijft actief.
   - Antwoordt de BNO055 niet bij het opstarten, dan meteen `START` sturen (liever te vroeg opnemen dan nooit).
6. **Na elke wijziging testen na een power-cycle zonder debugger.**

---

## 5. Valkuilen die we tegenkwamen

- **Sla `main.c` op vóór je in CubeMX op Generate Code klikt.** Niet-opgeslagen wijzigingen gaan anders verloren.
- Eigen code **alleen tussen `USER CODE BEGIN` / `END`**, anders overschrijft CubeMX ze.
- De `}` die de `while (1)` sluit staat **binnen** het `USER CODE 3`-blok. Verwijder je die per ongeluk, dan krijg je waarschuwingen als *"MX_I2C1_Init defined but not used"*.
- Na een power-cycle draait eerst de **oude firmware** (die bv. al `START` stuurt) voordat de debugger de nieuwe flasht.
- HAL verwacht het I2C-adres **één bit naar links** geschoven (`addr << 1`).
- De debugger stopt eerst op `HAL_Init()`: druk F8 om verder te gaan. Breakpoints blijven op een regelnummer staan en verschuiven niet mee als je code toevoegt.
- **Expressions** worden alleen bijgewerkt als het programma gepauzeerd is; **Live Expressions** werken terwijl het loopt.
- Op de LQFP32 delen **PA9/PA11** en **PA10/PA12** een fysieke pin (remap in SYS). I2C2 gebruikt PA10 (pin 21) en PA11 (pin 22).
- De **SPI1-instelling "NSS Signal Type: Software"** komt overeen met "Hardware NSS uitgeschakeld"; CS wordt via PA4 aangestuurd.

---

## 6. Verder werken met Claude

Een gedeelde Claude-chat is alleen een momentopname die je kunt lezen, niet verder kunt zetten. Om verder te werken:

1. Start een **nieuwe chat** met Claude.
2. Voeg **dit bestand** en **`payload/Core/Src/main.c`** toe.
3. Zeg waar je wilt beginnen, bijvoorbeeld: *"Lees HANDOVER.md en main.c. Ik wil verder met punt 4.1, de flashdriver."*
