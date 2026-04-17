#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TinyGPS++.h>
#include <OneWire.h>             
#include <DallasTemperature.h>   

// --- NASTAVENÍ PINŮ ---
#define ONE_WIRE_BUS 21 // Teplota
#define RPM_PIN 25      // Otáčkoměr z optočlenu

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature senzory(&oneWire);

TFT_eSPI tft = TFT_eSPI();
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); 

// --- PROMĚNNÉ PRO GPS A TEPLOTU ---
double tripVzdalenostKm = 0.0;
double posledniLat = 0.0;
double posledniLng = 0.0;
unsigned long posledniPrekresleni = 0;
unsigned long posledniMereniTeploty = 0; 
float aktualniTeplota = 0.0;             

// --- PROMĚNNÉ PRO OTÁČKOMĚR (Volatile znamená, že se mění z přerušení) ---
volatile unsigned long casPoslednihoImpulsu = 0;
volatile unsigned long delkaImpulsuMicros = 0;
int aktualniRPM = 0;

// --- FUNKCE PŘERUŠENÍ (Volá se sama při každé jiskře) ---
void IRAM_ATTR zmerOtacky() {
  unsigned long aktualniCas = micros();
  unsigned long rozdil = aktualniCas - casPoslednihoImpulsu;
  
  // Zvýšený Debounce filtr: 5000 mikrosekund = max 12 000 ot/min.
  // Pokud přijde další impuls dříve než za 5 ms, zahodíme ho jako rušení (dozvuk jiskry).
  if (rozdil > 5000) { 
    delkaImpulsuMicros = rozdil;
    casPoslednihoImpulsu = aktualniCas;
  }
}

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  // --- START SENZORŮ ---
  senzory.begin();
  senzory.setWaitForConversion(false); 
  senzory.requestTemperatures();       

  // --- START OTÁČKOMĚRU ---
  pinMode(RPM_PIN, INPUT_PULLUP); // Použijeme vnitřní odpor ESP32
  // Nastavíme přerušení na pin D25, bude reagovat na pád napětí (FALLING), když optočlen sepne
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), zmerOtacky, FALLING);

  // --- START DISPLEJE ---
  tft.init();
  tft.setRotation(0); 
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("KEEWAY SYSTEM", 120, 120, 4);
  delay(2000);
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  // GPS ČTENÍ (Běží pořád)
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // MĚŘENÍ TEPLOTY (1x za vteřinu)
  if (millis() - posledniMereniTeploty > 1000) {
    posledniMereniTeploty = millis();
    aktualniTeplota = senzory.getTempCByIndex(0); 
    senzory.requestTemperatures();                
  }

  // PŘEKRESLENÍ DISPLEJE (5x za vteřinu)
  if (millis() - posledniPrekresleni > 200) {
    posledniPrekresleni = millis();

    // --- VÝPOČET OTÁČEK (RPM) ---
    // Pokud jsme víc jak 0.5 vteřiny nedostali jiskru, motor stojí.
    if (micros() - casPoslednihoImpulsu > 500000) {
      aktualniRPM = 0;
    } else if (delkaImpulsuMicros > 0) {
      // Skútr je 2-takt = 1 jiskra za otáčku.
      // 60 milionů mikrosekund (1 minuta) / délka jednoho pulsu = otáčky za minutu
      int vypocitaneRPM = 60000000 / delkaImpulsuMicros; 
      
      // Mírné vyhlazení skoků (Bere 80 % staré hodnoty a 20 % nové)
      aktualniRPM = (aktualniRPM * 0.8) + (vypocitaneRPM * 0.2);
    }

    // Výpočet Tripu
    if (gps.location.isUpdated() && gps.location.isValid()) {
      if (posledniLat != 0.0 && posledniLng != 0.0) {
        double vzdalenostMetry = TinyGPSPlus::distanceBetween(
          gps.location.lat(), gps.location.lng(), 
          posledniLat, posledniLng
        );
        tripVzdalenostKm += (vzdalenostMetry / 1000.0);
      }
      posledniLat = gps.location.lat();
      posledniLng = gps.location.lng();
    }

    // --- KRESLENÍ NA DISPLEJ ---

    // 1. Hlava (Satelity a Čas)
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 10);
    tft.print("SATS: "); 
    tft.print(gps.satellites.value());
    tft.print("  "); 

    if (gps.time.isValid()) {
      tft.setCursor(160, 10);
      int hodina = gps.time.hour() + 2; // Letní čas
      if (hodina >= 24) hodina -= 24;
      tft.printf("%02d:%02d", hodina, gps.time.minute());
    }

    // 2. Střed (RYCHLOST)
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextPadding(120);
    tft.drawFloat(gps.speed.kmph(), 1, 120, 90, 7); // Posunuto mírně nahoru kvůli RPM
    tft.setTextPadding(0);
    tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
    tft.drawString("km/h", 120, 150, 4);

    // 3. BARGRAF PRO OTÁČKY (Pruh)
    tft.setCursor(10, 180);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("RPM: %04d", aktualniRPM);

    // Vykreslení samotného proužku otáček (max 10 000 ot/min)
    int sirkaPruhu = map(aktualniRPM, 0, 10000, 0, 220); // Zmapuje RPM na šířku displeje
    if (sirkaPruhu > 220) sirkaPruhu = 220; // Ochrana proti přetečení
    if (sirkaPruhu < 0) sirkaPruhu = 0;
    
    tft.drawRect(10, 195, 220, 12, TFT_DARKGREY); // Rámeček
    
    // Barva podle otáček (zelená/žlutá/červená)
    uint16_t barvaPruhu = TFT_GREEN;
    if (aktualniRPM > 7000) barvaPruhu = TFT_YELLOW;
    if (aktualniRPM > 8500) barvaPruhu = TFT_RED;
    
    tft.fillRect(10, 195, sirkaPruhu, 12, barvaPruhu); // Vyplněný pruh
    tft.fillRect(10 + sirkaPruhu, 195, 220 - sirkaPruhu, 12, TFT_BLACK); // Smazání zbytku pruhu

    // 4. Spodek (Vzdálenost a Teplota)
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 220);
    tft.print("TR: ");
    tft.print(tripVzdalenostKm, 1); 

    tft.setCursor(160, 220); 
    if (aktualniTeplota <= -100.0) {
      tft.print("ERR   "); 
    } else {
      tft.print(aktualniTeplota, 1); 
      tft.print("C  ");             
    }
  }
}