#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TinyGPS++.h>
#include <OneWire.h>             
#include <DallasTemperature.h>   

// --- NASTAVENÍ PINŮ ---
#define ONE_WIRE_BUS 21 // Teplota
#define RPM_PIN 25      // Otáčkoměr
#define PALIVO_PIN 33   // Plovák nádrže

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

// --- PROMĚNNÉ PRO OTÁČKOMĚR ---
volatile unsigned long casPoslednihoImpulsu = 0;
volatile unsigned long delkaImpulsuMicros = 0;
int aktualniRPM = 0;

// --- PROMĚNNÉ PRO PALIVOMĚR ---
int rawPlna = 130;    
int rawPrazdna = 845;  
int procentaPaliva = 0;
float vyhlazenyRawPalivo = -1.0; // Paměť pro extrémní softwarovou filtraci

// --- PAMĚŤ PRO PLYNULÉ VYKRESLOVÁNÍ (PROTI BLIKÁNÍ) ---
int staraSirkaRPM = -1;
uint16_t staraBarvaRPM = 0;
int staraSirkaFuel = -1;
uint16_t staraBarvaFuel = 0;

// --- FUNKCE PŘERUŠENÍ (RPM) ---
void IRAM_ATTR zmerOtacky() {
  unsigned long aktualniCas = micros();
  unsigned long rozdil = aktualniCas - casPoslednihoImpulsu;
  if (rozdil > 5000) { 
    delkaImpulsuMicros = rozdil;
    casPoslednihoImpulsu = aktualniCas;
  }
}

// --- VYKRESLENÍ STATICKÉ GRAFIKY PRO 480x320 ---
void nakresliRozhrani() {
  tft.fillScreen(TFT_BLACK);

  // Horní dělicí čára pro stavovou lištu
  tft.drawLine(0, 40, 480, 40, TFT_DARKGREY);
  
  // Spodní dělicí čára pod rychlostí (prostřední panel)
  tft.drawLine(0, 220, 480, 220, TFT_DARKGREY);

  // Rámečky pro bargrafy (Vyplňovat se budou uvnitř)
  tft.drawRect(10, 255, 460, 15, TFT_DARKGREY);
  tft.drawRect(10, 300, 460, 15, TFT_DARKGREY);

  // Statické nápisy uprostřed
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  tft.drawString("km/h", 240, 190, 4);

  // Statické nápisy vlevo
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("TRIP (km)", 10, 60, 2);
  tft.drawString("RPM", 10, 235, 2);
  tft.drawString("FUEL", 10, 280, 2);

  // Statické nápisy vpravo
  tft.setTextDatum(TR_DATUM);
  tft.drawString("TEMP", 470, 60, 2);
}

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  senzory.begin();
  senzory.setWaitForConversion(false); 
  senzory.requestTemperatures();       

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), zmerOtacky, FALLING);

  tft.init();
  tft.setRotation(1); // ZMĚNA ORIENTACE NA ŠÍŘKU
  tft.invertDisplay(true); // Ochrana proti bílému pozadí
  
  tft.fillScreen(TFT_BLACK);

  // Uvítací obrazovka
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("KEEWAY SYSTEM", 240, 160, 4);
  delay(2000);
  
  // Vykreslení statické "kostry" rozhraní
  nakresliRozhrani();
}

void loop() {
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  // --- MĚŘENÍ TEPLOTY (1x za sekundu) ---
  if (millis() - posledniMereniTeploty > 1000) {
    posledniMereniTeploty = millis();
    aktualniTeplota = senzory.getTempCByIndex(0); 
    senzory.requestTemperatures();                
  }

  // --- PŘEKRESLENÍ DISPLEJE (5x za sekundu) ---
  if (millis() - posledniPrekresleni > 200) {
    posledniPrekresleni = millis();

    // --- VÝPOČET OTÁČEK ---
    if (micros() - casPoslednihoImpulsu > 500000) {
      aktualniRPM = 0;
    } else if (delkaImpulsuMicros > 0) {
      int vypocitaneRPM = 60000000 / delkaImpulsuMicros; 
      aktualniRPM = (aktualniRPM * 0.8) + (vypocitaneRPM * 0.2);
    }

    // --- VÝPOČET PALIVA S EXTRÉMNÍ FILTRACÍ (EMA) ---
    int aktualniRaw = analogRead(PALIVO_PIN);
    
    // Rychlý náběh při prvním spuštění
    if (vyhlazenyRawPalivo < 0) {
      vyhlazenyRawPalivo = aktualniRaw;
    } else {
      // Magie filtru: 98 % staré hodnoty + 2 % z nového měření
      vyhlazenyRawPalivo = (vyhlazenyRawPalivo * 0.98) + (aktualniRaw * 0.02);
    }
    
    // Převod na procenta z vyhlazené hodnoty
    procentaPaliva = map((int)vyhlazenyRawPalivo, rawPrazdna, rawPlna, 0, 100);
    procentaPaliva = constrain(procentaPaliva, 0, 100);

    // --- VÝPOČET TRIPU ---
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

    // ==========================================
    // --- KRESLENÍ DYNAMICKÝCH DAT NA DISPLEJ ---
    // ==========================================

    // 1. Hlava (Satelity a Čas)
    tft.setTextPadding(80); 
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(10, 10);
    tft.printf("SATS: %d", gps.satellites.value());

    if (gps.time.isValid()) {
      tft.setTextDatum(TR_DATUM);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      int hodina = gps.time.hour() + 2; 
      if (hodina >= 24) hodina -= 24;
      char casString[10];
      sprintf(casString, "%02d:%02d", hodina, gps.time.minute());
      tft.drawString(casString, 470, 10, 4);
    }

    // 2. STŘEDOVÝ PANEL (RYCHLOST + TRIP + TEPLOTA)
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextPadding(150); 
    tft.drawFloat(gps.speed.kmph(), 0, 240, 130, 7); 

    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextPadding(100);
    tft.drawFloat(tripVzdalenostKm, 1, 10, 95, 4);

    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.setTextPadding(100);
    if (aktualniTeplota <= -100.0) {
      tft.drawString("ERR", 470, 95, 4); 
    } else {
      char teplotaStr[10];
      sprintf(teplotaStr, "%.0fC", aktualniTeplota);
      tft.drawString(teplotaStr, 470, 95, 4);
    }

    // 3. BARGRAF PRO OTÁČKY
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextPadding(80);
    char rpmStr[10];
    sprintf(rpmStr, "%04d", aktualniRPM);
    tft.drawString(rpmStr, 470, 235, 2);

    int sirkaPruhuRPM = map(aktualniRPM, 0, 10000, 0, 458);
    sirkaPruhuRPM = constrain(sirkaPruhuRPM, 0, 458);
    
    uint16_t barvaRPM = TFT_GREEN;
    if (aktualniRPM > 7000) barvaRPM = TFT_YELLOW;
    if (aktualniRPM > 8500) barvaRPM = TFT_RED;
    
    if (sirkaPruhuRPM != staraSirkaRPM || barvaRPM != staraBarvaRPM) {
      if (barvaRPM != staraBarvaRPM || staraSirkaRPM == -1) {
        tft.fillRect(11, 256, sirkaPruhuRPM, 13, barvaRPM);
        tft.fillRect(11 + sirkaPruhuRPM, 256, 458 - sirkaPruhuRPM, 13, TFT_BLACK);
      } else {
        if (sirkaPruhuRPM > staraSirkaRPM) {
          tft.fillRect(11 + staraSirkaRPM, 256, sirkaPruhuRPM - staraSirkaRPM, 13, barvaRPM);
        } else {
          tft.fillRect(11 + sirkaPruhuRPM, 256, staraSirkaRPM - sirkaPruhuRPM, 13, TFT_BLACK);
        }
      }
      staraSirkaRPM = sirkaPruhuRPM;
      staraBarvaRPM = barvaRPM;
    }

    // 4. PALIVOMĚR
    tft.setTextDatum(TR_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextPadding(80);
    char fuelStr[10];
    sprintf(fuelStr, "%d%%", procentaPaliva);
    tft.drawString(fuelStr, 470, 280, 2);

    int sirkaPruhuFuel = map(procentaPaliva, 0, 100, 0, 458);
    sirkaPruhuFuel = constrain(sirkaPruhuFuel, 0, 458);
    
    uint16_t barvaFuel = TFT_GREEN;
    if (procentaPaliva < 25) barvaFuel = TFT_YELLOW;
    if (procentaPaliva < 10) barvaFuel = TFT_RED;

    if (sirkaPruhuFuel != staraSirkaFuel || barvaFuel != staraBarvaFuel) {
      if (barvaFuel != staraBarvaFuel || staraSirkaFuel == -1) {
        tft.fillRect(11, 301, sirkaPruhuFuel, 13, barvaFuel);
        tft.fillRect(11 + sirkaPruhuFuel, 301, 458 - sirkaPruhuFuel, 13, TFT_BLACK);
      } else {
        if (sirkaPruhuFuel > staraSirkaFuel) {
          tft.fillRect(11 + staraSirkaFuel, 301, sirkaPruhuFuel - staraSirkaFuel, 13, barvaFuel);
        } else {
          tft.fillRect(11 + sirkaPruhuFuel, 301, staraSirkaFuel - sirkaPruhuFuel, 13, TFT_BLACK);
        }
      }
      staraSirkaFuel = sirkaPruhuFuel;
      staraBarvaFuel = barvaFuel;
    }

    // Vynulování paddingu na konci
    tft.setTextPadding(0);
  }
}