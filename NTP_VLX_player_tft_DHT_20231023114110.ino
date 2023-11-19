/*
 * Dr.TRonik / YouTube / KASIM 2023 / İzmir / Türkiye 
 * Belirli bir mesafeden karşısında durulduğunda sesli olarak zaman, takvim, sıcaklık, nem bildirimi...
 * ESP8266, ST7735s 128x160 1.8" TFT LCD, GYBMEP / DHT11, dfplayer, NTP clock...
 * Derleme ve karta yükleme öncesi, tüm kütüphaneler arduino ide'sine yüklenmiş olmalıdır...
 * YouTube: https://www.youtube.com/watch?v=GlaglWVU73U&t=337s
 * PCB: https://www.pcbway.com/project/shareproject/Data_to_Sound_4cf24a5e.html
 * ESP8266 Bağlantılar (Pin Etiketleri ile):  
      ST735:              DfPlayerMini:               VL53L0x:          BME280:         (DHT11)
      -----------------------------------------------------------------------------------------------
      CS_D2               Vcc_5V                      Vın_5V            Vın_5V          (Vinn_5V)
      RST_D0              GND_GND                     GND_GND           GND_GND         (GND_GND)
      DC_D1               Pin2_Rx - ESP Tx            SDA_D4            SCL_D3          (Data_D4)
      SCL_D5              Pin3_Tx - ESP Rx            SCL_D3            SDA_D4
      SDA_D7 
      VDD_5V
      GND_GND
      BLK_3.3V 
*/

/********************************************************************
  GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___GLOBALS___
 ********************************************************************/
#include <Wire.h>  // I²C

//ESP8266
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//VL53L0X
#include <VL53L0X.h>
VL53L0X sensor;
#define HIGH_SPEED

//ST7735
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library for ST7735
#include <SPI.h>              // SPI comminication

//ESP8266 için pin bağlantıları - Pin Connections for ESP8266
#define TFT_CS 4    // D2
#define TFT_RST 16  // D0
#define TFT_DC 5    // D1
//SCL D5
//SDA D7
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

//İnternet üzerinden zamanı alabilme - TimeClient
#include <NTPClient.h>
#include <Time.h>
#include <TimeLib.h>
#include "Timezone.h"

//DHT
#include <DHT.h>
#define DHTPin 2  //GPIO0 yani D4 etiketli pin
#define DHTType DHT11
DHT dht(DHTPin, DHTType);

//Mini Player
#include <DFPlayerMini_Fast.h>
#include <SoftwareSerial.h>
#define ESP_SERIAL_RX 3                                     // Player pin3 Tx->ESP8266 Rx GPIO 3 / RX
#define ESP_SERIAL_TX 1                                     // Player pin2 Rx-> ESP8266 Tx GPIO 1 / TX
SoftwareSerial sound_serial(ESP_SERIAL_RX, ESP_SERIAL_TX);  // ESP RX, ESP TX
DFPlayerMini_Fast sound;                                    //"sound" adında player objesi oluşturuldu...

//SSID ve Şifre Tanımlama - SSID PASS
#define STA_SSID "yourSSID"
#define STA_PASSWORD  "yourPASS"

//Zaman intervalleri - Time Intervals
#define NTP_OFFSET 60 * 60             // In seconds
#define NTP_INTERVAL 60 * 1000         // In miliseconds, dk da bir güncelleme
#define NTP_ADDRESS "tr.pool.ntp.org"  // The nearest ntp pool
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

//Değişkenler - Variables
time_t local, utc;
bool connected = false;
int temp, hum, pressure, altitude, distance;
int saat, dakika, saniye, gun, ay, dayWeek, yil;
char gun_isimleri[8][10] = { " ", "  PAZAR", "PAZARTESi", "  SALI", "CARSAMBA", "PERSEMBE", "  CUMA", "CUMARTESi" };

/********************************************************************
  SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___SETUP___
 ********************************************************************/
void setup() {
  Serial.begin(9600);        // Seri iletişim
  Wire.begin(2, 0);          // I²C: Wire.begin(SDA, SCL); / GPIO 2 = D4 = SDA, GPIO 0 = D3 = SCL
  sound_serial.begin(9600);  //DfPlayer Esp iletişimi default baud rate is 9600
  delay(100);
  sound.begin(sound_serial);
  delay(1000);
  sound.volume(15);  //Ses ayarı 0-30 arası...
  delay(1000);
  sound.play(98);  // İlk açılışta 098.mp3
  dht.begin();

  //VLX_sen.
  sensor.setTimeout(500);  // Set serial communictaion time out 500ms
  if (!sensor.init()) {
    // Bu if bloğu OLMALI!...
    // Serial.println("Failed to detect and initialize sensor!");
    // while (1) {} //Stop
  }

  sensor.startContinuous();

  //sensor.setMeasurementTimingBudget(20000);

  tft.initR(INITR_BLACKTAB);     // initialize a ST7735S chip, black tab
  tft.setTextWrap(false);        // Allow text to run off right edge
  tft.fillScreen(ST77XX_BLACK);  // Ekranı temizlemek VE canvası siyah yapabilmek için
  tft.setRotation(3);            // 0_0°, 1_90°, 2_180°, 3_270° rotasyon...

  timeClient.begin();  // Start the NTP UDP client

  // Serial.print("Trying WiFi connection to ");
  // Serial.println(STA_SSID);

  WiFi.setAutoReconnect(true);
  WiFi.begin(STA_SSID, STA_PASSWORD);

  read_data();  // Açılışta ekrana veriler boş gelmesin...
}

/********************************************************************
  LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__LOOP__
 ********************************************************************/
void loop() {

  static unsigned long timer, timer_1, timer_2, timer_3 = millis();


  //Handle Connection to Access Point
  if (WiFi.status() == WL_CONNECTED) {
    if (!connected) {
      connected = true;
    }
  } else {
    if (connected) {
      connected = false;
    }
  }

  //___millis başlangıcı___
  if (millis() - timer > 1000) {
    timer = millis();

    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();

    // convert received time stamp to time_t object

    utc = epochTime;

    // Then convert the UTC UNIX timestamp to local time
    // Eastern Daylight Time (EDT); Türkiye: sabit +UTC+3 nedeni ile  +2saat = +120dk ayarlanmalı
    // Eastern Time Zone: UTC- change this as needed; Türkiye için ulusal saat/dünya saat UTC değişikliklerine göre kasım ve mart aylarında güncelleme isteyebilir
    //Türkiye kış saati:
    TimeChangeRule usEDT = { "EDT", Second, Sun, Mar, 2 };
    TimeChangeRule usEST = { "EST", First, Sun, Nov, 2, +120 };
    // Türkiye yaz saati:
    // TimeChangeRule usEDT = { "EDT", Second, Sun, Mar, 2, +120  };
    // TimeChangeRule usEST = { "EST", First, Sun, Nov, 2,  };

    Timezone usEastern(usEDT, usEST);
    local = usEastern.toLocal(utc);

    saat = hour(local);  // hour(local); -->14:42  hourFormat12(local)--> 02:42;
    dakika = minute(local);
    saniye = second(local);
    gun = day(local);
    ay = month(local);
    yil = year(local);
    dayWeek = weekday(local);  //Haftanın günleri...

    //Gün, ay, yıl, haftanın günü verilerini tek bir veri olarak ekrana yazdırabilimek için verileri char takvim_ dizisine biçimlendirerek aktaralım...
    char takvim_[11];
    sprintf(takvim_, "%02d.%02d.%04d ", gun, ay, yil);
    tft.setCursor(20, 0);
    tft.setTextColor(0x07FF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
    tft.setTextSize(2);
    tft.print(takvim_);

    char takvim_1[10];
    sprintf(takvim_1, "%-9s", gun_isimleri[dayWeek]);
    if (dayWeek == 1 || dayWeek == 7) {
      tft.setCursor(25, 113);
      tft.setTextColor(0xF800, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
      tft.setTextSize(2);
      tft.print(takvim_1);
    } else {
      tft.setCursor(25, 113);
      tft.setTextColor(0x07FF, 0x0000);  // Veriler değiştiğinde ekranda üst üste binmesini engeller...
      tft.setTextSize(2);
      tft.print(takvim_1);
    }
    yaz();
  }
  //___millis sonu___


  // 10 saniyede bir bmp sensörden veri okumak için...
  // if (millis() - timer_1 > 60000) {
  //   timer_1 = millis();
  //   read_data();
  // }


  //Belirlenen sürede vl53 sensörden veri okumak için...
  if (millis() - timer_2 > 1 * 500) {
    timer_2 = millis();
    vlx();

    if (distance < 150) {
      takvim();
      time();
      meteo();
    }
  }



  //Belirlenen sürede lokalizasyon ve durum sinyal sesi çalma...
  if (millis() - timer_3 > 1 * 60000) {
    timer_3 = millis();
    sound.play(99);
    read_data();
  }
}

/********************************************************************
  VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs__VOIDs
********************************************************************/
//Saat Bilgisi...
void yaz() {
  char saat_[6];  //5 characters + null
  char sec[3];    //2 characters + null
  sprintf(saat_, "%02d:%02d", saat, dakika);
  sprintf(sec, "%02d", saniye);
  if ((saat >= 7) && (saat <= 22)) {
    tft.setTextColor(0xFFFF, 0x0000);  // tft.fillScreen(0); -->Causes flicker...
  } else {
    tft.setTextColor(0xF81F, 0x0000);
  }
  tft.setTextSize(5);
  tft.setCursor(5, 24);
  tft.print(saat_);

  // tft.setTextSize(1);
  // tft.setCursor(149, 24);
  // tft.print(sec);
}

// BME280 Sensörden veri okuma ve okunan verinin tft ekrana yazdırılması...
void read_data() {
  temp = dht.readTemperature();
  hum = dht.readHumidity();

  //distance = sensor.readRangeContinuousMillimeters();
  //distance = sensor.readRangeSingleMillimeters();

  char temp_[3];
  sprintf(temp_, "%02d", temp);
  tft.setCursor(0, 74);
  if ((saat >= 7) && (saat <= 22)) {
    tft.setTextColor(0xFFE0, 0x0000);
  } else {
    tft.setTextColor(0xF81F, 0x0000);
  }
  tft.setTextSize(4);
  tft.print(temp_);

  tft.setCursor(60, 74);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.print("C");
  tft.drawCircle(53, 73, 3, ST77XX_GREEN);

  char hum_[3];
  sprintf(hum_, "%02d", hum);
  tft.setCursor(112, 74);
  if ((saat >= 7) && (saat <= 22)) {
    tft.setTextColor(0xFFE0, 0x0000);
  } else {
    tft.setTextColor(0xF81F, 0x0000);
  }

  tft.setTextSize(4);
  tft.print(hum_);

  tft.setCursor(93, 74);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.print("%");

  tft.setCursor(91, 94);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(1);
  tft.print("NEM");
}
//CALENDAR
void takvim() {
  sound.play(90);
  delay(800);
  //GÜN
  sound.play(gun);
  delay(800);
  //AY
  if (ay == 1) {
    sound.play(71);
  } else if (ay == 2) {
    sound.play(72);
  } else if (ay == 3) {
    sound.play(73);
  } else if (ay == 4) {
    sound.play(74);
  } else if (ay == 5) {
    sound.play(75);
  } else if (ay == 6) {
    sound.play(76);
  } else if (ay == 7) {
    sound.play(77);
  } else if (ay == 8) {
    sound.play(78);
  } else if (ay == 9) {
    sound.play(79);
  } else if (ay == 10) {
    sound.play(80);
  } else if (ay == 11) {
    sound.play(81);
  } else if (ay == 12) {
    sound.play(82);
  }
  delay(1000);
  //Haftanın günü
  if (dayWeek == 1) {
    sound.play(83);
  } else if (dayWeek == 2) {
    sound.play(84);
  } else if (dayWeek == 3) {
    sound.play(85);
  } else if (dayWeek == 4) {
    sound.play(86);
  } else if (dayWeek == 5) {
    sound.play(87);
  } else if (dayWeek == 6) {
    sound.play(88);
  } else if (dayWeek == 7) {
    sound.play(89);
  }
  delay(1000);
  //YIL
  sound.play(97);
  delay(1300);
}
//SESLİ SAAT
void time() {
  sound.play(95);
  delay(800);
  sound.play(saat);
  delay(800);
  if (dakika == 0) {
    sound.play(96);
    delay(700);
    sound.play(96);
    delay(800);
  } else if (dakika > 0 && dakika < 10) {
    sound.play(96);
    delay(1000);
    sound.play(dakika);
    delay(800);
  } else {
    sound.play(dakika);
    delay(1000);
  }
}

//SESLİ SICAKLIK VE NEM
void meteo() {
  sound.play(91);
  delay(1000);
  sound.play(temp);
  delay(900);
  sound.play(92);
  delay(900);
  sound.play(93);
  delay(800);
  sound.play(94);
  delay(1000);
  sound.play(hum);
  delay(1000);
}

void vlx() {
  distance = sensor.readRangeContinuousMillimeters();
  // char mesafe_[5];
  // sprintf(mesafe_, "%04d", sensor.readRangeContinuousMillimeters());
  // tft.setCursor(90, 108);
  // tft.setTextColor(0xF800, 0x0000);
  // tft.setTextSize(1);
  // tft.print(mesafe_);

  // tft.setCursor(0, 114);
  // tft.setTextColor(ST77XX_GREEN);
  // tft.setTextSize(1);
  // tft.print("Mesafe:");
}




/* ___İletişim:
e-posta: bilgi@ronaer.com
https://www.instagram.com/dr.tronik2023/   
YouTube: Dr.TRonik: https://www.youtube.com/@DrTRonik
PCBWay: https://www.pcbway.com/project/member/shareproject/?bmbno=A0E12018-0BBC-4C

The nearest ntp pools:
 TURKİYE: "tr.pool.ntp.org"
 Worldwide: pool.ntp.org
 Asia:  asia.pool.ntp.org
 Europe:  europe.pool.ntp.org
 North America: north-america.pool.ntp.org
 Oceania: oceania.pool.ntp.org
 South America: south-america.pool.ntp.org
  
Color definitions for tft:
 #define BLACK 0x0000
 #define BLUE 0x001F
 #define RED 0xF800
 #define GREEN 0x07E0
 #define CYAN 0x07FF
 #define MAGENTA 0xF81F
 #define YELLOW 0xFFE0
 #define WHITE 0xFFFF

DFPlayer Mini:
 3.2v..5.0v, typical 4.2v
 15mA without flash drive, typical 24mA
 24-bit DAC with 90dB output dynamic range and SNR over 85dB
 micro SD-card, up to 32GB (FAT16, FAT32)
 USB-Disk up to 32GB (FAT16, FAT32)
 supports mp3 sampling rate 8KHz, 11.025KHz, 12KHz, 16KHz, 22.05KHz, 24KHz, 32KHz, 44.1KHz, 48KHz
 supports up to 100 folders, each folder can be assigned to 001..255 songs
 built-in 3W mono amplifier, NS8002 AB-Class with standby function
 UART to communicate, 9600bps (parity:none, data bits:8, stop bits:1, flow control:none)

*/
