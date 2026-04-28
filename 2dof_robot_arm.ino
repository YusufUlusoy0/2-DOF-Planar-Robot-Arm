#include <Servo.h>
#include <math.h>

// Kolların uzunluğu (cm)

const float L1 = 20.0; 
const float L2 = 20.0; 

Servo servo1;
Servo servo2;

const float PI_DEGERI = 3.14159265359;


// 1. HIZ AYARI (Ne kadar büyükse o kadar YAVAŞ)
int hareketHizi = 20; 

// 2. OFSET AYARLARI 
// Sola döndürmek için artır sağa döndürmek için azalt

int offsetS1_us = -30;   // 1. Kol
int offsetS2_us = 140;   // 2. Kol 

// Servo Sinyal Limitleri 
const int MIN_PULSE = 460;
const int MAX_PULSE = 2460;

// Konum Hafızası
float mevcutPulseS1 = 1472; 
float mevcutPulseS2 = 544;  

void setup() {
  Serial.begin(9600);
  servo1.attach(11, MIN_PULSE, MAX_PULSE);
  servo2.attach(12, MIN_PULSE, MAX_PULSE);
  
  hassasVeYavasSur(90.0, 0.0);
  
  Serial.println("--- KALIBRASYONLU MOD ---");
  Serial.print("2. Kol Ofseti: "); Serial.println(offsetS2_us);
  Serial.println("Robot 2. kolu duzeltmek icin bu degerle oynayin.");
}

void loop() {
  if (Serial.available() > 0) {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat(); 
    while (Serial.available() > 0) Serial.read();

    if (isnan(x) || isnan(y)) return;
    
    Serial.print("Hedef: "); Serial.print(x);
    Serial.print(", "); Serial.println(y);
    hesaplaVeGit(x, y);
  }
}

void hesaplaVeGit(float x, float y) {
  // Güvenlik
  if (y < 0) { Serial.println("HATA: Y < 0"); return; }
  float r = sqrt(sq(x) + sq(y));
  if (r > (L1 + L2)) { Serial.println("HATA: Menzil dışı"); return; }
  if (r < 12) { Serial.println("HATA: Hedef merkeze çok yakın!"); return; }

  // Ters Kinematik
  float cos_q2 = (sq(x) + sq(y) - sq(L1) - sq(L2)) / (2 * L1 * L2);
  if (cos_q2 < -1.0) cos_q2 = -1.0;
  if (cos_q2 > 1.0) cos_q2 = 1.0;
  
  float q2_rad = acos(cos_q2); 
  float q1_rad = atan2(y, x) - atan2(L2 * sin(q2_rad), L1 + L2 * cos(q2_rad));

  float q1_deg = q1_rad * (180.0 / PI_DEGERI);
  float q2_deg = q2_rad * (180.0 / PI_DEGERI);

  // Açı Uyarısı (İç açı daralırsa)
  if (q2_deg > 90.0) {
    Serial.println("UYARI: 2. Kol limiti zorlaniyor! (Dar Aci)");
  }

  Serial.print("Aci S1: "); Serial.print(q1_deg, 1);
  Serial.print(" S2: "); Serial.println(q2_deg, 1);

  hassasVeYavasSur(q1_deg, q2_deg);
}

void hassasVeYavasSur(float hedefS1_deg, float hedefS2_deg) {
  
  hedefS1_deg = constrain(hedefS1_deg, 0.0, 180.0);
  hedefS2_deg = constrain(hedefS2_deg, 0.0, 180.0);

  // Açı -> Sinyal Dönüşümü
  float hedefPulseS1 = MIN_PULSE + ((hedefS1_deg / 180.0) * (MAX_PULSE - MIN_PULSE));
  float hedefPulseS2 = MIN_PULSE + ((hedefS2_deg / 180.0) * (MAX_PULSE - MIN_PULSE));

  // --- OFSETLER BURADA EKLENİYOR ---
  hedefPulseS1 += offsetS1_us;
  hedefPulseS2 += offsetS2_us;

  // Hareket Döngüsü
  float farkS1 = hedefPulseS1 - mevcutPulseS1;
  float farkS2 = hedefPulseS2 - mevcutPulseS2;
  int adimSayisi = max(abs(farkS1), abs(farkS2)); 
  if (adimSayisi < 1) adimSayisi = 1;

  for (int i = 1; i <= adimSayisi; i++) {
    float anlikPulseS1 = mevcutPulseS1 + (farkS1 * i / adimSayisi);
    float anlikPulseS2 = mevcutPulseS2 + (farkS2 * i / adimSayisi);
    
    servo1.writeMicroseconds((int)anlikPulseS1);
    servo2.writeMicroseconds((int)anlikPulseS2);
    
    delayMicroseconds(hareketHizi * 100); 
  }

  mevcutPulseS1 = hedefPulseS1;
  mevcutPulseS2 = hedefPulseS2;
  
  // Hedefi son kez sabitle
  servo1.writeMicroseconds((int)hedefPulseS1);
  servo2.writeMicroseconds((int)hedefPulseS2);
}
