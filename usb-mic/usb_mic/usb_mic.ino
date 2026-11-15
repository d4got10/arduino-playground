// USB-микрофон: Arduino Leonardo (или Nano/Uno) + модуль KY-037 / KY-038.
//
// Плата оцифровывает сигнал с аналогового выхода модуля (AO) с частотой
// SAMPLE_RATE и шлёт сэмплы по USB-Serial. На компьютере их принимает
// host/usb_mic.py и выводит в звуковое устройство (например, VB-Cable,
// чтобы Discord/Zoom/OBS видели это как микрофон).
//
// Подключение:
//   KY-037  AO -> A0
//   KY-037  G  -> GND
//   KY-037  +  -> 5V
//   (DO и подстроечник не нужны: подстроечник влияет только на DO)
//
// Формат потока: каждый 10-битный сэмпл = 2 байта
//   1-й байт: 1 0 0 0 0 b9 b8 b7   (старший бит = 1 — маркер начала)
//   2-й байт: 0 b6 b5 b4 b3 b2 b1 b0
// Так приёмник сам находит границы сэмплов, даже если начал читать с середины.

#include <Arduino.h>

const uint16_t SAMPLE_RATE = 16000;  // Гц. 8000 — меньше шума от USB, но глуше звук
const uint8_t  MIC_PIN     = A0;

// 1 = внутренний опорник (2.56 В на Leonardo, 1.1 В на Nano/Uno) — выше
// чувствительность, но только если постоянная составляющая сигнала ниже
// опорного напряжения. Скрипт на ПК показывает её как "DC" — проверь там.
#define USE_INTERNAL_REF 0

// Кольцевой буфер на 256 сэмплов (16 мс при 16 кГц).
// uint8_t-индексы переполняются сами и читаются атомарно.
volatile uint16_t ring[256];
volatile uint8_t  head = 0;
volatile uint8_t  tail = 0;

ISR(TIMER1_COMPA_vect) {
  uint16_t v = ADC;          // результат предыдущего преобразования
  ADCSRA |= _BV(ADSC);       // запускаем следующее
  uint8_t next = head + 1;
  if (next != tail) {        // если буфер полон — сэмпл теряется
    ring[head] = v;
    head = next;
  }
}

void setup() {
  // На Leonardo скорость игнорируется (настоящий USB), на Nano/Uno нужен
  // 1 Мбод: 16000 сэмплов * 2 байта * 10 бит = 320 кбит/с.
  Serial.begin(1000000);
  pinMode(LED_BUILTIN, OUTPUT);

#if USE_INTERNAL_REF
  analogReference(INTERNAL);
#endif
  // analogRead() выставляет мультиплексор АЦП на нужный пин (на Leonardo
  // это нетривиально), дальше пользуемся этой настройкой напрямую.
  analogRead(MIC_PIN);
  analogRead(MIC_PIN);

  // Делитель АЦП 32: 16 МГц / 32 = 500 кГц, одно преобразование ~26 мкс
  // (у Arduino по умолчанию 128 — слишком медленно для звука).
  ADCSRA = (ADCSRA & ~0x07) | _BV(ADPS2) | _BV(ADPS0);

  // Timer1 в режиме CTC тикает ровно SAMPLE_RATE раз в секунду.
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);  // CTC, без предделителя
  TCNT1  = 0;
  OCR1A  = F_CPU / SAMPLE_RATE - 1;
  TIMSK1 = _BV(OCIE1A);
  interrupts();

  ADCSRA |= _BV(ADSC);
}

void loop() {
  // Пока на ПК никто не открыл порт, просто выбрасываем сэмплы.
  // Не используем `if (!Serial)`: на Leonardo эта проверка каждый раз
  // ждёт 10 мс, и из 16000 сэмплов в секунду доходило только ~3000.
#ifdef USBCON
  bool connected = Serial.dtr();
#else
  bool connected = true;
#endif
  if (!connected) {
    tail = head;
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }
  digitalWrite(LED_BUILTIN, HIGH);

  uint8_t out[64];  // 64 байта = один USB-пакет
  uint8_t n = 0;
  while (tail != head && n < sizeof(out)) {
    uint16_t v = ring[tail];
    tail = tail + 1;
    out[n++] = 0x80 | (v >> 7);
    out[n++] = v & 0x7F;
  }
  if (n) Serial.write(out, n);
}
