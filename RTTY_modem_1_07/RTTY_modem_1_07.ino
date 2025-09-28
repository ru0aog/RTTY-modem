// *****************************************************************
// RTTY-коммуникатор
// Приём/передача символов RTTY
// + цифровой фильтр принимаемого сигнала
// RU0AOG ver. 1.07 2025-09-27
// *****************************************************************

// назначим входные данные
#define AUDI_PIN_IN   2          // пин входа НЧ-сигнала (частота)
#define DIGI_PIN_IN   3          // пин входа сигнала (цифра)
#define DIGI_PIN_OUT  4          // пин выхода сигнала (цифра)
#define AUDI_PIN_OUT  5          // пин выхода НЧ-сигнала (частота)
#define INVERSE_PIN   6          // пин переключателя инверсии
#define BODSPDPIN1    7          // пин включения скорости1 протокола
#define LEDMARK_PIN   8          // пин светодиода индикации MARK
#define LEDSPACE_PIN  9          // пин светодиода индикации SPACE
#define BODSPDPIN2   10          // пин включения скорости2 протокола
#define BODSPDPIN3   11          // пин включения скорости3 протокола
#define BODSPDPIN4   12          // пин включения скорости4 протокола
#define PTT_PIN      14          // пин индикации активности передачи, использовать как РТТ

#define MARK       1
#define SPACE      0
#define FREQMARK   1170          // частота MARK
#define FREQSPACE  1000          // частота SPACE
#define DIFF_FREQ    50          // допустимое отклонение частот +/- Гц
#define RCV_TIME_OUT  5          // время таймаута приёма строки, сек
#define EEPROM_BUF  500          // длина буфера еепром, байт
#define RXTIK_ERROR   8          // кол-во верно принятых тиков (от 7 до 14) для бита принимаемого символа
#define BODESPEED 45.45          // скорость протокола по умолчанию
#define BODESPEED1  50           // скорость1 протокола
#define BODESPEED2 100           // скорость2 протокола
#define BODESPEED3 150           // скорость3 протокола
#define BODESPEED4 200           // скорость4 протокола

#define TIME_PAUSE 3             // пауза между повторной передачей, сек
#define EEPROM_ADDRESS 0x50      // адрес шины I2C для ЕЕПРОМ
#define LCDI2C_ADDRESS 0x27      // адрес шины I2C для LCD дисплея

// объявление констант и переменных
char MESSAGE[190] = "CQ CQ ПРИВЕТ! DE RU0AOG RU0AOG RTTY BEACON МАЯК";    // текст передачи маяка по умолчанию
String LINE;                                // принятая строка для LCD
String RCV_MESSAGE;                         // принятая строка для eeprom

volatile uint16_t Time_pause = TIME_PAUSE;        // значение паузы между посылками маяка, сек
volatile uint32_t TimeUp  = 0,  TimeUpPrv = 0;    // значение системного времени для вычисления периода частоты сигнала
volatile uint32_t TimeDur = 0;                    // длительность периода частоты сигнала

volatile uint32_t Freq;         // значение частоты сигнала
volatile uint8_t  Mark;         // флаг наличия MARK
volatile uint8_t  Space;        // флаг наличия SPACE

volatile uint8_t  mark[5];      // принятые марки
volatile uint8_t  space[5];     // принятые спейсы


volatile uint8_t rSq;           // состояние приёма данных (0 - ожидание, 1 - начата передача, 2 - принят стартовый бит, 3 - принимается символ)
volatile uint8_t ti;            // счётчик тиков таймера
volatile uint8_t dsp;           // флаг готовности принятого символа
volatile uint8_t baudotTX;      // регистр передаваемого символа
volatile uint8_t baudotTX1;     // подготовка кода Бодо передаваемого символа
volatile uint8_t baudotRX;      // код Бодо принятого символа
volatile uint8_t baudotRX_prv;  // предыдущий код Бодо принятого символа
volatile float   baudotSpeed;   // скорость протокола
volatile uint8_t INVERSE = 0;   // состояние инверсии (0 - нет инверсии, 1 - есть инверсия)

volatile uint8_t TXRX_state = 0;// флаг режима ПРМ/ПРД (0 - RX, 1 = TX)
volatile uint8_t fig;           // флаг цифр/букв кода Бодо   (0 - цифра, 1 - ЛАТ, 2 - РУС)
volatile int     fig_TX;        // флаг передаваемого символа (0 - цифра, 1 - ЛАТ, 2 - РУС)
volatile int     fig_TX_prv;    // флаг предыдущего символа   (0 - цифра, 1 - ЛАТ, 2 - РУС)
boolean  crLf;
volatile uint8_t snd;           // флаг разрешения передачи
volatile char    ch;            // принятый символ в ASCII

volatile uint8_t toneOn = 0;    // флаг разрешения тона
volatile uint8_t toneState;     // состояние пина тон

uint8_t  beacon = 0;
uint8_t  transmit = 0;
uint32_t currtime, prvtime;

const word address = 0;
const word eeprom_start_address = 0;

const byte count = 16;
byte inputBytes[count] = { 0 };
byte outputBytes[count] = { 0 };

uint8_t  eofRx = 0;                // флаг окончания
uint32_t RCVTimeOut;               // счётчик таймаута приёма строки
uint8_t  RCVInProgress = 0;        // флаг приёма строки
int      EOF_address;


uint8_t UTF_charConvert (uint8_t lcdstr) {
  // функция конвертации русских букв из UTF-8 в Windows-1251
  uint8_t letterin = 0;
  uint8_t letterout = 0;
  static uint8_t letterin_prv;
      letterin = lcdstr;
      if (letterin != 208 && letterin != 209) {   // если появился втрой байт - пропустить
        if (letterin >= 32 && letterin <= 127)    {letterout = letterin;}         // латинский шрифт
        if (letterin >= 97 && letterin <= 122)    {letterout = letterin - 32;}    // латинский шрифт: a -> A
        
        if (letterin >= 144 && letterin <= 175)   {letterout = letterin + 48;}    // русский шрифт
        if (letterin >= 176 && letterin <= 191)   {letterout = letterin + 16;}    // русский шрифт: a -> A
        if (letterin >= 128 && letterin <= 143)   {letterout = letterin + 80;}    // русский шрифт: р -> Р
        if (letterout == 218)                     {letterout = 220;}              // русский шрифт: заменить Ъ на Ь
        if (letterin == 129 && letterin_prv == 208) {letterout = 197;}            // русский шрифт: заменить Ё на Е
        if (letterin == 145 && letterin_prv == 209) {letterout = 197;}            // русский шрифт: заменить ё на Е
    }
    letterin_prv = letterin;
    return letterout;
}

// Подключаем библиотеки
#include "LiquidCrystal_I2C.h"
#include "LiquidCrystalCyr.h"                   // кодировка - Windows-1251
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Eeprom24C32_64.h>

LiquidCrystalCyr lcd (LCDI2C_ADDRESS, 20, 4);   // LCD дисплей 2 строки на 16 символов
Eeprom24C32_64 eeprom(EEPROM_ADDRESS);

tmElements_t tm;                                // переменная с датой/временем

void setup() {
// инициализация
 // определить входы/выходы
   pinMode(AUDI_PIN_IN, INPUT_PULLUP);     // пин входа НЧ-сигнала
   pinMode(DIGI_PIN_IN, INPUT_PULLUP);     // пин входа сигнала (цифра)
   pinMode(LEDMARK_PIN,  OUTPUT);          // пин светодиода MARK
   pinMode(LEDSPACE_PIN, OUTPUT);          // пин светодиода SPACE
   pinMode(PTT_PIN,      OUTPUT);          // пин индикации активности передачи, использовать как РТТ
   pinMode(AUDI_PIN_OUT, OUTPUT);          // пин выхода НЧ-сигнала (частота)
   pinMode(DIGI_PIN_OUT, OUTPUT);          // пин выхода сигнала (цифра)
   pinMode(INVERSE_PIN,INPUT_PULLUP);      // пин переключателя инверсии
   pinMode(BODSPDPIN1, INPUT_PULLUP);      // пин включения скорости1 протокола
   pinMode(BODSPDPIN2, INPUT_PULLUP);      // пин включения скорости2 протокола
   pinMode(BODSPDPIN3, INPUT_PULLUP);      // пин включения скорости3 протокола
   pinMode(BODSPDPIN4, INPUT_PULLUP);      // пин включения скорости4 протокола

 // назначить прерывание
   detachInterrupt(0);                     // отключить внешнее прерывание INT0
   attachInterrupt(0, ISR_AUDI, RISING);   // подключить прерывание INT0 к функции ISR_AUDI по фронту сигнала
   detachInterrupt(1);                     // отключить внешнее прерывание INT1
   attachInterrupt(1, ISR_DIGI, CHANGE);   // подключить прерывание INT1 к функции ISR_DIGI по изменению сигнала
 // инициализация
   eeprom.initialize();
   lcd.init();
   lcd.clear();
   lcd.begin(16, 2);
   Serial.begin(9600);
   toneOn = 0;                             // генератор таймера2 - ВЫКЛ
   fig = 0;                                // принимаем буквы
   eofRx = 1;                              // начинаем приём с новой строки
   snd = 0;
  // рапорт LCD
   lcd.backlight();
   lcd.setCursor(0, 0);
   lcd.print("S");
   lcd.print(FREQSPACE);
   LCD_Print("ГЦ");
   lcd.print("  M");
   lcd.print(FREQMARK);
   LCD_Print("ГЦ");
   lcd.setCursor(0, 1);
   Bod_speed(BODESPEED);                    // установить скорость протокола
   lcd.setCursor(0, 1);
   LINE += "  ГОТОВ";
   LCD_Print(LINE);                    // ГОТОВ
  // рапорт UART
   Serial.print("SPACE = ");Serial.print(FREQSPACE); Serial.println(" Гц");
   Serial.print("MARK  = "); Serial.print(FREQMARK); Serial.println(" Гц");
   Serial.println("ГОТОВ");
}


void SerialCommand() {
  // приём и обработка команд
  if (Serial.available()) {                              // если доступно к чтению более 0 байт, то
    String incoming = Serial.readStringUntil('\n');      // читать строку до LF
    String inc_after = "";
    String inc_after2 = "";
    int inc_after_i = 0;
    incoming.trim();                                     // убрать перевод строки LF и возврат каретки CR
    inc_after_i = incoming.indexOf('/');                 // искать в строке командный символ '/'
    if (inc_after_i != -1) {
    for (int i = inc_after_i+1; i <= incoming.length(); i++) {
        inc_after += incoming[i];}
    }

    if (inc_after == "bk") {                             // разрешить передачу маяка 
      lcd.print("bk");                                   // передаётся последнее переданное сообщение
      if (beacon == 0) {
        Serial.println("маяк ВКЛ");
        beacon = 1;
        prvtime = currtime - (Time_pause * 1000UL);
        }
      else {
        beacon = 0;
        Serial.println("маяк ВЫКЛ");
        }
    }
    else if (inc_after == "rx") {    // запустить приём
     lcd.clear();
     lcd.setCursor(0, 1);
      Serial.println("приём - ВКЛ");
      transmit = 0;
      beacon = 0;
      TXRX_state = 0;
      toneOn = 0;                    // запретить генерацию
    }
    else if (inc_after[0] == 'p' || inc_after[0] == 'P') {    // установить время паузы
    for (int i = 1; i <= inc_after.length(); i++) {
        inc_after2 += inc_after[i];}
        inc_after2.trim();
     Time_pause = inc_after2.toInt();
     lcd.clear();
     lcd.setCursor(0, 1);
      Serial.print("Время паузы маяка = "); Serial.print(Time_pause); Serial.println(" сек");
    }
    else if (inc_after == "?") {    // вывести инфо
     lcd.clear();
     lcd.setCursor(0, 1);
     lcd.print("RTFM!");
      Serial.println("Передаётся вводимый текст");
      Serial.println("посли символа / следует команда");
      Serial.println("/bk  - периодически передавать последнее сообщение в режиме маяка");
      Serial.println("/rx  - выйти из режима маяка");
      Serial.println("/pxxxx  - установить время паузы между посылками маяка, хххх в секундах");
    }
    else{
  // если нет команд, то это текст на передачу
      transmit = 1;
      beacon = 0;
      prvtime = currtime - (Time_pause * 1000UL);
      Serial.print(">>"); Serial.println(incoming);
      incoming.toCharArray(MESSAGE,incoming.length()+1);}  // преобразовать строку в массив символов
  }
}


void ISR_AUDI() {
  // обработчик прерывания INT0
  // измеряем период частоты
   cli();
     TimeUp = micros();                     // считать текущее время
     TimeDur = TimeUp - TimeUpPrv;          // длительность периода
     TimeUpPrv = TimeUp;                    // обновить таймер
     if (TimeDur > 0){
       Freq = 1000000/TimeDur;              // вычислить текущую частоту
       if      (Freq >= FREQMARK-DIFF_FREQ  && Freq <= FREQMARK+DIFF_FREQ)      {Mark  = 1; Space = 0;}     // пишем MARK
       else if (Freq >= FREQSPACE-DIFF_FREQ && Freq <= FREQSPACE+DIFF_FREQ)     {Space = 1; Mark  = 0;}     // пишем SPACE
       else {Mark  = 0; Space = 0;}                                             // если частота невнятная, то опустить все флаги
     }
   sei();
}

void ISR_DIGI() {
  // обработчик прерывания INT1
  // получаем готовый цифровой сигнал Бодо
   cli();
     // в работе :)
   sei();
}

ISR(TIMER1_COMPA_vect) {
  // обработчик прерывания по таймеру1
  // генерация тонов AFSK
  if (toneOn == 1) {                             // если генератор - ВКЛ
    digitalWrite(AUDI_PIN_OUT, toneState);
    toneState = !toneState;                      // менять состояние пина по прерыванию
  }
  if (toneOn == 0) {                             // если генератор - ВЫКЛ
    digitalWrite(AUDI_PIN_OUT, LOW);
    toneState = 0;
  }
}

ISR(TIMER2_COMPA_vect) {
  // обработчик прерывания по таймеру2
  // генерация тиков скорости бод
   cli();
   if (TXRX_state == 1) {                         // режим TX передача
     static boolean bit0;                         // создать блок локальных статических пременных
     static boolean bit1;
     static boolean bit2;
     static boolean bit3;
     static boolean bit4;
     if(snd == 1) {                             // выполняется передача символа
           switch(ti) {                            // в зависимости от текущего тика:
              case 0:
                 Mark  = 0;       // запускаем передачу стартового бита
                 Space = 1;      // запускаем передачу стартового бита
                 AFSKgenerator(SPACE);                 // генерировать частоту SPACE
                 bit0 = baudotTX & B00001;             // разобрать код на биты
                 bit1 = baudotTX & B00010;
                 bit2 = baudotTX & B00100;
                 bit3 = baudotTX & B01000;
                 bit4 = baudotTX & B10000;
                 break;
              case 22:                                   // с 22 тика - начало передачи бит0
                 Mark  = bit0;
                 Space = !bit0;
                 //digitalWrite(LEDMARK_PIN, bit0);        //   установить LEDMARK_PIN
                 //digitalWrite(LEDSPACE_PIN,!bit0);       //   установить LEDSPACE_PIN
                 if  (bit0 == 1) {AFSKgenerator(MARK);}  //   генерировать частоту MARK/SPACE
                 else            {AFSKgenerator(SPACE);}
                 break;
              case 44:                                   // с 44 тика - начало передачи бит1
                 Mark  = bit1;
                 Space = !bit1;
                 if  (bit1 == 1) {AFSKgenerator(MARK);}  //   генерировать частоту MARK/SPACE
                 else            {AFSKgenerator(SPACE);}
                 break;
              case 66:                                   // с 66 тика - начало передачи бит2
                 Mark  = bit2;
                 Space = !bit2;
                 if  (bit2 == 1) {AFSKgenerator(MARK);}  //   генерировать частоту MARK/SPACE
                 else            {AFSKgenerator(SPACE);}
                 break;
              case 88:                                   // с 88 тика - начало передачи бит3
                 Mark  = bit3;
                 Space = !bit3;
                 if  (bit3 == 1) {AFSKgenerator(MARK);}  //   генерировать частоту MARK/SPACE
                 else            {AFSKgenerator(SPACE);}
                 break;
              case 110:                                  // с 110 тика - начало передачи бит4
                 Mark  = bit4;
                 Space = !bit4;
                 if  (bit4 == 1) {AFSKgenerator(MARK);}  //   генерировать частоту MARK/SPACE
                 else            {AFSKgenerator(SPACE);}
                 break;
              case 132:                                  // с 132 тика - начало стоп-бита х1,5
                 Mark  = 1;        // стоп-бит
                 Space = 0;        // стоп-бит
                 AFSKgenerator(MARK);                    //   генерировать частоту MARK
                 break;
              case 165:                                  // с 165 тика - конец передачи
                 AFSKgenerator(0);                       //   выключить генератор AFSK
                 Mark  = 0;        // выключить
                 Space = 0;        // выключить
                 snd = 0;
                 ti = 0;
                 break;
           }
          ti++;
        }
   }
   else {                                         // режим RX приём (TXRX_state == 0)
    // обработка принимаемого сигнала
      ti++;
      if(rSq == 0 && Mark == 0 && Space == 0) {  // нет сигнала
            rSq = 0;                             //   установить режим ожидания сигнала
            ti = 0;}                             //   сбросить счётчик тиков
      if(rSq == 0 && Mark == 1) {                // появился марк, замечаем время и готовимся принять стартовый бит
            rSq = 1;                             //   установить режим контроля длительности стоп-бита
            ti = 0;}                             //   сбросить счётчик тиков
      if(rSq == 1 && ti > 31 && Space == 1) {    // марк длился как минимум 31 тик - принимаем стартовый бит
            rSq = 2;                             //   установить режим ожидания приёма старт-бита
            ti = 0;}                             //   сбросить счётчик тиков
      if(rSq == 2 && ti == 10) {                 // готовимся принимать стартовый бит, начало анализа от 8 до 12 тиков
            if(Space == 1) {                     //   если есть стартовый бит
                rSq = 3;                         //      установить режим приёма символа
                ti = 0;                          //      сбросить счётчик тиков
                for (uint8_t i = 0; i < 5; i++){ //      очистить счётчики верных тиков
                    mark[i] = 0;
                    space[i] = 0;}
            }
            else  {                              //   мы ждали стартовый бит, а его нет - ошибка
                rSq = 0;}                        //      установить режим ожидания сигнала
      }
    // ошибка приёма
      if (ti > 165) {                            // если тики вышли за пределы длительности символа - ошибка
          rSq = 0;                               //   установить режим ожидания сигнала
          ti = 0;}                               //   сбросить счётчик тиков 
    // режим приёма символа
    // бит 0
      if(rSq == 3 && ti >= 14 && ti < 30) {      // приём бита 0 - подсчёт состояния входного сигнала от 14 до 30 тиков
          if (Mark  == true) {mark[0]++;}        //   принимаем MARK  - добавляем единичку к счётчику марков  0-го бита
          if (Space == true) {space[0]++;}}      //   принимаем SPACE - добавляем единичку к счётчику спейсов 0-го бита
      if(rSq == 3 && ti == 30) {                 // на 30-ом тике подсчитываем сумму
           if      (mark[0] > RXTIK_ERROR) {bitWrite(baudotRX, 0, true);}  // если марков  больше порога ошибки - то бит0 = 1
           else if (space[0]> RXTIK_ERROR) {bitWrite(baudotRX, 0, false);} // если спейсов больше порога ошибки - то бит0 = 0
           else {                                                        // если порог ошибки не преодолели - ошибка
            rSq = 0;}                                                    //   установить режим ожидания сигнала
           mark[0] = 0;                          // сбросить счётчик марков  0-го бита
           space[0] = 0;}                        // сбросить счётчик спейсов 0-го бита
    // бит 1
      if(rSq == 3 && ti >= 35 && ti < 52) {      // приём бита 1 - подсчёт состояния входного сигнала от 35 до 52 тиков
          if (Mark  == true) {mark[1]++;}        //   принимаем MARK  - добавляем единичку к счётчику марков  1-го бита
          if (Space == true) {space[1]++;}}      //   принимаем SPACE - добавляем единичку к счётчику спейсов 1-го бита
      if(rSq == 3 && ti == 52) {                 // на 52-ом тике подсчитываем сумму
           if      (mark[1] > RXTIK_ERROR) {bitWrite(baudotRX, 1, true);}  // если марков  больше порога ошибки - то бит1 = 1
           else if (space[1]> RXTIK_ERROR) {bitWrite(baudotRX, 1, false);} // если спейсов больше порога ошибки - то бит1 = 0
           else {                                                        // если порог ошибки не преодолели - ошибка
            rSq = 0;}                                                    //   установить режим ожидания сигнала
           mark[1] = 0;                          // сбросить счётчик марков  1-го бита
           space[1] = 0;}                        // сбросить счётчик спейсов 1-го бита
    // бит 2
      if(rSq == 3 && ti >= 56 && ti < 73) {      // приём бита 2 - подсчёт состояния входного сигнала от 56 до 73 тиков
          if (Mark  == true) {mark[2]++;}        //   принимаем MARK  - добавляем единичку к счётчику марков  2-го бита
          if (Space == true) {space[2]++;}}      //   принимаем SPACE - добавляем единичку к счётчику спейсов 2-го бита
      if(rSq == 3 && ti == 73) {                 // на 73-ом тике подсчитываем сумму
           if      (mark[2] > RXTIK_ERROR) {bitWrite(baudotRX, 2, true);}  // если марков  больше порога ошибки - то бит2 = 1
           else if (space[2]> RXTIK_ERROR) {bitWrite(baudotRX, 2, false);} // если спейсов больше порога ошибки - то бит2 = 0
           else {                                                        // если порог ошибки не преодолели - ошибка
            rSq = 0;}                                                    //   установить режим ожидания сигнала
           mark[2] = 0;                          // сбросить счётчик марков  2-го бита
           space[2] = 0;}                        // сбросить счётчик спейсов 2-го бита
    // бит 3
      if(rSq == 3 && ti >= 79 && ti < 95) {      // приём бита 3 - подсчёт состояния входного сигнала от 79 до 95 тиков
          if (Mark  == true) {mark[3]++;}        //   принимаем MARK  - добавляем единичку к счётчику марков  3-го бита
          if (Space == true) {space[3]++;}}      //   принимаем SPACE - добавляем единичку к счётчику спейсов 3-го бита
      if(rSq == 3 && ti == 95) {                 // на 95-ом тике подсчитываем сумму
           if      (mark[3] > RXTIK_ERROR) {bitWrite(baudotRX, 3, true);}  // если марков  больше порога ошибки - то бит3 = 1
           else if (space[3]> RXTIK_ERROR) {bitWrite(baudotRX, 3, false);} // если спейсов больше порога ошибки - то бит3 = 0
           else {                                                        // если порог ошибки не преодолели - ошибка
            rSq = 0;}                                                    //   установить режим ожидания сигнала
           mark[3] = 0;                          // сбросить счётчик марков  3-го бита
           space[3] = 0;}                        // сбросить счётчик спейсов 3-го бита
    // бит 4
      if(rSq == 3 && ti >= 102 && ti < 119) {    // приём бита 4 - подсчёт состояния входного сигнала от 102 до 119 тиков
          if (Mark  == true) {mark[4]++;}        //   принимаем MARK  - добавляем единичку к счётчику марков  4-го бита
          if (Space == true) {space[4]++;}}      //   принимаем SPACE - добавляем единичку к счётчику спейсов 4-го бита
      if(rSq == 3 && ti == 119) {                 // на 119-ом тике подсчитываем сумму
           if      (mark[4] > RXTIK_ERROR) {bitWrite(baudotRX, 4, true);}  // если марков  больше порога ошибки - то бит4 = 1
           else if (space[4]> RXTIK_ERROR) {bitWrite(baudotRX, 4, false);} // если спейсов больше порога ошибки - то бит4 = 0
           else {                                                        // если порог ошибки не преодолели - ошибка
            rSq = 0;}                                                    //   установить режим ожидания сигнала
           mark[4] = 0;                          // сбросить счётчик марков  4-го бита
           space[4] = 0;                         // сбросить счётчик спейсов 4-го бита
           dsp = 1;                              // установить флаг - символ принят
           rSq = 0;}                             // установить режим ожидания сигнала
   }
   sei();
}







// основной цикл
void loop() {
   resetBod_speed();                                     // проверить переключатель скорости протокола
   SerialCommand();                                      // принять команды
   currtime = millis();
   if (transmit == 1 || beacon == 1) {                   // если передача разрешена
     if (currtime > ((Time_pause * 1000UL) + prvtime)){  // и время вышло, то готовимся к передаче
      //Serial.println("");
      lcd.clear();
      lcd.setCursor(0, 0);
      LCD_Print(String(LINE));
      LINE = "";
      lcd.setCursor(0, 1);
      LINE = "";
/*
       char lin1[20];
       char lin2[150];
       lin2[0] = '\0';
                       if (RTC.read(tm)) {                // метка времени
                         if (tm.Day    <10) {LINE = '0'+String(tm.Day)+'/';}   else {LINE = String(tm.Day)+'/';}
                         if (tm.Month  <10) {LINE += '0'+String(tm.Month)+' ';} else {LINE += String(tm.Month)+' ';}
                         if (tm.Hour   <10) {LINE += '0'+String(tm.Hour)+':';}  else {LINE += String(tm.Hour)+':';}
                         if (tm.Minute <10) {LINE += '0'+String(tm.Minute)+':';}else {LINE += String(tm.Minute)+':';}
                         if (tm.Second <10) {LINE += '0'+String(tm.Second);}    else {LINE += String(tm.Second);}
                        LINE += '>';
                        LINE.toCharArray(lin1, 20);
                        strcat(lin2, lin1);
                        strcat(lin2, MESSAGE);
                        LINE = "";
                       }
*/
       rttyTx(MESSAGE);                // передать сообщение
       currtime = millis();         // сбросить таймер
       prvtime = currtime;
     }
   }
          

if (dsp == 1 && TXRX_state == 0) {  // если в режиме приёма и принят символ
  decodeBodo();                     // декодировать символ
  }


  if (millis() > RCVTimeOut + (RCV_TIME_OUT*5000000) && RCVInProgress == 1) {   // если приём был давно, то завершить
    baudotRX_prv = B01000;
    baudotRX = B00010;
    dsp = 1;
    }

  // включить светодиоды LEDMARK/LEDSPACE и установить DIGI_PIN_OUT
  if (INVERSE == 0){
    if (Mark  == 1) digitalWrite(LEDMARK_PIN, HIGH);  else digitalWrite(LEDMARK_PIN, LOW);
    if (Space == 1) digitalWrite(LEDSPACE_PIN, HIGH); else digitalWrite(LEDSPACE_PIN, LOW);
    if (Mark == 1)  digitalWrite(DIGI_PIN_OUT, HIGH); else digitalWrite(DIGI_PIN_OUT, LOW);}
  else{
    if (Mark  == 0) digitalWrite(LEDMARK_PIN, HIGH);  else digitalWrite(LEDMARK_PIN, LOW);
    if (Space == 0) digitalWrite(LEDSPACE_PIN, HIGH); else digitalWrite(LEDSPACE_PIN, LOW);
    if (Mark == 0)  digitalWrite(DIGI_PIN_OUT, HIGH); else digitalWrite(DIGI_PIN_OUT, LOW);}
}



void rttyTxSymbol(uint8_t symbol) {
 // передача символа
 while (snd == 1) {}          // ждать, пока не окончится предыдущая передача
  if (fig_TX != -1){ 
  if (fig_TX != fig_TX_prv) { // если регистр изменился, то сначала передать код регистра
    switch(fig_TX) {          //   0 - ЛАТ, 1 - цифра, 2 - РУС
      case 0: baudotTX = B11111; break;  // 0 - ЛАТ
      case 1: baudotTX = B11011; break;  // 1 - цифры/символы
      case 2: baudotTX = B00000; break;  // 2 - РУС
    }
   ti = 0;                    // сбросить счётчик тиков таймера2
   snd = 1;                   // запустить передачу символа (сбросится в 0 по завершении передачи)
   //  Serial.println(baudotTX, BIN);
  while (snd == 1) {}         // ждать, пока не окончится передача
  fig_TX_prv = fig_TX;        // обновить состояние последнего регистра
  }
   baudotTX = baudotTX1;      // вписать в регистр передачи передаваемый символ
   //  Serial.println(baudotTX, BIN);
   ti = 0;                    // сбросить счётчик тиков таймера2
   snd = 1;                   // запустить передачу символа (сбросится в 0 по завершении передачи)
  }
}

void rttyTxDirect(uint8_t command) {
 // прямая передача кода
 while (snd == 1) {}         // ждать, пока не окончится передача
   baudotTX = command;
   //  Serial.println(baudotTX, BIN);
   ti = 0;                    // сбросить счётчик тиков таймера2
   snd = 1;                   // запустить передачу символа (сбросится в 0 по завершении передачи)
}

void rttyTx(char message[]) {
  // процедура передачи сообщения
    TXRX_state = 1;                          // передача начата
    digitalWrite(PTT_PIN, HIGH);             // активировать PTT_PIN - индикация процесса передачи
    delay(50);
    toneOn = 1;
    AFSKgenerator(MARK); delay(200);         // передавать MARK 0.2 с
  // начало сообщения
    rttyTxDirect(B11111);                     // переключиться на регистр ЛАТ
    rttyTxDirect(B11111);                     // переключиться на регистр ЛАТ
    fig_TX_prv = -1;
  // цикл по символам сообщения
   for(uint16_t i = 0; i < strlen(message); i++) {
     //Serial.print(message[i], HEX); Serial.print(" - ");
     //   Serial.println(message[i]);
     Serial.print(message[i]);
     uint8_t dd = UTF_charConvert(message[i]);
     //Serial.println(dd, DEC);
     if (dd != 0){
     codeBodo(dd);  // закодировать символ, код -> baudotTX1, регистр -> fig_TX
     rttyTxSymbol(dd);                // передать символ
    if (LINE.length() == 16) {      // скроллинг дисплея вверх
      lcd.clear();
      lcd.setCursor(0, 0);
      LCD_Print(String(LINE));
      LINE = "";
      lcd.setCursor(0, 1);
      }
    LINE += message[i];                          // добавить символ к строке
    LCD_Print(String(message[i]));
  }}
  // конец сообщения
    rttyTxDirect(B01000);                     // CR
    rttyTxDirect(B00010);                     // LF
   delay(200);
   digitalWrite(PTT_PIN, LOW);                // выключить PTT_PIN - индикация конца передачи
   toneOn = 0;
   //if (transmit == 1) Serial.println("передача завершена");
   Serial.println();
   transmit = 0;
   TXRX_state = 0;                          // передача окончена
}

void codeBodo(uint8_t chTX)
{
        //Serial.println(chTX, DEC);
        fig_TX = -1;
        baudotTX1 = 0;
        switch(chTX)
        {       
                case 'A': baudotTX1 = B00011; fig_TX = 0; break;
                case 'B': baudotTX1 = B11001; fig_TX = 0; break;
                case 'C': baudotTX1 = B01110; fig_TX = 0; break;
                case 'D': baudotTX1 = B01001; fig_TX = 0; break;
                case 'E': baudotTX1 = B00001; fig_TX = 0; break;
                case 'F': baudotTX1 = B01101; fig_TX = 0; break;
                case 'G': baudotTX1 = B11010; fig_TX = 0; break;
                case 'H': baudotTX1 = B10100; fig_TX = 0; break;
                case 'I': baudotTX1 = B00110; fig_TX = 0; break;
                case 'J': baudotTX1 = B01011; fig_TX = 0; break;
                case 'K': baudotTX1 = B01111; fig_TX = 0; break;
                case 'L': baudotTX1 = B10010; fig_TX = 0; break;
                case 'M': baudotTX1 = B11100; fig_TX = 0; break;
                case 'N': baudotTX1 = B01100; fig_TX = 0; break;
                case 'O': baudotTX1 = B11000; fig_TX = 0; break;
                case 'P': baudotTX1 = B10110; fig_TX = 0; break;
                case 'Q': baudotTX1 = B10111; fig_TX = 0; break;
                case 'R': baudotTX1 = B01010; fig_TX = 0; break;
                case 'S': baudotTX1 = B00101; fig_TX = 0; break;
                case 'T': baudotTX1 = B10000; fig_TX = 0; break;
                case 'U': baudotTX1 = B00111; fig_TX = 0; break;
                case 'V': baudotTX1 = B11110; fig_TX = 0; break;
                case 'W': baudotTX1 = B10011; fig_TX = 0; break;
                case 'X': baudotTX1 = B11101; fig_TX = 0; break;
                case 'Y': baudotTX1 = B10101; fig_TX = 0; break;
                case 'Z': baudotTX1 = B10001; fig_TX = 0; break;
                
                case '0': baudotTX1 = B10110; fig_TX = 1; break;
                case '1': baudotTX1 = B10111; fig_TX = 1; break;
                case '2': baudotTX1 = B10011; fig_TX = 1; break;
                case '3': baudotTX1 = B00001; fig_TX = 1; break;
                case '4': baudotTX1 = B01010; fig_TX = 1; break;
                case '5': baudotTX1 = B10000; fig_TX = 1; break;
                case '6': baudotTX1 = B10101; fig_TX = 1; break;
                case '7': baudotTX1 = B00111; fig_TX = 1; break;
                case '8': baudotTX1 = B00110; fig_TX = 1; break;
                case '9': baudotTX1 = B11000; fig_TX = 1; break;
                case '-': baudotTX1 = B00011; fig_TX = 1; break;
                case '+': baudotTX1 = B10001; fig_TX = 1; break;
                case '?': baudotTX1 = B11001; fig_TX = 1; break;
                case ':': baudotTX1 = B01110; fig_TX = 1; break;
                case '(': baudotTX1 = B01111; fig_TX = 1; break;
                case ')': baudotTX1 = B10010; fig_TX = 1; break;
                case '.': baudotTX1 = B11100; fig_TX = 1; break;
                case ',': baudotTX1 = B01100; fig_TX = 1; break;
                case '/': baudotTX1 = B11101; fig_TX = 1; break;
                case '\r':
                        baudotTX1 = B01000; fig_TX = 0; //CR
                        crLf = 1;
                        break;
                case '\n':
                        baudotTX1 = B01000; fig_TX = 0; //LF
                        crLf = 1;
                        break;
                case ' ':
                        baudotTX1 = B00100; fig_TX = 0; //SPACE
                        //space = 1;
                        break;
                case 192: baudotTX1 = B00011; fig_TX = 2; break; // А
                case 193: baudotTX1 = B11001; fig_TX = 2; break; // Б
                case 214: baudotTX1 = B01110; fig_TX = 2; break; // Ц
                case 196: baudotTX1 = B01001; fig_TX = 2; break; // Д
                case 197: baudotTX1 = B00001; fig_TX = 2; break; // Е
                case 212: baudotTX1 = B01101; fig_TX = 2; break; // Ф
                case 195: baudotTX1 = B11010; fig_TX = 2; break; // Г
                case 213: baudotTX1 = B10100; fig_TX = 2; break; // Х
                case 200: baudotTX1 = B00110; fig_TX = 2; break; // И
                case 201: baudotTX1 = B01011; fig_TX = 2; break; // Й
                case 202: baudotTX1 = B01111; fig_TX = 2; break; // К
                case 203: baudotTX1 = B10010; fig_TX = 2; break; // Л
                case 204: baudotTX1 = B11100; fig_TX = 2; break; // М
                case 205: baudotTX1 = B01100; fig_TX = 2; break; // Н
                case 206: baudotTX1 = B11000; fig_TX = 2; break; // О
                case 207: baudotTX1 = B10110; fig_TX = 2; break; // П
                case 223: baudotTX1 = B10111; fig_TX = 2; break; // Я
                case 208: baudotTX1 = B01010; fig_TX = 2; break; // Р
                case 209: baudotTX1 = B00101; fig_TX = 2; break; // С
                case 210: baudotTX1 = B10000; fig_TX = 2; break; // Т
                case 211: baudotTX1 = B00111; fig_TX = 2; break; // У
                case 198: baudotTX1 = B11110; fig_TX = 2; break; // Ж
                case 194: baudotTX1 = B10011; fig_TX = 2; break; // В
                case 220: baudotTX1 = B11101; fig_TX = 2; break; // Ь
                case 219: baudotTX1 = B10101; fig_TX = 2; break; // Ы
                case 199: baudotTX1 = B10001; fig_TX = 2; break; // З
                case 221: baudotTX1 = B01101; fig_TX = 1; break; // Э
                case 216: baudotTX1 = B11010; fig_TX = 1; break; // Ш
                case 217: baudotTX1 = B10100; fig_TX = 1; break; // Щ
                case 222: baudotTX1 = B01011; fig_TX = 1; break; // Ю
                case 215: baudotTX1 = B01010; fig_TX = 1; break; // Ч
                default:
                        break;
        }
}







void decodeBodo(){
     if(dsp == 1)   // если символ принят
        {
               if(eofRx == 1){                    // если до этого была окончена передача, то 
                       Serial.println("");        // начинаем с новой строки
                       lcd.clear();
                       lcd.setCursor(0, 0);
                       LCD_Print(String(LINE));
                       LINE = "";
                       lcd.setCursor(0, 1);
                       eofRx = 0;

                       if (RTC.read(tm)) {                // метка времени
                         LINE = String(tmYearToCalendar(tm.Year))+'/';
                         if (tm.Month  <10) {LINE += '0'+String(tm.Month)+'/';} else {LINE += String(tm.Month)+'/';}
                         if (tm.Day    <10) {LINE += '0'+String(tm.Day)+' ';}   else {LINE += String(tm.Day)+' ';}
                         if (tm.Hour   <10) {LINE += '0'+String(tm.Hour)+':';}  else {LINE += String(tm.Hour)+':';}
                         if (tm.Minute <10) {LINE += '0'+String(tm.Minute)+':';}else {LINE += String(tm.Minute)+':';}
                         if (tm.Second <10) {LINE += '0'+String(tm.Second);}    else {LINE += String(tm.Second);}
                        LINE += '>';
                        Serial.print(LINE);
                        RCV_MESSAGE = LINE;               // метка времени
                        LINE = "";
                       }
               }
                ch = '\0';
                     if(baudotRX == B11111){fig = 0; }                  // буквы латинские
                else if(baudotRX == B11011){fig = 1; }                  // цифры/символы
                else if(baudotRX == B00000){fig = 2; }                  // буквы русские
                 
                 else {
                if(baudotRX == B00010){}                             // LF
                if(baudotRX == B01000){}                             // CR
                if(baudotRX_prv == B01000 && baudotRX == B00010){    // CR+LF
                  eofRx = 1;                    // конец принятого сообщения
                  RCVInProgress = 0;
                  //eepromWrite(RCV_MESSAGE);     // сохранить принятую строку
                  RCV_MESSAGE = "";
                  }
                if(baudotRX == B00100){ch = ' ';}                  // пробел
                if(fig == 0)
                {
                             if(baudotRX == B00011){ch = 'A';}
                        else if(baudotRX == B11001){ch = 'B';}
                        else if(baudotRX == B01110){ch = 'C';}
                        else if(baudotRX == B01001){ch = 'D';}
                        else if(baudotRX == B00001){ch = 'E';}
                        else if(baudotRX == B01101){ch = 'F';}
                        else if(baudotRX == B11010){ch = 'G';}
                        else if(baudotRX == B10100){ch = 'H';}
                        else if(baudotRX == B00110){ch = 'I';}
                        else if(baudotRX == B01011){ch = 'J';}
                        else if(baudotRX == B01111){ch = 'K';}
                        else if(baudotRX == B10010){ch = 'L';}
                        else if(baudotRX == B11100){ch = 'M';}
                        else if(baudotRX == B01100){ch = 'N';}
                        else if(baudotRX == B11000){ch = 'O';}
                        else if(baudotRX == B10110){ch = 'P';}
                        else if(baudotRX == B10111){ch = 'Q';}
                        else if(baudotRX == B01010){ch = 'R';}
                        else if(baudotRX == B00101){ch = 'S';}
                        else if(baudotRX == B10000){ch = 'T';}
                        else if(baudotRX == B00111){ch = 'U';}
                        else if(baudotRX == B11110){ch = 'V';}
                        else if(baudotRX == B10011){ch = 'W';}
                        else if(baudotRX == B11101){ch = 'X';}
                        else if(baudotRX == B10101){ch = 'Y';}
                        else if(baudotRX == B10001){ch = 'Z';}
                }
                if(fig == 1)
                {
                             if(baudotRX == B00011){ch = '-';}
                        else if(baudotRX == B11001){ch = '?';}
                        else if(baudotRX == B01110){ch = ':';}
                        else if(baudotRX == B01001){         } // кто там?
                        else if(baudotRX == B00001){ch = '3';}
                        else if(baudotRX == B01101){ch = 'Э';}
                        else if(baudotRX == B11010){ch = 'Ш';}
                        else if(baudotRX == B10100){ch = 'Щ';}
                        else if(baudotRX == B00110){ch = '8';}
                        else if(baudotRX == B01011){ch = 'Ю';} // или звонок
                        else if(baudotRX == B01111){ch = '(';}
                        else if(baudotRX == B10010){ch = ')';}
                        else if(baudotRX == B11100){ch = '.';}
                        else if(baudotRX == B01100){ch = ',';}
                        else if(baudotRX == B11000){ch = '9';}
                        else if(baudotRX == B10110){ch = '0';}
                        else if(baudotRX == B10111){ch = '1';}
                        else if(baudotRX == B01010){ch = '4';}
                        else if(baudotRX == B00101){ch = '"';}
                        else if(baudotRX == B10000){ch = '5';}
                        else if(baudotRX == B00111){ch = '7';}
                        else if(baudotRX == B11110){ch = '=';}
                        else if(baudotRX == B10011){ch = '2';}
                        else if(baudotRX == B11101){ch = '/';}
                        else if(baudotRX == B10101){ch = '6';}
                        else if(baudotRX == B10001){ch = '+';}
                }
                if(fig == 2)
                {
                             if(baudotRX == B00011){ch = 'А';}
                        else if(baudotRX == B11001){ch = 'Б';}
                        else if(baudotRX == B01110){ch = 'Ц';}
                        else if(baudotRX == B01001){ch = 'Д';}
                        else if(baudotRX == B00001){ch = 'Е';}
                        else if(baudotRX == B01101){ch = 'Ф';}
                        else if(baudotRX == B11010){ch = 'Г';}
                        else if(baudotRX == B10100){ch = 'Х';}
                        else if(baudotRX == B00110){ch = 'И';}
                        else if(baudotRX == B01011){ch = 'Й';}
                        else if(baudotRX == B01111){ch = 'К';}
                        else if(baudotRX == B10010){ch = 'Л';}
                        else if(baudotRX == B11100){ch = 'М';}
                        else if(baudotRX == B01100){ch = 'Н';}
                        else if(baudotRX == B11000){ch = 'О';}
                        else if(baudotRX == B10110){ch = 'П';}
                        else if(baudotRX == B10111){ch = 'Я';}
                        else if(baudotRX == B01010){ch = 'Р';}
                        else if(baudotRX == B00101){ch = 'С';}
                        else if(baudotRX == B10000){ch = 'Т';}
                        else if(baudotRX == B00111){ch = 'У';}
                        else if(baudotRX == B11110){ch = 'Ж';}
                        else if(baudotRX == B10011){ch = 'В';}
                        else if(baudotRX == B11101){ch = 'Ь';}
                        else if(baudotRX == B10101){ch = 'Ы';}
                        else if(baudotRX == B10001){ch = 'З';}
                }
                 if (baudotRX != 0 && baudotRX != B00010 && baudotRX != B01000) {
                 if (ch >= -112 && ch <= -81) {Serial.write(208); Serial.write(ch);}  // обратная перекодировка из Win-1251 в Unicode
                 else {Serial.print(ch);}
                  LINE += ch;                          // добавить символ к строке
                  RCV_MESSAGE += ch;
                 LCD_Print(String(ch));
                 }
    if (LINE.length() == 16) {                         // скроллинг дисплея вверх
      lcd.clear();
      lcd.setCursor(0, 0);
      LCD_Print(String(LINE));
      LINE = "";
      lcd.setCursor(0, 1);
      }
                }
        dsp = 0;
        baudotRX_prv = baudotRX;
        RCVTimeOut = millis();
        }
}



void print2digits(int number) {
  if (number >= 0 && number < 10) {
    lcd.print("0");
  }
  lcd.print(String(number));
}


void eepromWrite(String StrToWrite) {
 // записать в EEPROM 24С32
  // найти EOF
  word address = 0;
   for (word address = 0; address < EEPROM_BUF; address++){
     char data = eeprom.readByte(address);
     if (data == 0x04) {   // ищем конец записи
      EOF_address = address;
      address = EEPROM_BUF + 1;}
     else {EOF_address = 0;}
   }
    for (word i = 0; i < StrToWrite.length(); i++)
    {
      address = EOF_address + i;
      if (address > EEPROM_BUF) {EOF_address = 0;}
        eeprom.writeByte(address, StrToWrite[i]);
        address++;
    }
  eeprom.writeByte(address, 0x03);               // вписать символ конца текста
  address++;
  eeprom.writeByte(address, 0x04);               // вписать символ конца передачи
}

void eepromRead() {
 // читать EEPROM 24С32
  // найти EOF
  word address = 0;
   for (word address = 0; address < EEPROM_BUF; address++){
     char data = eeprom.readByte(address);
     if (data == 0x04) {   // ищем конец записи
      EOF_address = address;
      address = EEPROM_BUF + 1;}
     else {EOF_address = 0;}
   }
EOF_address++;

   for (word i = 0; i < EEPROM_BUF; i++){      // вывести старые записи
    address = EOF_address + i;
    if (address > EEPROM_BUF) {EOF_address = 0; i = 0; address = EOF_address + i; Serial.println(EOF_address);}
     char data = eeprom.readByte(address);
     if (data == 0x04) {   // ищем конец записи
      i = EEPROM_BUF + 1;}
     if (data == 0x03) {                          // если нашли конец записи
      Serial.println(LINE);
      LINE = "";
      }
     if (data != 0xFF && data != 0x04 && data != 0x03) {LINE += data;}
   }
 Serial.print(LINE);
 
   for (word address = 0; address < EEPROM_BUF; address++){      // вывести новые записи
     char data = eeprom.readByte(address);
     if (data == 0x04) {   // ищем конец записи
      address = EEPROM_BUF + 1;}
     if (data == 0x03) {                          // если нашли конец записи
      Serial.println(LINE);
      LINE = "";
      }
     if (data != 0xFF && data != 0x04 && data != 0x03) {LINE += data;}
   }
 Serial.println(LINE);
}

void AFSKgenerator(bool mark) {
 //генератор тона AFSK
   switch (mark) {
     case SPACE:
       ISRTimer1(FREQSPACE);  // запустить таймер1 с частотой FREQSPACE Гц
       toneOn = 1;            // разрешить генерацию
     break;
     case MARK:
       ISRTimer1(FREQMARK);   // запустить таймер1 с частотой FREQMARK Гц
       toneOn = 1;            // разрешить генерацию
     break;
   }
}

void ISRTimer1 (float FrToneHz){    //  FrToneHz - частота, Гц
  // генератор прерываний
  if (FrToneHz == 0) {              // если FrToneHz = 0, то остановить таймер
   cli();
   TCCR1A = 0;
   TCCR1B = 0;
   sei();
   digitalWrite(AUDI_PIN_OUT, LOW);
   toneState = 0;
  }
  if (FrToneHz >= 150 && FrToneHz <= 3500) {      // если частота в границах 150...3500 Гц
      cli();                                      // отключить глобальные прерывания
      TCCR1A = 0;
      TCCR1B = 0;
      TCCR1B |= (1 << WGM12);                     // включить CTC режим > сброс таймера по совпадению
      TCCR1B |= (1 << CS10);                      // установить коэффициент деления = 1
      OCR1A = round(16000000/(2*FrToneHz)) - 1;   // вычислить значение счётчика
      TIMSK1 |= (1 << OCIE1A);                    // включить прерывание по совпадению таймера 
      sei();                                      // включить глобальные прерывания
  }
}

void Bod_speed(float Bodespeed) {    // Bodespeed - скорость передачи данных, бод
 // установка скорости протокола
  if (Bodespeed >= 3 && Bodespeed <= 600) {
    uint32_t Period_mks = round(45455/Bodespeed);  // вычислить период, мкс
    ISRTimer2(Period_mks);                         // настроить таймер2
    Serial.print("Длина тона "); Serial.print(22*Period_mks/1000); Serial.println(" мс");
    Serial.print("Период тиков "); Serial.print(Period_mks); Serial.println(" мкс");
    Serial.print("Скорость протокола "); Serial.print(Bodespeed); Serial.println(" Бод");
    LCD_Print(LINE);
    LINE = String(Bodespeed)+" бод";
    LCD_Print(LINE);
    rSq = 0;
    ti = 0;
    snd = 0;
    baudotSpeed = Bodespeed;}
}

void resetBod_speed() {
  // переключатель скоростей протокола
  if (digitalRead(BODSPDPIN1) == HIGH && digitalRead(BODSPDPIN2) == HIGH && digitalRead(BODSPDPIN3) == HIGH && digitalRead(BODSPDPIN4) == HIGH && baudotSpeed!= BODESPEED) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    Bod_speed(BODESPEED);
    lcd.setCursor(0, 1);
    }
  if (digitalRead(BODSPDPIN1) == LOW && baudotSpeed!= BODESPEED1) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    Bod_speed(BODESPEED1);
    lcd.setCursor(0, 1);
    }
  if (digitalRead(BODSPDPIN2) == LOW && baudotSpeed!= BODESPEED2) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    Bod_speed(BODESPEED2);
    lcd.setCursor(0, 1);
    }
  if (digitalRead(BODSPDPIN3) == LOW && baudotSpeed!= BODESPEED3) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    Bod_speed(BODESPEED3);
    lcd.setCursor(0, 1);
    }
  if (digitalRead(BODSPDPIN4) == LOW && baudotSpeed!= BODESPEED4) {
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    Bod_speed(BODESPEED4);
    lcd.setCursor(0, 1);
    }
}

void ISRTimer2 (uint32_t Period_mks){    //  Period_ms - период, мкс
  // генератор тиков
  if (Period_mks == 0) {                 // если Period_mks = 0, то остановить таймер2
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
   sei();}
  if (Period_mks >= 4050 && Period_mks <= 16200) {   // от 3 до 11 бод
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
     TCCR2B |= (1<<CS20); // делитель 1024: 16М/1024 = 64 мкс
     TCCR2B |= (1<<CS21);
     TCCR2B |= (1<<CS22);
     OCR2A = round(Period_mks/64) - 1;}
  else if (Period_mks >= 2000 && Period_mks <= 4050) {   // от 11 до 22 бод
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
     TCCR2B |= (1<<CS21); // делитель 256: 16М/256 = 16 мкс
     TCCR2B |= (1<<CS22);
     OCR2A = round(Period_mks/16) - 1;}
  else if (Period_mks >= 1000 && Period_mks <= 2000) {   // от 22 до 45 бод
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
     TCCR2B |= (1<<CS20); // делитель 128: 16М/128 = 8 мкс
     TCCR2B |= (1<<CS22);
     OCR2A = round(Period_mks/8) - 1;}
  else if (Period_mks >= 500 && Period_mks <= 1000) {   // от 45 до 90 бод
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
     TCCR2B |= (1<<CS22); // делитель 64: 16М/64 = 4 мкс
     OCR2A = round(Period_mks/4) - 1;}
  else if (Period_mks >= 125 && Period_mks <= 500) {   // от 90 до 360 бод
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
     TCCR2B |= (1<<CS20); // делитель 32: 16М/32 = 2 мкс
     TCCR2B |= (1<<CS21);
     OCR2A = round(Period_mks/2) - 1;}
  else if (Period_mks >= 35 && Period_mks <= 125) {   // от 360 до 1200 бод
   cli();
   TCCR2A = 0;
   TCCR2B = 0;
     TCCR2B |= (1<<CS21); // делитель 8: 16М/8 = 0,5 мкс
     OCR2A = round(Period_mks*2) - 1;}
   TCCR2A |= (1<<WGM21);  // в регистре TCCR2B установить бит WGM21 (режим CTC)
   TIMSK2 |= (1<<OCIE2A); // в регистре TIMSK2 установить бит OCIE2A
   sei();
}

void LCD_Print (String lcdstr) {
  // печасть русских букв на LCD
  int letterin;
  String letterout;
  for (uint8_t i = 0; i < lcdstr.length(); i++){
      letterin = lcdstr[i];
      if (letterin != -48 && letterin != -47) {   // если появился втрой байт - пропустить
        if (letterin <= -113 && letterin >= -128) {letterin = letterin + 32;}     // строчные рус -> в прописные
        if (letterin <= -65 && letterin >= -80)   {letterin = letterin - 32;}
        if (letterin >= 97 && letterin <= 122)    {letterin = letterin - 32;}     // строчные лат -> в прописные
        if (letterin >= 32 && letterin <= 127)    {letterout = char(letterin);}   // латинский шрифт
           if(letterin == 32)  {letterout = "\40";}   // пробел
      else if(letterin == -112){letterout = "\300";}  // А
      else if(letterin == -111){letterout = "\301";}  // Б
      else if(letterin == -110){letterout = "\302";}  // В
      else if(letterin == -109){letterout = "\303";}  // Г
      else if(letterin == -108){letterout = "\304";}  // Д
      else if(letterin == -107){letterout = "\305";}  // Е
      else if(letterin == -106){letterout = "\306";}  // Ж
      else if(letterin == -105){letterout = "\307";}  // З
      else if(letterin == -104){letterout = "\310";}  // И
      else if(letterin == -103){letterout = "\311";}  // Й
      else if(letterin == -102){letterout = "\312";}  // К
      else if(letterin == -101){letterout = "\313";}  // Л
      else if(letterin == -100){letterout = "\314";}  // М
      else if(letterin == -99) {letterout = "\315";}  // Н
      else if(letterin == -98) {letterout = "\316";}  // О
      else if(letterin == -97) {letterout = "\317";}  // П
      else if(letterin == -96) {letterout = "\320";}  // Р
      else if(letterin == -95) {letterout = "\321";}  // С
      else if(letterin == -94) {letterout = "\322";}  // Т
      else if(letterin == -93) {letterout = "\323";}  // У
      else if(letterin == -92) {letterout = "\324";}  // Ф
      else if(letterin == -91) {letterout = "\325";}  // Х
      else if(letterin == -90) {letterout = "\326";}  // Ц
      else if(letterin == -89) {letterout = "\327";}  // Ч
      else if(letterin == -88) {letterout = "\330";}  // Ш
      else if(letterin == -87) {letterout = "\331";}  // Щ
      else if(letterin == -86) {letterout = "\332";}  // Ъ
      else if(letterin == -85) {letterout = "\333";}  // Ы
      else if(letterin == -84) {letterout = "\334";}  // Ь
      else if(letterin == -83) {letterout = "\335";}  // Э
      else if(letterin == -82) {letterout = "\336";}  // Ю
      else if(letterin == -81) {letterout = "\337";}  // Я
      lcd.print(letterout);
      }
      letterout = "";
    }
  }
