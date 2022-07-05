#include <OneWire.h>
#include <DS18B20.h>

#define Switch 2            //(innterupt)
#define ImpFan 3            //(interrupt)

#define PWMFan 5            //Sygnał PWM dla wentylatora
#define PWM 6               //Sygnał PWM dla zasilacza
#define Temp 7              //Temperatura: 2 czujniki DS18B20
#define SensorsNum 2        //Ilość czujników

#define SHCPR 8             //Wejście sygnału zegarowego dla rejestru
#define STCPR 9             //Wejście sygnału zegarowego dla rejestru przechowującego
#define DSR 10              //Wejście danych szeregowych

#define Working 11          //Zasilanie podane z baterii   --> TRYB PRACY
#define Charging 12         //Zasilanie podane z ładowarki --> TRYB ŁADOWANIA

#define CurrentCharging A1  //Prąd ładowania baterii
#define CurrentMotor A2     //Prąd pobierany przez silnik główny
#define CurrentBattery A6   //Prąd pobieramy z baterii
#define AkuVoltage A7      //Napięcie na baterii

#define SINT A3            //Linia sygnałowa generująca przerwanie
//#define SCL A4              //Serial Clock
//#define SDA A5              //Serial Data

OneWire onewire(Temp);      //Deklaracja obiektu odpowiedzialnego za komunikację za pomocą 1-Wire. Argument konstruktora to nr pinu do którego jest podłączony czujnik
DS18B20 sensors(&onewire);  //Deklaracja obiektu odpowiedzialnego za obsługę czujników DS18B20. Argument konstruktora to wskaźnik do obiektu magistrali 1_Wire

//Adresy czujników
const byte address[SensorsNum][8] PROGMEM = {
  0x28, 0xEE, 0x16, 0xEE, 0x12, 0x16, 0x1, 0x85,  //Czujunik na baterii
  0x28, 0xA7, 0xFC, 0x5C, 0x5, 0x0, 0x0, 0xBF     //Czujnik na radiatorze
};

int ReadAnalogValue = 0;               //Zmienna do odczytu wartości z ADC
volatile double VoltAku = 0;           //Zmienna do przechowania wartości napiecia na baterii
double AverangeVoltAku = 0;            //ZMienna do przechowania wartosci sredniej napiecia na baterii
volatile double CurMotor = 0;          //Zmienna do przechowania wartosci prądu pobieranego przez silnik
volatile double CurCharging = 0;       //Zmienna do przechowania wartosci prądu ładowania
double AverangeCurCharging = 0;        //Zmienna do przechowania wartosci sredniej prądu ładowania
double LastCurCharging = 0;            //Zmienna do przechowania ostatniej zmierzonej wartosci prądu ładowania
volatile double CurBattery = 0;        //Zmienna do przechowania wartosci prądu pobieranego z baterii

const double MotorResistor = 0.1;      //Stała rezystora bocznikowego dla silnika
const double BatteryResistor = 0.1;    //Stała rezystora bocznikowego dla baterii

unsigned long TimeMultiplex = 0;
unsigned long pomoc = 0;
int NrCol = 0;
int StatusCharging = 0; //LED gorna, 0 - wylaczona, 1 - zielona, 2 - czerwona, 3 - pomaranczowa
boolean Register[8];  // Tablica wartosci dla 74595

int PWMPowerSuply = 0;                //Wartosc sygnału PWM (0 - 255) dla zasilacza łądowarki
double LastVoltAku = 0;
double VoltagePowerSuply = 0;
double MaxVoltageChrging = 12.6;      //Maksymalne napięcie ładowania 12.6
double ChargingCurrent = 0;           //Zmienna dla koncowego określenia prądu ładowania
double MaxChargingCurrent = 1.50;     //Maksymalny prąd ładowania (ako w dobrym stanie)
double MinChargingCurrent = 0.40;     //Minimalny prąd ładowania (ako w głębokim rozładowaniu)
double ErrorPercentCharging = 0;      //Procentowa wartość błędu prądu ładowania
double ErrorPercent = 0;              //
double PomocE1 = 0;                   //
double PomocE2 = 0;                   //

int PWMFanValue = 0;                  //Wartosc sygnału PWM (0 - 255) dla wentylatora         
int NormalFanPower = 70;              //Zmienna określająca normalną wartość PWM dla wiatraka
float TempBattery = 0;                //Zmienna dla temperatury baterii
float MaxTempBattery = 40;            //Zmienna dla maksymalnej wartości temperatury baterii
float TempRadiator = 0;               //Zmienna dla temperatury radiatora
float MaxTempRadiator = 50;           //Zmienna dla maksymalnej wartości temperatury radiatora

//Nastwy Kp dla TempSet = 35`
//Kp = MaxFanPWM(255) / (TempSet * Blad 5%) 

float TempSet = 30.0;                 //Zmienna dla temperatury zadanej
float Kp = 30;                      //Wzmocnienie członu proporcjonalnego
float Ki = 10;
float Kd = 0.04;

float P = 0;
float I = 0;
float D = 0;

float ErrorP = 0;                     //Błąd regulacji
float LastError = 0;
float ErrorSumI = 0;
float ErrorD = 0;

int StatusWork = 0;                   //

void setup() {
  pinMode(Switch, INPUT_PULLUP);
  pinMode(ImpFan, INPUT);
    digitalWrite(ImpFan, HIGH);

  pinMode(PWMFan, OUTPUT);
    analogWrite(PWMFan, PWMFanValue);  
  pinMode(PWM, OUTPUT);
    analogWrite(PWM, PWMPowerSuply);

  pinMode(SHCPR, OUTPUT);
  pinMode(STCPR, OUTPUT);
  pinMode(DSR, OUTPUT);

  TimeMultiplex = millis() + 1;
  pomoc = millis() + 4000;

  attachInterrupt(digitalPinToInterrupt(Switch), SwitchInt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ImpFan), ImpFanInt, FALLING);
  
Serial.begin(9600);

sensors.begin();    //Wyszukiwanie czujników i ustawienie rozdzielczości. Argumentem funkcji jest rozdzielczość czujnika (9, 10, 11, 12(dowyślnie))
sensors.request();  //Wysłanie rozkazu pomiaru temp. Argumentem funkcji jest adres czujnika (pusty nawias: rozkaz dla wszystkich)
ReadAkuVoltage();

Serial.println("ROBOT MOBILNY - Zasilacz!"); Serial.println(" ");

delay(1000);
/*
for(int i = 1; i < 11; i++)
{
  ReadAkuVoltage();
  Serial.print("Pomiar "); Serial.print(i); Serial.print(" : "); Serial.println(VoltAku);
  delay(500);
}

delay(2000);
Serial.println(" ");
Serial.println("Start!");
*/
}

void loop() {

if(digitalRead(Working) == HIGH) { StatusWork = 1; Serial.println("Status: PRACA!"); Serial.println(" "); }    //Jesli auto jest urzywane
if(digitalRead(Charging) == HIGH) { StatusWork = 2; Serial.println("Status: LADOWANIE!"); Serial.println(" "); }   //Jesli auto jest ładowane


/*
  PWMPowerSuply++;    //
  Serial.print("Wartosc PWM: "); Serial.println(PWMPowerSuply);
  analogWrite(PWM, PWMPowerSuply);
  delay(1000);
  ReadAkuVoltage();
  Serial.print("Napiecie: "); Serial.println(VoltAku);

  while(PWMPowerSuply == 255){  
  }
*/  

  if(StatusWork == 1)   //Praca robota
  {
    delay(1000);
    ReadAkuVoltage();
    if(VoltAku < 9.6) {   //Bateira rozładowana
      ColorPointBarGraph(1);  //Dioda czerwona na bargrafie
      delay(300);
      ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
      //delay(150);
    }   
  }

  if(StatusWork == 2)   //Ładowanie baterii
  {
    ReadAkuVoltage();       //Mierzę napięcie
    DisplayPointBarGraph(VoltAku);
    
    if(VoltAku == 0)  { //Warunek jeśli nie ma podłączonej baterii to migaj czerwoną diodą
      analogWrite(PWMFan, 0); 
      analogWrite(PWM, 0);
      Serial.println("Brak baterii!"); Serial.println(" ");
      while(VoltAku == 0)  {
        ColorPointBarGraph(1);  //Dioda czerwona na bargrafie
        delay(150);
        ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
        delay(150);
        ReadAkuVoltage();       //Mierzę napięcie
      }
    }

    if((VoltAku > 0) && (VoltAku <= 12.40))  {
      Serial.println("Start ladowania!"); Serial.println(" "); Serial.println("Ladowanie stalopradowe!"); Serial.println(" ");
      
      ReadAkuVoltage();       //Mierzę napięcie
      AverangeVoltAku = VoltAku;
      ReadCurrentCharging();  //Mierzę prąd ładowania
      AverangeCurCharging = CurCharging;
      
      PWMPowerSuply = -0.0518 * pow(VoltAku, 2) + 19.747 * VoltAku + 5.8881;  //Obliczam wartość PWM tak aby otrzymać napięcie na zasilaczu podobne do napięcia na baterii
      analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie
      //Serial.print("Zmierzone napiecie: "); Serial.println(VoltAku); Serial.print("Obliczone napiecie: "); Serial.println(PWMPowerSuply);
     
      while(1)  {   //Pętla while dla ładowania stałym prądem
        if((VoltAku <= 9.6)   && (VoltAku > 0))   {  ChargingCurrent = MinChargingCurrent; ErrorPercentCharging = 0.03; }   //Określam minimalny prąd ładowania dla ogniw głęboko rozładowanych
        if((VoltAku <= 12.40) && (VoltAku > 9.6)) {  ChargingCurrent = MaxChargingCurrent; ErrorPercentCharging = 0.01; }   //Określam maksymalny prąd ładowania dla ogniw w stanie normalnym
        ErrorPercent = ErrorPercentCharging * ChargingCurrent;
        
        ReadCurrentCharging();  //Mierzę prąd ładowania
        ReadAkuVoltage();       //Mierzę napięcie
        DisplayPointBarGraph(VoltAku);

        AverangeVoltAku = (AverangeVoltAku + VoltAku) / 2;
        AverangeCurCharging = (AverangeCurCharging + CurCharging) / 2;
        
        //PomocE1 = ChargingCurrent - ErrorPercent;
        //PomocE2 = ChargingCurrent + ErrorPercent;
                        
        //Serial.print("Error: "); Serial.println(ErrorPercent);
        //Serial.print("Zmierzone napiecie: "); Serial.println(VoltAku);
        //Serial.print("Srednie zmierzone napiecie: "); Serial.println(AverangeVoltAku);
        //Serial.print("Zmierzony prad: "); Serial.println(CurCharging);
        //Serial.print("Sredni zmierzony prad: "); Serial.println(AverangeCurCharging);
        //Serial.print("Ostatni zmierzony prad: "); Serial.println(LastCurCharging);
        //Serial.print("PomocE1 -: "); Serial.print(PomocE1); Serial.print(" : "); Serial.println(CurCharging - ErrorPercent);
        //Serial.print("PomocE2 +: "); Serial.print(PomocE2); Serial.print(" : "); Serial.println(CurCharging + ErrorPercent);
        
        if(AverangeVoltAku > MaxVoltageChrging) { break; } //Sprawdzam czy bateria osiągneła napięcie właściwe do przejscia w końcową fazę ładowania

        if((CurCharging < LastCurCharging) && (CurCharging <= 0.001)) {   //Sprawdzam czy ktos nie odlaczył baterii żeby uC nie zwiekszał napięcia z braku prądu
          Serial.println("Nie oczekiwany spadek pradu do zera!");
          while(CurCharging <= 0.001)  {                
            ColorPointBarGraph(1);  //Dioda czerwona na bargrafie
            delay(150);
            ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
            delay(150);
            ReadCurrentCharging();  //Mierzę prąd ładowania
            FanControl();
          }
        }

        if((VoltAku > 0.001) && (CurCharging <= 0.01)) {
          LastVoltAku = VoltAku;
          while(1) {
            ++PWMPowerSuply;
            analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie         
            ColorPointBarGraph(2);  //Dioda pomarańczowa na bargrafie
            delay(150);
            ColorPointBarGraph(0);
            delay(150);
            ReadCurrentCharging();  //Mierzę prąd ładowania
            ReadAkuVoltage();       //Mierzę napięcie
            if(CurCharging > 0.01) { break; }
              if(VoltAku > (LastVoltAku + 5))  {
                Serial.println("Podlacz obciazenie!");
                while(CurCharging <= 0.01) {
                  ColorPointBarGraph(1);  //Dioda czerwona na bargrafie
                  delay(150);
                  ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
                  delay(150); 
                  ReadCurrentCharging();  //Mierzę prąd ładowania
                  FanControl();
                }
              }
            FanControl();

            //Serial.println("++ PWM (sprawdzam)"); Serial.println(" ");
            Serial.print("PWM Zasilacz       (sprawdzam): "); Serial.println(PWMPowerSuply);
            Serial.print("Zmierzone napiecie (sprawdzam): "); Serial.println(VoltAku);
            Serial.print("Zmierzony prad     (sprawdzam): "); Serial.println(CurCharging);
          }
        }
                
        if(CurCharging < (ChargingCurrent - ErrorPercent)) {   //Jesli prąd ładowania jest mniejszy od założonego to zwiększ napięcie i tym samym przepłym prądu
          ++PWMPowerSuply;
          analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie
          //Serial.println("++ PWM"); Serial.println(" ");
        }
        if((CurCharging >= (ChargingCurrent - ErrorPercent)) && (CurCharging <= (ChargingCurrent + ErrorPercent))) {   //Jeśli prąd jest w "widełkach" to 
          //Serial.println("== PWM"); Serial.println(" ");
          //delay(500);
        }
        if(CurCharging > (ChargingCurrent + ErrorPercent)) {   //Jesli prąd ładowania jest większy od założonego to zmniejsz napięcie i tym samym przepłym prądu
          --PWMPowerSuply;
          analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie
          //Serial.println("-- PWM"); Serial.println(" ");
        }
       
        LastCurCharging = CurCharging;

        FanControl();
        SendDataUART();
        
        delay(500);   //zeby za szybko nie leciał
      }

      Serial.println(" "); Serial.println("Ladowanie stalonapieciowe!"); Serial.println(" ");
      
      PWMPowerSuply = -0.0518 * pow(MaxVoltageChrging, 2) + 19.747 * MaxVoltageChrging + 5.8881;  //Obliczam wartość PWM tak aby otrzymać napięcie na zasilaczu podobne do napięcia 12.6V
      //Serial.print("Obliczone napiecie: "); Serial.println(PWMPowerSuply);
      analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie
      while(1)  {
        ReadAkuVoltage();       //Mierzę napięcie
        ReadCurrentCharging();  //Mierzę prąd ładowania 
        AverangeVoltAku = (AverangeVoltAku + VoltAku) / 2;
        AverangeCurCharging = (AverangeCurCharging + CurCharging) / 2;
        
        if(VoltAku < (MaxVoltageChrging - ErrorPercent)) {   //Jesli prąd ładowania jest mniejszy od założonego to zwiększ napięcie i tym samym przepłym prądu
          ++PWMPowerSuply;
          analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie
          //Serial.println("++ PWM nap"); Serial.println(" ");
        }
        if((VoltAku >= (MaxVoltageChrging - ErrorPercent)) && (VoltAku <= (MaxVoltageChrging + ErrorPercent))) {   //Jeśli prąd jest w "widełkach" to 
          //Serial.println("== PWM nap"); 
          //Serial.println(" ");
          //Serial.println("Stale napiecie: "); Serial.println(VoltAku); Serial.println(" ");
          //delay(500);
          break;
        }
        if(VoltAku > (MaxVoltageChrging + ErrorPercent)) {   //Jesli prąd ładowania jest większy od założonego to zmniejsz napięcie i tym samym przepłym prądu
          --PWMPowerSuply;
          analogWrite(PWM, PWMPowerSuply);  //Ustawiam napięcie
          //Serial.println("-- PWM nap"); Serial.println(" ");
        }
        //Serial.println(VoltAku);
        FanControl();
        SendDataUART();

        delay(500);   //zeby za szybko nie leciał
      }
      
      AverangeCurCharging = 0;
      
      while(1)  {   //Pętla while dla ładowania stałym napięciem
        ReadAkuVoltage();       //Mierzę napięcie
        ReadCurrentCharging();  //Mierzę prąd ładowania  
        AverangeVoltAku = (AverangeVoltAku + VoltAku) / 2;
        AverangeCurCharging = (AverangeCurCharging + CurCharging) / 2;
        //Serial.print("Zmierzony prad (stale nap): "); Serial.println(CurCharging);  
        //Serial.print("Sredni zmierzony prad (stale nap): "); Serial.println(AverangeCurCharging); 
            
        if(AverangeCurCharging < 0.33)  {   //0.33
          analogWrite(PWMFan, 0); 
          analogWrite(PWM, 0);
          Serial.println(" "); Serial.println("Ladowanie zakonczone!");

          while(1){
            ColorPointBarGraph(3);  //Dioda zielona na bargrafie
            delay(150);
            ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
            delay(150);
            FanControl();
          }
          //break;
        }

      FanControl();
      SendDataUART();
      delay(500);   //zeby za szybko nie leciał
      }
    }
     
    if(VoltAku > 12.40) {
      Serial.println("Bateria naladowana!");
      analogWrite(PWMFan, 0); 
      analogWrite(PWM, 0);

      while(1){
        ColorPointBarGraph(3);  //Dioda zielona na bargrafie
        delay(150);
        ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
        delay(150);
      }
    }   
  }
}

void SwitchInt()
{
  ReadAkuVoltage();   //Mierzę napięcie
  BarGraph(VoltAku);  //Wyswietlam linijkę diod
  ClearBarGraph();
}

void ImpFanInt()
{
  //Serial.println("Wiatrak");
}

void FanControl()
{
  if (sensors.available()) {
    TempBattery = ReadTemperature(0);      //Mierzę temperature baterii
    TempRadiator = ReadTemperature(1);      //Mierzę temperature radiatora
  }
  
  if(TempBattery > MaxTempBattery) {
    analogWrite(PWM, 0);  //Ustawiam napięcie
    Serial.println("Temperatura na baterii za wysoka! Nastepuje chlodzenie");
    while(TempBattery > 35)  {
      if (sensors.available()) { TempBattery = ReadTemperature(0); }     //Mierzę temperature baterii
        ColorPointBarGraph(1);  //Dioda czerwona na bargrafie
        delay(150);
        ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
        delay(150); 
    }
  }
  
  if(TempRadiator > MaxTempRadiator)  {
    analogWrite(PWMFan, 255);
    analogWrite(PWM, 10);  //Ustawiam napięcie
    Serial.println("Temperatura na radiatorze za wysoka! Nastepuje chlodzenie");
    while(TempRadiator > 35)  {
      if (sensors.available()) { TempRadiator = ReadTemperature(1); }     //Mierzę temperature radiatora
      ColorPointBarGraph(1);  //Dioda czerwona na bargrafie
      delay(150);
      ColorPointBarGraph(0);  //Dioda zgaszona na bargrafie
      delay(150); 
    }
  }

  //ErrorP = TempSet - TempRadiator;                //Wyznaczenie błedu regulacji
  ErrorP = TempRadiator - TempSet;                //Wyznaczenie błedu regulacji
  
  P = Kp * ErrorP;                                //Obliczenie członu proporcjonalnego
    if(P > 255)                            //Ograniczenie wart czlonu P
    {
      P = 255;
    }   
  ErrorSumI = (ErrorSumI + ErrorP) * 0.5;         //Wyznaczenie całki
    if(ErrorSumI > 200)                            //Ograniczenie żeby ErrorSumI nie poszedł za wysoko
    {
      ErrorSumI = 200;
    }
    if(ErrorSumI < -200)
    {
      ErrorSumI = -200;
    }
  I = Ki * ErrorSumI;                             //Obliczenie członu całkującego
  ErrorD = (ErrorP - LastError) / 0.5;            //Wyznaczenie różniczki (przyrost błędu)
  D = Kd * ErrorD;                                //Obliczenie członu różniczkującego
  PWMFanValue = P + I + D;                  //Obliczenie wartości PWM
                           //PWM = 100 dla err = 0
  if(PWMFanValue > 254) { PWMFanValue = 254; }
  if(PWMFanValue < 60)  { PWMFanValue = 0;   }
  
  analogWrite(PWMFan, PWMFanValue);

  LastError = ErrorP;
}

float ReadTemperature(int x)
{ //Odczyt temperatury
  float temperature;
    temperature = sensors.readTemperature(FA(address[x]));

    //Serial.print(F("#")); Serial.print(x); Serial.print(F(": ")); Serial.print(temperature); Serial.println(F(" 'C"));

    sensors.request();
  return temperature;
}

void ReadAkuVoltage()
{ //Pomiar napięcia na baterii
  ReadAnalogValue = analogRead(AkuVoltage);            //Odczyt wartości z ADC
  VoltAku = ReadAnalogValue * (5.0 / 1023.0);          //Przeliczenie na wolty
  VoltAku = (VoltAku / 0.23356);                       //Korekta storunku dzielnika napięcia
  //Serial.print("Napiecie aku przed: "); Serial.print(VoltAku); Serial.println(" [V]");
  if(VoltAku == 0) { VoltAku = 0; }
  else {
    VoltAku = VoltAku - (0.0007*pow(VoltAku,2)+ 0.0585*VoltAku - 0.0502); //Obliczam wartość napięcia z korektą (zalezność napiecia arduino do różnicy miernika i arduino)
  }
  //VoltAku = VoltAku - (0.000006*(PWMPowerSuply*PWMPowerSuply) + 0.003*PWMPowerSuply - 0.0619);
  //Serial.print("Napiecie aku: "); Serial.print(VoltAku); Serial.println(" [V]");
  //Serial.println(" ");
}

void ReadCurrentCharging()
{ //Pomiar prądu ładowania
  ReadAnalogValue = analogRead(CurrentCharging);       //Odczyt wartości z ADC
  //Serial.print("Odczyt z ADC: "); Serial.println(ReadAnalogValue);
  //Serial.print("5.0 / 1023.0 = "); Serial.println(5.0 / 1023.0);
  CurCharging = ReadAnalogValue * (5.0 / 1023.0);     //Przeliczenie na wolty
  //Serial.print("Napiecie na rezystorze: "); Serial.print(CurCharging); Serial.println(" [V]");
  //CurCharging = (CurCharging / BatteryResistor) / 10.0;
  CurCharging = CurCharging - (0.0058*pow(CurCharging,2)+ 0.2912*CurCharging - 0.0003);
  //Serial.print("Prad Ladowania: "); Serial.print(CurCharging); Serial.println(" [A]");
  //Serial.println(" ");
}

void ReadCurrentMotor()
{ //Pomiar prądu pobieranego przez silnik
  ReadAnalogValue = analogRead(CurrentMotor);          //Odczyt wartości z ADC
  CurMotor = ReadAnalogValue * (5.0 / 1023.0);        //Przeliczenie na wolty
  CurMotor = (CurMotor / MotorResistor) / 10;
  Serial.print("Prad silnika: "); Serial.print(CurMotor); Serial.println(" [A]");
}

void ReadCurrentBattery()
{ //Pomiar prądu pobieranej z baterii
  ReadAnalogValue = analogRead(CurrentBattery);        //Odczyt wartości z ADC
  CurBattery = ReadAnalogValue * (5.0 / 1023.0);      //Przeliczenie na wolty
  CurBattery = (CurBattery / BatteryResistor) / 10;
  Serial.print("Prad baterii: "); Serial.print(CurBattery); Serial.println(" [A]");
}

void BarGraph(double Volt)
{
  //Serial.println("BarGraph");
  pomoc = 0;  //
  while(pomoc < 2000) {
    //Serial.println("while");
    delay(1);
      TimeMultiplex = millis() + 1;
      ++NrCol;  // Zwiekszam nr kolumny ktora ma byc teraz wlaczona
      if(NrCol > 5) NrCol = 1;  // Zabezpieczenie 
      
      DisplayBarGraph(NrCol, (Volt*100));  // Wywolanie funkcji do wyswietlenia na bargraphie
      ++pomoc;
  }
}

void ColorPointBarGraph(int Color)
{ //Wyswietlenie punktu na bargrafie w wybranym kolorze
  if(Color == 0) {                      // Dioda LED nieswieci
    ClearBarGraph();
    DisplayBarGraph(5, 0);              // Wywolanie funkcji do wyswietlenia na bargraphie
  }
  if(Color == 1) {                      // Dioda LED swieci na czerwono
    ClearBarGraph();
    DisplayBarGraph(5, (10*100));       // Wywolanie funkcji do wyswietlenia na bargraphie
  }
  if(Color == 2) {                      // Dioda LED swieci na pomaranczowo
    ClearBarGraph();
    DisplayBarGraph(5, (11.40*100));    // Wywolanie funkcji do wyswietlenia na bargraphie
  }
  if(Color == 3) {                      // Dioda LED swieci na zielono
    ClearBarGraph();
    DisplayBarGraph(5, (11.90*100));    // Wywolanie funkcji do wyswietlenia na bargraphie
  }
}

int DisplayPointBarGraph(double Voltt )
{
  if  (Voltt == 0)                           { ColorPointBarGraph(0); }   //Dioda LED zgaszona
  if ((Voltt <= 10.35)  && (Voltt > 0))      { ColorPointBarGraph(1); }   //Dioda LED swieci na czerwono
  if ((Voltt > 10.35)   && (Voltt <= 11.48)) { ColorPointBarGraph(2); }   //Dioda LED swieci na pomaranczowo
  if  (Voltt > 11.48)                        { ColorPointBarGraph(3); }   //Dioda LED swieci na zielono
}

void ClearBarGraph(void)
{
  // algorytm ladowania tablicy do 74595 i wyswietlenie na bargraphie
  digitalWrite(STCPR, LOW);     //Wylaczenie wyjsci 74595
  for(int a = 0; a < 8; a++) {  // Wpisanie kazdego bitu tablicy do rejestru 74595
    digitalWrite(SHCPR, LOW);   // Sygnal zegara
    digitalWrite(DSR, 0);       // Dane
    digitalWrite(SHCPR, HIGH);  // Sygnal zegara 
  }  
  digitalWrite(STCPR, HIGH);    // Wlaczenie wyjsc 74595
}

void DisplayBarGraph(int Column, int Voltage)
{
  int Value = 0;

// Min Volt = 9,6V (0 LED), Max Volt = 12,6V (8 LED), 12,6-9,6 = 3V, 3/8 = 0,375V
// Wyznaczenie zmiennej Value od 0 do 8 zaleznie od napiecia na baterii  
  if (Voltage <= 960)                         { Value = 0; }
  if ((Voltage > 960)   && (Voltage <= 998))  { Value = 1; }
  if ((Voltage > 998)   && (Voltage <= 1035)) { Value = 2; }
  if ((Voltage > 1035)  && (Voltage <= 1073)) { Value = 3; } 
  if ((Voltage > 1073)  && (Voltage <= 1110)) { Value = 4; }
  if ((Voltage > 1110)  && (Voltage <= 1148)) { Value = 5; }
  if ((Voltage > 1148)  && (Voltage <= 1185)) { Value = 6; } 
  if ((Voltage > 1185)  && (Voltage <= 1223)) { Value = 7; }
  if (Voltage > 1223)                         { Value = 8; }      

  if  (Voltage == 0)                          { StatusCharging = 0; }   //Dioda LED zgaszona
  if ((Voltage <= 1035)  && (Voltage > 0))    { StatusCharging = 2; }   //Dioda LED swieci na czerwono
  if ((Voltage > 1035)  && (Voltage <= 1148)) { StatusCharging = 3; }   //Dioda LED swieci na pomaranczowo
  if  (Voltage > 1148)                        { StatusCharging = 1; }   //Dioda LED swieci na zielono

// Wypelnianie tablicy dla 74595 jakie dane ma pokazac bargrapf
  if (Column == 1) {  // Instrukcje dla kolumny nr 1 i 2
    Register[0] = HIGH; // Wpisanie do rejestru bit wlaczenia kolumny 1 i 2
    Register[4] = LOW; Register[3] = LOW; Register[2] = LOW; Register[1] = LOW; // Wpisanie do rejestru bitow wylaczenia kolumn 3 i 4, 5 i 6, 7 i 8, 9 i 10

    if (Value == 0) { // Wpisanie do rejestru bitow wl/wyl odpowiednie diody LED w linijce
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = HIGH; // Nie swieci się zadna dioda LED
    }
    if (Value == 1) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = LOW;  // Swieci się tylko pierwsza dioda LED
    }
    if (Value >= 2) {
      Register[7] = LOW; Register[6] = HIGH; Register[5] = LOW;   // Swiecą się dwie pierwsze diody LED 
    }
  }
  if (Column == 2) {  // Instrukcje dla kolumny nr 3 i 4
    Register[1] = HIGH; // Wpisanie do rejestru bit wlaczenia kolumny 3 i 4
    Register[4] = LOW; Register[3] = LOW; Register[2] = LOW; Register[0] = LOW; // Wpisanie do rejestru bitow wylaczenia kolumn 1 i 2, 5 i 6, 7 i 8, 9 i 10

    if (Value <= 2) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = HIGH;
    }
    if (Value == 3) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = LOW;
    }
    if (Value == 4) {
      Register[7] = LOW; Register[6] = HIGH; Register[5] = LOW;
    }
  }
  if (Column == 3) {  // Instrukcje dla kolumny nr 5 i 6
    Register[2] = HIGH; // Wpisanie do rejestru bit wlaczenia kolumny 5 i 6
    Register[4] = LOW; Register[3] = LOW; Register[1] = LOW; Register[0] = LOW; // Wpisanie do rejestru bitow wylaczenia kolumn 1, 2, 4, 5
    
    if (Value <= 4) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = HIGH;
    }
    if (Value == 5) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = LOW;
    }
    if (Value == 6) {
      Register[7] = LOW; Register[6] = HIGH; Register[5] = LOW;
    }
  }
  if (Column == 4) {  // Instrukcje dla kolumny nr 7 i 8
    Register[3] = HIGH; // Wpisanie do rejestru bit wlaczenia kolumny 7 i 8
    Register[4] = LOW; Register[2] = LOW; Register[1] = LOW; Register[0] = LOW; // Wpisanie do rejestru bitow wylaczenia kolumn 1, 2, 3, 5

    if (Value <= 6) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = HIGH;
    }
    if (Value == 7) {
      Register[7] = HIGH; Register[6] = HIGH; Register[5] = LOW;
    }
    if (Value == 8) {
      Register[7] = LOW; Register[6] = HIGH; Register[5] = LOW;
    }
  }  
  if (Column == 5) {  // Instrukcje dla kolumny nr 9 i 10
    Register[4] = HIGH; // Wpisanie do rejestru bit wlaczenia kolumny 9 i 10
    Register[3] = LOW; Register[2] = LOW; Register[1] = LOW; Register[0] = LOW; // Wpisanie do rejestru bitow wylaczenia kolumn 1, 2, 3, 4
    // Zapalenie diody LED w zaleznosci od statusu
    if (StatusCharging == 0) {  // Dioda LED zgaszona
      Register[5] = HIGH; Register[6] = HIGH; Register[7] = HIGH;
    }
    if (StatusCharging == 1) {  // Dioda LED swieci na zielono
      Register[5] = HIGH; Register[6] = HIGH; Register[7] = LOW;
    }
    if (StatusCharging == 2) {  // Dioda LED swieci na czerwono
      Register[5] = HIGH; Register[6] = LOW; Register[7] = HIGH;
    }
    if (StatusCharging == 3) {  // Dioda LED swieci na pomaranczowo
      Register[5] = HIGH; Register[6] = LOW; Register[7] = LOW;
    }
  }

  /* Znaczenie bitow w tablicy !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  Register[7] = HIGH; // Led GREEN wyzej  -->  Q0
  Register[6] = LOW;  // Led RED wyzej    -->  Q1
  Register[5] = HIGH; // Led GREEN nizej  -->  Q2
  Register[4] = HIGH; // Kolumna 9 10     -->  Q3
  Register[3] = LOW;  // Kolumna 7 8      -->  Q4
  Register[2] = LOW;  // Kolumna 5 6      -->  Q5
  Register[1] = LOW;  // Kolumna 3 4      -->  Q6
  Register[0] = LOW;  // Kolumna 1 2      -->  Q7
  */
 
  // algorytm ladowania tablicy do 74595 i wyswietlenie na bargraphie
  digitalWrite(STCPR, LOW);     //Wylaczenie wyjsci 74595
  for(int a = 0; a < 8; a++) {  // Wpisanie kazdego bitu tablicy do rejestru 74595
    digitalWrite(SHCPR, LOW);   // Sygnal zegara
    digitalWrite(DSR, Register[a]);  // Dane
    digitalWrite(SHCPR, HIGH);  // Sygnal zegara 
  }  
  digitalWrite(STCPR, HIGH);    // Wlaczenie wyjsc 74595   
}

void ReadAddresTemp() 
{
  byte address[8];

  onewire.reset_search();
  while(onewire.search(address))
  {
    if (address[0] != 0x28)
      continue;

    if (OneWire::crc8(address, 7) != address[7])
    {
      Serial.println(F("Błędny adres, sprawdz polaczenia"));
      break;
    }

    for (byte i=0; i<8; i++)
    {
      Serial.print(F("0x"));
      Serial.print(address[i], HEX);

      if (i < 7)
        Serial.print(F(", "));
    }
    Serial.println();
  }

  while(1);
}

void SendDataUART()
{
  //Serial.print("Error: "); Serial.println(ErrorPercent);
  Serial.print("PWM Zasilacz:                   ");  Serial.print(PWMPowerSuply);        Serial.println(" ;");
  //Serial.print("Zmierzone napiecie:             ");  Serial.print(VoltAku);              Serial.println(" ;");
  Serial.print("Srednie zmierzone napiecie:     ");  Serial.print(AverangeVoltAku);      Serial.println(" ;");
  Serial.print("Ustawiona wartosc napiecia:     ");  Serial.print(MaxVoltageChrging);    Serial.println(" ;");
  //Serial.print("Zmierzony prad:                 ");  Serial.print(CurCharging);          Serial.println(" ;");
  Serial.print("Sredni zmierzony prad:          ");  Serial.print(AverangeCurCharging);  Serial.println(" ;");
  Serial.print("Ustawiona wartosc pradu:        ");  Serial.print(MaxChargingCurrent);   Serial.println(" ;");
  //Serial.print("Ostatni zmierzony prad:     "); Serial.println(LastCurCharging);

  //Serial.print("Blad regulacji:                 ");  Serial.print(ErrorP);               Serial.println(" ;");
  //Serial.print("Czlon P:                        ");  Serial.print(P);                    Serial.println(" ;");
  //Serial.print("Czlon I:                        ");  Serial.print(I);                    Serial.println(" ;");
  // Serial.print("Czlon D:                        ");  Serial.print(D);                    Serial.println(" ;");
  //Serial.print("Wartosc PWM Fan:                ");  Serial.print(PWMFanValue);          Serial.println(" ;");
  //Serial.print("Ustawiona wartosc temp:         ");  Serial.print(TempSet);              Serial.println(" ;");

  Serial.print("Temp radiatora:                 ");  Serial.print(TempRadiator);         Serial.println(" ;");
  Serial.print("Temp baterii:                   ");  Serial.print(TempBattery);          Serial.println(" ;");
  Serial.println(" "); 

  /*  //Serial.print("Error: "); Serial.println(ErrorPercent);
  Serial.println(PWMPowerSuply);        
  Serial.println(VoltAku);              
  Serial.println(AverangeVoltAku);      
  Serial.println(MaxVoltageChrging);    
  Serial.println(CurCharging);          
  Serial.println(AverangeCurCharging);  
  Serial.println(MaxChargingCurrent);  
  
  Serial.println(ErrorP);             
  Serial.println(P);                   
  Serial.println(I);                   
  Serial.println(D);                   
  Serial.println(PWMFanValue);         
  Serial.println(TempSet);             

  Serial.println(TempRadiator);        
  Serial.println(TempBattery);    */      
}

