#include <Arduino.h>
#include <Adafruit_MAX31865.h>
#include <TimerFive.h>
#include <TimerOne.h>
#include <LibPrintf.h>
#include <LiquidCrystal_PCF8574.h>    // <LiquidCrystal_I2C>
#include <fuzzy.h>
//#include <EEPROM.h>

#define RREF      430.0
#define RNOMINAL  100.0
#define PERIODO_PWM 8000000L
#define PRESION_C 1
#define PRESION_J 0

#define VENT      31  //26
#define VCAM      30  //27
#define VESC      29  //28     // Valvula de escape - NC al activar se cierra
#define VAIR      28  //29
#define VAC1      27  //30
#define VAC2      26  //31
#define PUERTA    38
#define LED_rojo  43      //43
#define LED_verd  45      //44
#define LED_btn1  41      //41
#define LED_btn2  42      //42
#define LED_btn3  40      //40
#define LED_btn4  44      //45
#define btn1      4     //4      //rojo
#define btn2      2     //2      //verde
#define btn3      6     //7      //paquetes
#define btn4      7     //6      //embolsados
#define btn5      5     //5      //liquidos
#define btn6      3     //3      //sin embolsar
#define salBuzzr  23

#define P_VACIO   -5.688  // 27 = -10PSI
#define PRES_VAC  -4.6    // 35 <- 50
#define PRES_AIR  -0.316  // 95 
#define PRES_OUT  1.264   // 100 = 0PSI
#define VOFF      0
#define VON_OUT   1
#define VIN       2
#define VVAC      3
#define AIRE      4
#define VENFRIA   5
#define PAQUETS   1
#define LIQUI2    2
#define EMBOLSA2  3
#define SINBOLSA  4

#define txSelCic        " Seleccionar Ciclo: "
#define txCicl12        "1 Paquete 2 Liquidos"
#define txCicl3         "  3 Embolsados      "
#define txCicl4         "  4 Sin Embolsar    "
#define txPaqtes        "Esterilizar Paquetes"
#define txLiqui2        "Esterilizar Liquidos"
#define txEmbolsa2      "  Est. Embolsados   "
#define txSinBolsa      " Est. Sin Embolsar  "
#define txBtnInicio     "  Presionar Inicio  "
#define txSnsors        "     Sensores       "
#define txTempBj1       "  Temperatura baja  "
#define txTempBj2       "  espere T2 > 121   "
#define txTemperaturas  "T1=       T2=       "
#define txPresiones     "P1=       P2=       "
#define txPrevacio      " Prevacio 1 - IN    "  // 10 - 14
#define txTiempoEster   " Esterilizado 07:00 "
#define txEstVac        " Tiempo vacio 20:00 "
#define txEstEsc        "Escape Esterilizado "
#define txEnfria        " Enfriando Liquidos "
#define txEstAire       "        Aire        "
#define txTerminado     "     Terminado      "
#define txBorraLinea    "                    "
#define txCicloCancel   "  Ciclo cancelado   "
#define txVacioBajo     "     Vacio bajo     "

bool puertAbierta = true;
bool esperaBtnInicio = false;
bool finProceso = false;
bool enProceso = false;
byte prevacios = 0;
byte estadoValvulas = 0;
byte cicloSelec = 0;
int tiempo = 0;
int tiempoVacio;  //*60;
bool verSensores = false;
bool activo = false;
bool enref = false;
bool control = false;
//byte celsiusSymbol_glyphIndex = 0;
float t1=50.4, t2=100.0;
float p1=0, p2=0;
unsigned long msT, usAlto, msBtn, msVacio, msPuerta;
bool edoPWM = true;
int pwmDc = 0;
bool testVal = false;
byte lineaPrevacio = 0;
bool calentando = false;
byte pantalla = 0;
byte t1txlant, t2txlant, p1txlant, p2txlant;
//String cadenaMnjs;
int presionCero = 100;
bool ajustePresCero = false;
bool bntContinuo = false;
bool buzzer = false;
int buzTs;
bool vacioBajo = false;

Adafruit_MAX31865 ptc = Adafruit_MAX31865(48);
Adafruit_MAX31865 ptj = Adafruit_MAX31865(53);
LiquidCrystal_PCF8574 lcd(0x27);
fuzzy ctrlDifuso;

float UIN[] = {-3, 0.1};
float EPP[] = {-0.1, 0.0, 0.1,'T'}; 
float EC[] = {-0.45, -0.35, -0.25, 'T'};
float ENP[] = {-3, -1.5, 0, 'T'};
float USAL[] = {0, 10};
float AT[] = {4, 7, 10, 'T'};
float ZE[] = {1, 3, 5, 'T'};
float DT[] = {0, 1, 2, 'T'};
float paso = 1; //Se define la cantidad mínima de variación para los numeros del conjunto de salida
float r1 = 134.2, error;
const int tam = ctrlDifuso.calc_size(USAL,paso); // (abs(USAL[0]) + abs(USAL[1]))/paso

/*namespace glyphs {
uint8_t celsiusSymbol[8] = {
  0b11000,
  0b11000,
  0b00000,
  0b00111,
  0b01000,
  0b01000,
  0b01000,
  0b00111
};
}*/

void Activar() {
  activo = true;
  control = true;
  Serial.print(F(" Inicia control "));
  
  Timer1.start();
  usAlto = micros();
  Timer5.start(); 
}

void FinalizaControl() {
  Timer1.stop();
  Timer5.stop();
  control = false;
  tiempo = 0;
  edoPWM = true;
  enref = false;
  digitalWrite(VENT, edoPWM);
}

void ProbarSalida(int salida) {
  digitalWrite(salida, testVal);
  testVal = !testVal;
}

void Valvulas(int pv3) {
  switch (pv3) {
    case VIN:
      estadoValvulas = VIN;
      digitalWrite(VAC1, HIGH);
      digitalWrite(VAC2, HIGH);
      digitalWrite(VESC, LOW);
      digitalWrite(VCAM, LOW);
      digitalWrite(VENT, LOW);
      break;
    case VVAC:
      estadoValvulas = VVAC;
      digitalWrite(VESC, LOW);
      digitalWrite(VAC1, LOW);
      digitalWrite(VAC2, LOW);
      break;
    case AIRE:
      estadoValvulas = AIRE;
      digitalWrite(VAC1, HIGH);
      digitalWrite(VAC2, HIGH);
      digitalWrite(VAIR, LOW);
      break;
    case VON_OUT:
      estadoValvulas = VON_OUT;
      digitalWrite(VENT, LOW);
      digitalWrite(VCAM, HIGH);
      digitalWrite(VESC, HIGH);
      digitalWrite(VAIR, HIGH);
      digitalWrite(VAC1, HIGH);
      digitalWrite(VAC2, HIGH);
      break;
    case VENFRIA:
      estadoValvulas = VENFRIA;
      digitalWrite(VESC, LOW);
      digitalWrite(VCAM, HIGH);
      digitalWrite(VENT, HIGH);
    default:
      estadoValvulas = VOFF;
      digitalWrite(VENT, HIGH);
      digitalWrite(VCAM, HIGH);
      digitalWrite(VESC, HIGH);
      digitalWrite(VAIR, HIGH);
      digitalWrite(VAC1, HIGH);
      digitalWrite(VAC2, HIGH);
      break;
  }
}

void SeleCiclo (bool ledOn) {  
  if (cicloSelec != PAQUETS)
    digitalWrite(LED_btn1, ledOn);
  if (cicloSelec != LIQUI2)
    digitalWrite(LED_btn2, ledOn);
  if (cicloSelec != EMBOLSA2)
    digitalWrite(LED_btn3, ledOn);
  if (cicloSelec != SINBOLSA)
    digitalWrite(LED_btn4, ledOn);  
}

void PantallaPrincipal() {
  pantalla = 1;
  verSensores = false;
  cicloSelec = 0;
  SeleCiclo(true);
  lcd.setCursor(0, 0);
  lcd.print(F(txSelCic));
  lcd.setCursor(0, 1);
  lcd.print(F(txCicl12));
  lcd.setCursor(0, 2);
  lcd.print(F(txCicl3));
  lcd.setCursor(0, 3);
  lcd.print(F(txCicl4));
}
void PantallaProceso(String procesoSelecionado) {
  pantalla = 3;
  verSensores = true;
  lcd.clear();  
  lcd.print(procesoSelecionado);
  if (procesoSelecionado != txSnsors) {
    lcd.setCursor(0, 1);
    lcd.print(F(txBtnInicio));
  }
  lcd.setCursor(0, 2);      // Hasta despues de presionar inicio
  lcd.print(F(txPresiones));        
  lcd.setCursor(0, 3);
  lcd.print(F(txTemperaturas));
}

/*void PantallaCancelado() {
  if (pantalla != 4) {
    pantalla = 4;
    lcd.setCursor(0, 0);
    lcd.print(F(txCicloCancel));
    lcd.setCursor(0, 1);
    lcd.print(F(txVacioBajo));
  }
}*/

void InterfaceSerial() {
  if (Serial.available()) {
    String dato;
    dato = Serial.readString();
    switch ((int)dato[0]) {
      case (int)'i':
        if (!activo && !verSensores) {
          for (unsigned int i = 1; i < dato.length(); i++) {
            if (dato[i] == '\n') {
              String raux = dato.substring(1, i);
              r1 = raux.toFloat();
            }
          }     
          Activar();
          tiempo = -1;
          Valvulas(VIN);
          PantallaProceso(txSnsors);
          printf("SP- %.1f °C\n", r1);
        }
        break;       
      case (int)'c':
        if (!activo && !verSensores) {
          for (unsigned int i = 1; i < dato.length(); i++) {
            if (dato[i] == '\n') {
              String raux = dato.substring(1, i);
              pwmDc = raux.toInt();
            }
          }
          control = false;      
          error = 1.0;
          Activar();
          Valvulas(VIN);
          PantallaProceso(txSnsors);
          printf("Ciclo de trabajo = %d%c\n", pwmDc*10, '%');
        }
        break;
      case (int)'s':             
        if (!activo && !verSensores ) {
          verSensores = true;
          PantallaProceso(txSnsors);
        }
        break;
      case (int)'p':
        if (activo)    
          FinalizaControl();      
        if (verSensores)
          verSensores = false;
        if (estadoValvulas == VIN)
          Valvulas(VON_OUT);
        PantallaPrincipal();
        break;
      case (int)'v':
        switch ((int)dato[1]) {
          case (int)'1':
            ProbarSalida(VENT);
            break;
          case (int)'2':
            ProbarSalida(VCAM);
            break;
          case (int)'3':
            ProbarSalida(VESC);
            break;
          case (int)'4':
            ProbarSalida(VAIR);
            break;
          case (int)'5':
            ProbarSalida(VAC1);
            break;
          case (int)'6':
            ProbarSalida(VAC2);
            break;
          default:
            break;
        }
        break;
      
      case (int)'l':
        switch ((int)dato[1]) {
          case (int)'1':
            ProbarSalida(LED_rojo);
            break;
          case (int)'2':
            ProbarSalida(LED_verd);
            break;
          case (int)'3':
            ProbarSalida(LED_btn1);
            break;
          case (int)'4':
            ProbarSalida(LED_btn2);
            break;
          case (int)'5':
            ProbarSalida(LED_btn3);
            break;
          case (int)'6':
            ProbarSalida(LED_btn4);
            break;
          default:
            break;
        }
        break;
      case (int)'e':
        switch ((int)dato[1]) {
          case (int)'1':
              if (t2 > 121 && cicloSelec == 0 && estadoValvulas == VON_OUT && !puertAbierta) {
                r1 = 134;
                cicloSelec = PAQUETS;
                SeleCiclo(false);
                digitalWrite(LED_verd, HIGH);                
                //menu.next_screen();
                prevacios = 0;
                Valvulas(VIN);
              } 
            break;
          default:
            break;
        }
        break;
      default:     
        Serial.println(dato);
        break;
    }
  }
}

void  Muestreo() {  
  uint8_t fault = ptc.readFault();
  if (fault) {
    Serial.print(" F1-x"); Serial.print(fault, HEX);
    ptc.clearFault();
    //delay(50);
  } else
    t1 = ptc.temperature(RNOMINAL, RREF);
  fault = ptj.readFault();
  if (fault) {
    Serial.print(" F2-x"); Serial.print(fault, HEX);
    ptj.clearFault();
    //delay(50);
  } else
    t2 = ptj.temperature(RNOMINAL, RREF + 0.8);
  //p1 = analogRead(PRESION_C);  
  p1 = ((float)analogRead(PRESION_C)-presionCero)*0.079;
  //p2 = analogRead(PRESION_J);
  p2 = ((float)analogRead(PRESION_J)-99)*0.079;
  error = r1 - t1;

  // printf("\n%d, %d, ", activo, pantalla);
  // Serial.println();
  // Serial.print(analogRead(PRESION_C));
  // Serial.print(", ");
  // Serial.print(analogRead(PRESION_J));
  // Serial.print(" >");
  // Serial.print(pantalla);
  // Serial.print(" >");
  // Serial.print(activo);

  // ---- LCD  sensores ----
  if ((activo || verSensores) && (pantalla > 1)) {
    String varToString = String(p1, 1);
    if (varToString.length() < p1txlant) {
      varToString = String(varToString + ' ');
      p1txlant = varToString.length();
    } else if (varToString.length() > p1txlant) {
      p1txlant = varToString.length();
    }
    lcd.setCursor(3, 2);
    lcd.print(varToString);
    varToString = String(p2, 1);
    if (varToString.length() < p2txlant) {
      varToString = String(varToString + ' ');
      p1txlant = varToString.length();
    } else if (varToString.length() > p2txlant) {
      p1txlant = varToString.length();
    }
    lcd.setCursor(13, 2);
    lcd.print(varToString);    
    varToString = String(t1, 1);
    lcd.setCursor(3, 3);
    lcd.print(varToString);
    varToString = String(t2, 1);
    lcd.setCursor(13, 3);
    lcd.print(varToString);
  
    // ---- tiempo o reloj ----
    if (tiempo != 0 && lineaPrevacio == 0 && !esperaBtnInicio) {
      varToString = String(tiempo/60);      
      if (varToString.length() == 1)
        varToString = String('0' + varToString);
      lcd.setCursor(14, 1);
      lcd.print(varToString);
      lcd.print(':');
      varToString = String(tiempo%60);
      if (varToString.length() == 1)
        varToString = String('0' + varToString);
      lcd.print(varToString);
    }
  }
}

void Difuso(void) {
  if (error > 0.1) {
    pwmDc = 10;
  } else {
    float B[tam];
    ctrlDifuso.inicio(B,tam);
    ctrlDifuso.regla_simple(ENP,UIN,error,DT,USAL,B,tam);
    ctrlDifuso.regla_simple(EC,UIN,error,ZE,USAL,B,tam);
    ctrlDifuso.regla_simple(EPP,UIN,error,AT,USAL,B,tam);    
    float res = ctrlDifuso.defusi(B,USAL,tam);
    pwmDc = round(res);
  }
}

void ConteoTiempo() {
  if (enref) {
    if (tiempo > 0)
      tiempo--;
  }  
  if (control)
    Difuso();
}

void bfPWM() {
  if (activo) { 
    if (edoPWM == true && pwmDc > 0) {
      usAlto = micros();
      edoPWM = false;
      digitalWrite(VENT, edoPWM);
    }
  }
}

void ParoSeguridad() {
  cicloSelec = 0;
  SeleCiclo(false);
  if (!puertAbierta)
    digitalWrite(LED_verd, LOW);
  if (activo) {
    FinalizaControl();
    activo = false;
  }
  if (estadoValvulas > VON_OUT) 
    Valvulas(VON_OUT);
  if (verSensores) 
    verSensores = false;  
  if (enProceso)
    enProceso = false;

}

void PantallaCalentamiento() {
  pantalla = 2;
  tiempo = 0;
  verSensores = true;
  lcd.clear();  
  lcd.print(F(txTempBj1));
  lcd.setCursor(0, 1);
  lcd.print(F(txTempBj2));
  lcd.setCursor(0, 2);
  lcd.print(F(txPresiones));
  lcd.setCursor(0, 3);
  lcd.print(F(txTemperaturas));
}

void setup() {
  Timer1.attachInterrupt(ConteoTiempo);
  Timer1.initialize(1000000);
  Timer1.stop();
  Timer5.attachInterrupt(bfPWM);
  Timer5.initialize(PERIODO_PWM);
  Timer5.stop();

  pinMode(VENT, OUTPUT);
  pinMode(VCAM, OUTPUT);
  pinMode(VESC, OUTPUT);
  pinMode(VAIR, OUTPUT);
  pinMode(VAC1, OUTPUT);
  pinMode(VAC2, OUTPUT);
  Valvulas(VON_OUT);
  pinMode(LED_rojo, OUTPUT);
  pinMode(LED_verd, OUTPUT);
  pinMode(LED_btn1, OUTPUT);
  pinMode(LED_btn2, OUTPUT);
  pinMode(LED_btn3, OUTPUT);
  pinMode(LED_btn4, OUTPUT);
  pinMode(salBuzzr, OUTPUT);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(btn5, INPUT_PULLUP);
  pinMode(btn6, INPUT_PULLUP);
  pinMode(PUERTA, INPUT_PULLUP);
  digitalWrite(LED_rojo, HIGH);

  Serial.begin(115200);
  msT = millis();
  lcd.begin(20, 4);
  lcd.init();
  lcd.setBacklight(128);  
  //lcd.createChar(celsiusSymbol_glyphIndex, glyphs::celsiusSymbol); 
  //line4i.add_variable(celsiusSymbol_glyphIndex);
  PantallaCalentamiento();  

  ptc.begin(MAX31865_3WIRE);
  ptj.begin(MAX31865_3WIRE);

  /*t2 = ptj.temperature(RNOMINAL, RREF);
  Serial.print(F("Ajuste "));
  Serial.print(t2);
  Serial.print(F(" "));  
  if (t2 > 50) {
    presionCero = analogRead(PRESION_C);
    //Serial.print(t2);
    //Serial.print(F(" "));
    if (presionCero < 110 && presionCero > 85) {
      EEPROM.write(1, presionCero);
      EEPROM.write(0, 0x0A);
      Serial.print(F("Cero "));
    } else {
      presionCero = 100;
      // Error sesor de presion
    }
  }*/
  Serial.print(F("Ajuste presion cero = "));
  Serial.println(presionCero);
  /*byte eepromUsada = EEPROM.read(0);
  if (eepromUsada == 0x0A) {
    presionCero = EEPROM.read(1);
  } else {
    presionCero = 99;
  }*/

  /*Serial.println(F("i + Temperatura deseada"));
  Serial.println(F("c + Ciclo de trabajo (1-9)"));
  Serial.println(F("s -> Valor de sensores LCD"));
  Serial.println(F("p -> Paro cancelacion"));
  Serial.println(F("v + Valvula(1-6) cambia estado actual"));
  Serial.println(F("e1 -> Inicia Esterilizacion Paquetes"));*/
}

void loop() {

  /*if (!digitalRead(btn1)) {
    Serial.print(" boton 1");
    delay(80);
  }
  if (!digitalRead(btn2)) {
    Serial.print(" boton 2");
    delay(80);
  }
  if (!digitalRead(btn3)) {
    Serial.print(" boton 3");
    delay(80);
  }
  if (!digitalRead(btn4)) {
    Serial.print(" boton 4");
    delay(80);
  }
  if (!digitalRead(btn5)) {
    Serial.print(" boton 5");
    delay(80);
  }
  if (!digitalRead(btn6)) {
    Serial.print(" boton 6");
    delay(80);
  }*/

  //InterfaceSerial();

  if (digitalRead(PUERTA)) {
    delay(300);
    if (digitalRead(PUERTA)) {
      puertAbierta = true;
      //cicloSelec = 0;
      ParoSeguridad();
      if (pantalla != 0) {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(F("   PUERTA ABIERTA   "));
        pantalla = 0;
        ajustePresCero = false;
        msPuerta = millis();
        Serial.print(F("PAbierta"));
      }      
    } 
  } else {
    delay(250);
    if (!digitalRead(PUERTA)) {
      if (puertAbierta) {
        if (estadoValvulas == VON_OUT) {
          if (t2 < 121 && pantalla != 2) {
            PantallaCalentamiento();
            Serial.print(" puertaPC ");
          }
          else if (pantalla != 1) {
            PantallaPrincipal();
            Serial.print(" puertaPP ");
          }
        }
        /*if (estadoValvulas == VOFF) {

        }*/
      }
      puertAbierta = false;
    }
  }

  if (!puertAbierta) {
    if (cicloSelec == 0 && estadoValvulas == VON_OUT) {
      if (t2 > 121) {      
        if (pantalla == 2 && !enProceso) { 
          PantallaPrincipal();
          Serial.print(" PPt2 ");
        }
        if (!digitalRead(btn3)) {       // Boton selecciona proceso
          esperaBtnInicio = true;
          r1 = 134.15;   // Temperatura
          tiempo = 7*60;
          tiempoVacio = 20*60;
          cicloSelec = PAQUETS;
          SeleCiclo(false);          
          digitalWrite(LED_verd, HIGH);
          PantallaProceso(txPaqtes);   // Pantalla <- presionar inicio
        }
        if (!digitalRead(btn5)) {       // Boton selecciona proceso
          esperaBtnInicio = true;
          r1 = 121.15;   // Temperatura
          tiempo = 3*60; 
          //tiempoVacio = 20; 
          cicloSelec = LIQUI2;
          SeleCiclo(false);          
          digitalWrite(LED_verd, HIGH);
          PantallaProceso(txLiqui2);   // Pantalla <- presionar inicio
          Serial.print(F(" BtnLiquidos "));
          Serial.print(estadoValvulas);
        }
        if (!digitalRead(btn4)) {       // Boton selecciona proceso
          esperaBtnInicio = true;
          r1 = 134.15;   // Temperatura
          tiempo = 7*60; 
          tiempoVacio = 15*60;  
          cicloSelec = EMBOLSA2;
          SeleCiclo(false);          
          digitalWrite(LED_verd, HIGH);
          PantallaProceso(txEmbolsa2);   // Pantalla <- presionar inicio
        }
        if (!digitalRead(btn6)) {       // Boton selecciona proceso
          esperaBtnInicio = true;
          r1 = 134.15;   // Temperatura
          tiempo = 3*60; 
          tiempoVacio = 6*60;  
          cicloSelec = SINBOLSA;
          SeleCiclo(false);          
          digitalWrite(LED_verd, HIGH);
          PantallaProceso(txSinBolsa);   // Pantalla <- presionar inicio
        }
      } else {
        cicloSelec = 0;
        SeleCiclo(false);
        if (pantalla == 1) {
          PantallaCalentamiento();
          Serial.print(" temp baja ");
        }
      }    
    }
  } else {
    if ((millis() - msPuerta)/1000 >= 60) {
      if (!ajustePresCero && t2 > 121) {
        ajustePresCero = true;
        presionCero = analogRead(PRESION_C);
        Serial.print("=");
        Serial.print(presionCero);
      }
    }
  } 

  if (!digitalRead(btn1)) {         // --- PARO DE EMERGENCIA ---
    delay(125);
    if (!digitalRead(btn1)) { 
      Serial.print(" bntParo ");
      ParoSeguridad();
      Valvulas(VOFF);
      digitalWrite(LED_rojo, LOW);
      digitalWrite(LED_verd, HIGH);
      if (pantalla != 0) {
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(F(" PARO DE EMERGENCIA "));
        lcd.setCursor(0, 2);
        lcd.print(txBtnInicio); 
        pantalla = 0;
      }
    }
  }

  if (!digitalRead(btn2)) {       // ---  BOTON INICIO  ---
    if (estadoValvulas == VOFF && pantalla == 0) {      
      if (t2 < 121 && pantalla != 2) {
        PantallaCalentamiento();
        Serial.print(" bntIniPC ");
      } else if (pantalla != 1) {
        PantallaPrincipal();
        Serial.print(" bntIniPP ");
      }
      Valvulas(VON_OUT);
      digitalWrite(LED_verd, LOW);
      digitalWrite(LED_rojo, HIGH);
    } 
    if (estadoValvulas == VON_OUT && pantalla == 3) {  
      if (esperaBtnInicio && cicloSelec > 0) {        
        prevacios = 0;
        enProceso = true;
        esperaBtnInicio = false;        
        digitalWrite(LED_verd, LOW);
        lcd.setCursor(0, 1);
        lcd.print(txPrevacio);        // Linea prevacio 1 - IN
        lineaPrevacio = 1;
        lcd.setCursor(0, 3);
        lcd.print(F(txTemperaturas));
        Valvulas(VIN);
        Serial.print(" Vin-BtnInicio ");
      }
      if (finProceso && cicloSelec == 0) {
        finProceso = false;
        pantalla = 2;
        buzzer = false;
        digitalWrite(LED_verd, LOW);
        digitalWrite(salBuzzr, LOW);
      }  
    }      
  }

  /*if (estadoValvulas == VON_OUT && pantalla == 2){ // !!!!! Temperatura baja
    if (!digitalRead(btn5) && !digitalRead(btn6)) {
      delay(100);
      if (!digitalRead(btn5) && !digitalRead(btn6)) {
        if (!bntContinuo) {
          //Serial.print('?');
          msBtn = millis();
          bntContinuo = true;
        } 
        if (bntContinuo) {
          if ((millis() - msBtn) >= 2500) {
            presionCero = analogRead(PRESION_C);
            //Serial.print(presionCero);
            EEPROM.write(1, presionCero);
            EEPROM.write(0, 0x0A);
            msBtn = millis();
            //bntContinuo = false;
            //Serial.print(('>'));
            //Serial.print(EEPROM.read(1));
          }
        }
      }
    }
  }*/

  if (estadoValvulas == VIN && enProceso) {    
    if (cicloSelec == LIQUI2) {
      if (t1 > 105 && t2 > 105) {
        if (prevacios >= 1) {
          if (t1 > 115) {
            if (lineaPrevacio == 1) {
              lcd.setCursor(14, 1);
              lcd.print("OUT");
              lineaPrevacio = 2;
              Valvulas(VON_OUT);
              Serial.print(" OUT");
            }
          }
        } else if (lineaPrevacio == 1) {
          lcd.setCursor(14, 1);
          lcd.print("OUT");
          lineaPrevacio = 2;
          Valvulas(VON_OUT);
          Serial.print(" OUT");
        }
      }
    } else if (t1 > 115 && t2 > 115) {
      if (prevacios >= 1) {
        if (t1 > 125) {
          if (lineaPrevacio == 1) {
            lcd.setCursor(14, 1);
            lcd.print("OUT");
            lineaPrevacio = 2;
            Valvulas(VON_OUT);
            Serial.print(" OUT");
          }
        }
      } else if (lineaPrevacio == 1) {
        lcd.setCursor(14, 1);
        lcd.print("OUT");
        lineaPrevacio = 2;
        Valvulas(VON_OUT);
        Serial.print(" OUT");
      }
    }
  }

  if (estadoValvulas == VON_OUT && cicloSelec > 0 && enProceso) {       
    if (p1 < PRES_OUT) {
      if (cicloSelec == LIQUI2) {
        lineaPrevacio = 3;
        goto SinVacio;
      }
      if (lineaPrevacio == 2) {
        lcd.setCursor(14, 1);
        lcd.print("VAC");
        lineaPrevacio = 3;        
        Valvulas(VVAC);
        Serial.print(" VAC ");
        vacioBajo = false;
        msVacio = millis();
        //Serial.print(msVacio);
      } else if (lineaPrevacio == 0 && pantalla == 3) {
        lcd.setCursor(0, 1);
        lcd.print(txEstVac);
        tiempo = tiempoVacio;
        Valvulas(VVAC);
      }
    }
  }
  if (estadoValvulas == VVAC && cicloSelec > 0 && enProceso) {
    if (activo && !enref) {
      Timer1.start();
      enref = true;
      Serial.print(F(" Vacio 20min "));
    }
    if (p1 < PRES_VAC && !activo) {
SinVacio:
      prevacios++;
      if (prevacios == 3) {
        prevacios = 0;        
        Serial.print(F(" Tercer prevacio "));
        verSensores = false;
        edoPWM = false;
        pwmDc = 10;
        Activar();
        Serial.print(!edoPWM);
        Serial.print(' ');
        lcd.setCursor(0, 1);
        lcd.print(txTiempoEster);
        lineaPrevacio = 0;
        Valvulas(VIN);
        Serial.print(" Vin ");        
      }       
      if (lineaPrevacio == 3) {
        lcd.setCursor(10, 1);
        lcd.print(prevacios + 1);
        lcd.setCursor(14, 1);
        lcd.print("IN ");
        lineaPrevacio = 1;        
        Valvulas(VIN);
        Serial.print(" Vin ");
        Serial.print(prevacios);
      }      
      // Falta monitorear  P1 < -4psi (después de 5 min.), t2 >130°C
      // solo Mensaje vacio bajo, boton para finalizar ciclo.
    }
  }

  if (estadoValvulas == AIRE && cicloSelec > 0) {
    if (p1 > PRES_AIR) { 
FinCiclo:     
      cicloSelec = 0;
      lcd.setCursor(0, 1);
      lcd.print(txTerminado);
      lcd.setCursor(0, 2);
      lcd.print(txBtnInicio);
      finProceso = true;
      enProceso = false;
      digitalWrite(LED_verd, HIGH);
      Serial.print(F(" Proceso Terminado "));
      Valvulas(VON_OUT);
      digitalWrite(salBuzzr, HIGH);
      // buzzer = true;    // *******  Activar buzzer
      // buzTs = 0;
    }
  } 
  if (estadoValvulas == VENFRIA && cicloSelec == LIQUI2) {
    if (p1 < 102) {
      activo = false; 
      lcd.setCursor(0, 3);
      lcd.print(txBorraLinea);
      goto FinCiclo;
    }
  }
  
  if ((millis() - msT) >= 1000) {
    msT = millis();

    Muestreo();
    if (activo) {
      if (tiempo == 0) {
        if (control) {
          FinalizaControl();
          Serial.print(F(" Fin Control "));
          if (cicloSelec != LIQUI2) {
            lcd.setCursor(0, 1);
            lcd.print(txEstEsc);                       
            Valvulas(VON_OUT);
          } else {
            lcd.setCursor(0, 1);
            lcd.print(txEnfria); 
            Valvulas(VENFRIA);
          }        
        }
        if (estadoValvulas == VVAC) {
          Timer1.stop();
          enref = false;
          lcd.setCursor(0, 1);
          lcd.print(txEstAire);
          lcd.setCursor(0, 2);
          lcd.print(txBorraLinea);
          lcd.setCursor(0, 3);
          lcd.print(txBorraLinea);
          Valvulas(AIRE);
          Serial.print(F(" AIRE "));
          activo = false;          
        }
      }      
    } 
    if (!vacioBajo && lineaPrevacio == 3) {      
      if ((millis() - msVacio)/1000 >= 8*60) {        
        vacioBajo = true;
        cicloSelec = 0;
        lcd.setCursor(0, 0);
        lcd.print(F(txCicloCancel));
        lcd.setCursor(0, 1);
        lcd.print(F(txVacioBajo));
        finProceso = true;
        enProceso = false;        
        Serial.print(F(" Vacio bajo "));
        Valvulas(AIRE);        
      }
    }  
  /*  if (buzzer) {
      //Serial.print(buzTs);
      //Serial.print("-");
      if (buzTs == 0) {
        digitalWrite(salBuzzr, HIGH);
        Serial.print(F("on"));
      }
      if (buzTs == 4) {
        digitalWrite(salBuzzr, LOW);
        Serial.print(F("-off "));
      }
      if (buzTs == 20) {
        buzTs = 0;
        //Serial.print('\n');
      }
      buzTs++;
    } */   
  }

  if (estadoValvulas == VIN && control) {
    if (edoPWM == false && pwmDc != 10) {
      if ((micros() - usAlto) >= (unsigned long)pwmDc*(PERIODO_PWM/10)) {
        edoPWM = true;
        digitalWrite(VENT, edoPWM);
      }
    }
    if (error <= 0.02) {
      if (!enref) {
        enref = true;
        Serial.print(F(" Ref "));
      }
    }
    if (cicloSelec == LIQUI2) {
      if (t1 >= 121.5)
        digitalWrite(VCAM, HIGH);
      if (t1 <= 121) 
        digitalWrite(VCAM, LOW);      
    } else {
      if (t1 >= 134.5)
        digitalWrite(VCAM, HIGH);
      if (t1 <= 134) 
        digitalWrite(VCAM, LOW);
    }
  } 
}