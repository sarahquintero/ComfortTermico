// Sistema de monitoreo ambiental y seguridad para Arduino

// Sección de Inclusión de Librerías
#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <string.h>
#include "DHT.h"
#include <Servo.h>
#include <SPI.h>
#include <MFRC522.h>

// Sección de Definición de Pines
#define PIN_LED_RED 25
#define PIN_LED_GREEN 23
#define PIN_LED_BLUE 24
#define PIN_LUZ A0
#define DHTPIN 22
#define DHTTYPE DHT11
#define BUZZER_PIN 7
#define PIN_VENTILADOR 6
#define PIN_SERVO 10

// Sección de Configuración RFID
const int RST_PIN = 33;
const int SS_PIN = 53;
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Sección de Configuración de Pantalla LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Sección de Configuración de Teclado Matricial
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[ROWS] = { 28, 30, 32, 34 };
byte colPins[COLS] = { 36, 38, 40, 42 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Sección de Prototipos de Funciones
void readLuz();
void readTemp();
void readHumidity();
void encenderLedSeguridad(int pin);
void parpadearLedBloqueado();
void activarBuzzerAlarma();
void parpadearLedAlarma();
void onEnterInit();
void onExitInit();
void onEnterMonitoreo();
void onExitMonitoreo();
void onEnterBloqueado();
void onExitBloqueado();
void onEnterAlarma();
void onExitAlarma();
void handleInit();
void handleMonitoreo();
void handleBloqueado();
void handleAlarma();
void onEnterPMVAlto();
void onExitPMVAlto();
void handlePMVAlto();
void calculatePMV();
void readRFID();
void printArray(byte* buffer, byte bufferSize);
bool isEqualArray(byte* arrayA, byte* arrayB, int length);

// Sección de Variables Globales para Datos de Sensores y Control de Tiempo
int luzValue = 0;
float tempValue = NAN;
float humidityValue = NAN;
DHT dht(DHTPIN, DHTTYPE);
unsigned long monitoreoEnterTime = 0;
const long SENSOR_STABILIZE_TIME = 3000;
bool sensorsStable = false;
unsigned long pmvBajoEnterTime = 0;
const long PMV_BAJO_DURATION_MS = 4000;

// Sección de Tabla de Valores PMV (Predicted Mean Vote)
const float PMV_TABLE_TEMP[] = { 30.0, 29.0, 28.0, 27.0, 26.0, 25.0, 24.0, 23.0, 22.0, 21.0, 20.0, 19.0, 18.0, 29.5, 27.5, 25.5, 23.5, 21.5, 19.5, 20.5 };
const float PMV_TABLE_HUM[] = { 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 80.0, 70.0, 60.0, 50.0, 40.0, 30.0, 20.0, 45.0, 65.0, 85.0, 75.0, 55.0, 35.0, 45.0 };
const float PMV_TABLE_VALUE[] = { 2.9, 2.4, 1.9, 1.4, 0.9, 0.4, 0.0, 1.4, -0.9, -1.4, -1.9, -2.4, -2.9, 2.6, 1.6, 0.6, 1.2, -1.2, -2.2, -1.6 };
const int PMV_TABLE_SIZE = 20;

float pmvValue = NAN;

// Sección de UID para Tarjetas RFID
byte tarjetaPMVBajo[4] = { 0x53, 0xF3, 0xD2, 0x2E };
byte llaveroPMVAlto[4] = { 0x81, 0xAA, 0xE0, 0x26 };

// Sección de Declaración de Tareas Asíncronas
AsyncTask TaskLuz(1000, true, readLuz);
AsyncTask TaskTemp(2000, true, readTemp);
AsyncTask TaskHumidity(2500, true, readHumidity);
AsyncTask TaskBuzzerAlarma(100, true, activarBuzzerAlarma);
AsyncTask TaskLedAlarma(100, true, parpadearLedAlarma);
AsyncTask TaskPMV(3000, true, calculatePMV);
AsyncTask TaskVentilador(7000, false, []() {
  digitalWrite(PIN_VENTILADOR, LOW);
  Serial.println("Ventilador OFF (AsyncTask)");
});

// Sección de Enumeración de Estados de la Máquina de Estados
enum State {
  INIT = 0,
  MONITOREO_AMBIENTAL = 1,
  BLOQUEADO = 2,
  ALARMA = 3,
  PMV_ALTO = 4,
  PMV_BAJO = 5
};

// Sección de Enumeración de Entradas/Eventos para la Máquina de Estados
enum Input {
  CLAVE_CORRECTA = 0,
  CLAVE_INCORRECTA = 1,
  INTENTOS_AGOTADOS = 2,
  DESBLOQUEO = 3,
  NINGUNA = 4,
  PMV_ALTO_DETECTED = 5,
  PMV_BAJO_DETECTED = 6
};

StateMachine stateMachine(6, 10);

// Sección de Variables para Manejo de Contraseña y Bloqueo
char password[] = "2269";
char user_input[6] = "";
unsigned char pass_idx = 0;
int pass_intentos_incorrectos = 0;
const int MAX_PASS_INTENTOS = 3;
unsigned long bloqueo_led_millis = 0;
bool bloqueo_led_state = LOW;
const long BLOQUEO_LED_ON_TIME = 500;
const long BLOQUEO_LED_OFF_TIME = 200;

// Sección de Variables para Control de Buzzer y Alarma
unsigned long buzzerFreq = 200;
unsigned long lastBuzzerChange = 0;
const int BUZZER_FREQ_STEP = 50;
const int BUZZER_MIN_FREQ = 200;
const int BUZZER_MAX_FREQ = 800;
const long BUZZER_CHANGE_INTERVAL = 5;
bool buzzerDirectionUp = true;
unsigned long alarmaStartTime = 0;
const long ALARMA_DURATION_MS = 5000;

// Sección de Variables para Conteo de Alarmas
int alarm_entry_count = 0;
const int MAX_ALARM_ENTRIES = 3;
unsigned long ledAlarmaBlinkMillis = 0;
bool ledAlarmaState = LOW;
const long ALARMA_LED_ON_MS = 200;
const long ALARMA_LED_OFF_MS = 100;

Input currentInput = NINGUNA;

//State Machine Setup Function
/**
 * @brief Configura las transiciones y callbacks de la máquina de estados.
 * 
 * Esta función inicializa y define todas las transiciones posibles entre los diferentes
 * estados del sistema, así como las funciones que se ejecutan al entrar y salir de cada estado.
 * Es crucial para el comportamiento general del sistema.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Configuración de la Máquina de Estados (Transiciones y Callbacks)
void setupStateMachine() {
  stateMachine.AddTransition(INIT, MONITOREO_AMBIENTAL, []() {
    return currentInput == CLAVE_CORRECTA;
  });
  stateMachine.AddTransition(INIT, BLOQUEADO, []() {
    return currentInput == INTENTOS_AGOTADOS;
  });
  stateMachine.AddTransition(BLOQUEADO, INIT, []() {
    return currentInput == DESBLOQUEO;
  });
  stateMachine.AddTransition(MONITOREO_AMBIENTAL, ALARMA, []() {
    return sensorsStable && (luzValue < 10 || tempValue > 40.0);
  });
  stateMachine.AddTransition(ALARMA, MONITOREO_AMBIENTAL, []() {
    return (millis() - alarmaStartTime >= ALARMA_DURATION_MS) && (alarm_entry_count < MAX_ALARM_ENTRIES);
  });
  stateMachine.AddTransition(ALARMA, BLOQUEADO, []() {
    return (millis() - alarmaStartTime >= ALARMA_DURATION_MS) && (alarm_entry_count >= MAX_ALARM_ENTRIES);
  });
  stateMachine.AddTransition(MONITOREO_AMBIENTAL, PMV_ALTO, []() {
    return currentInput == PMV_ALTO_DETECTED;
  });
  stateMachine.AddTransition(MONITOREO_AMBIENTAL, PMV_BAJO, []() {
    return currentInput == PMV_BAJO_DETECTED;
  });
  stateMachine.AddTransition(PMV_ALTO, MONITOREO_AMBIENTAL, []() {
    return currentInput != PMV_ALTO_DETECTED;
  });
  stateMachine.AddTransition(PMV_BAJO, MONITOREO_AMBIENTAL, []() {
    return (millis() - pmvBajoEnterTime >= PMV_BAJO_DURATION_MS);
  });

  stateMachine.SetOnEntering(INIT, onEnterInit);
  stateMachine.SetOnEntering(MONITOREO_AMBIENTAL, onEnterMonitoreo);
  stateMachine.SetOnEntering(BLOQUEADO, onEnterBloqueado);
  stateMachine.SetOnEntering(ALARMA, onEnterAlarma);
  stateMachine.SetOnEntering(PMV_ALTO, onEnterPMVAlto);
  stateMachine.SetOnEntering(PMV_BAJO, onEnterPMVBajo);

  stateMachine.SetOnLeaving(INIT, onExitInit);
  stateMachine.SetOnLeaving(MONITOREO_AMBIENTAL, onExitMonitoreo);
  stateMachine.SetOnLeaving(BLOQUEADO, onExitBloqueado);
  stateMachine.SetOnLeaving(ALARMA, onExitAlarma);
  stateMachine.SetOnLeaving(PMV_ALTO, onExitPMVAlto);
  stateMachine.SetOnLeaving(PMV_BAJO, onExitPMVBajo);
}

//State Enter/Exit Functions
/**
 * @brief Acciones al entrar al estado INIT (Inicial).
 * 
 * Esta función se ejecuta cada vez que el sistema entra en el estado de inicialización.
 * Se encarga de limpiar la pantalla LCD, mostrar el mensaje de bienvenida y resetea
 * las variables relacionadas con la entrada de la clave de acceso.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Funciones "onEnter" (al entrar a un estado)
void onEnterInit() {
  lcd.clear();
  lcd.print("Ingrese clave:");
  Serial.println("-> Estado Init: Ingrese clave:");
  pass_idx = 0;
  memset(user_input, 0, sizeof(user_input));
  pass_intentos_incorrectos = 0;
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
  TaskLuz.Start();
  TaskTemp.Start();
  TaskHumidity.Start();
  alarm_entry_count = 0;
}

/**
 * @brief Acciones al salir del estado INIT (Inicial).
 * 
 * Esta función se ejecuta cada vez que el sistema sale del estado de inicialización.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Funciones "onExit" (al salir de un estado)
void onExitInit() {
  Serial.println("<- Saliendo de Init");
}

/**
 * @brief Acciones al entrar al estado MONITOREO_AMBIENTAL.
 * 
 * Esta función se ejecuta cada vez que el sistema entra en el estado de monitoreo ambiental.
 * Activa las tareas de lectura de sensores (luz, temperatura, humedad y PMV) y enciende
 * el LED verde para indicar el monitoreo activo.
 * 
 * @return No retorna ningún valor.
*/
void onEnterMonitoreo() {
  TaskVentilador.Stop();
  lcd.clear();
  lcd.print("Monitoreo activo");
  Serial.println("-> Estado Monitoreo: Monitoreo activo");
  digitalWrite(PIN_LED_GREEN, HIGH);

  TaskLuz.Start();
  TaskTemp.Start();
  TaskHumidity.Start();
  TaskPMV.Start();

  monitoreoEnterTime = millis();
  sensorsStable = false;
}

/**
 * @brief Acciones al salir del estado MONITOREO_AMBIENTAL.
 * 
 * Esta función se ejecuta cada vez que el sistema sale del estado de monitoreo ambiental.
 * Se encarga de apagar el LED verde que indica el monitoreo.
 * 
 * @return No retorna ningún valor.
*/
void onExitMonitoreo() {
  Serial.println("<- Saliendo de Monitoreo");
  digitalWrite(PIN_LED_GREEN, LOW);
}

/**
 * @brief Acciones al entrar al estado BLOQUEADO.
 * 
 * Esta función se ejecuta cada vez que el sistema entra en el estado bloqueado.
 * Muestra un mensaje en la LCD indicando que el sistema está bloqueado y las
 * instrucciones para desbloquearlo.
 * 
 * @return No retorna ningún valor.
*/
void onEnterBloqueado() {
  lcd.clear();
  lcd.print("Sistema bloqueado");
  lcd.setCursor(0, 1);
  lcd.print("Presione #");
  Serial.println("-> Estado Bloqueado: Sistema bloqueado. Presione #");
  bloqueo_led_millis = millis();
}

/**
 * @brief Acciones al salir del estado BLOQUEADO.
 * 
 * Esta función se ejecuta cada vez que el sistema sale del estado bloqueado.
 * Se encarga de apagar el LED rojo de bloqueo y resetear el contador de intentos.
 * 
 * @return No retorna ningún valor.
*/
void onExitBloqueado() {
  Serial.println("<- Saliendo de Bloqueado");
  digitalWrite(PIN_LED_RED, LOW);
  pass_intentos_incorrectos = 0;
}

/**
 * @brief Acciones al entrar al estado ALARMA.
 * 
 * Esta función se ejecuta cada vez que el sistema entra en el estado de alarma.
 * Muestra un mensaje de advertencia en la LCD, inicia el zumbador y el parpadeo del LED
 * de alarma, y registra el número de veces que se ha activado la alarma.
 * 
 * @return No retorna ningún valor.
*/
void onEnterAlarma() {
  lcd.clear();
  lcd.print("!!! ALARMA !!!");
  Serial.println("-> Estado ALARMA: !!! ALARMA !!!");
  alarmaStartTime = millis();
  alarm_entry_count++;
  Serial.print("Alarmas activadas: ");
  Serial.println(alarm_entry_count);
  TaskBuzzerAlarma.Start();
  TaskLedAlarma.Start();
}

/**
 * @brief Acciones al salir del estado ALARMA.
 * 
 * Esta función se ejecuta cada vez que el sistema sale del estado de alarma.
 * Detiene el zumbador y el parpadeo del LED de alarma, y reinicia las tareas
 * de lectura de sensores.
 * 
 * @return No retorna ningún valor.
*/
void onExitAlarma() {
  Serial.println("<- Saliendo de Alarma");
  noTone(BUZZER_PIN);
  TaskBuzzerAlarma.Stop();
  TaskLedAlarma.Stop();
  TaskLuz.Start();
  TaskTemp.Start();
  TaskHumidity.Start();
  TaskPMV.Start();
}

// Sección de Función setup() (Inicialización del Arduino)
void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();

  tempValue = dht.readTemperature();
  humidityValue = dht.readHumidity();

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LUZ, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_VENTILADOR, OUTPUT);

  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_VENTILADOR, LOW);
  noTone(BUZZER_PIN);

  SPI.begin();
  mfrc522.PCD_Init();

  setupStateMachine();
  stateMachine.SetState(INIT, false, true);
}

// Sección de Función loop() (Bucle Principal del Programa)
void loop() {
  TaskLuz.Update();
  TaskTemp.Update();
  TaskHumidity.Update();
  TaskPMV.Update();
  readRFID();

  TaskBuzzerAlarma.Update();
  TaskLedAlarma.Update();
  TaskVentilador.Update();

  char key = keypad.getKey();
  Input keypadInput = NINGUNA;

  if (key) {
    if (stateMachine.GetState() == INIT) {
      if (pass_idx < sizeof(user_input) - 1) {
        lcd.setCursor(pass_idx % 16, 1);
        lcd.print('*');
        user_input[pass_idx++] = key;
        user_input[pass_idx] = '\0';
      }

      if (pass_idx == strlen(password)) {
        if (strcmp(password, user_input) == 0) {
          digitalWrite(PIN_LED_GREEN, HIGH);
          keypadInput = CLAVE_CORRECTA;
        } else {
          pass_intentos_incorrectos++;
          if (pass_intentos_incorrectos >= MAX_PASS_INTENTOS) {
            keypadInput = INTENTOS_AGOTADOS;
          } else {
            keypadInput = CLAVE_INCORRECTA;
          }
        }
      }
    } else if (stateMachine.GetState() == BLOQUEADO && key == '#') {
      keypadInput = DESBLOQUEO;
    }
  }

  if (currentInput == NINGUNA ||
      (currentInput == PMV_ALTO_DETECTED && stateMachine.GetState() != MONITOREO_AMBIENTAL) ||
      (currentInput == PMV_BAJO_DETECTED && stateMachine.GetState() != MONITOREO_AMBIENTAL) ) {
      currentInput = keypadInput;
  }
  stateMachine.Update();

  switch (stateMachine.GetState()) {
    case INIT:
      handleInit();
      break;
    case MONITOREO_AMBIENTAL:
      handleMonitoreo();
      break;
    case BLOQUEADO:
      handleBloqueado();
      parpadearLedBloqueado();
      break;
    case ALARMA:
      handleAlarma();
      break;
    case PMV_ALTO:
      handlePMVAlto();
      break;
    case PMV_BAJO:
      handlePMVBajo();
      break;
  }

  if (keypadInput != NINGUNA) {
      currentInput = NINGUNA;
  }
  if (stateMachine.GetState() != PMV_ALTO && stateMachine.GetState() != PMV_BAJO &&
      (currentInput == PMV_ALTO_DETECTED || currentInput == PMV_BAJO_DETECTED)) {
      currentInput = NINGUNA;
  }

  delay(50);
}

// Sección de Función para Obtener Entrada del Teclado (redundante, se maneja en loop)
Input getInput() {
  char key = keypad.getKey();
  if (stateMachine.GetState() == INIT) {
    if (key) {
      if (pass_idx < sizeof(user_input) - 1) {
        lcd.setCursor(pass_idx % 16, 1);
        lcd.print('*');
        user_input[pass_idx++] = key;
        user_input[pass_idx] = '\0';
      }

      if (pass_idx == strlen(password)) {
        if (strcmp(password, user_input) == 0) {
          digitalWrite(PIN_LED_GREEN, HIGH);
          return CLAVE_CORRECTA;
        } else {
          pass_intentos_incorrectos++;
          if (pass_intentos_incorrectos >= MAX_PASS_INTENTOS) {
            return INTENTOS_AGOTADOS;
          }
          return CLAVE_INCORRECTA;
        }
      }
    }
  } else if (stateMachine.GetState() == BLOQUEADO && key == '#') {
    return DESBLOQUEO;
  }
  return NINGUNA;
}

//State Handlers
/**
 * @brief Maneja la lógica en el estado INIT.
 * 
 * Esta función se encarga de procesar la entrada de la clave de usuario
 * y determinar si es correcta, incorrecta o si se han agotado los intentos.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Manejadores de Estado (handle functions)
void handleInit() {
  if (currentInput == CLAVE_CORRECTA) {
    lcd.clear();
    lcd.print("Acceso concedido");
    Serial.println("Acceso concedido");
  } else if (currentInput == CLAVE_INCORRECTA) {
    lcd.clear();
    lcd.print("Clave incorrecta");
    Serial.print("Clave incorrecta. Intentos: ");
    Serial.println(pass_intentos_incorrectos);
    encenderLedSeguridad(PIN_LED_BLUE);
    pass_idx = 0;
    memset(user_input, 0, sizeof(user_input));
    lcd.clear();
    lcd.print("Ingrese clave:");
    Serial.println("Ingrese clave:");
  }
}

/**
 * @brief Maneja la lógica en el estado MONITOREO_AMBIENTAL.
 * 
 * Esta función actualiza la pantalla LCD con los valores de los sensores
 * (luz, temperatura, humedad y PMV) y evalúa si se cumplen las condiciones
 * para activar una alarma.
 * 
 * @return No retorna ningún valor.
*/
void handleMonitoreo() {
  static unsigned long lastUpdate = 0;

  if (!sensorsStable && (millis() - monitoreoEnterTime >= SENSOR_STABILIZE_TIME)) {
      sensorsStable = true;
  }
  if (sensorsStable && (luzValue < 10 || tempValue > 40.0)) {
    if (stateMachine.GetState() != ALARMA) {
        stateMachine.SetState(ALARMA, true, true);
        return;
    }
  }

  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();

    lcd.clear();
    lcd.setCursor(0, 0);

    lcd.print("L:");
    lcd.print(luzValue);
    lcd.print(" T:");
    if (isnan(tempValue)) {
      lcd.print("Err");
    } else {
      lcd.print(tempValue, 1);
      lcd.print("C");
    }

    lcd.setCursor(0, 1);

    lcd.print("H:");
    if (isnan(humidityValue)) {
      lcd.print("Err");
    } else {
      lcd.print(humidityValue, 1);
      lcd.print("%");
    }

    lcd.print(" PMV:");
    if (isnan(pmvValue)) {
      lcd.print("N/A");
    } else {
      lcd.print(pmvValue, 1);
    }

    Serial.print("Luz:");
    Serial.print(luzValue);
    Serial.print(" Temp:");
    Serial.print(isnan(tempValue) ? "Error" : String(tempValue, 1));
    Serial.print("C");
    Serial.print(" Hum:");
    Serial.print(isnan(humidityValue) ? "Error" : String(humidityValue, 1));
    Serial.print("%");
    Serial.print(" PMV:");
    Serial.println(isnan(pmvValue) ? "N/A" : String(pmvValue, 1));
  }
}

/**
 * @brief Maneja la lógica en el estado BLOQUEADO.
 * 
 * Esta función se ejecuta continuamente mientras el sistema está en el
 * estado bloqueado. Su principal tarea es mantener el LED de bloqueo
 * parpadeando.
 * 
 * @return No retorna ningún valor.
*/
void handleBloqueado() {
  // Este manejador de estado puede estar vacío si la lógica principal se maneja en onEnter/onExit y parpadearLedBloqueado.
}

/**
 * @brief Maneja la lógica en el estado ALARMA.
 * 
 * Esta función se ejecuta continuamente mientras el sistema está en el estado
 * de alarma. Se encarga de coordinar el zumbador y el parpadeo del LED de alarma.
 * 
 * @return No retorna ningún valor.
*/
void handleAlarma() {
  // Este manejador de estado puede estar vacío si la lógica principal se maneja en onEnter/onExit y las tareas asíncronas.
}

/**
 * @brief Lee el valor del sensor de luz.
 * 
 * Esta función realiza una lectura analógica del pin conectado al sensor de luz
 * y actualiza la variable global `luzValue` con el valor obtenido.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Funciones de Lectura de Sensores
void readLuz() {
  luzValue = analogRead(PIN_LUZ);
}

/**
 * @brief Lee la temperatura del sensor DHT11.
 * 
 * Esta función solicita una lectura de temperatura al sensor DHT11. Si la lectura
 * es exitosa, actualiza la variable global `tempValue`. En caso de fallo,
 * establece `tempValue` como `NAN` (Not a Number).
 * 
 * @return No retorna ningún valor.
*/
void readTemp() {
  float t = dht.readTemperature();
  if (isnan(t)) {
    Serial.println("DHT11: Failed to read temperature!");
    tempValue = NAN;
  } else {
    tempValue = t;
  }
}

/**
 * @brief Lee la humedad del sensor DHT11.
 * 
 * Esta función solicita una lectura de humedad al sensor DHT11. Si la lectura
 * es exitosa, actualiza la variable global `humidityValue`. En caso de fallo,
 * establece `humidityValue` como `NAN`.
 * 
 * @return No retorna ningún valor.
*/
void readHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("DHT11: Failed to read humidity!");
    humidityValue = NAN;
  } else {
    humidityValue = h;
  }
}

/**
 * @brief Calcula el valor PMV (Predicted Mean Vote).
 * 
 * Esta función calcula el valor PMV basándose en las lecturas actuales de
 * temperatura y humedad. Busca la combinación más cercana en una tabla predefinida
 * y asigna el valor PMV correspondiente. Si el PMV calculado es alto,
 * establece la entrada para una posible transición de estado.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Funciones de Cálculo y Detección
void calculatePMV() {
  if (currentInput == PMV_ALTO_DETECTED || currentInput == PMV_BAJO_DETECTED) {
    Serial.println("PMV: RFID detected, skipping sensor PMV calculation.");
    return;
  }

  if (isnan(tempValue) || isnan(humidityValue)) {
    pmvValue = NAN;
    Serial.println("PMV: Temp or Hum is NAN, PMV set to NAN.");
    return;
  }

  float minDistance = 1000000.0;
  int bestMatchIndex = -1;
  const float MAX_PMV_MATCH_DISTANCE = 5.0;

  for (int i = 0; i < PMV_TABLE_SIZE; i++) {
    float distTemp = tempValue - PMV_TABLE_TEMP[i];
    float distHum = humidityValue - PMV_TABLE_HUM[i];
    float currentDistance = sqrt(distTemp * distTemp + distHum * distHum);

    if (currentDistance < minDistance) {
      minDistance = currentDistance;
      bestMatchIndex = i;
    }
  }

  if (bestMatchIndex != -1 && minDistance <= MAX_PMV_MATCH_DISTANCE) {
    pmvValue = PMV_TABLE_VALUE[bestMatchIndex];

    if (pmvValue > 1.0) {
      if (stateMachine.GetState() == MONITOREO_AMBIENTAL) {
        currentInput = PMV_ALTO_DETECTED;
      }
    } else {
      if (stateMachine.GetState() == PMV_ALTO) {
        currentInput = NINGUNA;
      } else {
        currentInput = NINGUNA;
      }
    }
  } else {
    pmvValue = NAN;
    Serial.println("PMV: No close match or out of range, PMV set to NAN.");
    if (stateMachine.GetState() == PMV_ALTO) {
      currentInput = NINGUNA;
    }
  }
}

/* *
 * @brief Lee información de tarjetas RFID.
 * 
 * Esta función detecta si una nueva tarjeta RFID está presente y lee su UID.
 * Si el UID coincide con tarjetas predefinidas (tarjetaPMVBajo o llaveroPMVAlto),
 * establece la entrada correspondiente para la máquina de estados.
 *
 * @return No retorna ningún valor.
*/
void readRFID() {
  if (stateMachine.GetState() != MONITOREO_AMBIENTAL) {
    return;
  }

  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {
      Serial.print(F("Card UID:"));
      printArray(mfrc522.uid.uidByte, mfrc522.uid.size);
      Serial.println();

      if (isEqualArray(mfrc522.uid.uidByte, tarjetaPMVBajo, 4)) {
        currentInput = PMV_BAJO_DETECTED;
        Serial.println("Tarjeta PMV Bajo detectada (RFID) - currentInput set.");
      } else if (isEqualArray(mfrc522.uid.uidByte, llaveroPMVAlto, 4)) {
        currentInput = PMV_ALTO_DETECTED;
        Serial.println("Llavero PMV Alto detectado (RFID) - currentInput set.");
      } else {
        Serial.println("Tarjeta desconocida (RFID).");
      }
      mfrc522.PICC_HaltA();
    }
  }
}

/**
 * @brief Compara dos arreglos de bytes.
 * 
 * Esta función compara los contenidos de dos arreglos de bytes para determinar
 * si son idénticos en la longitud especificada.
 * 
 * @param arrayA Puntero al primer arreglo de bytes.
 * @param arrayB Puntero al segundo arreglo de bytes.
 * @param length Longitud de los arreglos a comparar.
 * @return true Si ambos arreglos son idénticos hasta la longitud especificada.
 * @return false Si los arreglos difieren en cualquier posición o no tienen la misma longitud efectiva.
*/
bool isEqualArray(byte* arrayA, byte* arrayB, int length) {
  for (int index = 0; index < length; index++) {
    if (arrayA[index] != arrayB[index]) return false;
  }
  return true;
}

/**
 * @brief Imprime un arreglo de bytes en el monitor serial.
 * 
 * Esta función recorre un arreglo de bytes e imprime cada byte en formato
 * hexadecimal en el monitor serial, útil para depuración de UIDs de RFID.
 * 
 * @param buffer Puntero al arreglo de bytes a imprimir.
 * @param bufferSize Tamaño del arreglo de bytes.
 * @return No retorna ningún valor.
*/
void printArray(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

/**
 * @brief Hace parpadear el LED de bloqueo.
 * 
 * Esta función controla el parpadeo del LED rojo cuando el sistema está
 * en el estado BLOQUEADO, alternando entre encendido y apagado a intervalos definidos.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Funciones de Control de LEDs y Buzzer
void parpadearLedBloqueado() {
  if (stateMachine.GetState() == State::BLOQUEADO) {
    unsigned long currentMillis = millis();
    long interval = bloqueo_led_state ? BLOQUEO_LED_ON_TIME : BLOQUEO_LED_OFF_TIME;
    if (currentMillis - bloqueo_led_millis >= interval) {
      bloqueo_led_millis = currentMillis;
      bloqueo_led_state = !bloqueo_led_state;
      digitalWrite(PIN_LED_RED, bloqueo_led_state);
    }
  } else {
    if (stateMachine.GetState() != State::ALARMA) {
      digitalWrite(PIN_LED_RED, LOW);
    }
  }
}

/**
 * @brief Enciende momentáneamente un LED de seguridad.
 * 
 * Esta función enciende un LED específico durante un breve periodo (1 segundo)
 * y luego lo apaga, utilizado para indicaciones visuales rápidas, como una
 * clave incorrecta.
 * 
 * @param pin El número del pin del LED a encender.
 * @return No retorna ningún valor.
*/
void encenderLedSeguridad(int pin) {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
}

/**
 * @brief Activa el zumbador de alarma con frecuencia variable.
 * 
 * Esta función genera un sonido de alarma en el zumbador. La frecuencia
 * del sonido varía de forma cíclica entre un mínimo y un máximo, creando
 * un efecto de sirena. Solo se activa si el sistema está en el estado ALARMA.
 * 
 * @return No retorna ningún valor.
*/
void activarBuzzerAlarma() {
  if (stateMachine.GetState() == State::ALARMA) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastBuzzerChange >= BUZZER_CHANGE_INTERVAL) {
      lastBuzzerChange = currentMillis;

      if (buzzerDirectionUp) {
        buzzerFreq += BUZZER_FREQ_STEP;
        if (buzzerFreq >= BUZZER_MAX_FREQ) {
          buzzerFreq = BUZZER_MAX_FREQ;
          buzzerDirectionUp = false;
        }
      } else {
        buzzerFreq -= BUZZER_FREQ_STEP;
        if (buzzerFreq <= BUZZER_MIN_FREQ) {
          buzzerFreq = BUZZER_MIN_FREQ;
          buzzerDirectionUp = true;
        }
      }
      tone(BUZZER_PIN, buzzerFreq);
    }
  } else {
    noTone(BUZZER_PIN);
  }
}

/**
 * @brief Hace parpadear el LED de alarma.
 * 
 * Esta función controla el parpadeo del LED rojo cuando el sistema está
 * en el estado ALARMA, alternando entre encendido y apagado a intervalos definidos.
 * 
 * @return No retorna ningún valor.
*/
void parpadearLedAlarma() {
  if (stateMachine.GetState() == State::ALARMA) {
    unsigned long currentMillis = millis();
    long interval = ledAlarmaState ? ALARMA_LED_ON_MS : ALARMA_LED_OFF_MS;

    if (currentMillis - ledAlarmaBlinkMillis >= interval) {
      ledAlarmaBlinkMillis = currentMillis;
      ledAlarmaState = !ledAlarmaState;
      digitalWrite(PIN_LED_RED, ledAlarmaState);
    }
  } else {
    digitalWrite(PIN_LED_RED, LOW);
  }
}

// Sección de Funciones "onEnter" para PMV
void onEnterPMVAlto() {
  lcd.clear();
  lcd.print("PMV ALTO!");
  lcd.setCursor(0, 1);
  lcd.print("Ventilador ON");
  Serial.println("-> Estado PMV ALTO! Activando ventilador.");

  if (stateMachine.GetState() != ALARMA) {
    digitalWrite(PIN_LED_RED, HIGH);
  }

  digitalWrite(PIN_VENTILADOR, HIGH);
  TaskVentilador.Start();
  delay(7000);
}

/**
 * @brief Acciones al salir del estado PMV_ALTO.
 * 
 * Esta función se ejecuta cada vez que el sistema sale del estado de PMV Alto.
 * Apaga el LED rojo y el ventilador si estaban encendidos.
 * 
 * @return No retorna ningún valor.
*/

void onExitPMVAlto() {
  Serial.println("<- Saliendo de PMV Alto. Apagando ventilador (si estaba encendido).");
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_VENTILADOR, LOW);
}

/**
 * @brief Maneja la lógica en el estado PMV_ALTO.
 * 
 * Esta función se ejecuta mientras el sistema está en el estado de PMV Alto.
 * Actualiza la información del PMV en la pantalla LCD.
 * 
 * @return No retorna ningún valor.
*/
// Sección de Manejadores de Estado para PMV
void handlePMVAlto() {
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {
    lastDisplayUpdate = millis();
    lcd.setCursor(0, 0);
    lcd.print("PMV ALTO!");
    lcd.setCursor(0, 1);
    lcd.print("PMV: ");
    if (isnan(pmvValue)) {
      lcd.print("N/A");
    } else {
      lcd.print(pmvValue, 1);
    }
  }
}

void onEnterPMVBajo() {
  lcd.clear();
  lcd.print("PMV BAJO!");
  lcd.setCursor(0, 1);
  lcd.print("Ambiente Frio");
  Serial.println("-> Estado PMV BAJO! Ambiente frío.");

  digitalWrite(PIN_LED_BLUE, HIGH);
  pmvBajoEnterTime = millis();
}

/**
 * @brief Acciones al entrar al estado PMV_BAJO.
 * 
 * Esta función se ejecuta cada vez que el sistema entra en el estado de PMV Bajo.
 * Muestra un mensaje en la LCD indicando el PMV bajo y enciende el LED azul.
 * 
 * @return No retorna ningún valor.
*/
void onExitPMVBajo() {
  Serial.println("<- Saliendo de PMV Bajo.");
  digitalWrite(PIN_LED_BLUE, LOW);
}

/**
 * @brief Maneja la lógica en el estado PMV_BAJO.
 * 
 * Esta función se ejecuta mientras el sistema está en el estado de PMV Bajo.
 * Actualiza la información del PMV en la pantalla LCD.
 * 
 * @return No retorna ningún valor.
*/
void handlePMVBajo() {
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {
    lastDisplayUpdate = millis();
    lcd.setCursor(0, 0);
    lcd.print("PMV BAJO!");
    lcd.setCursor(0, 1);
    lcd.print("PMV: ");
    if (isnan(pmvValue)) {
      lcd.print("N/A");
    } else {
      lcd.print(pmvValue, 1);
    }
  }
}
