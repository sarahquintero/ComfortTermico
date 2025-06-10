#include "StateMachineLib.h"
#include "AsyncTaskLib.h"
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <string.h>
#include "DHT.h"
#include <Servo.h>    // Include the Servo library
#include <SPI.h>      // ¡NUEVO! Librería SPI para RFID
#include <MFRC522.h>  // ¡NUEVO! Librería para el módulo RFID

















// --- DEFINICIONES DE PINES ---
#define PIN_LED_RED 25
#define PIN_LED_GREEN 23
#define PIN_LED_BLUE 24
#define PIN_LUZ A0
#define DHTPIN 22
#define DHTTYPE DHT11
#define BUZZER_PIN 7  // Pin para el buzzer
#define PIN_VENTILADOR 6
#define PIN_SERVO 10




// Pines para el MFRC522 RFID
const int RST_PIN = 33;            // Pin 32 para el reset del RC522
const int SS_PIN = 53;             // Pin 53 para el SS (SDA) del RC522
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Crear instancia del MFRC522



// Configuración LCD
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
















// Configuración del Teclado
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
















// --- PROTOTIPOS DE FUNCIONES ---
void readLuz();
void readTemp();
void readHumidity();
void encenderLedSeguridad(int pin);
void parpadearLedBloqueado();
void activarBuzzerAlarma();  // Para el buzzer en el estado ALARMA
void parpadearLedAlarma();   // NUEVO: Para el LED rojo en alarma
void onEnterInit();
void onExitInit();
void onEnterMonitoreo();
void onExitMonitoreo();
void onEnterBloqueado();
void onExitBloqueado();
void onEnterAlarma();  // Al entrar al estado ALARMA
void onExitAlarma();   // Al salir del estado ALARMA
void handleInit();
void handleMonitoreo();
void handleBloqueado();
void handleAlarma();    // Manejador del estado ALARMA
void onEnterPMVAlto();  // NUEVO
void onExitPMVAlto();   // NUEVO
void handlePMVAlto();   // NUEVO
void calculatePMV();
void readRFID();                                            // ¡NUEVO! Prototipo para la lectura RFID
void printArray(byte* buffer, byte bufferSize);             // ¡NUEVO! Prototipo para imprimir UID
bool isEqualArray(byte* arrayA, byte* arrayB, int length);  // ¡NUEVO! Prototipo para comparar UIDs

















// Variables de sensores
int luzValue = 0;
float tempValue = NAN;
float humidityValue = NAN;
DHT dht(DHTPIN, DHTTYPE);
unsigned long monitoreoEnterTime = 0;
const long SENSOR_STABILIZE_TIME = 3000;  // 3 segundos para que los sensores se estabilicen
bool sensorsStable = false;               // Flag para indicar si los sensores están estables para evaluación de alarma
unsigned long pmvBajoEnterTime = 0;
const long PMV_BAJO_DURATION_MS = 4000; // 4 segundos








// PMV Lookup Table Data (as provided)
const float PMV_TABLE_TEMP[] = { 30.0, 29.0, 28.0, 27.0, 26.0, 25.0, 24.0, 23.0, 22.0, 21.0, 20.0, 19.0, 18.0, 29.5, 27.5, 25.5, 23.5, 21.5, 19.5, 20.5 };
const float PMV_TABLE_HUM[] = { 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 80.0, 70.0, 60.0, 50.0, 40.0, 30.0, 20.0, 45.0, 65.0, 85.0, 75.0, 55.0, 35.0, 45.0 };
const float PMV_TABLE_VALUE[] = { 2.9, 2.4, 1.9, 1.4, 0.9, 0.4, 0.0, 1.4, -0.9, -1.4, -1.9, -2.4, -2.9, 2.6, 1.6, 0.6, 1.2, -1.2, -2.2, -1.6 };
const int PMV_TABLE_SIZE = 20;  // Number of entries in your table



// To store calculated PMV
float pmvValue = NAN;

byte tarjetaPMVBajo[4] = { 0x53, 0xF3, 0xD2, 0x2E };  // Tarjeta PMV Bajo (valor -0.5)
byte llaveroPMVAlto[4] = { 0x81, 0xAA, 0xE0, 0x26 };  // Llavero PMV Alto (valor 2)



// Tasks para sensores
AsyncTask TaskLuz(1000, true, readLuz);
AsyncTask TaskTemp(2000, true, readTemp);
AsyncTask TaskHumidity(2500, true, readHumidity);
AsyncTask TaskBuzzerAlarma(100, true, activarBuzzerAlarma);
AsyncTask TaskLedAlarma(100, true, parpadearLedAlarma);
// Ensure TaskPMV runs slightly after TaskHumidity to get fresh values
AsyncTask TaskPMV(2600, true, calculatePMV); // Adjusted from 3000
AsyncTask TaskVentilador(7000, false, []() {
  digitalWrite(PIN_VENTILADOR, LOW);
  Serial.println("Ventilador OFF (AsyncTask)");
});
















// --- DEFINICIÓN DE ESTADOS ---
enum State {
  INIT = 0,
  MONITOREO_AMBIENTAL = 1,
  BLOQUEADO = 2,
  ALARMA = 3,
  PMV_ALTO = 4,
  PMV_BAJO = 5  // ¡NUEVO ESTADO!
};




enum Input {
  CLAVE_CORRECTA = 0,
  CLAVE_INCORRECTA = 1,
  INTENTOS_AGOTADOS = 2,
  DESBLOQUEO = 3,
  NINGUNA = 4,
  PMV_ALTO_DETECTED = 5,
  PMV_BAJO_DETECTED = 6  // ¡NUEVA ENTRADA!
};
















// Máquina de estados
StateMachine stateMachine(6, 10);  // Ahora 5 estados (INIT, MONITOREO, BLOQUEADO, ALARMA, PMV_ALTO) y al menos 8 transiciones.
















// Variables de seguridad
char password[] = "2269";
char user_input[6] = "";
unsigned char pass_idx = 0;
int pass_intentos_incorrectos = 0;
const int MAX_PASS_INTENTOS = 3;
unsigned long bloqueo_led_millis = 0;
bool bloqueo_led_state = LOW;
const long BLOQUEO_LED_ON_TIME = 500;
const long BLOQUEO_LED_OFF_TIME = 200;
















// Variables específicas para la alarma
unsigned long buzzerFreq = 200;  // Frecuencia inicial para el buzzer
unsigned long lastBuzzerChange = 0;
const int BUZZER_FREQ_STEP = 50;  // Cuánto cambia la frecuencia
const int BUZZER_MIN_FREQ = 200;
const int BUZZER_MAX_FREQ = 800;
const long BUZZER_CHANGE_INTERVAL = 5;  // Ms para cambiar la frecuencia
bool buzzerDirectionUp = true;
unsigned long alarmaStartTime = 0;     // Para rastrear cuándo comenzó la alarma
const long ALARMA_DURATION_MS = 5000;  // Duración de la alarma (5 segundos)








// NUEVO: Variables para el contador de alarmas
int alarm_entry_count = 0;
const int MAX_ALARM_ENTRIES = 3;
unsigned long ledAlarmaBlinkMillis = 0;  // Para el parpadeo del LED de alarma
bool ledAlarmaState = LOW;               // Estado del LED de alarma (ON/OFF)
const long ALARMA_LED_ON_MS = 200;       // LED on for 200ms
const long ALARMA_LED_OFF_MS = 100;      // LED off for 100ms
















// Almacena la última entrada
Input currentInput = NINGUNA;
















void setupStateMachine() {
  // Configurar transicionesloop
  stateMachine.AddTransition(INIT, MONITOREO_AMBIENTAL, []() {
    return currentInput == CLAVE_CORRECTA;
  });
  stateMachine.AddTransition(INIT, BLOQUEADO, []() {
    return currentInput == INTENTOS_AGOTADOS;
  });
  stateMachine.AddTransition(BLOQUEADO, INIT, []() {
    return currentInput == DESBLOQUEO;
  });



  // Transición de MONITOREO_AMBIENTAL a ALARMA
  // Se activa si la temperatura es > 40 O la luz es < 100
  // Transición de MONITOREO_AMBIENTAL a ALARMA
  stateMachine.AddTransition(MONITOREO_AMBIENTAL, ALARMA, []() {
    return sensorsStable && (luzValue < 100 || tempValue > 40.0);  // Prioriza la alarma
  });








  // Transición de ALARMA a MONITOREO_AMBIENTAL
  // Se activa 5 segundos después de que sonara la alarma
  stateMachine.AddTransition(ALARMA, MONITOREO_AMBIENTAL, []() {
    return (millis() - alarmaStartTime >= ALARMA_DURATION_MS) && (alarm_entry_count < MAX_ALARM_ENTRIES);  // NUEVO: También verifica el contador
  });








  // NUEVO: Transición de ALARMA a BLOQUEADO
  // Se activa cuando el contador de alarmas alcanza el límite, después de la duración de la alarma.
  stateMachine.AddTransition(ALARMA, BLOQUEADO, []() {
    return (millis() - alarmaStartTime >= ALARMA_DURATION_MS) && (alarm_entry_count >= MAX_ALARM_ENTRIES);
  });


  stateMachine.AddTransition(MONITOREO_AMBIENTAL, PMV_ALTO, []() {
    return currentInput == PMV_ALTO_DETECTED; // No need for the NOT alarm condition here anymore if alarm is handled directly
  });
  stateMachine.AddTransition(MONITOREO_AMBIENTAL, PMV_BAJO, []() {
    return currentInput == PMV_BAJO_DETECTED; // Same here
  });




  // Transición de PMV_ALTO de vuelta a MONITOREO_AMBIENTAL
  // Esta condición significa "salir de PMV_ALTO si ya no se detecta la condición de PMV_ALTO".
  stateMachine.AddTransition(PMV_ALTO, MONITOREO_AMBIENTAL, []() {
    return currentInput != PMV_ALTO_DETECTED;
  });




stateMachine.AddTransition(PMV_BAJO, MONITOREO_AMBIENTAL, []() {
  // NUEVO: Transición basada en tiempo
  return (millis() - pmvBajoEnterTime >= PMV_BAJO_DURATION_MS);
});




  // Configurar acciones al entrar/salir de estados
  stateMachine.SetOnEntering(INIT, onEnterInit);
  stateMachine.SetOnEntering(MONITOREO_AMBIENTAL, onEnterMonitoreo);
  stateMachine.SetOnEntering(BLOQUEADO, onEnterBloqueado);
  stateMachine.SetOnEntering(ALARMA, onEnterAlarma);
  stateMachine.SetOnEntering(PMV_ALTO, onEnterPMVAlto);  // ¡NUEVO!
  stateMachine.SetOnEntering(PMV_BAJO, onEnterPMVBajo);



  stateMachine.SetOnLeaving(INIT, onExitInit);
  stateMachine.SetOnLeaving(MONITOREO_AMBIENTAL, onExitMonitoreo);
  stateMachine.SetOnLeaving(BLOQUEADO, onExitBloqueado);
  stateMachine.SetOnLeaving(ALARMA, onExitAlarma);
  stateMachine.SetOnLeaving(PMV_ALTO, onExitPMVAlto);  // ¡NUEVO!
  stateMachine.SetOnLeaving(PMV_BAJO, onExitPMVBajo);
}




// Funciones de entrada/salida de estados
void onEnterInit() {
  lcd.clear();
  lcd.print("Ingrese clave:");
  Serial.println("-> Estado Init: Ingrese clave:");
  pass_idx = 0;
  memset(user_input, 0, sizeof(user_input));
  pass_intentos_incorrectos = 0;  // Resetear intentos al entrar a Init
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
  TaskLuz.Start();
  TaskTemp.Start();
  TaskHumidity.Start();
  alarm_entry_count = 0;  // NUEVO: Resetear contador de alarmas al volver a INIT
}





void onExitInit() {
  Serial.println("<- Saliendo de Init");
}



void onEnterMonitoreo() {
  TaskVentilador.Stop();
  //Serial.println("-> Estado Monitoreo: Entering onEnterMonitoreo()");  // DEBUG
  lcd.clear();
  lcd.print("Monitoreo activo");
  Serial.println("-> Estado Monitoreo: Monitoreo activo");
  digitalWrite(PIN_LED_GREEN, HIGH);  // LED verde para indicar monitoreo activo


  TaskLuz.Start();
  //Serial.println("DEBUG: TaskLuz Started.");  // DEBUG
  TaskTemp.Start();
  //Serial.println("DEBUG: TaskTemp Started.");  // DEBUG
  TaskHumidity.Start();
  //Serial.println("DEBUG: TaskHumidity Started.");  // DEBUG
  TaskPMV.Start();
  //Serial.println("DEBUG: TaskPMV Started.");  // DEBUG


  // Restablecer valores de sensores para evitar lecturas "viejas"
  //luzValue = analogRead(PIN_LUZ); // Leer la luz inmediatamente al entrar (puede ser 0 al principio)
  //tempValue = NAN; // Forzar a DHT a leer desde cero en su próxima ejecución
  //humidityValue = NAN; // Forzar a DHT a leer desde cero en su próxima ejecución
  //pmvValue = NAN; // Forzar a PMV a recalcular


  monitoreoEnterTime = millis();                                                  // Registrar el tiempo de entrada
  sensorsStable = false;                                                          // Inicializar como inestable
  //Serial.println("DEBUG: Monitoreo entered. Sensors not yet stable for alarm.");  // DEBUG
}




void onExitMonitoreo() {
  Serial.println("<- Saliendo de Monitoreo");
  digitalWrite(PIN_LED_GREEN, LOW);  // Apaga el LED verde al salir
}




void onEnterBloqueado() {
  lcd.clear();
  lcd.print("Sistema bloqueado");
  lcd.setCursor(0, 1);
  lcd.print("Presione #");
  Serial.println("-> Estado Bloqueado: Sistema bloqueado. Presione #");
  bloqueo_led_millis = millis();
}





void onExitBloqueado() {
  Serial.println("<- Saliendo de Bloqueado");
  digitalWrite(PIN_LED_RED, LOW);  // Asegurarse que el LED esté apagado
  pass_intentos_incorrectos = 0;   // Resetear intentos al salir de Bloqueado
}





void onEnterAlarma() {
  lcd.clear();
  lcd.print("!!! ALARMA !!!");
  Serial.println("-> Estado ALARMA: !!! ALARMA !!!");
  alarmaStartTime = millis();
  alarm_entry_count++;
  Serial.print("Alarmas activadas: ");
  Serial.println(alarm_entry_count);
  Serial.println("DEBUG ALARMA: Iniciando TaskBuzzerAlarma.Start() y TaskLedAlarma.Start()"); // <-- ¡AÑADE ESTO!
  TaskBuzzerAlarma.Start();
  TaskLedAlarma.Start(); // El LED rojo parpadea
}




void onExitAlarma() {
  Serial.println("<- Saliendo de Alarma");
  noTone(BUZZER_PIN);       // Apagar buzzer
  TaskBuzzerAlarma.Stop();  // Detener la tarea del buzzer
  // Eliminado: digitalWrite(PIN_LED_RED, LOW); // Esto lo manejará TaskLedAlarma al detenerse
  TaskLedAlarma.Stop();  // Detener la tarea del LED de alarma (asegurará que el LED se apague)
  TaskLuz.Start();
  TaskTemp.Start();
  TaskHumidity.Start();
  TaskPMV.Start();
}
















void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  dht.begin();

  // Force initial sensor reads to populate values, mitigating initial NANs
  tempValue = dht.readTemperature();
  humidityValue = dht.readHumidity();
  // Add a small delay after initial reads for sensor stabilization if needed
  if (isnan(tempValue) || isnan(humidityValue)) {
      Serial.println("Initial DHT read failed or returned NAN. Will retry in loop.");
  } else {
      Serial.println("Initial DHT read successful.");
  }

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

  SPI.begin();       // Iniciar SPI
  mfrc522.PCD_Init(); // Iniciar MFRC522

  setupStateMachine();
  stateMachine.SetState(INIT, false, true);
}
















void loop() {
  // Update sensor tasks
  TaskLuz.Update();
  TaskTemp.Update();
  TaskHumidity.Update();
  TaskPMV.Update(); // This is where calculatePMV() runs and might set currentInput
  readRFID();       // This is where readRFID() runs and might set currentInput

  // Update alarm and actuator tasks
  TaskBuzzerAlarma.Update();
  TaskLedAlarma.Update();
  TaskVentilador.Update();

  // Handle keypad input: Only set currentInput if it's not already set by PMV/RFID.
  // This prevents keypad input from immediately overwriting PMV_ALTO_DETECTED/PMV_BAJO_DETECTED.
  char key = keypad.getKey(); // Read key once per loop
  Input keypadInput = NINGUNA;
  if (key) { // If a key is pressed
    if (stateMachine.GetState() == INIT) {
      // ... (your existing getInput() logic for INIT state, but extract it here) ...
      // Example:
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

  // Assign keypadInput to currentInput ONLY if currentInput wasn't set by PMV/RFID this cycle.
  // Or, if the currentInput was from PMV/RFID, but the state machine already transitioned.
  if (currentInput == NINGUNA ||
      (currentInput == PMV_ALTO_DETECTED && stateMachine.GetState() != MONITOREO_AMBIENTAL) || // If we've already left monitoreo due to PMV
      (currentInput == PMV_BAJO_DETECTED && stateMachine.GetState() != MONITOREO_AMBIENTAL) ) {
      currentInput = keypadInput;
  }
  // The line `currentInput = NINGUNA;` at the beginning of loop was problematic, remove it.

  // Update state machine (evaluates transitions, including the forced alarm in handleMonitoreo)
  stateMachine.Update();

  // Execute state-specific logic (this is where handleMonitoreo is called)
  switch (stateMachine.GetState()) {
    case INIT:
      handleInit();
      break;
    case MONITOREO_AMBIENTAL:
      handleMonitoreo(); // Alarm logic (SetState) is inside here
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

  // --- IMPORTANT: Reset currentInput *after* stateMachine.Update() ---
  // Reset `currentInput` at the end of the loop, but only if it's an input that
  // signifies a 'one-shot' event (like CLAVE_CORRECTA, etc.).
  // PMV_ALTO_DETECTED/PMV_BAJO_DETECTED should only be reset if the PMV is no longer high/low
  // and the system is not in the PMV_ALTO/BAJO state.
  // The `calculatePMV()` function and `readRFID()` set `currentInput` based on current conditions.
  // The state machine handles transitions out of PMV_ALTO/BAJO when `currentInput` changes.

  // Reset these "trigger" inputs after the state machine has had a chance to use them.
  if (keypadInput != NINGUNA) { // Only reset keypad inputs once they've been processed.
      currentInput = NINGUNA;
  }


  // If the state is not PMV_ALTO or PMV_BAJO, and currentInput is still a PMV-related detection, reset it.
  // This helps ensure `currentInput` is `NINGUNA` when not actively in a PMV-driven state.
  if (stateMachine.GetState() != PMV_ALTO && stateMachine.GetState() != PMV_BAJO &&
      (currentInput == PMV_ALTO_DETECTED || currentInput == PMV_BAJO_DETECTED)) {
      currentInput = NINGUNA;
  }

  delay(50); // Keep this delay for stability
}

// Remove the separate getInput() function and integrate its logic directly into loop() for better control.
// If you want to keep getInput() as a separate function for clarity, then change its return type to char
// and let loop() handle the `currentInput` assignment based on `keypadInput`.
// For simplicity and direct control, I've integrated it above.
















// Obtener entrada del sistema (teclado)
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
          //delay(2000);
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
  // Las condiciones de ALARMA son manejadas directamente en las transiciones
  // de la máquina de estados, no necesitan un 'Input' aquí.
  return NINGUNA;
}
















// Manejadores de estado (contienen la lógica que se ejecuta mientras el sistema está en ese estado)
void handleInit() {
  if (currentInput == CLAVE_CORRECTA) {
    lcd.clear();
    lcd.print("Acceso concedido");
    Serial.println("Acceso concedido");
    //delay(2000);  // Muestra el mensaje por 2 segundos
  } else if (currentInput == CLAVE_INCORRECTA) {
    lcd.clear();
    lcd.print("Clave incorrecta");
    Serial.print("Clave incorrecta. Intentos: ");
    Serial.println(pass_intentos_incorrectos);
    encenderLedSeguridad(PIN_LED_BLUE);  // Parpadeo azul
    //delay(1000);                         // Muestra el mensaje por 1 segundo
    pass_idx = 0;
    memset(user_input, 0, sizeof(user_input));
    lcd.clear();
    lcd.print("Ingrese clave:");
    Serial.println("Ingrese clave:");
  }
}


void handleMonitoreo() {
  static unsigned long lastUpdate = 0;

  // Verificar si los sensores han tenido suficiente tiempo para estabilizarse
  if (!sensorsStable && (millis() - monitoreoEnterTime >= SENSOR_STABILIZE_TIME)) {
      sensorsStable = true;
      Serial.println("DEBUG: Sensors are now considered stable for alarm evaluation.");
  }

  // --- NUEVA LÓGICA DE PRIORIDAD PARA LA ALARMA ---
  // Si los sensores están estables Y hay una condición de alarma (luz baja O temp alta)
  if (sensorsStable && (luzValue < 100 || tempValue > 40.0)) {
    // Si NO estamos ya en el estado ALARMA, forzamos la transición a ALARMA.
    // Esto sobrescribe cualquier otra potencial transición a PMV_ALTO/BAJO.
    if (stateMachine.GetState() != ALARMA) {
        // Directamente cambiamos el estado, bypassando las transiciones si es necesario.
        // La librería StateMachineLib permite cambiar el estado directamente con SetState.
        Serial.println("DEBUG: ALARMA CONDITION MET! Forcing state change to ALARMA.");
        stateMachine.SetState(ALARMA, true, true); // (Estado, ejecutar onExit del estado actual, ejecutar onEnter del nuevo estado) // true para ejecutar onEnterAlarma()
        return; // Salimos de handleMonitoreo para que la máquina de estados procese el nuevo estado.
    }
  }
  // --- FIN NUEVA LÓGICA DE PRIORIDAD PARA LA ALARMA ---


  if (millis() - lastUpdate > 1000) { // Actualiza LCD cada segundo
    lastUpdate = millis();

    lcd.clear();
    lcd.setCursor(0, 0);

    // Line 1: Light and Temperature
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

    // Line 2: Humidity and PMV
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

    // Mostrar en Serial (always show all data)
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


void handleBloqueado() {
  // La función parpadearLedBloqueado() se llama en el loop() principal
}

void handleAlarma() {
  // No hay lógica continua aquí más allá de lo que hacen las tareas AsyncTask (buzzer y LED)
  // La salida de la alarma se maneja por la máquina de estados con el temporizador y el contador.
}
















// Funciones de sensores
void readLuz() {
  luzValue = analogRead(PIN_LUZ);
}


void readTemp() {
  float t = dht.readTemperature();
  // Check if the reading is valid *before* assigning to tempValue
  if (isnan(t)) {
    Serial.println("DHT11: Failed to read temperature!");
    tempValue = NAN;  // Ensure tempValue is NAN if reading failed
  } else {
    tempValue = t;
    //Serial.print("DHT11: Temperature = ");
    //Serial.print(tempValue);
    //Serial.println(" *C");
  }
}




void readHumidity() {
  float h = dht.readHumidity();
  // Check if the reading is valid *before* assigning to humidityValue
  if (isnan(h)) {
    Serial.println("DHT11: Failed to read humidity!");
    humidityValue = NAN;  // Ensure humidityValue is NAN if reading failed
  } else {
    humidityValue = h;
    //Serial.print("DHT11: Humidity = ");
    //Serial.print(humidityValue);
    //Serial.println(" %");
  }
}








void calculatePMV() {

  // La lectura RFID ahora tiene prioridad para establecer el currentInput si una tarjeta es detectada
  // Solo calculamos PMV basado en sensores si no hay una tarjeta detectada.
  if (currentInput == PMV_ALTO_DETECTED || currentInput == PMV_BAJO_DETECTED) {
    // Si ya se detectó una tarjeta RFID que cambia el PMV, no calculamos por sensores
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
  const float MAX_PMV_MATCH_DISTANCE = 5.0;  // Define your max distance threshold


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
    //Serial.print("PMV: Calculated PMV: ");
    //Serial.println(pmvValue, 2);
    //Serial.print("DEBUG PMV Value: ");
    //Serial.println(pmvValue, 2);
    //Serial.print("DEBUG Current State: ");
    //Serial.println(stateMachine.GetState());


    if (pmvValue > 1.0) {  // Umbral para PMV ALTO
      if (stateMachine.GetState() == MONITOREO_AMBIENTAL) {
        currentInput = PMV_ALTO_DETECTED;
        //Serial.println("DEBUG: Setting currentInput to PMV_ALTO_DETECTED!");
      }
    } else {  // PMV is not high (means it's <= 1.0)
      if (stateMachine.GetState() == PMV_ALTO) {
        currentInput = NINGUNA;
        Serial.println("DEBUG: PMV no longer ALTO, setting currentInput to NINGUNA.");
      } else {
        currentInput = NINGUNA;
      }
    }
  } else {
    pmvValue = NAN;
    Serial.println("PMV: No close match or out of range, PMV set to NAN.");
    if (stateMachine.GetState() == PMV_ALTO) {
      currentInput = NINGUNA;
      Serial.println("DEBUG: PMV is NAN, attempting to exit PMV_ALTO state.");
    }
  }
  //Serial.print("DEBUG: currentInput at end of calculatePMV(): ");
  //Serial.println(currentInput);
  //Serial.println("--- Exiting calculatePMV() ---");
}


void readRFID() {
  // Solo intentar leer RFID si estamos en el estado MONITOREO_AMBIENTAL
  if (stateMachine.GetState() != MONITOREO_AMBIENTAL) {
    return;
  }
  //Serial.println("DEBUG: readRFID() is active in MONITOREO_AMBIENTAL."); // <-- AÑADE ESTA LÍNEA

  if (mfrc522.PICC_IsNewCardPresent()) {
    Serial.println("DEBUG: RFID New Card Present.");  // <-- AÑADE ESTA LÍNEA
    if (mfrc522.PICC_ReadCardSerial()) {
      Serial.print(F("Card UID:"));
      printArray(mfrc522.uid.uidByte, mfrc522.uid.size);
      Serial.println();

      if (isEqualArray(mfrc522.uid.uidByte, tarjetaPMVBajo, 4)) {
        currentInput = PMV_BAJO_DETECTED;
        Serial.println("Tarjeta PMV Bajo detectada (RFID) - currentInput set.");  // <-- MODIFICA ESTA LÍNEA
      } else if (isEqualArray(mfrc522.uid.uidByte, llaveroPMVAlto, 4)) {
        currentInput = PMV_ALTO_DETECTED;
        Serial.println("Llavero PMV Alto detectado (RFID) - currentInput set.");  // <-- MODIFICA ESTA LÍNEA
      } else {
        Serial.println("Tarjeta desconocida (RFID).");
      }
      mfrc522.PICC_HaltA();
    }
  }
}

// Función para comparar dos arrays de bytes (UIDs)
bool isEqualArray(byte* arrayA, byte* arrayB, int length) {
  for (int index = 0; index < length; index++) {
    if (arrayA[index] != arrayB[index]) return false;
  }
  return true;
}

// Función para imprimir el UID de la tarjeta
void printArray(byte* buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}



// Funciones auxiliares
void parpadearLedBloqueado() {
  // Asegurarse de que esta función solo afecte el LED ROJO cuando está en el estado BLOQUEADO
  if (stateMachine.GetState() == State::BLOQUEADO) {
    unsigned long currentMillis = millis();
    long interval = bloqueo_led_state ? BLOQUEO_LED_ON_TIME : BLOQUEO_LED_OFF_TIME;
    if (currentMillis - bloqueo_led_millis >= interval) {
      bloqueo_led_millis = currentMillis;
      bloqueo_led_state = !bloqueo_led_state;
      digitalWrite(PIN_LED_RED, bloqueo_led_state);
    }
  } else {
    // Si NO estamos en BLOQUEADO, y el estado actual NO es ALARMA (donde TaskLedAlarma lo controla)
    // entonces apagamos el LED ROJO por seguridad.
    if (stateMachine.GetState() != State::ALARMA) {
      digitalWrite(PIN_LED_RED, LOW);
    }
  }
}
















void encenderLedSeguridad(int pin) {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
}
















// Función para la tarea del buzzer de alarma
void activarBuzzerAlarma() {
  // Asegurarse de que el buzzer solo esté activo en el estado ALARMA
  if (stateMachine.GetState() == State::ALARMA) {
    //Serial.println("DEBUG ALARMA: activandoBuzzerAlarma() está en estado ALARMA."); // Puede ser muy ruidoso en Serial
    unsigned long currentMillis = millis();

    // Controlar el patrón de frecuencia del buzzer
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
      Serial.print("DEBUG ALARMA: Tone activado en pin "); // <-- ¡AÑADE ESTO!
      Serial.print(BUZZER_PIN);
      Serial.print(" con frecuencia: ");
      Serial.println(buzzerFreq);
    }
  } else {
    noTone(BUZZER_PIN); // Asegurarse de que el buzzer esté apagado si no está en ALARMA
    //Serial.println("DEBUG ALARMA: noTone() ejecutado (no en estado ALARMA)."); // Puede ser muy ruidoso
  }
}








// NUEVO: Función para la tarea de parpadeo del LED de alarma
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
    digitalWrite(PIN_LED_RED, LOW);  // Asegurarse de que el LED esté apagado si no está en ALARMA
  }
}  //Esta bien por ahora:(


// --- NUEVAS Funciones de Entrada/Salida para PMV_ALTO ---
void onEnterPMVAlto() {
  //digitalWrite(PIN_VENTILADOR, HIGH);
  lcd.clear();
  lcd.print("PMV ALTO!");
  lcd.setCursor(0, 1);
  lcd.print("Ventilador ON");  // Mostrar en LCD
  Serial.println("-> Estado PMV ALTO! Activando ventilador.");




  if (stateMachine.GetState() != ALARMA) { // Solo si no está en alarma
    digitalWrite(PIN_LED_RED, HIGH); // LED rojo fijo
  }




  // --- ¡ACTIVAR EL VENTILADOR Y SU TAREA! ---
  digitalWrite(PIN_VENTILADOR, HIGH);  // Encender el ventilador
  TaskVentilador.Start();              // Iniciar la tarea que lo apagará después de 7 segundos
  delay(7000);
}




void onExitPMVAlto() {
  Serial.println("<- Saliendo de PMV Alto. Apagando ventilador (si estaba encendido).");
  digitalWrite(PIN_LED_RED, LOW);  // Apagar el LED rojo




  // --- ASEGURAR QUE EL VENTILADOR ESTÉ APAGADO ---
  // Detener la tarea (esto también la resetea para la próxima vez)
  digitalWrite(PIN_VENTILADOR, LOW);  // Asegurarse de apagar explícitamente el pin del ventilador
}




// --- Manejador de Estado PMV_ALTO ---
void handlePMVAlto() {
  // Lógica que se ejecuta mientras el sistema está en el estado PMV_ALTO.
  // Por ahora, no hay lógica compleja aquí, ya que las transiciones lo manejan.
  // Podrías mostrar el PMV actual en el LCD.
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {  // Update every second
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

  // NUEVO: Registrar el tiempo de entrada
  pmvBajoEnterTime = millis();
}

void onExitPMVBajo() {
  Serial.println("<- Saliendo de PMV Bajo.");
  digitalWrite(PIN_LED_BLUE, LOW);  // Apagar el LED AZUL
}

// --- Manejador de Estado PMV_BAJO ---
void handlePMVBajo() {
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {  // Actualizar cada segundo
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
