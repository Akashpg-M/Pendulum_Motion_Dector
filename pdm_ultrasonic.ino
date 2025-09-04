// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// // ====== Hardware Pins ======
// #define IR_SENSOR_PIN 4
// #define BUZZER_PIN 5
// #define SERVO_PIN 18

// // ====== Task Handles ======
// TaskHandle_t sensorTaskHandle = NULL;
// TaskHandle_t buzzerTaskHandle = NULL;
// TaskHandle_t servoTaskHandle = NULL;
// TaskHandle_t lcdTaskHandle = NULL;

// // ====== Servo ======
// Servo myServo;

// // ====== LCD ======
// LiquidCrystal_I2C lcd(0x27, 16, 2); // adjust address if needed

// // ====== Global Flags & Counters ======
// volatile int detectionCount = 0;    // raw IR detections
// volatile int swingCounter = 0;      // full swings
// volatile bool servoPosFlag = false; // false = safe side 1, true = safe side 2
// volatile bool buzzerFlag = false;   // trigger buzzer

// // ====== Task 1: IR Sensor Detection ======
// void sensorTask(void *pvParameters) {
//   pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
//   Serial.println("Sensor Task Started...");

//   while (1) {
//     if (digitalRead(IR_SENSOR_PIN) == LOW) {
//       detectionCount++;
//       Serial.println(detectionCount);

//       // Set buzzer and servo flags
//       buzzerFlag = true;
//       servoPosFlag = !servoPosFlag;

//       // Every 2 detections = 1 full swing
//       if (detectionCount % 2 == 0) {
//         swingCounter++;
//         if (swingCounter >= 25) swingCounter = 0; // reset after 25
//         Serial.print("Full Swing Count: ");
//         Serial.println(swingCounter);
//       }

//       // Wait until pendulum leaves sensor
//       while (digitalRead(IR_SENSOR_PIN) == LOW) {
//         vTaskDelay(pdMS_TO_TICKS(10));
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(50)); // check every 50ms
//   }
// }

// // ====== Task 2: Buzzer ======
// void buzzerTask(void *pvParameters) {
//   pinMode(BUZZER_PIN, OUTPUT);

//   while (1) {
//     if (buzzerFlag) {
//       digitalWrite(BUZZER_PIN, HIGH);
//       vTaskDelay(pdMS_TO_TICKS(100));
//       digitalWrite(BUZZER_PIN, LOW);
//       buzzerFlag = false; // reset flag
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// // ====== Task 3: Servo Motion ======
// void servoTask(void *pvParameters) {
//   myServo.attach(SERVO_PIN);
//   myServo.write(0); // initial safe angle

//   while (1) {
//     if (servoPosFlag) {
//       myServo.write(180); // move to safe side 2
//     } else {
//       myServo.write(0); // move to safe side 1
//     }
//     vTaskDelay(pdMS_TO_TICKS(50)); // small delay
//   }
// }

// // ====== Task 4: LCD Update ======
// void lcdTask(void *pvParameters) {
//   lcd.init();
//   lcd.backlight();
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Swing Count:");

//   while (1) {
//     lcd.setCursor(0, 1);
//     lcd.print("                "); // clear previous number
//     lcd.setCursor(0, 1);
//     lcd.print(swingCounter);     // display current swing count
//     vTaskDelay(pdMS_TO_TICKS(200)); // update every 200ms
//   }
// }

// // ====== Setup ======
// void setup() {
//   Serial.begin(115200);
//   Serial.println("Pendulum Detection System Starting...");

//   myServo.attach(SERVO_PIN);

//   // Create FreeRTOS tasks
//   xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 1, &sensorTaskHandle, 1);
//   xTaskCreatePinnedToCore(buzzerTask, "Buzzer Task", 1024, NULL, 1, &buzzerTaskHandle, 0);
//   xTaskCreatePinnedToCore(servoTask, "Servo Task", 2048, NULL, 1, &servoTaskHandle, 1);
//   xTaskCreatePinnedToCore(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle, 1);
// }

// // ====== Loop ======
// void loop() {
//   // Empty, FreeRTOS tasks handle everything
// }


////////////////////////////////////////////////////////Full Working For IR Sensor
// #include <Arduino.h>
// #include <ESP32Servo.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

// // ====== Hardware Pins ======
// #define IR_SENSOR_PIN 4
// #define BUZZER_PIN 5
// #define SERVO_PIN 18

// // ====== Task Handles ======
// TaskHandle_t sensorTaskHandle = NULL;
// TaskHandle_t buzzerTaskHandle = NULL;
// TaskHandle_t servoTaskHandle = NULL;
// TaskHandle_t lcdTaskHandle = NULL;

// // ====== Servo ======
// Servo myServo;

// // ====== LCD ======
// LiquidCrystal_I2C lcd(0x27, 16, 2); // adjust address if needed

// // ====== Global Flags & Counters ======
// volatile int detectionCount = 0;    // raw IR detections
// volatile int swingCounter = 0;      // full swings
// volatile bool servoPosFlag = false; // false = safe side 1, true = safe side 2
// volatile bool buzzerFlag = false;   // trigger buzzer

// // ====== Task 1: IR Sensor Detection ======
// void sensorTask(void *pvParameters) {
//   pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
//   Serial.println("Sensor Task Started...");

//   while (1) {
//     if (digitalRead(IR_SENSOR_PIN) == LOW) {
//       detectionCount++;
//       Serial.println(detectionCount);

//       // Set buzzer and servo flags
//       buzzerFlag = true;
//       servoPosFlag = !servoPosFlag;

//       // Every 2 detections = 1 full swing
//       // if (detectionCount % 2 == 0) {
//         swingCounter++;
//         if (swingCounter > 25) swingCounter = 0; // reset after 25
//         Serial.print("Full Swing Count: ");
//         Serial.println(swingCounter);
//       // }

//       // Wait until pendulum leaves sensor
//       while (digitalRead(IR_SENSOR_PIN) == LOW) {
//         vTaskDelay(pdMS_TO_TICKS(10));
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(50)); // check every 50ms
//   }
// }

// // ====== Task 2: Buzzer ======
// void buzzerTask(void *pvParameters) {
//   pinMode(BUZZER_PIN, OUTPUT);

//   while (1) {
//     if (buzzerFlag) {
//       digitalWrite(BUZZER_PIN, HIGH);
//       vTaskDelay(pdMS_TO_TICKS(100));
//       digitalWrite(BUZZER_PIN, LOW);
//       buzzerFlag = false; // reset flag
//     }
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// // ====== Task 3: Servo Motion ======
// void servoTask(void *pvParameters) {
//   myServo.attach(SERVO_PIN);
//   myServo.write(0); // initial safe angle

//   while (1) {
//     if (servoPosFlag) {
//       myServo.write(180); // move to safe side 2
//     } else {
//       myServo.write(0); // move to safe side 1
//     }
//     vTaskDelay(pdMS_TO_TICKS(50)); // small delay
//   }
// }

// // ====== Task 4: LCD Update ======
// void lcdTask(void *pvParameters) {
//   lcd.init();
//   lcd.backlight();
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Swing Count:");

//   while (1) {
//     lcd.setCursor(0, 1);
//     lcd.print("                "); // clear previous number
//     lcd.setCursor(0, 1);
//     lcd.print(swingCounter);     // display current swing count
//     vTaskDelay(pdMS_TO_TICKS(200)); // update every 200ms
//   }
// }

// // ====== Setup ======
// void setup() {
//   Serial.begin(115200);
//   Serial.println("Pendulum Detection System Starting...");

//   myServo.attach(SERVO_PIN);

//   // Create FreeRTOS tasks
//   xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 1, &sensorTaskHandle, 1);
//   xTaskCreatePinnedToCore(buzzerTask, "Buzzer Task", 1024, NULL, 1, &buzzerTaskHandle, 0);
//   xTaskCreatePinnedToCore(servoTask, "Servo Task", 2048, NULL, 1, &servoTaskHandle, 1);
//   xTaskCreatePinnedToCore(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle, 1);
// }

// // ====== Loop ======
// void loop() {
//   // Empty, FreeRTOS tasks handle everything
// }


/////////////////////////////////////////////testing ur sensor

// #include <Arduino.h>

// // ====== Hardware Pins ======
// #define ULTRASONIC_TRIG_PIN 23
// #define ULTRASONIC_ECHO_PIN 19

// // ====== Function to measure distance ======
// long readUltrasonicDistance() {
//   digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

//   long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000); // timeout 30ms
//   long distance = duration * 0.034 / 2; // cm
//   return distance;
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
//   pinMode(ULTRASONIC_ECHO_PIN, INPUT);
//   Serial.println("Ultrasonic Sensor Test Started...");
// }

// void loop() {
//   long distance = readUltrasonicDistance();
//   if (distance > 0) {
//     Serial.print("Distance: ");
//     Serial.print(distance);
//     Serial.println(" cm");
//   } else {
//     Serial.println("Out of range");
//   }
//   delay(500); // read every 0.5 seconds
// }

////////////////////////////////////////////////////////////////ur

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ====== Hardware Pins ======
#define ULTRASONIC_TRIG_PIN 23
#define ULTRASONIC_ECHO_PIN 19
#define BUZZER_PIN 5
#define SERVO_PIN 18

// ====== FreeRTOS Task Handles ======
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t buzzerTaskHandle = NULL;
TaskHandle_t servoTaskHandle = NULL;
TaskHandle_t lcdTaskHandle = NULL;

// ====== Servo ======
Servo myServo;

// ====== LCD ======
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ====== Global Flags & Counters ======
volatile int detectionCount = 0;    // raw detections
volatile int swingCounter = 0;      // full swings
volatile bool servoPosFlag = false; // false = safe side 1, true = safe side 2
volatile bool buzzerFlag = false;   // trigger buzzer

// Thresholds
#define PENDULUM_THRESHOLD 60   // distance in cm to detect pendulum
#define MAX_DISTANCE 500        // max distance to consider

// ====== Function to measure distance ======
long readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000); // timeout 30ms
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

// ====== Task 1: Ultrasonic Sensor Detection ======
void sensorTask(void *pvParameters) {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  Serial.println("Sensor Task Started...");

  while (1) {
    long distance = readUltrasonicDistance();

    // Only detect pendulum if within threshold
    if (distance > 0 && distance <= PENDULUM_THRESHOLD) {
      detectionCount++;
      Serial.print("Detection #: ");
      Serial.println(detectionCount);

      // Trigger buzzer and servo
      buzzerFlag = true;
      servoPosFlag = !servoPosFlag;

      // Count full swings
      swingCounter++;
      if (swingCounter > 25) swingCounter = 0; // reset after 25
      Serial.print("Full Swing Count: ");
      Serial.println(swingCounter);
      

      // Wait until pendulum moves away (avoid multiple detections)
      while (readUltrasonicDistance() <= PENDULUM_THRESHOLD) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ====== Task 2: Buzzer ======
void buzzerTask(void *pvParameters) {
  pinMode(BUZZER_PIN, OUTPUT);

  while (1) {
    if (buzzerFlag) {
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(BUZZER_PIN, LOW);
      buzzerFlag = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====== Task 3: Servo Motion ======
void servoTask(void *pvParameters) {
  myServo.attach(SERVO_PIN);
  myServo.write(0); // initial safe angle

  while (1) {
    if (servoPosFlag) {
      myServo.write(180); // move to safe side 2
    } else {
      myServo.write(0);   // move to safe side 1
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ====== Task 4: LCD Update ======
void lcdTask(void *pvParameters) {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Swing Count:");

  while (1) {
    lcd.setCursor(0, 1);
    lcd.print("    "); // clear previous number
    lcd.setCursor(0, 1);
    lcd.print(swingCounter);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  Serial.println("Pendulum Detection System Starting...");

  myServo.attach(SERVO_PIN);

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 2048, NULL, 1, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(buzzerTask, "Buzzer Task", 1024, NULL, 1, &buzzerTaskHandle, 0);
  xTaskCreatePinnedToCore(servoTask, "Servo Task", 2048, NULL, 1, &servoTaskHandle, 1);
  xTaskCreatePinnedToCore(lcdTask, "LCD Task", 2048, NULL, 1, &lcdTaskHandle, 1);
}

// ====== Loop ======
void loop() {
  // Nothing here; FreeRTOS handles tasks
}
