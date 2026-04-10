#include <AccelStepper.h>

// Piny - uprav podle svého zapojení
#define ROTACE_STEP   2
#define ROTACE_DIR    3
#define ROTACE_ENABLE 6

#define POJEZD_STEP   4
#define POJEZD_DIR    5
#define POJEZD_ENABLE 7

AccelStepper rotace(AccelStepper::DRIVER, ROTACE_STEP, ROTACE_DIR);
AccelStepper pojezd(AccelStepper::DRIVER, POJEZD_STEP, POJEZD_DIR);

bool rotaceDone = true;
bool pojezdDone = true;

void setup() {
  Serial.begin(115200);

  // Enable piny jako výstup
  pinMode(ROTACE_ENABLE, OUTPUT);
  pinMode(POJEZD_ENABLE, OUTPUT);

  // Výchozí stav - motory vypnuté
  disableMotor('R');
  disableMotor('Z');

  // Výchozí nastavení rotace
  rotace.setMaxSpeed(10000);
  rotace.setAcceleration(1000);

  // Výchozí nastavení pojezdu
  pojezd.setMaxSpeed(10000);
  pojezd.setAcceleration(1000);

  Serial.println("READY");
}

void loop() {
  rotace.run();
  pojezd.run();

  // Detekce dokončení pohybu
  if (!rotaceDone && rotace.distanceToGo() == 0) {
    rotaceDone = true;
    disableMotor('R');        // Vypni motor po dokončení
    Serial.println("DONE R");
  }
  if (!pojezdDone && pojezd.distanceToGo() == 0) {
    pojezdDone = true;
    disableMotor('Z');        // Vypni motor po dokončení
    Serial.println("DONE Z");
  }

  // Příjem příkazů ze sériové linky
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    parseCommand(cmd);
  }
}

// -------------------------------------------------------------------------
// Enable / Disable motorů
// -------------------------------------------------------------------------

void enableMotor(char osa) {
  if (osa == 'R') {
    digitalWrite(ROTACE_ENABLE, HIGH);  // HIGH = motor zapnutý (SON = OFF)
    Serial.println("ENABLE R");
  } else if (osa == 'Z') {
    digitalWrite(POJEZD_ENABLE, HIGH);
    Serial.println("ENABLE Z");
  }
}

void disableMotor(char osa) {
  if (osa == 'R') {
    digitalWrite(ROTACE_ENABLE, LOW);   // LOW = motor vypnutý (SON = ON)
    Serial.println("DISABLE R");
  } else if (osa == 'Z') {
    digitalWrite(POJEZD_ENABLE, LOW);
    Serial.println("DISABLE Z");
  }
}

// -------------------------------------------------------------------------
// Parser příkazů
// -------------------------------------------------------------------------

void parseCommand(String cmd) {
  if (cmd.length() < 2) return;

  char osa = cmd.charAt(0);  // R nebo Z
  char typ = cmd.charAt(1);  // +/- = move, S = speed, A = accel, X = stop, E = enable, D = disable

  // --- MOVE: R+1000 nebo Z-500 ---
  if (typ == '+' || typ == '-' || isDigit(typ)) {
    long kroky = cmd.substring(1).toInt();

    if (osa == 'R') {
      enableMotor('R');           // Zapni motor před pohybem
      rotace.move(kroky);
      rotaceDone = false;
      Serial.print("MOVING R ");
      Serial.println(kroky);
    } else if (osa == 'Z') {
      enableMotor('Z');
      pojezd.move(kroky);
      pojezdDone = false;
      Serial.print("MOVING Z ");
      Serial.println(kroky);
    } else {
      Serial.println("ERR unknown axis");
    }

  // --- SPEED: RS5000 nebo ZS3000 ---
  } else if (typ == 'S') {
    float speed = cmd.substring(2).toFloat();
    if (speed <= 0) { Serial.println("ERR invalid speed"); return; }

    if (osa == 'R') {
      rotace.setMaxSpeed(speed);
      Serial.print("SPEED R ");
      Serial.println(speed);
    } else if (osa == 'Z') {
      pojezd.setMaxSpeed(speed);
      Serial.print("SPEED Z ");
      Serial.println(speed);
    }

  // --- ACCELERATION: RA1000 nebo ZA500 ---
  } else if (typ == 'A') {
    float acc = cmd.substring(2).toFloat();
    if (acc <= 0) { Serial.println("ERR invalid acceleration"); return; }

    if (osa == 'R') {
      rotace.setAcceleration(acc);
      Serial.print("ACCEL R ");
      Serial.println(acc);
    } else if (osa == 'Z') {
      pojezd.setAcceleration(acc);
      Serial.print("ACCEL Z ");
      Serial.println(acc);
    }

  // --- EMERGENCY STOP: RX nebo ZX ---
  } else if (typ == 'X') {
    if (osa == 'R') {
      rotace.stop();
      rotaceDone = true;
      disableMotor('R');
      Serial.println("STOP R");
    } else if (osa == 'Z') {
      pojezd.stop();
      pojezdDone = true;
      disableMotor('Z');
      Serial.println("STOP Z");
    }

  // --- MANUÁLNÍ ENABLE: RE nebo ZE ---
  } else if (typ == 'E') {
    if (osa == 'R' || osa == 'Z') {
      enableMotor(osa);
    } else {
      Serial.println("ERR unknown axis");
    }

  // --- MANUÁLNÍ DISABLE: RD nebo ZD ---
  } else if (typ == 'D') {
    if (osa == 'R' || osa == 'Z') {
      disableMotor(osa);
    } else {
      Serial.println("ERR unknown axis");
    }

  } else {
    Serial.println("ERR unknown command");
  }
}