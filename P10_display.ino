#include <SPI.h>
#include <DMD.h>
#include <TimerOne.h>
#include "Arial14.h"

#define PANELS_WIDTH 3
#define PANELS_HEIGHT 1
#define FONT Arial_14

DMD led_module(PANELS_WIDTH, PANELS_HEIGHT);

char currentSpeed[8] = "0";
char currentETA[8] = "0";
bool speedReceived = false;
bool isClear = true;
bool isCautionActive = false; // NEW FEATURE

bool showSlow = false;
unsigned long slowStart = 0;
const int slowDuration = 1200; 

char serialBuf[32];
byte serialIdx = 0;

unsigned long lastPacketTime = 0;
const unsigned long TIMEOUT_THRESHOLD = 8000;

void scan_module() { led_module.scanDisplayBySPI(); }

void drawCentered(const char *text) {
  led_module.clearScreen(true);
  int screenWidth = 32 * PANELS_WIDTH;
  int totalWidth = strlen(text) * 9; 
  int x = (screenWidth - totalWidth) / 2;
  if (x < 0) x = 0;
  led_module.drawString(x, 1, text, strlen(text), GRAPHICS_NORMAL);
}

void displayInfo() {
  char msg[32];
  snprintf(msg, sizeof(msg), "%skm/h ETA:%ss", currentSpeed, currentETA);
  drawCentered(msg);
}

void setup() {
  Timer1.initialize(2000);
  Timer1.attachInterrupt(scan_module);
  Serial.begin(9600);
  led_module.selectFont(FONT);
  drawCentered("GO AHEAD");
  lastPacketTime = millis();
}

void handleCommand(char *cmd) {
  lastPacketTime = millis();

  // 1. CAUTION TRIGGER (New)
  if (!strcmp(cmd, "CAUTION")) {
    isCautionActive = true;
    isClear = false;
    showSlow = false; 
    drawCentered("CAUTION");
    return;
  }

  // 2. GREEN RESET
  if (!strcmp(cmd, "GO AHEAD")) {
    isClear = true;
    isCautionActive = false;
    showSlow = false;
    speedReceived = false;
    drawCentered("GO AHEAD");
    return;
  }

  // 3. SLOW DOWN TRIGGER
  if (!strcmp(cmd, "SLOW DOWN") || !strcmp(cmd, "KEEP_RED")) {
    if (isClear && !isCautionActive) {
      isClear = false;
      showSlow = true;
      slowStart = millis();
      drawCentered("SLOW DOWN");
    }
    return;
  }

  // 4. SPEED DATA
  if (cmd[0] == 'S' && cmd[1] == ':') {
    strncpy(currentSpeed, cmd + 2, sizeof(currentSpeed) - 1);
    currentSpeed[sizeof(currentSpeed) - 1] = '\0';
    speedReceived = true;
  }

  // 5. ETA DATA
  if (cmd[0] == 'E' && cmd[1] == ':') {
    strncpy(currentETA, cmd + 2, sizeof(currentETA) - 1);
    currentETA[sizeof(currentETA) - 1] = '\0';
    
    //Speed shows even if light is RED
    if (!showSlow) {
      displayInfo(); 
    }
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuf[serialIdx] = '\0';
      handleCommand(serialBuf);
      serialIdx = 0;
    } else if (serialIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialIdx++] = c;
    }
  }

 if (showSlow && millis() - slowStart > slowDuration) {
    showSlow = false;
    if (speedReceived) { // Now it shows Speed/ETA regardless of light color
        displayInfo();
    }
  }

  if (!isClear && millis() - lastPacketTime > TIMEOUT_THRESHOLD) {
    isClear = true;
    isCautionActive = false;
    speedReceived = false;
    drawCentered("GO AHEAD");
  }
}
