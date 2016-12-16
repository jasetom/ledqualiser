/*

Ledqualiser code for physical computing project by Tomas Jasevicius

Libraries required to run this project:
MAX72xx.h
SPI.h

This project is a audio visualiser, which takes in audio signal, analyses it and produces
visuals on the 8x8 LED matrix.

IC used to analyse audio is MSEQG7.
IC used to display visuals on 8x8 matrix is MAX721.

IC used to aplify audio signal and play it out loud is PAM8406 (no code is needed in order to use it).

Digital pin connection:
 pin 04 is connected to the Strobe (MSEQG7)
 pin 05 is connected to the Reset (MSEQG7)
 pin 06 is connected to the Button to control visuals displayed on the LEDS.
 pin 07 is connected to the Button to control visuals displayed on the LEDS.
 pin 11 is connected to the DataIn (MAX721)
 pin 13 is connected to the CLK (MAX721)
 pin 10 is connected to LOAD (MAX721)

Analog pin connection:
 pin A0 is connected to the ADUIO_OUT (MSEQG7)

*/

// Include MAX721 library
#include <MD_MAX72xx.h>
#if USE_LIBRARY_SPI
#include <SPI.h>
#endif

//Pins
// MAX721
#define	CLK_PIN		13
#define	DATA_PIN	11
#define	CS_PIN		10
// MSEQG7
#define STROBE_PIN      4
#define RST_PIN         5  //resetpin
#define DATA_IN         A0
// Button pins
#define BTN1_PIN        6
#define BTN2_PIN        7
//Defining other variables
#define DELAYTIME       50 //milliseconds
#define MAX_DEVICES     1  // how many max721 chips we have

int bands[7];              // store band values in this arrays
int btn1State = 0;         // variable for reading the pushbutton status
int btn2State = 0;
int animationState;        //variable to store overall animation states

int levels [9] = {0, 1, 3, 7, 15, 31, 63, 127, 255};

// Create MAX721 hardware object
MD_MAX72XX leds = MD_MAX72XX(CS_PIN, MAX_DEVICES);

// Read bands from MSEQG7 and store them in bands array
void readMSEQG7() {
  // Reset chip everytime to get a new reading.
  digitalWrite(RST_PIN, HIGH);
  digitalWrite(RST_PIN, LOW);

  // Loop throught bands in MSEQG7
  for (int i = 0; i < 7; i++) {
    digitalWrite(STROBE_PIN, LOW); // strobe pin- kicks the IC up to the next band
    delayMicroseconds(30);
    bands[i] = analogRead(DATA_IN); // store bands reading in the array
    digitalWrite(STROBE_PIN, HIGH);
  }
}

void welcomeAnimation() {
  // Adapted from MAX721 library example
  // Animation of a diagonal stripe moving across the display
  // with points plotted outside the display region ignored.
  const uint16_t maxCol = MAX_DEVICES * ROW_SIZE;
  const uint8_t	stripeWidth = 10;

  int milliseconds = 50;

  leds.clear();

  for (uint16_t col = 0; col < maxCol + ROW_SIZE + stripeWidth; col++) {
    for (uint8_t row = 0; row < ROW_SIZE; row++) {
      leds.setPoint(row, col - row, true);
      leds.setPoint(row, col - row - stripeWidth, false);
    }
    delay(milliseconds);
  }
}

void column() {

  leds.clear();

  for (int i = 0; i < 8; i++) {
    digitalWrite(STROBE_PIN, HIGH);
    delay(1);
    digitalWrite(STROBE_PIN, LOW);
    delay(1);
    leds.setColumn(0, i, levels[map(analogRead(DATA_IN), 50, 600, 0, 11)]);

    leds.update();
    leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);

  }
}


void bullseye() {
  //Adapted from MAX7xx library example
  // Demonstrate the use of buffer based repeated patterns
  // across all devices.

  leds.clear();
  leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  for (uint8_t n = 0; n < 3; n++) {
    byte  b = 0xff;
    int   i = 0;

    while (b != 0x00) {
      for (uint8_t j = 0; j < MAX_DEVICES + 1; j++) {
        leds.setRow(j, i, b);
        leds.setColumn(j, i, b);
        leds.setRow(j, ROW_SIZE - 1 - i, b);
        leds.setColumn(j, COL_SIZE - 1 - i, b);
      }
      leds.update();
      delay(3 * DELAYTIME);
      for (uint8_t j = 0; j < MAX_DEVICES + 1; j++) {
        leds.setRow(j, i, 0);
        leds.setColumn(j, i, 0);
        leds.setRow(j, ROW_SIZE - 1 - i, 0);
        leds.setColumn(j, COL_SIZE - 1 - i, 0);
      }

      bitClear(b, i);
      bitClear(b, 7 - i);
      i++;
    }

    while (b != 0xff) {
      for (uint8_t j = 0; j < MAX_DEVICES + 1; j++) {
        leds.setRow(j, i, b);
        leds.setColumn(j, i, b);
        leds.setRow(j, ROW_SIZE - 1 - i, b);
        leds.setColumn(j, COL_SIZE - 1 - i, b);
      }
      leds.update();
      delay(3 * DELAYTIME);
      for (uint8_t j = 0; j < MAX_DEVICES + 1; j++) {
        leds.setRow(j, i, 0);
        leds.setColumn(j, i, 0);
        leds.setRow(j, ROW_SIZE - 1 - i, 0);
        leds.setColumn(j, COL_SIZE - 1 - i, 0);
      }

      i--;
      bitSet(b, i);
      bitSet(b, 7 - i);
    }
  }

  leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void lines() {
  //Adapted from MAX7xx library example
  // Demonstrate the use of drawLine().
  // fan out lines from each corner for up to 4 device blocks

  const uint8_t stepSize = 3;
  const uint8_t maxDev = (MAX_DEVICES > 4 ? 4 : MAX_DEVICES);

  leds.clear();
  for (uint16_t c = 0; c < (maxDev * COL_SIZE) - 1; c += stepSize) {
    leds.drawLine(0, 0, ROW_SIZE - 1, c, true);
    delay(DELAYTIME);
  }

  leds.clear();
  for (uint16_t c = 0; c < (maxDev * COL_SIZE) - 1; c += stepSize) {
    leds.drawLine(ROW_SIZE - 1, 0, 0, c, true);
    delay(DELAYTIME);
  }

  leds.clear();
  for (uint16_t c = 0; c < (maxDev * COL_SIZE) - 1; c += stepSize) {
    leds.drawLine(ROW_SIZE - 1, (MAX_DEVICES * COL_SIZE) - 1, 0, (MAX_DEVICES * COL_SIZE) - 1 - c, true);
    delay(DELAYTIME);
  }

  leds.clear();
  for (uint16_t c = 0; c < (maxDev * COL_SIZE) - 1; c += stepSize) {
    leds.drawLine(0, (MAX_DEVICES * COL_SIZE) - 1, ROW_SIZE - 1, (MAX_DEVICES * COL_SIZE) - 1 - c, true);
    delay(DELAYTIME);
  }
}

void cross() {
  // Adapted from MAX72xx library examples
  // Combination of setRow() and setColumn()
  // display updates to ensure concurrent changes.

  leds.clear();
  leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  // diagonally down the display R to L
  for (uint8_t i = 0; i < ROW_SIZE; i++) {
    for (uint8_t j = 0; j < MAX_DEVICES; j++) {
      leds.setColumn(j, i, 0xff);
      leds.setRow(j, i, 0xff);
    }
    leds.update();
    delay(3 * DELAYTIME);
    for (uint8_t j = 0; j < MAX_DEVICES; j++) {
      leds.setColumn(j, i, 0x00);
      leds.setRow(j, i, 0x00);
    }
  }

  // moving up the display on the R
  for (int8_t i = ROW_SIZE - 1; i >= 0; i--) {
    for (uint8_t j = 0; j < MAX_DEVICES; j++) {
      leds.setColumn(j, i, 0xff);
      leds.setRow(j, ROW_SIZE - 1, 0xff);
    }
    leds.update();
    delay(3 * DELAYTIME);
    for (uint8_t j = 0; j < MAX_DEVICES; j++) {
      leds.setColumn(j, i, 0x00);
      leds.setRow(j, ROW_SIZE - 1, 0x00);
    }
  }

  // diagonally up the display L to R
  for (uint8_t i = 0; i < ROW_SIZE; i++) {
    for (uint8_t j = 0; j < MAX_DEVICES; j++) {
      leds.setColumn(j, i, 0xff);
      leds.setRow(j, ROW_SIZE - 1 - i, 0xff);
    }
    leds.update();
    delay(3 * DELAYTIME);
    for (uint8_t j = 0; j < MAX_DEVICES; j++) {
      leds.setColumn(j, i, 0x00);
      leds.setRow(j, ROW_SIZE - 1 - i, 0x00);
    }
  }
  leds.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

void readButtonsStates() {

  // Read button state
  btn1State = digitalRead(BTN1_PIN);
  btn2State = digitalRead(BTN2_PIN);

  // If both buttons pressed, clear the screen.
  if (btn2State == HIGH && btn1State == HIGH) {
    delay(300);
    animationState = 0;
  } else if (btn1State == HIGH && btn2State == LOW) {
    delay(200);
    animationState++;
    if (animationState >= 6) {
      animationState = 2;

    }
  } else if (btn2State == HIGH && btn1State == LOW) {
    delay(200);
    animationState--;
    if (animationState < 2) {
      animationState = 5;

    }
  }

}

void setup() {
  //setting up pinModes
  //buttons
  pinMode(BTN1_PIN, INPUT);
  pinMode(BTN2_PIN, INPUT);
  pinMode(DATA_IN, INPUT);

  //MAX721
  pinMode(RST_PIN, OUTPUT); // reset
  pinMode(STROBE_PIN, OUTPUT); // strobe

  //setting up MSEQG7
  digitalWrite(RST_PIN, LOW);
  digitalWrite(STROBE_PIN, HIGH);

  //starting up LEDS and Serial for communication
  leds.begin();
  Serial.begin(115200);

  //Welcome animation so that the user knows that ledqualiser is on.
  welcomeAnimation();
  //Set animationState to 0 so that the leds would stay off.
  animationState = 0;
  delay(1000);

}

void loop() {
  // Everytime we loop, we read the audio chip first.
  readMSEQG7();
  readButtonsStates();


  switch (animationState) {
    case 1:
      leds.clear();
      break;
    case 2:
      lines();
      break;
    case 3:
      column();
      break;
    case 4:
      bullseye();
      break;
    case 5:
      cross();
      break;
    default:
      leds.clear();
      break;
  }

  Serial.println(animationState);
}






