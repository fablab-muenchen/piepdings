#include <avr/power.h>
#include <avr/sleep.h>

#define LED1  9 //PB0
#define LED2 11 //PB2
#define LED3 14 //PB5
#define LED4 16 //PB7

#define BUTTON1 10 //PB1
#define BUTTON2 12 //PB3
#define BUTTON3 13 //PB4
#define BUTTON4 15 //PB6

#define BUZZER1 6
#define BUZZER2 7

#define TONE1 880
#define TONE2 1760
#define TONE3 2000
#define TONE4 3000

#define I0_PIN 4

#define ENTRY_TIME_LIMIT 3000

bool soundEnabled = true;

void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  pinMode(I0_PIN, INPUT);

  if (digitalRead(BUTTON1) == LOW) {
    soundEnabled = false;
  }
}

void loop() {
  //go to sleep
  sleep_enable(); // set safety pin to allow cpu sleep
  attachInterrupt(0, intrpt, LOW); // attach interrupt 0 (pin 2) and run function intrpt when pin 2 gets LOW
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // set sleep mode to have most power savings
  cli(); // disable interrupts during timed sequence
  sei(); // set Global Interrupt Enable
  sleep_cpu(); // power down cpu
  
  //wake up here
  sleep_disable(); // set safety pin to NOT allow cpu sleep
  detachInterrupt(0); // detach interrupt to allow other usage of this pin

  //play game
  play_all_tones();
  
  int difficulty = 6;
  while (one_round(difficulty)) {

    //TODO Show difficulty here
    
    difficulty++;

    //C5 E5 G5 C6
    digitalWrite(LED2, HIGH);
    play_tone(10, 200);
    play_tone(11, 200);
    play_tone(12, 200);
    play_tone(13, 200);
    digitalWrite(LED2, LOW);
  }

  //C6 G5 C5
  digitalWrite(LED1, HIGH);
  play_tone(13, 200);
  play_tone(12, 200);
  play_tone(10, 400);
  digitalWrite(LED1, LOW);
}

void intrpt() {
  //do nothing here, just wake up
}

bool one_round(int difficulty) {
  difficulty = difficulty / 2;
  
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);
  digitalWrite(LED4, HIGH);
  delay(500);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  delay(500);
  
  int rnd[difficulty];

  //play tone sequence
  for (int i = 0; i < difficulty; i++) {
    rnd[i] = random(1, 5);

    digitalWrite(nr_to_led_pin(rnd[i]), HIGH);
    play_tone(rnd[i], 400);
    digitalWrite(nr_to_led_pin(rnd[i]), LOW);
    delay(100);
  }

  //wait for correct user input
  for (int i = 0; i < difficulty; i++) {
    int input_nr = wait_for_button();
    if (input_nr != rnd[i]) {
      return false;
    }
  }
  return true;
}

// plays all tones in a row and lights the corresponding LED
void play_all_tones() {
  digitalWrite(LED1, HIGH);
  play_tone(1, 400);
  digitalWrite(LED1, LOW);
  delay(100);

  digitalWrite(LED2, HIGH);
  play_tone(2, 400);
  digitalWrite(LED2, LOW);
  delay(100);

  digitalWrite(LED3, HIGH);
  play_tone(3, 400);
  digitalWrite(LED3, LOW);
  delay(100);

  digitalWrite(LED4, HIGH);
  play_tone(4, 400);
  digitalWrite(LED4, LOW);
  delay(100);
}

void play_tone(int tone_nr, int length_ms) {
  int tone_hz = tone_nr_to_hz(tone_nr);
  if (soundEnabled) {
    tone(BUZZER1, tone_hz);
  }
  delay(length_ms);
  noTone(BUZZER1);
}

int tone_nr_to_hz(int nr) {
  switch(nr) {
    case 1:
      return TONE1;
    case 2:
      return TONE2;
    case 3:
      return TONE3;
    case 4:
      return TONE4;

    //winning / loosing tones
    case 10:
      return 523;//C5
    case 11:
      return 659;//E5
    case 12:
      return 784;//G5
    case 13:
      return 1047;//C6
    default:
      return -1;
  }
}

int wait_for_button() {
  long start_time = millis();

  //wait a max of ENTRY_TIME_LIMIT for a button press
  while ((millis() - start_time) < ENTRY_TIME_LIMIT) {
    int button_nr = check_button();

    if (button_nr != -1) {//if a button is pressed
      digitalWrite(nr_to_led_pin(button_nr), HIGH);
      play_tone(button_nr, 150);
      digitalWrite(nr_to_led_pin(button_nr), LOW);

      while (check_button() != -1) {//wait for button release
        //do nothing
      }
      delay(10);//software debounce

      return button_nr;
    }
  }
}

int check_button() {
  if (digitalRead(BUTTON1) == 0) return 1; 
  else if (digitalRead(BUTTON2) == 0) return 2; 
  else if (digitalRead(BUTTON3) == 0) return 3; 
  else if (digitalRead(BUTTON4) == 0) return 4;

  return -1; // If no button is pressed, return -1
}

int nr_to_led_pin(int nr) {
  switch(nr) {
    case 1:
      return LED1;
    case 2:
      return LED2;
    case 3:
      return LED3;
    case 4:
      return LED4;
    default:
      return -1;
  }
}

