#define __DELAY_BACKWARD_COMPATIBLE__ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay_basic.h>


// ATMEL ATTINY2313
//
//                   +-\/-+
//      (D 17) PA2  1|    |20  VCC
// RX   (D  0) PD0  2|    |19  PB7 (D  16)
// TX   (D  1) PD1  3|    |18  PB6 (D  15)
//      (D  2) PA1  4|    |17  PB5 (D  14)
//      (D  3) PA0  5|    |16  PB4 (D  13)
// INT0 (D  4) PD2  6|    |15  PB3 (D  12)
// INT1 (D  5) PD3  7|    |14  PB2 (D  11)
//      (D  6) PD4  8|    |13  PB1 (D  10)
//      (D  7) PD5  9|    |12  PB0 (D  9)
//             GND 10|    |11  PD6 (D  8)
//                   +----+
// PCINT11=PD0

#define PORT_LED PORTB // if changed, change setup() too!
#define LED1    PB0
#define LED2    PB2
#define LED3    PB5
#define LED4    PB7

#define PORT_BUTTON PINB  // if changed, change setup() too!
#define BUTTON1 PB1
#define BUTTON2 PB3
#define BUTTON3 PB4
#define BUTTON4 PB6

#define PORT_BUZZER PORTD  // if changed, change setup() too!
#define PORT_PIN_BUZZER PIND
#define BUZZER1_PIN PD4
#define BUZZER2_PIN PD5

#define PORT_IO PIND  // if changed, change setup() too!
#define I0_PIN 4

#define TONE1 880
#define TONE2 1760
#define TONE3 2000
#define TONE4 3000


#define ENTRY_TIME_LIMIT_MS 3000
const uint16_t ENTRY_TIME_LIMIT_10MS = ENTRY_TIME_LIMIT_MS/10;

bool soundEnabled = true;

//////////////////////////////////////////////////////
// Helper functions

#define HIGH 0x1
#define LOW  0x0

void setOutputPin(volatile uint8_t* port, uint8_t pin, uint8_t val) {
    if (val) {
        *port |= (1 << pin);  // Set the pin high
    } 
    else {
        *port &= ~(1 << pin); // Set the pin low
    }
}

void digitalWriteLed(uint8_t pin, uint8_t val) {
    setOutputPin(&PORT_LED, pin, val);
}

uint8_t getInputPin(volatile uint8_t* port, uint8_t pin) {
    // Return pin state
    return (*port & (1 << pin)) ? 1 : 0;
}

uint8_t digitalReadButton(uint8_t pin) {
    return getInputPin(&PORT_BUTTON, pin);
}

const uint16_t count10ms = F_CPU / 4 / 100; // each _delay_loop_2 loop takes 4 CPU cycles

// delay factor * 10ms, 
inline void delay10ms(uint8_t factor) {
  for (uint8_t i=factor; i>0; i--)
    _delay_loop_2(count10ms);
}

ISR(TIMER1_COMPA_vect) {
    // Toggle buzzer pins
    PORT_PIN_BUZZER |= (1 << BUZZER1_PIN) | (1 << BUZZER2_PIN);
}

void tone(uint16_t frequency) {
    // Calculate the toggle count based on the desired frequency
    // and clock prescaler of 8 (CS11)
    uint16_t toggle_count = (F_CPU / 8) / frequency - 1;

    // Set up Timer1 in CTC mode
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    OCR1A = toggle_count;
    TIMSK |= (1 << OCIE1A); // enable timer compare interrupt
}

void noTone() {
    // Disable the Timer1 output
    TCCR1A = 0;
    TCCR1B = 0;
}

void setupTimer0() {
    // Set Timer0 to normal mode
    TCCR0A = 0;
    // Enable Timer0 overflow interrupt
    TIMSK |= (1 << TOIE0);
    // Set prescaler to 1:1 mode
    TCCR0B = (1 << CS00);
}

volatile static uint8_t overflowCounter = 0;

// ISR for Timer0 Overflow
ISR(TIMER0_OVF_vect) {
    overflowCounter++;  // Increment the overflow counter every time Timer0 overflows
}

static uint8_t counterLO = 0xAB;
static uint8_t counterHI = 0x89;
static uint8_t bitLO = 0;
static uint8_t bitHI = 0;
static uint8_t sequenceNumber = 0;

void initRandomFromClock() {
    counterLO = TCNT0 ^ 0xAB;
    counterHI = overflowCounter ^ 0x89;
    bitLO = 0;
    bitHI = 0;
    sequenceNumber = 0;
}

// get a 2-bit random number (0-3)
uint8_t getRandom(void) {
    // get one bit from counterLO and one from counterHI to form a 2-bit number
    uint8_t random = ((counterLO & (1 << bitLO)) ? 1 : 0) | 
                     ((counterHI & (1 << bitHI)) ? 2 : 0);
    // move to next bit position for next random number
    bitLO++;
    bitHI++;
    if (++sequenceNumber >= 8) {
        // after 8 runs, there are no more unused bits
        // in case really needed, resort to combining the bits differently  
        bitLO += 2;
        bitHI += 5;
        sequenceNumber = 0;
    }
    bitLO &= 0b0111; // modulo 8
    bitHI &= 0b0111; // modulo 8

    return random;
}


///////////////////////////


void setup() {
    // Configure pins for output 
    DDRB |= (1 << LED1) | (1 << LED2) | (1 << LED3) | (1 << LED4);
    DDRD |= (1 << BUZZER1_PIN) | (1 << BUZZER2_PIN);
    // and input
    DDRB &= ~((1 << BUTTON1) | (1 << BUTTON2) | (1 << BUTTON3) | (1 << BUTTON4));
    DDRD &= ~(1 << I0_PIN);
    
    // Enable pull-ups for button pins 
    PORTB |= (1 << BUTTON1) | (1 << BUTTON2) | (1 << BUTTON3) | (1 << BUTTON4);
    
    setOutputPin(&PORT_BUZZER, BUZZER1_PIN, LOW);
    setOutputPin(&PORT_BUZZER, BUZZER2_PIN, HIGH);

    // Check if BUTTON1 is pressed, and if so, disable sound
    if (digitalReadButton(BUTTON1) == 0) {
        soundEnabled = false;
    }
}

const uint8_t nr_to_led_pin[] = {0, LED1, LED2, LED3, LED4};


int tone_nr_to_hz(uint8_t nr) {
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


 
void play_tone(uint8_t tone_nr, uint8_t times10ms) {
  int tone_hz = tone_nr_to_hz(tone_nr);
  if (soundEnabled) {
    tone(tone_hz);
  }
  
  delay10ms(times10ms);
  noTone();
}

// plays all tones in a row and lights the corresponding LED
void play_all_tones() {
  digitalWriteLed(LED1, HIGH);
  play_tone(1, 40);
  digitalWriteLed(LED1, LOW);
  delay10ms(10);

  digitalWriteLed(LED2, HIGH);
  play_tone(2, 40);
  digitalWriteLed(LED2, LOW);
  delay10ms(10);

  digitalWriteLed(LED3, HIGH);
  play_tone(3, 40);
  digitalWriteLed(LED3, LOW);
  delay10ms(10);

  digitalWriteLed(LED4, HIGH);
  play_tone(4, 40);
  digitalWriteLed(LED4, LOW);
  delay10ms(10);
}





int8_t check_button() {
  if (digitalReadButton(BUTTON1) == 0) return 1; 
  else if (digitalReadButton(BUTTON2) == 0) return 2; 
  else if (digitalReadButton(BUTTON3) == 0) return 3; 
  else if (digitalReadButton(BUTTON4) == 0) return 4;

  return -1; // If no button is pressed, return -1
}


uint8_t wait_for_button() {
  uint16_t time10ms = 0;
  const uint8_t delay_10ms = 1;

  //wait a max of ENTRY_TIME_LIMIT for a button press
  while (time10ms < ENTRY_TIME_LIMIT_10MS) {
    uint8_t button_nr = check_button();
    if (button_nr != -1) {//if a button is pressed
        digitalWriteLed(nr_to_led_pin[button_nr], HIGH);
        play_tone(button_nr, 15);
        digitalWriteLed(nr_to_led_pin[button_nr], LOW);

        while (check_button() != -1) {//wait for button release
          //do nothing
        }
        initRandomFromClock(); // use this moment to save clock for ramdom numbers
        delay10ms(1);//software debounce

        return button_nr;
    }
    delay10ms(delay_10ms);
    time10ms += delay_10ms;
  }
  return 0;
}



bool one_round(uint8_t difficulty) {
  difficulty = difficulty / 2;
  
  digitalWriteLed(LED1, HIGH);
  digitalWriteLed(LED2, HIGH);
  digitalWriteLed(LED3, HIGH);
  digitalWriteLed(LED4, HIGH);
  delay10ms(50);
  digitalWriteLed(LED1, LOW);
  digitalWriteLed(LED2, LOW);
  digitalWriteLed(LED3, LOW);
  digitalWriteLed(LED4, LOW);
  delay10ms(50);
  
  uint8_t rnd[difficulty];

  //play tone sequence
  for (uint8_t i = 0; i < difficulty; i++) {
    rnd[i] = 1 + getRandom();

    digitalWriteLed(nr_to_led_pin[rnd[i]], HIGH);
    play_tone(rnd[i], 40);
    digitalWriteLed(nr_to_led_pin[rnd[i]], LOW);
    delay10ms(10);
  }

  //wait for correct user input
  for (uint8_t i = 0; i < difficulty; i++) {
    uint8_t input_nr = wait_for_button();
    if (input_nr != rnd[i]) {
      return false;
    }
  }
  return true;
}

/*
ISR (PCINT2_vect) {
  // this is the Interrupt Service Routine
  //do nothing here, just wake up
}

void attachInterrupt() {
  // interrupts
  PCMSK2 |= (1 << PCINT11); // want pin PCINT11 = PD0 = pin2
  GIFR   |= (1 << PCIF2);   // clear any outstanding interrupts
  GIMSK  |= (1 << PCIE2);   // enable pin change interrupts 
}

void detachInterrupt() {
  GIMSK &= ~(1 << PCIE2);
}
*/

void loop() {
  //go to sleep
  ///sleep_enable(); // set safety pin to allow cpu sleep
  ///attachInterrupt(0, intrpt, LOW); // attach interrupt 0 (pin 2) and run function intrpt when pin 2 gets LOW
  ///set_sleep_mode(SLEEP_MODE_PWR_DOWN); // set sleep mode to have most power savings
  ///cli(); // disable interrupts during timed sequence
  sei(); // set Global Interrupt Enable
  ///sleep_cpu(); // power down cpu
  
  //wake up here
  ///sleep_disable(); // set safety pin to NOT allow cpu sleep
  ///detachInterrupt(0); // detach interrupt to allow other usage of this pin

  setupTimer0();

  //play game
  play_all_tones();
  
  uint8_t difficulty = 6;
  while (one_round(difficulty)) {

    //TODO Show difficulty here
    
    difficulty++;

    //C5 E5 G5 C6
    digitalWriteLed(LED2, HIGH);
    play_tone(10, 20);
    play_tone(11, 20);
    play_tone(12, 20);
    play_tone(13, 20);
    digitalWriteLed(LED2, LOW);
  }

  //C6 G5 C5
  digitalWriteLed(LED1, HIGH);
  play_tone(13, 20);
  play_tone(12, 20);
  play_tone(10, 40);
  digitalWriteLed(LED1, LOW);
}


int main()
{
    setup();
    while (1)
        loop();
}