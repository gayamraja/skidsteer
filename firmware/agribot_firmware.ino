/**
 * agribot_firmware.ino — Agribot real-time motor controller
 *
 * Responsibilities:
 *  1. Receive CMD packets from Raspberry Pi over USB serial at 50 Hz
 *  2. Enforce soft-start slew rate (even if RPi glitches full-speed)
 *  3. Enforce 150 ms relay dead-time on direction change
 *  4. Backlash compensation: brief low-power pulse to take up chain slack
 *  5. Monitor ACS758 current sensors — cut power on >40 A per channel
 *  6. Watchdog: stop motors if no CMD received within 200 ms
 *  7. Send FB packets (encoder ticks, current, status) back to RPi at 50 Hz
 *
 * Serial protocol (115200 baud, ASCII newline-terminated):
 *   RPi  → Arduino : CMD:<left> <right>\n   (float -1.0 to +1.0)
 *   Arduino → RPi  : FB:<l_ticks> <r_ticks> <l_amps> <r_amps> <status>\n
 *   status values  : OK | ESTOP | WATCHDOG | OVERCURRENT
 *
 * Target board: Arduino Nano R3 (ATmega328P)
 *
 * Throttle output: Digital potentiometer chip (X9C103 or equivalent)
 *   INC pin — pulse LOW then HIGH to step wiper one position
 *   U/D pin — HIGH = increment up, LOW = increment down
 *   CS  pin — LOW to enable chip, HIGH to disable
 *   Wiper output connects directly to motor controller throttle wire.
 *   Steps 0-99: step 0 = 0V (stopped), step 34 = minimum active throttle,
 *   step 99 = maximum throttle (~4.2V on a 5V supply).
 *
 * Direction output: Relay module driven by DIR pin
 *   LOW  = Forward  (relay de-energised, matches previous project)
 *   HIGH = Reverse  (relay energised)
 *   150 ms dead-time enforced before switching direction.
 *
 * Pin assignments:
 *   Left  motor : INC=5, UD=10, CS=11, DIR=4
 *   Right motor : INC=6, UD=12, CS=13, DIR=7
 *   Left  encoder : A=2 (INT0 hardware interrupt), B=3
 *   Right encoder : A=8 (PCINT0 pin-change interrupt), B=9
 *   Left  current : A0  (ACS758 VIOUT)
 *   Right current : A1  (ACS758 VIOUT)
 *
 *   Shared GND between Arduino and both motor controllers is mandatory.
 */

// ── Pin definitions ──────────────────────────────────────────────────────────
#define LEFT_DIR_PIN    4
#define LEFT_POT_INC    5
#define RIGHT_POT_INC   6
#define RIGHT_DIR_PIN   7

#define LEFT_ENC_A      2    // INT0 — hardware interrupt
#define LEFT_ENC_B      3    // read inside ISR for direction
#define RIGHT_ENC_A     8    // PCINT0 — pin-change interrupt (Nano PB0)
#define RIGHT_ENC_B     9    // read inside PCINT ISR for direction

#define LEFT_POT_UD     10
#define LEFT_POT_CS     11
#define RIGHT_POT_UD    12
#define RIGHT_POT_CS    13

#define LEFT_CURRENT_PIN   A0
#define RIGHT_CURRENT_PIN  A1

// ── Tunable parameters ───────────────────────────────────────────────────────
#define BAUD_RATE           115200
#define LOOP_HZ             50          // control + feedback rate
#define LOOP_MS             (1000 / LOOP_HZ)

#define SLEW_RATE           0.015f      // max normalised change per cycle
#define RELAY_DEAD_TIME_MS  150         // direction-change pause (ms)
#define BACKLASH_POWER      0.20f       // normalised power during backlash take-up
#define BACKLASH_MS         50          // duration of backlash pulse (ms)
#define WATCHDOG_MS         200         // stop if no CMD within this window (ms)

// Digital pot step range
// RH=5V, RL=GND → wiper voltage = (step/99) × 5V
// E-rickshaw controller throttle range: 0.8V (min) to 4.2V (max)
// Step 34 = 1.72V (above 0.8V threshold) — minimum active
// Step 83 = 4.19V (≈4.2V max) — cap here to avoid over-voltage on controller
#define MAX_STEP            83          // (83/99 × 5V = 4.19V ≈ controller max)
#define MIN_ACTIVE_STEP     34          // (34/99 × 5V = 1.72V, above 0.8V threshold)

#define ACS758_SENSITIVITY  0.04f       // V/A for ACS758-050B (adjust for your model)
#define ACS758_VREF         2.5f        // zero-current output voltage (V)
#define ADC_VREF            5.0f        // Arduino ADC reference voltage
#define MAX_CURRENT_AMPS    40.0f       // per-channel cutoff

// ── State machine for each motor channel ─────────────────────────────────────
enum ChannelState {
    STATE_IDLE,
    STATE_BACKLASH,
    STATE_RELAY_PAUSE,
    STATE_RUNNING
};

struct Channel {
    uint8_t dir_pin;
    uint8_t cur_pin;
    uint8_t pot_inc_pin;
    uint8_t pot_ud_pin;
    uint8_t pot_cs_pin;
    int     pot_step;        // current wiper position (0-99)

    float   target;          // normalised target (-1.0 to +1.0)
    float   actual;          // current normalised output (after slew)
    int8_t  direction;       // +1 forward, -1 reverse, 0 stop

    ChannelState state;
    uint32_t     state_enter_ms;

    volatile long encoder_ticks;
    float         amps;
};

static Channel left_ch  = {LEFT_DIR_PIN,  LEFT_CURRENT_PIN,
                            LEFT_POT_INC,  LEFT_POT_UD,  LEFT_POT_CS,  0,
                            0, 0, 0, STATE_IDLE, 0, 0, 0};

static Channel right_ch = {RIGHT_DIR_PIN, RIGHT_CURRENT_PIN,
                            RIGHT_POT_INC, RIGHT_POT_UD, RIGHT_POT_CS, 0,
                            0, 0, 0, STATE_IDLE, 0, 0, 0};

// ── Watchdog ──────────────────────────────────────────────────────────────────
static uint32_t last_cmd_ms    = 0;
static bool     watchdog_fired = false;
static bool     overcurrent    = false;
static bool     estop          = false;

// ── Serial input buffer ───────────────────────────────────────────────────────
static char    rx_buf[64];
static uint8_t rx_idx = 0;

// ── Encoder ISRs ─────────────────────────────────────────────────────────────

// Left encoder — hardware interrupt on D2 (INT0)
void left_enc_isr() {
    if (digitalRead(LEFT_ENC_B) == HIGH) left_ch.encoder_ticks++;
    else                                  left_ch.encoder_ticks--;
}

// Right encoder — pin-change interrupt on D8 (PCINT0, PB0)
// PCINT fires on both rising and falling edges; we detect rising edge manually.
ISR(PCINT0_vect) {
    static bool prev_state = false;
    bool current = (PINB & (1 << PB0)) != 0;   // read D8 directly from port register
    if (current && !prev_state) {               // rising edge only
        if (digitalRead(RIGHT_ENC_B) == HIGH) right_ch.encoder_ticks++;
        else                                   right_ch.encoder_ticks--;
    }
    prev_state = current;
}

// ── Digital pot helpers ───────────────────────────────────────────────────────

// Reset wiper to position 0 by stepping down 99 times.
// Call once in setup() before any motion.
void reset_pot(Channel& ch) {
    digitalWrite(ch.pot_ud_pin, LOW);   // direction: down
    digitalWrite(ch.pot_cs_pin, LOW);   // enable chip
    for (int i = 0; i < 99; i++) {
        digitalWrite(ch.pot_inc_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(ch.pot_inc_pin, HIGH);
        delayMicroseconds(2);
    }
    digitalWrite(ch.pot_cs_pin, HIGH);  // disable chip
    ch.pot_step = 0;
}

// Move wiper to target_step (0-99) from current position.
void step_pot_to(Channel& ch, int target_step) {
    target_step = constrain(target_step, 0, 99);
    int diff = target_step - ch.pot_step;
    if (diff == 0) return;

    bool up    = (diff > 0);
    int  steps = abs(diff);

    digitalWrite(ch.pot_ud_pin, up ? HIGH : LOW);
    digitalWrite(ch.pot_cs_pin, LOW);   // enable
    for (int i = 0; i < steps; i++) {
        digitalWrite(ch.pot_inc_pin, LOW);
        delayMicroseconds(2);
        digitalWrite(ch.pot_inc_pin, HIGH);
        delayMicroseconds(2);
    }
    digitalWrite(ch.pot_cs_pin, HIGH);  // disable
    ch.pot_step = target_step;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);

    // Direction relay pins — default LOW = forward
    pinMode(LEFT_DIR_PIN,  OUTPUT);  digitalWrite(LEFT_DIR_PIN,  LOW);
    pinMode(RIGHT_DIR_PIN, OUTPUT);  digitalWrite(RIGHT_DIR_PIN, LOW);

    // Digital pot pins — CS high (disabled), INC high (idle)
    pinMode(LEFT_POT_INC,  OUTPUT);  digitalWrite(LEFT_POT_INC,  HIGH);
    pinMode(LEFT_POT_UD,   OUTPUT);  digitalWrite(LEFT_POT_UD,   HIGH);
    pinMode(LEFT_POT_CS,   OUTPUT);  digitalWrite(LEFT_POT_CS,   HIGH);

    pinMode(RIGHT_POT_INC, OUTPUT);  digitalWrite(RIGHT_POT_INC, HIGH);
    pinMode(RIGHT_POT_UD,  OUTPUT);  digitalWrite(RIGHT_POT_UD,  HIGH);
    pinMode(RIGHT_POT_CS,  OUTPUT);  digitalWrite(RIGHT_POT_CS,  HIGH);

    // Encoder pins — internal pull-ups
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // Left encoder — hardware interrupt (INT0 on D2)
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), left_enc_isr, RISING);

    // Right encoder — pin-change interrupt (PCINT0 on D8 = PB0)
    PCICR  |= (1 << PCIE0);    // enable PCINT0..7 group
    PCMSK0 |= (1 << PCINT0);   // enable PCINT0 specifically (D8)

    // Reset both digital pots to wiper position 0
    reset_pot(left_ch);
    reset_pot(right_ch);

    last_cmd_ms = millis();
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();
    static uint32_t last_loop_ms = 0;

    // Read all available serial bytes into line buffer
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            rx_buf[rx_idx] = '\0';
            parse_cmd(rx_buf);
            rx_idx = 0;
        } else if (rx_idx < sizeof(rx_buf) - 1) {
            rx_buf[rx_idx++] = c;
        }
    }

    // Only run control + feedback at LOOP_HZ
    if ((now - last_loop_ms) < (uint32_t)LOOP_MS) return;
    last_loop_ms = now;

    // ── Watchdog check ──────────────────────────────────────────────────────
    if ((now - last_cmd_ms) > WATCHDOG_MS) {
        watchdog_fired = true;
        left_ch.target  = 0.0f;
        right_ch.target = 0.0f;
    }

    // ── Current sensing ─────────────────────────────────────────────────────
    left_ch.amps  = read_current(left_ch.cur_pin);
    right_ch.amps = read_current(right_ch.cur_pin);

    if (left_ch.amps > MAX_CURRENT_AMPS || right_ch.amps > MAX_CURRENT_AMPS) {
        overcurrent = true;
        left_ch.target  = 0.0f;
        right_ch.target = 0.0f;
    } else {
        overcurrent = false;
    }

    // ── Update each channel ─────────────────────────────────────────────────
    if (!estop) {
        update_channel(left_ch,  now);
        update_channel(right_ch, now);
    } else {
        set_motor_output(left_ch,  0.0f);
        set_motor_output(right_ch, 0.0f);
    }

    // ── Send feedback ───────────────────────────────────────────────────────
    const char* status = "OK";
    if      (estop)          status = "ESTOP";
    else if (watchdog_fired) status = "WATCHDOG";
    else if (overcurrent)    status = "OVERCURRENT";

    // Snapshot encoder ticks atomically
    noInterrupts();
    long lt = left_ch.encoder_ticks;
    long rt = right_ch.encoder_ticks;
    interrupts();

    Serial.print("FB:");
    Serial.print(lt);              Serial.print(' ');
    Serial.print(rt);              Serial.print(' ');
    Serial.print(left_ch.amps,  2); Serial.print(' ');
    Serial.print(right_ch.amps, 2); Serial.print(' ');
    Serial.println(status);
}

// ── Parse CMD packet ──────────────────────────────────────────────────────────
void parse_cmd(const char* buf) {
    if (strncmp(buf, "CMD:", 4) != 0) return;

    float l, r;
    if (sscanf(buf + 4, "%f %f", &l, &r) != 2) return;

    l = constrain(l, -1.0f, 1.0f);
    r = constrain(r, -1.0f, 1.0f);

    left_ch.target  = l;
    right_ch.target = r;

    last_cmd_ms    = millis();
    watchdog_fired = false;
}

// ── Per-channel state machine ─────────────────────────────────────────────────
void update_channel(Channel& ch, uint32_t now) {
    int8_t target_dir = (ch.target > 0.0001f) ? 1 : (ch.target < -0.0001f) ? -1 : 0;

    switch (ch.state) {

        case STATE_IDLE:
        case STATE_RUNNING: {
            bool dir_change = (target_dir != 0) && (ch.direction != 0) && (target_dir != ch.direction);

            if (dir_change) {
                // Ramp down to zero first
                ch.target = 0.0f;
                slew(ch);
                if (fabsf(ch.actual) < 0.001f) {
                    // Motor at zero — begin relay dead-time
                    ch.state          = STATE_RELAY_PAUSE;
                    ch.state_enter_ms = now;
                    set_motor_output(ch, 0.0f);
                }
            } else {
                slew(ch);
                ch.direction = target_dir;
                set_motor_output(ch, ch.actual);
                ch.state = (fabsf(ch.actual) > 0.001f) ? STATE_RUNNING : STATE_IDLE;
            }
            break;
        }

        case STATE_RELAY_PAUSE:
            set_motor_output(ch, 0.0f);
            if ((now - ch.state_enter_ms) >= (uint32_t)RELAY_DEAD_TIME_MS) {
                // Dead-time elapsed — begin backlash compensation
                ch.state          = STATE_BACKLASH;
                ch.state_enter_ms = now;
                ch.direction      = target_dir;
                set_motor_output(ch, BACKLASH_POWER * target_dir);
            }
            break;

        case STATE_BACKLASH:
            set_motor_output(ch, BACKLASH_POWER * (float)ch.direction);
            if ((now - ch.state_enter_ms) >= (uint32_t)BACKLASH_MS) {
                ch.actual = BACKLASH_POWER * (float)ch.direction;
                ch.state  = STATE_RUNNING;
            }
            break;
    }
}

// ── Slew rate limiter ─────────────────────────────────────────────────────────
void slew(Channel& ch) {
    float diff = ch.target - ch.actual;
    if (fabsf(diff) <= SLEW_RATE) {
        ch.actual = ch.target;
    } else {
        ch.actual += (diff > 0.0f) ? SLEW_RATE : -SLEW_RATE;
    }
}

// ── Apply normalised value to motor (digital pot + relay) ─────────────────────
void set_motor_output(Channel& ch, float norm) {
    norm = constrain(norm, -1.0f, 1.0f);

    bool  forward   = (norm >= 0.0f);
    float magnitude = fabsf(norm);

    // Map magnitude to pot step (0-99), enforcing minimum active step
    int target_step = 0;
    if (magnitude > 0.001f) {
        target_step = (int)(MIN_ACTIVE_STEP + magnitude * (MAX_STEP - MIN_ACTIVE_STEP));
        target_step = constrain(target_step, MIN_ACTIVE_STEP, MAX_STEP);
    }

    // Direction relay: LOW = forward, HIGH = reverse
    digitalWrite(ch.dir_pin, forward ? LOW : HIGH);

    // Move digital pot wiper to target step
    step_pot_to(ch, target_step);
}

// ── ACS758 current reading ────────────────────────────────────────────────────
float read_current(uint8_t pin) {
    int   raw     = analogRead(pin);
    float voltage = (raw / 1023.0f) * ADC_VREF;
    float amps    = fabsf((voltage - ACS758_VREF) / ACS758_SENSITIVITY);
    return amps;
}
