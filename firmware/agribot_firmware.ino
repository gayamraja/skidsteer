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
 * Pin assignments — adjust to your wiring:
 *   Left  motor : PWM = 5,  DIR = 4
 *   Right motor : PWM = 6,  DIR = 7
 *   Left  encoder : A = 2 (INT0),  B = 3  (INT1)
 *   Right encoder : A = 18 (INT5), B = 19 (INT4)  [Mega pins]
 *   Left  current : A0
 *   Right current : A1
 *
 * Designed for Arduino Mega 2560 (interrupt pins 2,3,18,19 available).
 * For Uno, only INT0 (pin 2) and INT1 (pin 3) are available — use
 * software pin-change interrupts for the second encoder.
 */

// ── Pin definitions ──────────────────────────────────────────────────────────
#define LEFT_PWM_PIN    5
#define LEFT_DIR_PIN    4
#define RIGHT_PWM_PIN   6
#define RIGHT_DIR_PIN   7

#define LEFT_ENC_A      2    // INT0
#define LEFT_ENC_B      3    // INT1
#define RIGHT_ENC_A     18   // INT5 (Mega only)
#define RIGHT_ENC_B     19   // INT4 (Mega only)

#define LEFT_CURRENT_PIN   A0
#define RIGHT_CURRENT_PIN  A1

// ── Tunable parameters ───────────────────────────────────────────────────────
#define BAUD_RATE           115200
#define LOOP_HZ             50          // control + feedback rate
#define LOOP_MS             (1000 / LOOP_HZ)

#define SLEW_RATE           0.015f      // max normalised change per cycle (matches ROS soft-start)
#define RELAY_DEAD_TIME_MS  150         // direction-change pause
#define BACKLASH_POWER      0.20f       // normalised power during backlash take-up
#define BACKLASH_MS         50          // duration of backlash pulse
#define WATCHDOG_MS         200         // stop if no CMD within this window

#define MAX_PWM             255         // Arduino analogWrite ceiling
#define MIN_ACTIVE_PWM      87          // corresponds to Step 34 (34/99 * 255 ≈ 87)

#define ACS758_SENSITIVITY  0.04f       // V/A for ACS758-050B (adjust for your model)
#define ACS758_VREF         2.5f        // zero-current output voltage (V)
#define ADC_VREF            5.0f        // Arduino ADC reference voltage
#define MAX_CURRENT_AMPS    40.0f       // per-channel cutoff

// ── State machine for each motor channel ────────────────────────────────────
enum ChannelState {
    STATE_IDLE,
    STATE_BACKLASH,
    STATE_RELAY_PAUSE,
    STATE_RUNNING
};

struct Channel {
    uint8_t pwm_pin;
    uint8_t dir_pin;
    uint8_t cur_pin;

    float   target;          // normalised target (-1.0 to +1.0)
    float   actual;          // current normalised output (after slew)
    int8_t  direction;       // +1 forward, -1 reverse, 0 stop

    ChannelState state;
    uint32_t     state_enter_ms;

    volatile long encoder_ticks;
    float         amps;
};

static Channel left_ch  = {LEFT_PWM_PIN,  LEFT_DIR_PIN,  LEFT_CURRENT_PIN,  0,0,0, STATE_IDLE,0,0,0};
static Channel right_ch = {RIGHT_PWM_PIN, RIGHT_DIR_PIN, RIGHT_CURRENT_PIN, 0,0,0, STATE_IDLE,0,0,0};

// ── Watchdog ─────────────────────────────────────────────────────────────────
static uint32_t last_cmd_ms = 0;
static bool     watchdog_fired = false;
static bool     overcurrent    = false;
static bool     estop          = false;

// ── Serial input buffer ───────────────────────────────────────────────────────
static char    rx_buf[64];
static uint8_t rx_idx = 0;

// ── Encoder ISRs ─────────────────────────────────────────────────────────────
void IRAM_ATTR left_enc_isr() {
    if (digitalRead(LEFT_ENC_B) == HIGH) left_ch.encoder_ticks++;
    else                                  left_ch.encoder_ticks--;
}

void IRAM_ATTR right_enc_isr() {
    if (digitalRead(RIGHT_ENC_B) == HIGH) right_ch.encoder_ticks++;
    else                                   right_ch.encoder_ticks--;
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);

    pinMode(LEFT_PWM_PIN,  OUTPUT);
    pinMode(LEFT_DIR_PIN,  OUTPUT);
    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);

    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_enc_isr,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_enc_isr, RISING);

    set_motor_pwm(left_ch,  0.0f);
    set_motor_pwm(right_ch, 0.0f);

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
        set_motor_pwm(left_ch,  0.0f);
        set_motor_pwm(right_ch, 0.0f);
    }

    // ── Send feedback ───────────────────────────────────────────────────────
    const char* status = "OK";
    if      (estop)       status = "ESTOP";
    else if (watchdog_fired) status = "WATCHDOG";
    else if (overcurrent) status = "OVERCURRENT";

    // Snapshot ticks atomically
    noInterrupts();
    long lt = left_ch.encoder_ticks;
    long rt = right_ch.encoder_ticks;
    interrupts();

    Serial.print("FB:");
    Serial.print(lt);   Serial.print(' ');
    Serial.print(rt);   Serial.print(' ');
    Serial.print(left_ch.amps,  2); Serial.print(' ');
    Serial.print(right_ch.amps, 2); Serial.print(' ');
    Serial.println(status);
}

// ── Parse CMD packet ──────────────────────────────────────────────────────────
void parse_cmd(const char* buf) {
    if (strncmp(buf, "CMD:", 4) != 0) return;

    float l, r;
    if (sscanf(buf + 4, "%f %f", &l, &r) != 2) return;

    // Clamp
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
                // First ramp down to zero
                ch.target = 0.0f;
                slew(ch);
                if (fabsf(ch.actual) < 0.001f) {
                    // Begin relay dead-time
                    ch.state          = STATE_RELAY_PAUSE;
                    ch.state_enter_ms = now;
                    set_motor_pwm(ch, 0.0f);
                }
            } else {
                slew(ch);
                ch.direction = target_dir;
                set_motor_pwm(ch, ch.actual);
                ch.state = (fabsf(ch.actual) > 0.001f) ? STATE_RUNNING : STATE_IDLE;
            }
            break;
        }

        case STATE_RELAY_PAUSE:
            set_motor_pwm(ch, 0.0f);
            if ((now - ch.state_enter_ms) >= (uint32_t)RELAY_DEAD_TIME_MS) {
                // Begin backlash compensation
                ch.state          = STATE_BACKLASH;
                ch.state_enter_ms = now;
                ch.direction      = target_dir;
                set_motor_pwm(ch, BACKLASH_POWER * target_dir);
            }
            break;

        case STATE_BACKLASH:
            set_motor_pwm(ch, BACKLASH_POWER * (float)ch.direction);
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

// ── Apply normalised value to motor pins ──────────────────────────────────────
void set_motor_pwm(Channel& ch, float norm) {
    norm = constrain(norm, -1.0f, 1.0f);

    bool forward = (norm >= 0.0f);
    float magnitude = fabsf(norm);

    // Map non-zero magnitude to MIN_ACTIVE_PWM..MAX_PWM
    int pwm = 0;
    if (magnitude > 0.001f) {
        pwm = (int)(MIN_ACTIVE_PWM + magnitude * (MAX_PWM - MIN_ACTIVE_PWM));
        pwm = constrain(pwm, MIN_ACTIVE_PWM, MAX_PWM);
    }

    digitalWrite(ch.dir_pin, forward ? HIGH : LOW);
    analogWrite(ch.pwm_pin, pwm);
}

// ── ACS758 current reading ────────────────────────────────────────────────────
float read_current(uint8_t pin) {
    int   raw     = analogRead(pin);
    float voltage = (raw / 1023.0f) * ADC_VREF;
    float amps    = fabsf((voltage - ACS758_VREF) / ACS758_SENSITIVITY);
    return amps;
}
