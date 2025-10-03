/*
  Work Hours Counter v4.7
  I needed a way to keep track of the time spent repairing amplifiers.
  Since I often work on several units in parallel—sometimes pausing one while waiting for parts—
  I decided to build a dedicated work hour counter. I had a nice 4-button keypad at stock,
  so I configured the system to handle four independent timers, one for each device.

  Although originally designed for amplifier repairs,
  this timer can be used to measure and manage the working time of any activity you choose.

  3A + CRC16 + compact SaveImage
  Version History:
  - 3A: Incremental H/M/S timekeeping (no div/mod in hot path for better performance)
  - 4 : CRC-16/X25 instead of CRC-32 (saves flash memory)
  - 5 : Smaller SaveImage header, stores H/M/S directly (no division on load)

  Behavior:
  - Reset countdown appears after 1 second of holding button
  - No run/pause toggle allowed during countdown
  - Reset completes after 3 seconds of continuous hold

  Hardware Requirements:
  - Arduino with LCD (16x2)
  - 4 buttons (one per counter/amplifier)
  - Optional: FRAM I2C module for unlimited write cycles
  - Optional: PWM contrast control and beeper
*/

#include <LiquidCrystal.h> // Library for standard HD44780 LCD control
#include <EEPROM.h>        // Library for built-in EEPROM non-volatile storage

// =========================== Feature Flags ===========================

// ENABLE_STARTUP_ANIM: Shows a smooth two-line progress bar on boot
// Provides visual feedback that system is initializing
// Duration: ~80ms (configurable in setup())
#define ENABLE_STARTUP_ANIM 1

// USE_FRAM: Enable FRAM (Ferroelectric RAM) instead of EEPROM
// FRAM advantages: unlimited write cycles, faster writes, no wear leveling needed
// EEPROM limitations: ~100,000 write cycles per cell
// Set to 1 if Adafruit FRAM I2C breakout is connected
#define USE_FRAM 0

// USE_COUNTDOWN: Enables visual countdown timer during hold-to-reset
// When enabled (1):
//   - After holding reset button for 1 second, displays "Hold reset: X" countdown
//   - Shows remaining seconds (3, 2, 1) until reset completes at 3 seconds
//   - Prevents run/pause toggle once countdown begins (visual feedback only)
//   - Provides clear indication that reset is in progress
// When disabled (0):
//   - Reset still works at 3 seconds, but without visual countdown
//   - Saves ~200 bytes of flash memory
#define USE_COUNTDOWN 1

// VERIFY_WRITES: Enables read-back verification after writing to storage
// When enabled (1):
//   - After each EEPROM/FRAM write, reads data back and verifies it matches
//   - Compares sequence numbers to ensure write succeeded
//   - Slightly increases write time (~1-2ms) but adds safety
// When disabled (0):
//   - Assumes all writes succeed (faster but less safe)
//   - Use only if storage is known to be 100% reliable
#define VERIFY_WRITES 1

// USE_PASSIVE_BUZZER: Select beeper hardware type
// 0 = ACTIVE buzzer (on/off only; we simulate "tones" as ON pulses)
// 1 = PASSIVE piezo (supports tone() with frequencies)
#define USE_PASSIVE_BUZZER 0

// How many EEPROM slots to use for ring-buffer wear leveling.
// Must fit in EEPROM. Each slot stores one SaveImage.
#ifndef EEPROM_NUM_SLOTS
#define EEPROM_NUM_SLOTS 16
#endif

// Conditional includes based on feature flags
#if USE_FRAM
  #include <Wire.h>               // I2C communication library
  #include <Adafruit_FRAM_I2C.h>  // Library for Adafruit FRAM module
#endif

// =========================== PROGMEM Support ===========================
// Use AVR-specific flash functions for AVR architecture (Uno, Nano, etc.)
#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#else
  // Fallback for non-AVR architectures (e.g., ESP32, Due) that use standard pgmspace
  #include <pgmspace.h>
#endif

// =========================== Customizable Text ===========================
// Texts stored in PROGMEM (Flash memory) to save precious SRAM
const char kTxtHeaderPrefix[] PROGMEM = "  Amplifier "; // Prefix for the counter display (e.g., "Amplifier 1")
const char kTxtHoldReset[]   PROGMEM = "Hold reset: "; // Text shown during reset countdown
constexpr uint8_t TXT_HOLD_RESET_LEN = sizeof(kTxtHoldReset) - 1; // Length of reset text

// Storage detection messages shown on startup if FRAM is enabled
const char kTxtUsingFRAM[]    PROGMEM = " Using FRAM    ";
const char kTxtFRAMNotFound[] PROGMEM = " FRAM not found";
const char kTxtUsingEEPROM[]  PROGMEM = " Using EEPROM  ";

// =========================== Hardware Configuration ===========================
constexpr uint8_t NUM_COUNTERS = 4;  // Number of independent timers/buttons
constexpr uint8_t LCD_COLS     = 16; // 16 columns
constexpr uint8_t LCD_ROWS     = 2;  // 2 rows

// LCD pin assignments (4-bit mode)
constexpr uint8_t PIN_LCD_RS = 2; // Register Select pin
constexpr uint8_t PIN_LCD_EN = 8; // Enable pin
constexpr uint8_t PIN_LCD_D4 = 4; // Data pin 4
constexpr uint8_t PIN_LCD_D5 = 5; // Data pin 5
constexpr uint8_t PIN_LCD_D6 = 6; // Data pin 6
constexpr uint8_t PIN_LCD_D7 = 7; // Data pin 7

// Peripheral pins
constexpr uint8_t PIN_CONTRAST_PWM = 3;                     // PWM pin for LCD contrast control
constexpr uint8_t PIN_BEEPER       = 13;                    // Pin for buzzer/piezo
constexpr uint8_t PIN_BUTTONS[NUM_COUNTERS] = {11, 12, 9, 10}; // Digital pins for the four buttons (Active LOW)

// =========================== Timing Constants ===========================
constexpr uint8_t       DEBOUNCE_MS               = 10;      // Milliseconds for button debounce stabilization
constexpr uint8_t       CONTRAST_PWM_DUTY         = 0;       // PWM value for LCD contrast (0-255)
constexpr unsigned long SAVE_PERIOD_MS            = 10000UL; // 10 seconds auto-save interval
constexpr unsigned long RESET_HOLD_MS             = 3000UL;  // 3 seconds continuous hold required for reset
constexpr unsigned long COUNTDOWN_SHOW_DELAY_MS   = 1000UL;  // 1 second hold before the countdown display starts

#if USE_FRAM
  constexpr uint8_t FRAM_I2C_ADDR = 0x50; // Default I2C address for FRAM module
#endif

// =========================== Global Objects ===========================
// Instantiate the LiquidCrystal object with the hardware configuration
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

#if USE_FRAM
Adafruit_FRAM_I2C fram; // Instantiate FRAM library object
bool useFRAM = true;    // Flag indicating that FRAM is available and used
#else
bool useFRAM = false;   // Flag indicating that EEPROM is used
#endif

// =========================== PROGMEM Helper ===========================
// Helper function to print strings stored in Program Memory (PROGMEM) to the LCD
static inline void lcdPrint_P(const char* p) {
  char c;
  while ((c = pgm_read_byte(p++)) != 0) lcd.print(c);
}

// =========================== Button Debouncing ===========================
// Structure to hold state for a single debounced button
struct Deb {
  uint8_t pin;          // Arduino pin number
  uint8_t stable;       // Last confirmed stable reading (HIGH/LOW)
  uint8_t last;         // Last raw reading
  unsigned long tmark;  // Timestamp of the last state change (raw)
  uint8_t fellEv;       // Flag set on a confirmed transition to LOW (press)
  uint8_t roseEv;       // Flag set on a confirmed transition to HIGH (release)
};

// Class containing static methods for managing the Deb structure
struct Debounce {
  // Initialize a Deb structure and set up the corresponding pin
  static void init(Deb& d, uint8_t pin) {
    d.pin = pin;
    pinMode(pin, INPUT_PULLUP);
    uint8_t r = digitalRead(pin);
    d.stable = d.last = r;
    d.tmark = millis();
    d.fellEv = d.roseEv = 0;
  }
  // Main update function: reads the pin and checks for stable state changes
  static void update(Deb& d) {
    uint8_t r = digitalRead(d.pin);
    unsigned long now = millis();
    if (r != d.last) { d.last = r; d.tmark = now; } // Detected new raw state, reset timer
    else if (r != d.stable && (now - d.tmark) >= DEBOUNCE_MS) { // State stable for DEBOUNCE_MS
      d.stable = r;
      if (r == LOW)  d.fellEv = 1; // Confirmed press (fall)
      else           d.roseEv = 1; // Confirmed release (rise)
    }
  }
  // Check for press event (and clear the event flag)
  static bool fell (Deb& d){ if (d.fellEv) { d.fellEv = 0; return true; } return false; }
  // Check for release event (and clear the event flag)
  static bool rose (Deb& d){ if (d.roseEv) { d.roseEv = 0; return true; } return false; }
  // Get the current stable state of the button (LOW=pressed)
  static uint8_t read(const Deb& d){ return d.stable; }
};

Deb debs[NUM_COUNTERS]; // Array of Debounce structures for all 4 buttons

// =========================== State (3A: H/M/S) ===========================
// Structure to hold one counter's time data. H is 32-bit for max 4.2 billion hours.
struct Counter { uint32_t h; uint8_t m; uint8_t s; };

Counter       counters[NUM_COUNTERS] = {}; // Array of all counters (initialized to zero)
uint8_t       activeIndex  = 0;            // The index of the currently active/displayed counter (0-3)
bool          isRunning    = false;        // Flag: true if the active counter is currently timing

unsigned long zeroTime     = 0;            // Timestamp for the start of the current 1-second interval
uint32_t      runAccumMs   = 0;            // Accumulated milliseconds since the last full second tick

unsigned long lastSaveMs   = 0;            // Timestamp of the last time data was saved to storage
uint16_t      saveSeq      = 0;            // Sequence number for persistence (used for finding latest slot/wear leveling)

// =========================== Button Press/Hold FSM ===========================
// State variables for the button press/hold Finite State Machine
unsigned long pressStartMs = 0;      // Timestamp when the active button was first pressed
uint8_t       armedIndex   = 255;    // The index of the button currently being held (255 if none)
uint8_t       armedMask    = 0;      // Bitmask of the armed button (convenience for checking hold)
bool          togglePending= false;  // Flag: true if the current active button, if released soon, will toggle run/pause

#if USE_COUNTDOWN
bool          holdCountdownActive = false; // Flag: true if the reset countdown is currently displayed
uint8_t       holdSecsLeft        = 0;     // Number of seconds remaining in the hold-to-reset countdown
#endif

// =========================== UI Dirty-Cache ===========================
// Cache of the last displayed state to minimize slow LCD updates (only redraws on change)
struct UiState {
  uint8_t  shownIndex  = 255;         // Last shown counter index
  uint32_t shownH      = 0xFFFFFFFFUL;// Last shown hours
  uint8_t  shownM      = 255;         // Last shown minutes
  uint8_t  shownS      = 255;         // Last shown seconds
  uint8_t  flags       = 0;           // bit0=running, bit1=countdown status
  uint8_t  holdSec     = 255;         // 255 = no countdown shown, otherwise the seconds remaining
} ui;

// =========================== CRC-16/X25 ===========================
using crc_t = uint16_t;               // Define CRC type as 16-bit
constexpr size_t CRC_SZ = sizeof(crc_t); // Size of the CRC checksum

// Function to update the CRC for one byte
static uint16_t crc16_update(uint16_t crc, uint8_t b){
  crc ^= b;
  for(uint8_t i=0;i<8;++i) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
  return crc;
}
// Function to compute the CRC-16/X25 checksum for a block of memory
static uint16_t crc16_compute(const uint8_t* p, size_t n){
  uint16_t c = 0xFFFF; // Initial value
  for(size_t i=0;i<n;++i) c = crc16_update(c, p[i]);
  return ~c; // Final XOR
}

// =========================== SaveImage ===========================
constexpr uint16_t SAVE_MAGIC   = 0xA11F; // Magic number for storage header validity check
constexpr uint16_t SAVE_VERSION = 1;      // Storage structure version

// Structure that defines the layout of data saved to EEPROM/FRAM
struct SaveImage {
  uint16_t magic;                 // Magic number
  uint16_t version;               // Version number
  uint16_t seq;                   // Sequence number (for wear leveling)
  uint32_t hours[NUM_COUNTERS];   // Stored hours for all counters
  uint8_t  minutes[NUM_COUNTERS]; // Stored minutes
  uint8_t  seconds[NUM_COUNTERS]; // Stored seconds
  crc_t    crc;                   // CRC checksum for data integrity
} __attribute__((packed)); // Ensures no padding between members

const int EEPROM_SLOT_SIZE = sizeof(SaveImage); // Size of one save slot in EEPROM/FRAM

// Calculates the starting address in EEPROM for a given slot index
static inline int  eeprom_slot_base(uint8_t slot){ return int(slot) * EEPROM_SLOT_SIZE; }
// Compares sequence numbers for ring-buffer logic (handles rollover)
static inline bool seq_newer(uint16_t a, uint16_t b){ return uint16_t(a - b) < 0x8000; }

// =========================== EEPROM Ring-Buffer Store ===========================
class EepromStore {
public:
  // Read an arbitrary number of bytes from EEPROM
  static void read_bytes(int addr, uint8_t* buf, int len) {
    for (int i=0;i<len;++i) buf[i] = EEPROM.read(addr+i);
  }
  // Write bytes to EEPROM using update() to only write if the value changes (reduces wear)
  static void write_bytes(int addr, const uint8_t* buf, int len) {
    for (int i=0;i<len;++i) EEPROM.update(addr+i, buf[i]);
  }
  // Load a SaveImage from a specific EEPROM slot, checks magic, version, and CRC
  static bool load_slot(uint8_t slot, SaveImage& img) {
    const int base = eeprom_slot_base(slot);
    read_bytes(base, (uint8_t*)&img, EEPROM_SLOT_SIZE);
    if (img.magic != SAVE_MAGIC || img.version != SAVE_VERSION) return false;
    crc_t calc = crc16_compute((uint8_t*)&img, EEPROM_SLOT_SIZE-CRC_SZ);
    return calc == img.crc;
  }
  // Save a SaveImage to a specific EEPROM slot
  static bool save_to_slot(uint8_t slot, const SaveImage& img) {
    const int base = eeprom_slot_base(slot);
    write_bytes(base, (const uint8_t*)&img, EEPROM_SLOT_SIZE);
    #if VERIFY_WRITES
      SaveImage v{};
      return load_slot(slot, v) && v.seq == img.seq; // Verify write success
    #else
      return true;
    #endif
  }
  // Scan all EEPROM slots to find the one with the newest sequence number
  static bool load_latest(SaveImage& out, uint8_t& outSlot) {
    const uint16_t maxSlotsBySize = EEPROM.length() / EEPROM_SLOT_SIZE;
    const uint8_t  slots = (EEPROM_NUM_SLOTS <= maxSlotsBySize) ? EEPROM_NUM_SLOTS : maxSlotsBySize;

    bool found = false;
    SaveImage best{};
    uint8_t    bestSlot = 0;

    for (uint8_t s=0; s<slots; ++s) {
      SaveImage tmp{};
      if (!load_slot(s, tmp)) continue; // Skip invalid or corrupt slots
      if (!found || seq_newer(tmp.seq, best.seq)) { best = tmp; bestSlot = s; found = true; } // Find newer sequence
    }
    if (found) { out = best; outSlot = bestSlot; }
    return found;
  }
  // Determine the next slot to save to (circular buffer logic) and save the image
  static bool save_next(const SaveImage& img) {
    const uint16_t maxSlotsBySize = EEPROM.length() / EEPROM_SLOT_SIZE;
    const uint8_t  slots = (EEPROM_NUM_SLOTS <= maxSlotsBySize) ? EEPROM_NUM_SLOTS : maxSlotsBySize;

    uint8_t latestSlot = 0;
    SaveImage dummy{};
    bool haveLatest = load_latest(dummy, latestSlot);

    // Calculate the next slot index in the ring buffer
    uint8_t target = haveLatest ? (uint8_t)((latestSlot + 1) % slots) : 0;
    return save_to_slot(target, img);
  }
};

// =========================== FRAM Storage ===========================
#if USE_FRAM
// Class for FRAM storage API (simpler, as it does not need wear leveling)
class FramStore {
public:
  // Write a block of bytes to FRAM
  static void write_bytes(uint16_t addr, const uint8_t* buf, uint16_t len) {
    for (uint16_t i=0;i<len;++i) fram.write(addr+i, buf[i]);
  }
  // Read a block of bytes from FRAM
  static void read_bytes(uint16_t addr, uint8_t* buf, uint16_t len) {
    for (uint16_t i=0;i<len;++i) buf[i] = fram.read(addr+i);
  }
  // Load SaveImage from FRAM (address 0) and verify integrity
  static bool load(SaveImage& img) {
    read_bytes(0, (uint8_t*)&img, sizeof(img));
    if (img.magic != SAVE_MAGIC || img.version != SAVE_VERSION) return false;
    crc_t calc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);
    return calc == img.crc;
  }
  // Save SaveImage to FRAM (address 0)
  static bool save(const SaveImage& img) {
    write_bytes(0, (const uint8_t*)&img, sizeof(img));
    #if VERIFY_WRITES
      SaveImage v{};
      return load(v) && v.seq == img.seq; // Verify write success
    #else
      return true;
    #endif
  }
};
#endif

// =========================== Persistence API ===========================
// Loads counter values from storage or initializes to default (zero) if load fails
static void load_from_storage_or_default() {
  SaveImage img{};
  bool ok = false;

  if (useFRAM) {
  #if USE_FRAM
    ok = FramStore::load(img);
    if (!ok) { // If FRAM load fails, initialize and save a clean image
      img.magic=SAVE_MAGIC; img.version=SAVE_VERSION; img.seq=0;
      for (uint8_t i=0;i<NUM_COUNTERS;++i){ img.hours[i]=0; img.minutes[i]=0; img.seconds[i]=0; }
      img.crc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);
      (void)FramStore::save(img); // Save the default image
      ok = true;
    }
  #endif
  } else {
    uint8_t slot=0;
    ok = EepromStore::load_latest(img, slot);
    if (!ok) { // If EEPROM load fails, initialize and save a clean image to slot 0
      img.magic=SAVE_MAGIC; img.version=SAVE_VERSION; img.seq=0;
      for (uint8_t i=0;i<NUM_COUNTERS;++i){ img.hours[i]=0; img.minutes[i]=0; img.seconds[i]=0; }
      img.crc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);
      (void)EepromStore::save_to_slot(0, img); // Save the default image
      ok = true;
    }
  }

  // Copy loaded/default data into the active counter structures
  for (uint8_t i=0;i<NUM_COUNTERS;++i) {
    counters[i].h = img.hours[i];
    counters[i].m = img.minutes[i];
    counters[i].s = img.seconds[i];
  }
  saveSeq = img.seq; // Update the current sequence number
}

// Packages current counter data into a SaveImage and writes it to storage
static bool save_to_storage() {
  SaveImage img{};
  img.magic=SAVE_MAGIC;
  img.version=SAVE_VERSION;
  img.seq=uint16_t(saveSeq+1); // Increment sequence number for new save

  // Copy current counter state into the SaveImage
  for (uint8_t i=0;i<NUM_COUNTERS;++i){
    img.hours[i]   = counters[i].h;
    img.minutes[i] = counters[i].m;
    img.seconds[i] = counters[i].s;
  }
  img.crc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ); // Calculate CRC

  bool ok = false;
  if (useFRAM) {
  #if USE_FRAM
    ok = FramStore::save(img);
  #endif
  } else {
    ok = EepromStore::save_next(img); // Use ring-buffer logic for EEPROM
  }
  if (ok) saveSeq = img.seq; // Only update sequence number if save was successful
  return ok;
}

// Checks if the auto-save period has elapsed and saves if needed
static void maybe_autosave(bool force=false) {
  unsigned long now = millis();
  if (!force && (now - lastSaveMs) < SAVE_PERIOD_MS) return;
  (void)save_to_storage();
  lastSaveMs = now;
}

// =========================== Beeper (Non-Blocking, Class-Based) ===========================
// Replaces the old blocking beep() that used delay(). Plays short, distinct cues without blocking.
struct BeepNote { uint16_t freq; uint16_t dur; uint16_t gap; }; // Frequency, ON duration, OFF duration

class Beeper {
public:
  // Initialize the beeper pin
  static void begin(uint8_t pin) {
    buzPin = pin;
    pinMode(buzPin, OUTPUT);
    digitalWrite(buzPin, LOW);
  }
  // Start playing a sequence of notes
  static void play(const BeepNote* seq, uint8_t count) {
    if (!count) { stop(); return; }
    seqPtr = seq; seqLen = count; idx = 0; phase = 0; tMark = millis(); active = true;
    startOnPhase();
  }
  // Stop playback immediately
  static void stop() {
  #if USE_PASSIVE_BUZZER
    noTone(buzPin);
  #else
    digitalWrite(buzPin, LOW);
  #endif
    active = false;
  }
  // Called repeatedly in loop() to progress the playback state machine
  static void update() {
    if (!active) return;
    unsigned long now = millis();
    const BeepNote &n = seqPtr[idx];
    if (phase == 0) { // ON phase
      if ((now - tMark) >= n.dur) {
      #if USE_PASSIVE_BUZZER
        noTone(buzPin);
      #else
        digitalWrite(buzPin, LOW);
      #endif
        phase = 1; tMark = now; // Switch to OFF phase
      }
    } else {          // OFF phase (gap)
      if ((now - tMark) >= n.gap) {
        if (++idx >= seqLen) { stop(); return; } // End of sequence
        phase = 0; tMark = now; startOnPhase();  // Start next note
      }
    }
  }

  // Convenience triggers for various sound events
  static void click();
  static void startTone();
  static void pauseTone();
  static void resetConfirmTone();
  static void countdownTick();

private:
  // Starts the tone/pin for the current note
  static void startOnPhase() {
    const BeepNote &n = seqPtr[idx];
  #if USE_PASSIVE_BUZZER
    if (n.freq) tone(buzPin, n.freq, n.dur); else noTone(buzPin); // Passive uses tone()
  #else
    if (n.freq) digitalWrite(buzPin, HIGH); else digitalWrite(buzPin, LOW); // Active uses digitalWrite
  #endif
  }

  // Runtime state variables
  static uint8_t buzPin;
  static const BeepNote* seqPtr;
  static uint8_t seqLen, idx, phase; // phase: 0=ON, 1=OFF
  static unsigned long tMark;
  static bool active;

  // Pattern tables (defined out-of-class)
  static const BeepNote kClick[];
  static const BeepNote kStart[];
  static const BeepNote kPause[];
  static const BeepNote kResetConfirm[];
  static const BeepNote kTick[];
};

// ---- Beeper static data definitions (required) ----
uint8_t        Beeper::buzPin   = 255;
const BeepNote*Beeper::seqPtr   = nullptr;
uint8_t        Beeper::seqLen   = 0;
uint8_t        Beeper::idx      = 0;
uint8_t        Beeper::phase    = 0;
unsigned long  Beeper::tMark    = 0;
bool           Beeper::active   = false;

#if USE_PASSIVE_BUZZER
// Passive piezo: musical-ish tones
const BeepNote Beeper::kClick[]        = { {1800, 18,  8} };
const BeepNote Beeper::kStart[]        = { {1100, 35, 20}, {1500, 55, 0} };
const BeepNote Beeper::kPause[]        = { { 900, 45,  0} };
const BeepNote Beeper::kResetConfirm[] = { {1600, 60, 40}, {1200, 90, 0} };
const BeepNote Beeper::kTick[]         = { {1200, 16,  0} };
#else
// Active buzzer: freq field just means "on" (1 = ON)
const BeepNote Beeper::kClick[]        = { {1, 16,  0} };
const BeepNote Beeper::kStart[]        = { {1, 30, 25}, {1, 45, 0} };
const BeepNote Beeper::kPause[]        = { {1, 40,  0} };
const BeepNote Beeper::kResetConfirm[] = { {1, 70, 40}, {1,120, 0} };
const BeepNote Beeper::kTick[]         = { {1, 12,  0} };
#endif

// ---- Beeper convenience methods (use the tables above) ----
// Calls the main play function with the specified pattern
void Beeper::click()            { play(kClick,        sizeof(kClick)/sizeof(kClick[0])); }
void Beeper::startTone()        { play(kStart,        sizeof(kStart)/sizeof(kStart[0])); }
void Beeper::pauseTone()        { play(kPause,        sizeof(kPause)/sizeof(kPause[0])); }
void Beeper::resetConfirmTone() { play(kResetConfirm, sizeof(kResetConfirm)/sizeof(kResetConfirm[0])); }
void Beeper::countdownTick()    { play(kTick,         sizeof(kTick)/sizeof(kTick[0])); }

// =========================== UI Helpers ===========================
// Helper function to print a number with leading zero if less than 10
inline void print2(uint8_t v){ if (v<10) lcd.print('0'); lcd.print(v); }

// Draws the standard header (e.g., "Amplifier 1") on row 0
static void draw_header_normal() {
  lcd.setCursor(0,0);
  lcdPrint_P(kTxtHeaderPrefix);
  lcd.print(uint16_t(activeIndex+1)); // Display 1-based index (1-4)
}

#if USE_COUNTDOWN
// Draws the reset countdown header (e.g., "Hold reset: 3") on row 0
static void draw_header_hold(uint8_t secLeft) {
  lcd.setCursor(0,0);
  lcdPrint_P(kTxtHoldReset);
  lcd.print(secLeft);
  uint8_t used = TXT_HOLD_RESET_LEN + (secLeft >= 10 ? 2 : 1);
  for (uint8_t i=used; i<LCD_COLS; ++i) lcd.print(' '); // Clear the rest of the line
}
#endif

// Draws the main time display (HHH:MM:SS) on row 1
static void draw_time(uint32_t h, uint8_t m, uint8_t s){
  if(h>999) h=999; // Cap display at 999 hours for 16-char screen
  lcd.setCursor(3,1); // Start position on row 1
  // Print hours with leading zeros (up to 3 digits)
  if(h<10) lcd.print(F("00"));
  else if(h<100) lcd.print(F("0"));
  lcd.print(h);
  // Print minutes and seconds with colons and leading zeros
  lcd.print(':'); print2(m); lcd.print(':'); print2(s);
}

// Checks the dirty cache (ui struct) against the current state and redraws if necessary
static void maybe_redraw() {
  // Assemble current status flags
  uint8_t newFlags = (isRunning?1:0)
  #if USE_COUNTDOWN
                    | (holdCountdownActive?2:0)
  #endif
                    ;

  // Check if the header needs to be redrawn (countdown state or seconds left changed)
  bool headerChanged =
  #if USE_COUNTDOWN
      ((ui.flags & 2) != (newFlags & 2)) ||
      (ui.holdSec != (holdCountdownActive ? holdSecsLeft : 255));
  #else
      false;
  #endif

  // Check if the main timer display or active counter index changed
  bool otherChanged = (ui.shownIndex != activeIndex) ||
                      (ui.shownH     != counters[activeIndex].h) ||
                      (ui.shownM     != counters[activeIndex].m) ||
                      (ui.shownS     != counters[activeIndex].s) ||
                      ((ui.flags & 1) != (newFlags & 1));

  if (!(headerChanged || otherChanged)) return; // Nothing changed, skip redraw

  // Redraw the header (Normal or Countdown)
  #if USE_COUNTDOWN
    if (holdCountdownActive) draw_header_hold(holdSecsLeft);
    else                     draw_header_normal();
  #else
    draw_header_normal();
  #endif

  // Redraw the time and update the cache
  draw_time(counters[activeIndex].h, counters[activeIndex].m, counters[activeIndex].s);

  ui.shownIndex = activeIndex;
  ui.shownH     = counters[activeIndex].h;
  ui.shownM     = counters[activeIndex].m;
  ui.shownS     = counters[activeIndex].s;
  ui.flags      = newFlags;
  #if USE_COUNTDOWN
    ui.holdSec = holdCountdownActive ? holdSecsLeft : 255;
  #endif
}

// =========================== Button Event Handlers ===========================
// Handler for a confirmed button press (fall event)
static void on_button_fall(uint8_t idx){
  armedIndex = idx;
  pressStartMs = millis();
  armedMask = (1 << idx);
  Beeper::click(); // Audible feedback

  if (idx == activeIndex) {
    togglePending = true; // This is the active counter, prepare for run/pause toggle on release
  } else {
  #if USE_COUNTDOWN
    holdCountdownActive = false; // Cancel any existing countdown
  #endif
    togglePending = false;
    isRunning = false;       // Stop the previously active counter
    maybe_autosave(true);    // Save immediately when switching timers
    activeIndex = idx;       // Switch to the new counter
  }
}

// Handler for a confirmed button release (rise event)
static void on_button_rise(uint8_t idx){
  // Disarm the button if it's the one that was held
  if (armedIndex == idx) { armedIndex = 255; armedMask = 0; }

#if USE_COUNTDOWN
  // If released during countdown, only cancel the countdown
  if (holdCountdownActive) {
    holdCountdownActive = false;
    togglePending = false;
    return;
  }
#endif

  // If it's the active counter and a toggle was pending (not long-held)
  if (idx == activeIndex && togglePending) {
    togglePending = false;
    isRunning = !isRunning; // Toggle run/pause state
    if (isRunning) {
      zeroTime = millis(); // Reset time anchor for accurate timing
      Beeper::startTone();
    } else {
      maybe_autosave(true); // Immediate save on pause
      Beeper::pauseTone();
    }
  }
}

// =========================== Hold-to-Reset ===========================
// Checks if the active button has been held long enough to initiate a reset
static void check_hold_to_reset() {
  // Only proceed if a button is armed, is the active counter, and is still pressed
  if (armedIndex == 255) return;
  if (!(armedMask & (1 << armedIndex))) return;
  if (armedIndex != activeIndex) return;
  Deb& d = debs[armedIndex];
  if (Debounce::read(d) != LOW) return;

  unsigned long held = millis() - pressStartMs;

  // Check for full reset condition (3 seconds)
  if (held >= RESET_HOLD_MS) {
    counters[activeIndex].h = 0;
    counters[activeIndex].m = 0;
    counters[activeIndex].s = 0;
    runAccumMs = 0;
    isRunning  = false;
    armedIndex = 255; armedMask = 0;
    togglePending = false;
  #if USE_COUNTDOWN
    holdCountdownActive = false;
  #endif
    Beeper::resetConfirmTone();
    maybe_autosave(true); // Save the zeroed counter immediately
    return;
  }

#if USE_COUNTDOWN
  // Handle visual countdown display
  static uint8_t lastShown = 255;
  if (held >= COUNTDOWN_SHOW_DELAY_MS) {
    unsigned long remain = RESET_HOLD_MS - held;
    uint8_t secsLeft = (remain + 999UL) / 1000UL; // Calculate ceiling of remaining seconds
    if (secsLeft==0) secsLeft=1; // Ensure 1 is the last second shown before reset
    holdCountdownActive = true;
    holdSecsLeft = secsLeft;
    if (secsLeft != lastShown) { Beeper::countdownTick(); lastShown = secsLeft; } // Beeper tick on new second
    togglePending = false; // Prevent run/pause when countdown is active
  } else {
    holdCountdownActive = false;
    lastShown = 255;
  }
#endif
}

// =========================== Two-line Startup Animation ===========================
#if ENABLE_STARTUP_ANIM
// Custom character definitions for a 5-level progress bar
static const uint8_t kBarLevels[6][8] PROGMEM = {
  { 0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000 }, // 0: Empty
  { 0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000 }, // 1: 1-pixel bar
  { 0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000 }, // 2
  { 0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100 }, // 3
  { 0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110 }, // 4
  { 0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111 }  // 5: Full cell
};

// Loads the custom bar characters into the LCD's CGRAM
static void lcdLoadBarChars() {
  uint8_t buf[8];
  for (uint8_t i = 0; i < 6; ++i) {
    for (uint8_t r = 0; r < 8; ++r) buf[r] = pgm_read_byte(&kBarLevels[i][r]);
    lcd.createChar(i, buf);
  }
}

// Draws a single row of the progress bar animation
static inline void drawBarRow(uint8_t row, uint8_t cells, uint8_t fullCells, uint8_t partial) {
  lcd.setCursor(0, row);
  for (uint8_t i = 0; i < fullCells && i < cells; ++i) lcd.write((uint8_t)5); // Write full cells
  if (fullCells < cells) {
    lcd.write(partial); // Write the partial cell (0-4)
    for (uint8_t i = fullCells + 1; i < cells; ++i) lcd.write((uint8_t)0); // Write empty cells
  }
}

// Main function to run the progress bar startup animation
static void lcdStartupAnim(uint16_t total_ms = 1200) {
  lcd.clear();
  lcdLoadBarChars();
  const uint8_t  cells = LCD_COLS;
  const uint16_t steps = cells * 5; // Total animation frames (cells * sub-steps)
  const uint16_t frameDelay = steps ? (total_ms / steps) : total_ms;

  for (uint16_t step = 0; step <= steps; ++step) {
    const uint8_t fullCells = step / 5;
    const uint8_t partial   = step % 5;
    drawBarRow(0, cells, fullCells, partial); // Draw on row 0
    drawBarRow(1, cells, fullCells, partial); // Draw on row 1
    if (frameDelay) delay(frameDelay); // Delay for frame rate control
  }
  lcd.clear();
}
#endif

// =========================== Setup & Loop ===========================
void setup() {
  // Initialize LCD and contrast
  lcd.begin(LCD_COLS, LCD_ROWS); // Start LCD library
  pinMode(PIN_CONTRAST_PWM, OUTPUT);
  analogWrite(PIN_CONTRAST_PWM, CONTRAST_PWM_DUTY); // Set contrast PWM duty cycle

#if ENABLE_STARTUP_ANIM
  lcdStartupAnim(80); // Run brief progress bar animation
#endif

  // Beeper init (non-blocking)
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);
  Beeper::begin(PIN_BEEPER); // Initialize Beeper class

  // Initialize all 4 button debouncers
  for (uint8_t i=0;i<NUM_COUNTERS;++i) Debounce::init(debs[i], PIN_BUTTONS[i]);

#if USE_FRAM
  // Initialize I2C and check for FRAM
  Wire.begin();
  if (!fram.begin(FRAM_I2C_ADDR)) { // Check if FRAM chip responds
    useFRAM = false;
    lcd.setCursor(0,0); lcdPrint_P(kTxtFRAMNotFound);
    lcd.setCursor(0,1); lcdPrint_P(kTxtUsingEEPROM);
    delay(700); lcd.clear();
  } else {
    lcd.setCursor(0,0); lcdPrint_P(kTxtUsingFRAM);
    delay(500); lcd.clear();
  }
#endif

  // Load counters from persistent storage
  load_from_storage_or_default();

  // Initial UI drawing and state cache initialization
  draw_header_normal();
  draw_time(counters[activeIndex].h, counters[activeIndex].m, counters[activeIndex].s);
  ui.shownIndex = activeIndex;
  ui.shownH = counters[activeIndex].h;
  ui.shownM = counters[activeIndex].m;
  ui.shownS = counters[activeIndex].s;
  ui.flags = (isRunning ? 1 : 0);
#if USE_COUNTDOWN
  ui.flags |= (holdCountdownActive ? 2 : 0);
  ui.holdSec = 255;
#endif

  zeroTime   = millis(); // Set time anchor for time accumulation
  lastSaveMs = millis(); // Set time anchor for auto-save
}

void loop() {
  // 1. Handle all button events (debouncing, press, release)
  for (uint8_t i=0;i<NUM_COUNTERS;++i) {
    Debounce::update(debs[i]);
    if (Debounce::fell(debs[i])) on_button_fall(i);
    if (Debounce::rose(debs[i])) on_button_rise(i);
  }

  // 2. Check for hold-to-reset (must be done before time accumulation)
  check_hold_to_reset();

  // 3. Time accumulation (only if running)
  if (isRunning) {
    unsigned long now = millis();
    // Calculate elapsed time since last loop iteration
    unsigned long elapsed = now - zeroTime;
    zeroTime = now; // Anchor is now the current time
    runAccumMs += elapsed;

    // Roll accumulated milliseconds into H/M/S
    while (runAccumMs >= 1000) {
      runAccumMs -= 1000;
      // 3A: Increment H/M/S without div/mod (performance optimization)
      Counter& c = counters[activeIndex];
      // Check for second, minute, and hour roll-over
      if (++c.s == 60) { c.s = 0; if (++c.m == 60) { c.m = 0; ++c.h; } }
    }
    maybe_autosave(false); // Check and perform auto-save if period elapsed
  }

  // 4. UI: Redraw the display only if the state (time, index, running) has changed
  maybe_redraw();

  // 5. Beeper progression (non-blocking): handles playing any active tone sequence
  Beeper::update();
}
