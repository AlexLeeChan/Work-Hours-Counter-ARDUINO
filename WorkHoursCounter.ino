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

#include <LiquidCrystal.h>
#include <EEPROM.h>

// =========================== Feature Flags ===========================

// ENABLE_STARTUP_ANIM: Shows a smooth two-line progress bar on boot
// Provides visual feedback that system is initializing
// Duration: ~80ms (configurable in setup())
#define ENABLE_STARTUP_ANIM  1

// USE_FRAM: Enable FRAM (Ferroelectric RAM) instead of EEPROM
// FRAM advantages: unlimited write cycles, faster writes, no wear leveling needed
// EEPROM limitations: ~100,000 write cycles per cell
// Set to 1 if Adafruit FRAM I2C breakout is connected
#define USE_FRAM             0

// USE_COUNTDOWN: Enables visual countdown timer during hold-to-reset
// When enabled (1):
//   - After holding reset button for 1 second, displays "Hold reset: X" countdown
//   - Shows remaining seconds (3, 2, 1) until reset completes at 3 seconds
//   - Prevents run/pause toggle once countdown begins (visual feedback only)
//   - Provides clear indication that reset is in progress
// When disabled (0):
//   - Reset still works at 3 seconds, but without visual countdown
//   - Saves ~200 bytes of flash memory
#define USE_COUNTDOWN        1

// VERIFY_WRITES: Enables read-back verification after writing to storage
// When enabled (1):
//   - After each EEPROM/FRAM write, reads data back and verifies it matches
//   - Compares sequence numbers to ensure write succeeded
//   - Slightly increases write time (~1-2ms) but adds safety
// When disabled (0):
//   - Assumes all writes succeed (faster but less safe)
//   - Use only if storage is known to be 100% reliable
#define VERIFY_WRITES        1

// USE_PASSIVE_BUZZER: Select beeper hardware type
// 0 = ACTIVE buzzer (on/off only; we simulate "tones" as ON pulses)
// 1 = PASSIVE piezo (supports tone() with frequencies)
#define USE_PASSIVE_BUZZER   0

// Conditional includes based on feature flags
#if USE_FRAM
  #include <Wire.h>
  #include <Adafruit_FRAM_I2C.h>
#endif

// =========================== PROGMEM Support ===========================
// Platform-specific includes for storing constants in flash memory instead of RAM
#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#else
  #include <pgmspace.h>
#endif

// =========================== Customizable Text ===========================
// LCD display strings stored in PROGMEM (flash) to save precious RAM
// Each string reserves 1 byte in RAM (pointer) vs full string length
const char kTxtHeaderPrefix[]   PROGMEM = "  Amplifier ";
const char kTxtHoldReset[]      PROGMEM = "Hold reset: ";
constexpr uint8_t TXT_HOLD_RESET_LEN = sizeof(kTxtHoldReset) - 1; // Compile-time length (excludes null terminator)

// Storage detection messages shown on startup if FRAM is enabled
const char kTxtUsingFRAM[]      PROGMEM = " Using FRAM    ";
const char kTxtFRAMNotFound[]   PROGMEM = " FRAM not found";
const char kTxtUsingEEPROM[]    PROGMEM = " Using EEPROM  ";

// =========================== Hardware Configuration ===========================
constexpr uint8_t  NUM_COUNTERS        = 4; // Number of independent work hour counters
constexpr uint8_t  LCD_COLS            = 16; // LCD width in characters
constexpr uint8_t  LCD_ROWS            = 2; // LCD height in rows

// LCD pin assignments (standard 4-bit parallel interface)
constexpr uint8_t  PIN_LCD_RS          = 2; // Register Select (command/data)
constexpr uint8_t  PIN_LCD_EN          = 8; // Enable (clock)
constexpr uint8_t  PIN_LCD_D4          = 4; // Data bit 4
constexpr uint8_t  PIN_LCD_D5          = 5; // Data bit 5
constexpr uint8_t  PIN_LCD_D6          = 6; // Data bit 6
constexpr uint8_t  PIN_LCD_D7          = 7; // Data bit 7

// Peripheral pin assignments
constexpr uint8_t  PIN_CONTRAST_PWM    = 3; // PWM output for LCD contrast control (0-255)
constexpr uint8_t  PIN_BEEPER          = 13; // Active buzzer for button feedback (or passive piezo)
constexpr uint8_t  PIN_BUTTONS[NUM_COUNTERS] = {11, 12, 9, 10}; // Button inputs (active LOW with internal pullup)

// =========================== Timing Constants ===========================
constexpr uint8_t  DEBOUNCE_MS         = 10; // Button debounce time (mechanical switch settling)
constexpr uint8_t  CONTRAST_PWM_DUTY   = 0; // LCD contrast PWM duty cycle (0=max contrast, 255=min)
constexpr unsigned long SAVE_PERIOD_MS = 10000UL; // Auto-save interval (10 seconds) to reduce EEPROM wear
constexpr unsigned long RESET_HOLD_MS  = 3000UL; // Hold button this long to reset counter
constexpr unsigned long COUNTDOWN_SHOW_DELAY_MS = 1000UL; // Delay before showing countdown (prevents accidental display)

// FRAM I2C address (standard for Adafruit FRAM breakout)
#if USE_FRAM
  constexpr uint8_t FRAM_I2C_ADDR = 0x50;
#endif

// =========================== Global Objects ===========================
// LCD instance using 4-bit mode (saves 4 data pins vs 8-bit mode)
LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);

// FRAM instance and storage selection flag
#if USE_FRAM
Adafruit_FRAM_I2C fram;
bool useFRAM = true; // Will be set to false if FRAM initialization fails
#else
bool useFRAM = false;
#endif

// =========================== PROGMEM Helper Function ===========================
// Prints a string from PROGMEM (flash memory) to LCD character-by-character
// This avoids copying entire string to RAM before printing
static inline void lcdPrint_P(const char* p) {
  char c;
  while ((c = pgm_read_byte(p++)) != 0) lcd.print(c);
}

// =========================== Button Debouncing System ===========================

// Deb: Debouncer state structure for a single button
// Tracks both raw and stable (debounced) button states, plus edge events
struct Deb {
  uint8_t pin; // Arduino pin number this debouncer monitors
  uint8_t stable; // Current debounced state (HIGH/LOW) - noise filtered
  uint8_t last; // Last raw pin reading (may contain noise/bounce)
  unsigned long tmark; // Timestamp (millis) of last state change
  uint8_t fellEv; // Edge event flag: 1 if HIGH→LOW transition occurred
  uint8_t roseEv; // Edge event flag: 1 if LOW→HIGH transition occurred
};

// Debounce: Static utility class for button debouncing logic
// Implements time-based debouncing: state changes only accepted after
// DEBOUNCE_MS milliseconds of stability to filter mechanical bounce
struct Debounce {
  static void init(Deb& d, uint8_t pin) {
    d.pin = pin;
    pinMode(pin, INPUT_PULLUP); // Enable internal pull-up resistor (~20-50kΩ)
    uint8_t r = digitalRead(pin);
    d.stable = d.last = r; // Initialize both states to current reading
    d.tmark = millis(); // Record initialization time
    d.fellEv = d.roseEv = 0; // Clear edge event flags
  }
  // update: Poll button state and update debounced value
  static void update(Deb& d) {
    uint8_t r = digitalRead(d.pin);
    unsigned long now = millis();
    if (r != d.last) { d.last = r; d.tmark = now; } // Raw state changed - restart debounce timer
    else if (r != d.stable && (now - d.tmark) >= DEBOUNCE_MS) { // Debounce period elapsed
      d.stable = r; // Update debounced state
      if (r == LOW)  d.fellEv = 1; // Button pressed (HIGH→LOW with pullup)
      else           d.roseEv = 1; // Button released (LOW→HIGH)
    }
  }
  // fell: Check for HIGH→LOW edge (button press with INPUT_PULLUP)
  static bool fell(Deb& d){ if (d.fellEv){ d.fellEv=0; return true; } return false; }
  // rose: Check for LOW→HIGH edge (button release with INPUT_PULLUP)
  static bool rose(Deb& d){ if (d.roseEv){ d.roseEv=0; return true; } return false; }
  // read: Get current debounced button state (LOW = pressed, HIGH = released)
  static uint8_t read(const Deb& d){ return d.stable; }
};

// Debouncer instances - one per counter button
Deb debs[NUM_COUNTERS];

// =========================== State (3A: H/M/S) ===========================
// Counter: Stores elapsed time as separate Hour/Minute/Second fields
// Optimization: Avoids expensive division/modulo in loop() by incrementing directly
struct Counter { uint32_t h; uint8_t m; uint8_t s; };

// Array of all counters - one per amplifier/workstation (4 total)
Counter counters[NUM_COUNTERS] = {};

// Currently active counter (0-3) - which one is displayed and accumulating time
uint8_t  activeIndex  = 0;

// Running state - true means active counter is counting up
bool     isRunning    = false;

// High-precision timekeeping variables (sub-second accuracy)
unsigned long zeroTime   = 0; // Last millis() reading - used to calculate elapsed time
uint32_t      runAccumMs = 0; // Accumulated milliseconds waiting to roll into seconds

// Persistent storage tracking
unsigned long lastSaveMs   = 0; // millis() timestamp of last save (for 10-second auto-save)
uint16_t      saveSeq      = 0; // Save sequence number for wear leveling (wraps around at 65535)

// =========================== Button Press/Hold State Machine ===========================
unsigned long pressStartMs = 0; // millis() when button was first pressed (for hold timing)
uint8_t       armedIndex   = 255; // Which button is held (255 = none pressed)
uint8_t       armedMask    = 0; // Bitmask tracking armed buttons (bit i = button i)
bool          togglePending= false; // True = run/pause toggle will happen on button release

#if USE_COUNTDOWN
// Countdown visualization state (appears after 1 second of holding reset button)
bool          holdCountdownActive = false; // True when "Hold reset: X" is shown on screen
uint8_t       holdSecsLeft        = 0; // Countdown value shown (3→2→1)
#endif

// =========================== UI State Cache (Dirty-Bit Pattern) ===========================
// Stores what's currently on the LCD to avoid slow, redundant rewrites
struct UiState {
  uint8_t  shownIndex  = 255; // Last drawn counter index (255 = never drawn)
  uint32_t shownH      = 0xFFFFFFFFUL; // Last drawn hours (invalid initial value forces first draw)
  uint8_t  shownM      = 255; // Last drawn minutes
  uint8_t  shownS      = 255; // Last drawn seconds
  uint8_t  flags       = 0; // Composite state: bit0=running, bit1=holdCountdownActive
  uint8_t  holdSec     = 255; // Last drawn countdown seconds (255 = not showing countdown)
} ui;

// =========================== CRC-16/X25 Checksum Implementation ===========================
// Data integrity protection for EEPROM/FRAM writes
// Algorithm: CRC-16/X25 (polynomial 0x8408)
using crc_t = uint16_t;
constexpr size_t CRC_SZ = sizeof(crc_t);
static uint16_t crc16_update(uint16_t crc, uint8_t b){
  crc ^= b;
  for(uint8_t i=0;i<8;++i) crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
  return crc;
}
static uint16_t crc16_compute(const uint8_t* p, size_t n){
  uint16_t c = 0xFFFF; // Standard CRC-16/X25 initial value
  for(size_t i=0;i<n;++i) c = crc16_update(c, p[i]);
  return ~c; // Final XOR (invert all bits)
}

// =========================== SaveImage Structure ===========================
// Version 5 optimization: Compact binary format for persistent storage
constexpr uint16_t SAVE_MAGIC    = 0xA11F; // Format identifier
constexpr uint16_t SAVE_VERSION  = 1; // Format version
struct SaveImage {
  uint16_t magic;
  uint16_t version;
  uint16_t seq; // Write sequence number (for dual-slot selection)
  uint32_t hours[NUM_COUNTERS];
  uint8_t  minutes[NUM_COUNTERS];
  uint8_t  seconds[NUM_COUNTERS];
  crc_t    crc; // CRC-16 checksum (must be last field)
} __attribute__((packed)); // Prevent compiler padding - exact memory layout

// EEPROM layout: Two alternating slots for wear leveling
const int EEPROM_SLOT_SIZE = sizeof(SaveImage);

// Helper to compute the base address for a given slot index.
static inline int eeprom_slot_base(uint8_t slot){ return int(slot) * EEPROM_SLOT_SIZE; }

// Compare two uint16_t sequence numbers with rollover handling.
// Returns true if a is newer than b.
static inline bool seq_newer(uint16_t a, uint16_t b){
  return uint16_t(a - b) < 0x8000;
}

// =========================== EEPROM Storage Class (Multi-Slot Ring) ===========================
// Manages multi-slot wear leveling, CRC validation, and write verification
#ifndef EEPROM_NUM_SLOTS
#define EEPROM_NUM_SLOTS 16 // Configure how many EEPROM slots to use for wear leveling
#endif

class EepromStore {
public:
  // read_bytes: Raw byte-level EEPROM read
  static void read_bytes(int addr, uint8_t* buf, int len) {
    for (int i=0;i<len;++i) buf[i] = EEPROM.read(addr+i);
  }
  // write_bytes: Smart EEPROM write - only writes changed bytes
  static void write_bytes(int addr, const uint8_t* buf, int len) {
    for (int i=0;i<len;++i) EEPROM.update(addr+i, buf[i]);
  }
  // load_slot: Load and validate SaveImage from one slot
  static bool load_slot(uint8_t slot, SaveImage& img) {
    const int base = eeprom_slot_base(slot);
    read_bytes(base, (uint8_t*)&img, EEPROM_SLOT_SIZE);
    if (img.magic != SAVE_MAGIC || img.version != SAVE_VERSION) return false;
    crc_t calc = crc16_compute((uint8_t*)&img, EEPROM_SLOT_SIZE-CRC_SZ);
    return calc == img.crc;
  }
  // save_to_slot: Write SaveImage to one slot with optional verification
  static bool save_to_slot(uint8_t slot, const SaveImage& img) {
    const int base = eeprom_slot_base(slot);
    write_bytes(base, (const uint8_t*)&img, EEPROM_SLOT_SIZE);
    #if VERIFY_WRITES
      // Read back and verify sequence number matches
      SaveImage v{};
      return load_slot(slot, v) && v.seq == img.seq;
    #else
      return true;
    #endif
  }

  // load_latest: scan all configured slots; return newest valid image and its slot index.
  static bool load_latest(SaveImage& out, uint8_t& outSlot) {
    const uint16_t maxSlotsBySize = EEPROM.length() / EEPROM_SLOT_SIZE;
    const uint8_t  slots = (EEPROM_NUM_SLOTS <= maxSlotsBySize) ? EEPROM_NUM_SLOTS : maxSlotsBySize;

    bool found = false;
    SaveImage best{};
    uint8_t    bestSlot = 0;

    for (uint8_t s=0; s<slots; ++s) {
      SaveImage tmp{};
      if (!load_slot(s, tmp)) continue;
      if (!found || seq_newer(tmp.seq, best.seq)) { best = tmp; bestSlot = s; found = true; }
    }
    if (found) { out = best; outSlot = bestSlot; }
    return found;
  }

  // save_next: write to the slot after the latest valid (or slot 0 if none valid)
  // This yields a simple ring buffer wear leveling across N slots.
  static bool save_next(const SaveImage& img) {
    const uint16_t maxSlotsBySize = EEPROM.length() / EEPROM_SLOT_SIZE;
    const uint8_t  slots = (EEPROM_NUM_SLOTS <= maxSlotsBySize) ? EEPROM_NUM_SLOTS : maxSlotsBySize;

    uint8_t latestSlot = 0;
    SaveImage dummy{};
    bool haveLatest = load_latest(dummy, latestSlot);

    uint8_t target = haveLatest ? (uint8_t)((latestSlot + 1) % slots) : 0;
    return save_to_slot(target, img);
  }
};

// =========================== FRAM Storage Class ===========================
// Simpler than EEPROM - FRAM has unlimited write cycles, so no wear leveling needed
#if USE_FRAM
class FramStore {
public:
  // write_bytes: Raw byte-level FRAM write
  static void write_bytes(uint16_t addr, const uint8_t* buf, uint16_t len) {
    for (uint16_t i=0;i<len;++i) fram.write(addr+i, buf[i]);
  }
  // read_bytes: Raw byte-level FRAM read
  static void read_bytes(uint16_t addr, uint8_t* buf, uint16_t len) {
    for (uint16_t i=0;i<len;++i) buf[i] = fram.read(addr+i);
  }
  // load: Load and validate SaveImage from FRAM (single slot at address 0)
  static bool load(SaveImage& img) {
    read_bytes(0, (uint8_t*)&img, sizeof(img));
    if (img.magic != SAVE_MAGIC || img.version != SAVE_VERSION) return false;
    crc_t calc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);
    return calc == img.crc;
  }
  // save: Write SaveImage to FRAM (always address 0, no wear leveling)
  static bool save(const SaveImage& img) {
    write_bytes(0, (const uint8_t*)&img, sizeof(img));
    #if VERIFY_WRITES
      SaveImage v{};
      return load(v) && v.seq == img.seq;
    #else
      return true;
    #endif
  }
};
#endif

// =========================== Persistence API (High-Level) ===========================

// load_from_storage_or_default: Initialize counters from storage or zero them
// Called once in setup() to restore state from previous session
static void load_from_storage_or_default() {
  SaveImage img{};
  bool ok = false;

  if (useFRAM) {
  #if USE_FRAM
    ok = FramStore::load(img);
    if (!ok) {
      // FRAM empty/corrupt: initialize with zeros and save
      img.magic=SAVE_MAGIC; img.version=SAVE_VERSION; img.seq=0;
      for (uint8_t i=0;i<NUM_COUNTERS;++i){ img.hours[i]=0; img.minutes[i]=0; img.seconds[i]=0; }
      img.crc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);
      (void)FramStore::save(img); // Cast to void = ignore return value
      ok = true;
    }
  #endif
  } else {
    // EEPROM mode: try loading latest from ring buffer
    uint8_t slot=0;
    ok = EepromStore::load_latest(img, slot);
    if (!ok) {
      // EEPROM empty/corrupt: initialize with zeros and save to slot 0
      img.magic=SAVE_MAGIC; img.version=SAVE_VERSION; img.seq=0;
      for (uint8_t i=0;i<NUM_COUNTERS;++i){ img.hours[i]=0; img.minutes[i]=0; img.seconds[i]=0; }
      img.crc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);
      (void)EepromStore::save_to_slot(0, img);
      ok = true;
    }
  }

  // Copy loaded data into runtime counter array
  for (uint8_t i=0;i<NUM_COUNTERS;++i) {
    counters[i].h = img.hours[i];
    counters[i].m = img.minutes[i];
    counters[i].s = img.seconds[i];
  }
  saveSeq = img.seq; // Remember sequence for next save
}

// save_to_storage: Persist all counters with incremented sequence number
static bool save_to_storage() {
  SaveImage img{};
  img.magic=SAVE_MAGIC; 
  img.version=SAVE_VERSION; 
  img.seq=uint16_t(saveSeq+1); // Increment sequence (uint16_t rollover is OK)

  // Copy runtime counters into save image
  for (uint8_t i=0;i<NUM_COUNTERS;++i){
    img.hours[i]   = counters[i].h;
    img.minutes[i] = counters[i].m;
    img.seconds[i] = counters[i].s;
  }
  // Calculate CRC over everything except CRC field itself
  img.crc = crc16_compute((uint8_t*)&img, sizeof(img)-CRC_SZ);

  bool ok = false;
  if (useFRAM) {
  #if USE_FRAM
    ok = FramStore::save(img); // Single-slot FRAM save
  #endif
  } else {
    ok = EepromStore::save_next(img);   // write to next slot in ring for wear leveling
  }
  if (ok) saveSeq = img.seq; // Only commit sequence if write verified
  return ok;
}

// maybe_autosave: Periodic auto-save to minimize data loss on power failure
// Normal: saves every 10 seconds (SAVE_PERIOD_MS)
// force=true: immediate save (used when pausing or switching counters)
static void maybe_autosave(bool force=false) {
  unsigned long now = millis();
  if (!force && (now - lastSaveMs) < SAVE_PERIOD_MS) return; // Not time yet
  (void)save_to_storage(); // Ignore return value - continue even if save fails
  lastSaveMs = now;
}

// =========================== Beeper (Non-Blocking, Class-Based) ===========================
// Replaces the old blocking beep() that used delay(). This version plays short, distinct
// audio cues without blocking loop(), so UI stays responsive and timing stays accurate.
struct BeepNote { uint16_t freq; uint16_t dur; uint16_t gap; };
class Beeper {
public:
  // begin: initialize beeper pin and idle state
  static void begin(uint8_t pin) {
    buzPin = pin;
    pinMode(buzPin, OUTPUT);
    digitalWrite(buzPin, LOW);
  }
  // play: start a new pattern immediately
  static void play(const BeepNote* seq, uint8_t count) {
    if (!count) { stop(); return; }
    seqPtr = seq; seqLen = count; idx = 0; phase = 0; tMark = millis(); active = true;
    startOnPhase(); // kick off first ON phase
  }
  // stop: silence beeper and clear state
  static void stop() {
    #if USE_PASSIVE_BUZZER
      noTone(buzPin);
    #else
      digitalWrite(buzPin, LOW);
    #endif
    active = false;
  }
  // update: advance the pattern timing; must be called every loop() iteration
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
        phase = 1; tMark = now;
      }
    } else { // OFF phase
      if ((now - tMark) >= n.gap) {
        if (++idx >= seqLen) { stop(); return; } // End of pattern
        phase = 0; tMark = now; startOnPhase(); // Next note
      }
    }
  }
  // Convenience one-liners to trigger common UI cues
  static void click()            { play(kClick,        sizeof(kClick)/sizeof(kClick[0])); }
  static void startTone()        { play(kStart,        sizeof(kStart)/sizeof(kStart[0])); }
  static void pauseTone()        { play(kPause,        sizeof(kPause)/sizeof(kPause[0])); }
  static void resetConfirmTone() { play(kResetConfirm, sizeof(kResetConfirm)/sizeof(kResetConfirm[0])); }
  static void countdownTick()    { play(kTick,         sizeof(kTick)/sizeof(kTick[0])); }
private:
  // startOnPhase: engage the ON phase for current note (ACTIVE or PASSIVE)
  static void startOnPhase() {
    const BeepNote &n = seqPtr[idx];
    #if USE_PASSIVE_BUZZER
      if (n.freq) tone(buzPin, n.freq, n.dur); // hardware-timed tone
      else        noTone(buzPin);
    #else
      if (n.freq) digitalWrite(buzPin, HIGH); // Active buzzer ON
      else        digitalWrite(buzPin, LOW);
    #endif
  }
  
  // Internal state (Static members must be defined outside the class in C++)
  static uint8_t buzPin;
  static const BeepNote* seqPtr;
  static uint8_t seqLen, idx, phase; // phase: 0=ON, 1=OFF
  static unsigned long tMark;
  static bool active;

  // Built-in short patterns
  #if USE_PASSIVE_BUZZER
    // Passive piezo: musical-ish tones
    static constexpr BeepNote kClick[]        = { {1800, 18, 8} };
    static constexpr BeepNote kStart[]        = { {1100, 35, 20}, {1500, 55, 0} };
    static constexpr BeepNote kPause[]        = { { 900, 45, 0}  };
    static constexpr BeepNote kResetConfirm[] = { {1600, 60, 40}, {1200, 90, 0} };
    static constexpr BeepNote kTick[]         = { {1200, 16, 0}  };
  #else
    // Active buzzer: treat any non-zero freq as ON (no pitch)
    static constexpr BeepNote kClick[]        = { {1, 16, 0} };
    static constexpr BeepNote kStart[]        = { {1, 30, 25}, {1, 45, 0} };
    static constexpr BeepNote kPause[]        = { {1, 40, 0} };
    static constexpr BeepNote kResetConfirm[] = { {1, 70, 40}, {1, 120, 0} };
    static constexpr BeepNote kTick[]         = { {1, 12, 0} };
  #endif
};

// FIX: Define Beeper static members outside of the class (required by C++ standard)
uint8_t Beeper::buzPin;
const BeepNote* Beeper::seqPtr = nullptr;
uint8_t Beeper::seqLen = 0, Beeper::idx = 0, Beeper::phase = 0;
unsigned long Beeper::tMark = 0;
bool Beeper::active = false;


// =========================== UI Helper Functions ===========================

// print2: Print 2-digit number with leading zero padding
inline void print2(uint8_t v){ if (v<10) lcd.print('0'); lcd.print(v); }

// draw_header_normal: Display standard header line (e.g., "  Amplifier 1")
static void draw_header_normal() {
  lcd.setCursor(0,0); // Row 0, column 0
  lcdPrint_P(kTxtHeaderPrefix); // "  Amplifier " from flash
  lcd.print(uint16_t(activeIndex+1)); // Convert 0-3 to 1-4 for display
}

#if USE_COUNTDOWN
// draw_header_hold: Display countdown during reset button hold (e.g., "Hold reset: 3")
static void draw_header_hold(uint8_t secLeft) {
  lcd.setCursor(0,0);
  lcdPrint_P(kTxtHoldReset); // "Hold reset: "
  lcd.print(secLeft);
  // Clear rest of line
  uint8_t used = TXT_HOLD_RESET_LEN + (secLeft >= 10 ? 2 : 1);
  for (uint8_t i=used; i<LCD_COLS; ++i) lcd.print(' ');
}
#endif

// draw_time: Display time in HHH:MM:SS format on bottom row
static void draw_time(uint32_t h, uint8_t m, uint8_t s){
  if(h>999) h=999; // Cap display at 999 hours
  lcd.setCursor(3,1); // Row 1 (bottom), column 3
  // Print hours with leading zeros to 3 digits
  if(h<10) lcd.print(F("00"));
  else if(h<100) lcd.print(F("0"));
  lcd.print(h);
  lcd.print(':'); print2(m); lcd.print(':'); print2(s);
}

// maybe_redraw: Update LCD only if something changed (Dirty-bit pattern optimization)
static void maybe_redraw() {
  // Build composite flags byte for efficient comparison
  uint8_t newFlags = (isRunning?1:0)
                  #if USE_COUNTDOWN
                     | (holdCountdownActive?2:0)
                  #endif
                     ;
  // Check if header (top line) changed
  bool headerChanged =
  #if USE_COUNTDOWN
      ((ui.flags & 2) != (newFlags & 2)) ||
      (ui.holdSec != (holdCountdownActive ? holdSecsLeft : 255));
  #else
      false;
  #endif
  // Check if time display (bottom line) or counter changed
  bool otherChanged = (ui.shownIndex != activeIndex) ||
                      (ui.shownH     != counters[activeIndex].h) ||
                      (ui.shownM     != counters[activeIndex].m) ||
                      (ui.shownS     != counters[activeIndex].s) ||
                      ((ui.flags & 1) != (newFlags & 1));
  // Early exit if nothing changed
  if (!(headerChanged || otherChanged)) return;

  // Redraw header if changed
  #if USE_COUNTDOWN
    if (holdCountdownActive) draw_header_hold(holdSecsLeft);
    else                     draw_header_normal();
  #else
    draw_header_normal();
  #endif
  // Always redraw time (bottom line) if we got here
  draw_time(counters[activeIndex].h, counters[activeIndex].m, counters[activeIndex].s);

  // Update UI state cache
  ui.shownIndex = activeIndex;
  ui.shownH     = counters[activeIndex].h;
  ui.shownM     = counters[activeIndex].m;
  ui.shownS     = counters[activeIndex].s;
  ui.flags      = newFlags;
  #if USE_COUNTDOWN
    ui.holdSec = holdCountdownActive ? holdSecsLeft : 255;
  #endif
}

// =========================== Button Event Handlers ===========================
static void on_button_fall(uint8_t idx){
  // Record press start time and button index
  armedIndex = idx;
  pressStartMs = millis();
  armedMask = (1 << idx);
  Beeper::click(); // Feedback sound for button press

  if (idx == activeIndex) {
    // Same button pressed: flag for potential run/pause toggle on release
    togglePending = true;
  } else {
    // Different button pressed: switch counter immediately
    #if USE_COUNTDOWN
      holdCountdownActive = false;
    #endif
    togglePending = false;
    isRunning = false; // Pause any running timer when switching
    maybe_autosave(true); // Force save of previous counter
    activeIndex = idx; // Switch to new counter
  }
}
static void on_button_rise(uint8_t idx){
  // Clear armed state if the released button was the armed one
  if (armedIndex == idx) { armedIndex = 255; armedMask = 0; }
  
  #if USE_COUNTDOWN
    // If released during countdown, only clear the state machine, don't toggle run/pause
    if (holdCountdownActive) {
      holdCountdownActive = false;
      togglePending = false;
      return;
    }
  #endif

  // If the released button was the active one AND the toggle was still pending (not held too long)
  if (idx == activeIndex && togglePending) {
    togglePending = false;
    isRunning = !isRunning; // Toggle run/pause state
    if (isRunning) {
      zeroTime = millis(); // Reset time anchor
      Beeper::startTone(); 
    } else {
      maybe_autosave(true); // Force save on pause
      Beeper::pauseTone(); 
    }
  }
}

// =========================== Hold-to-Reset ===========================
static void check_hold_to_reset() {
  // Check only if a button is armed and it's the active counter's button
  if (armedIndex == 255) return;
  if (!(armedMask & (1 << armedIndex))) return;
  if (armedIndex != activeIndex) return;

  Deb& d = debs[armedIndex];
  if (Debounce::read(d) != LOW) return; // Check it's still physically pressed

  unsigned long held = millis() - pressStartMs;

  if (held >= RESET_HOLD_MS) {
    // Reset complete: clear counter and state
    counters[activeIndex].h = 0;
    counters[activeIndex].m = 0;
    counters[activeIndex].s = 0;
    runAccumMs = 0;
    isRunning  = false;
    armedIndex = 255; armedMask = 0;
    togglePending = false;
    #if USE_COUNTDOWN
      holdCountdownActive = false;
    #endif
    Beeper::resetConfirmTone(); // Confirmation sound
    maybe_autosave(true); // Force save of zeroed counter
    return;
  }

  #if USE_COUNTDOWN
  // Handle countdown display and ticks
  static uint8_t lastShown = 255;
  if (held >= COUNTDOWN_SHOW_DELAY_MS) {
    unsigned long remain = RESET_HOLD_MS - held;
    uint8_t secsLeft = (remain + 999UL) / 1000UL; // Ceiling division
    if (secsLeft==0) secsLeft=1; // Ensure it shows '1' at the last second
    holdCountdownActive = true;
    holdSecsLeft = secsLeft;
    if (secsLeft != lastShown) { Beeper::countdownTick(); lastShown = secsLeft; } // Play tick on second change
    togglePending = false; // Disable run/pause toggle once countdown starts
  } else {
    holdCountdownActive = false;
    lastShown = 255;
  }
  #endif
}

// =========================== Two-line Startup Animation ===========================
#if ENABLE_STARTUP_ANIM
// Custom character definitions for progress bar (0-5 filled pixels wide)
static const uint8_t kBarLevels[6][8] PROGMEM = {
  { 0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000 }, // 0: Empty
  { 0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000 }, // 1: 1-pixel bar
  { 0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000 }, // 2
  { 0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100 }, // 3
  { 0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110 }, // 4
  { 0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111 } // 5: Full cell
};
// Loads custom characters 0-5 into the LCD CGRAM
static void lcdLoadBarChars() {
  uint8_t buf[8];
  for (uint8_t i = 0; i < 6; ++i) {
    for (uint8_t r = 0; r < 8; ++r) buf[r] = pgm_read_byte(&kBarLevels[i][r]);
    lcd.createChar(i, buf);
  }
}
// Draws one row of the progress bar
static inline void drawBarRow(uint8_t row, uint8_t cells, uint8_t fullCells, uint8_t partial) {
  lcd.setCursor(0, row);
  for (uint8_t i = 0; i < fullCells && i < cells; ++i) lcd.write((uint8_t)5); // Full cells (char 5)
  if (fullCells < cells) {
    lcd.write(partial); // Partial cell (char 0-4)
    for (uint8_t i = fullCells + 1; i < cells; ++i) lcd.write((uint8_t)0); // Remaining empty cells (char 0)
  }
}
// Main animation routine
static void lcdStartupAnim(uint16_t total_ms = 1200) {
  lcd.clear();
  lcdLoadBarChars();
  const uint8_t cells   = LCD_COLS;
  const uint16_t steps  = cells * 5; // Total steps = 16 cells * 5 sub-steps/cell
  const uint16_t frameDelay = steps ? (total_ms / steps) : total_ms;
  for (uint16_t step = 0; step <= steps; ++step) {
    const uint8_t fullCells = step / 5;
    const uint8_t partial   = step % 5;
    drawBarRow(0, cells, fullCells, partial);
    drawBarRow(1, cells, fullCells, partial);
    if (frameDelay) delay(frameDelay);
  }
  lcd.clear();
}
#endif

// =========================== Setup & Loop ===========================
void setup() {
  // Initialize LCD and contrast
  lcd.begin(LCD_COLS, LCD_ROWS);
  pinMode(PIN_CONTRAST_PWM, OUTPUT);
  analogWrite(PIN_CONTRAST_PWM, CONTRAST_PWM_DUTY);

  #if ENABLE_STARTUP_ANIM
    lcdStartupAnim(80); // Run brief progress bar animation
  #endif

  // Beeper init (non-blocking)
  pinMode(PIN_BEEPER, OUTPUT);
  digitalWrite(PIN_BEEPER, LOW);
  Beeper::begin(PIN_BEEPER); 

  // Initialize all 4 button debouncers
  for (uint8_t i=0;i<NUM_COUNTERS;++i) Debounce::init(debs[i], PIN_BUTTONS[i]);

  #if USE_FRAM
    // Initialize I2C and check for FRAM
    Wire.begin();
    if (!fram.begin(FRAM_I2C_ADDR)) {
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
  ui.shownH = counters[activeIndex].h; ui.shownM = counters[activeIndex].m; ui.shownS = counters[activeIndex].s;
  ui.flags = (isRunning?1:0);
  #if USE_COUNTDOWN
    ui.flags |= (holdCountdownActive?2:0); ui.holdSec = 255;
  #endif

  zeroTime = millis(); // Set time anchor for loop()
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
      if (++c.s == 60) { c.s = 0; if (++c.m == 60) { c.m = 0; ++c.h; } }
    }
    // Check for auto-save condition
    maybe_autosave(false);
  }

  // 4. Update display if state has changed
  maybe_redraw();

  // 5. Keep beeper patterns running (non-blocking audio)
  Beeper::update(); 
}
