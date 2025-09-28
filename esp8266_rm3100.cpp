#include <SPI.h> // Library for SPI communication
#include <math.h> // Library for mathematical functions like atan2

// --- ESP8266 Pin Definitions ---
#define PIN_DRDY 5  // Data Ready Pin, connected to GPIO5 (D1 on NodeMCU)
#define PIN_CS   15 // Chip Select Pin, connected to GPIO15 (D8 on NodeMCU)

// --- RM3100 Register Definitions ---
#define RM3100_POLL_REG   0x00
#define RM3100_CMM_REG    0x01
#define RM3100_CCX1_REG   0x04
#define RM3100_CCX0_REG   0x05
#define RM3100_STATUS_REG 0x34
#define RM3100_REVID_REG  0x36

// --- Configuration Settings ---
#define initialCC 200 // Set the initial cycle count (e.g., 200)

// --- CALIBRATION OFFSETS ---
// **IMPORTANT**: You must find these values for your specific setup.
// See the instructions after the code for how to find these.
// For now, these are just placeholders.
float mag_offset_x = 0.0;
float mag_offset_y = 0.0;


uint16_t cycleCount;
float gain;

/**
 * @brief Reads a single 8-bit value from an RM3100 register.
 * @param addr The 7-bit address of the register to read from.
 * @return The 8-bit data read from the register.
 */
uint8_t readReg(uint8_t addr) {
  uint8_t data = 0;
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr | 0x80); // Set MSB to 1 for a read operation
  data = SPI.transfer(0);    // Send dummy byte to receive data
  digitalWrite(PIN_CS, HIGH);
  return data;
}

/**
 * @brief Writes a single 8-bit value to an RM3100 register.
 * @param addr The 7-bit address of the register to write to.
 * @param data The 8-bit data to write.
 */
void writeReg(uint8_t addr, uint8_t data) {
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(addr & 0x7F); // Set MSB to 0 for a write operation
  SPI.transfer(data);
  digitalWrite(PIN_CS, HIGH);
}

/**
 * @brief Changes the cycle count for all three axes (X, Y, Z).
 * @param newCC The new 16-bit cycle count value.
 */
void changeCycleCount(uint16_t newCC) {
  uint8_t CCMSB = (newCC >> 8) & 0xFF; // Most significant byte
  uint8_t CCLSB = newCC & 0xFF;        // Least significant byte

  digitalWrite(PIN_CS, LOW);
  SPI.transfer(RM3100_CCX1_REG & 0x7F); // Start writing at the CCX1 register
  SPI.transfer(CCMSB); SPI.transfer(CCLSB); // X-axis
  SPI.transfer(CCMSB); SPI.transfer(CCLSB); // Y-axis
  SPI.transfer(CCMSB); SPI.transfer(CCLSB); // Z-axis
  digitalWrite(PIN_CS, HIGH);
}

void setup() {
  pinMode(PIN_DRDY, INPUT);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH); // Deselect the sensor initially

  Serial.begin(115200);
  delay(100);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  uint8_t revid = readReg(RM3100_REVID_REG);
  if (revid != 0x22) {
    Serial.println("RM3100 Magnetometer not found!");
    while(1) delay(100); // Halt if sensor not found
  }
  Serial.println("RM3100 Found. Initializing...");

  changeCycleCount(initialCC);

  uint8_t cc_msb = readReg(RM3100_CCX1_REG);
  uint8_t cc_lsb = readReg(RM3100_CCX0_REG);
  cycleCount = (cc_msb << 8) | cc_lsb;
  gain = (0.3671 * (float)cycleCount) + 1.5;

  Serial.print("Cycle Counts = "); Serial.println(cycleCount);
  Serial.print("Gain = "); Serial.println(gain);
  Serial.println("Starting compass readings...");

  writeReg(RM3100_CMM_REG, 0x79); // All axes, DRDY enabled
}

void loop() {
  // Wait until the sensor has new data ready
  while ((readReg(RM3100_STATUS_REG) & 0x80) == 0) {
      delay(1);
  }

  // Read all 9 bytes of measurement data
  digitalWrite(PIN_CS, LOW);
  SPI.transfer(0xA4);
  uint8_t x2 = SPI.transfer(0); uint8_t x1 = SPI.transfer(0); uint8_t x0 = SPI.transfer(0);
  uint8_t y2 = SPI.transfer(0); uint8_t y1 = SPI.transfer(0); uint8_t y0 = SPI.transfer(0);
  uint8_t z2 = SPI.transfer(0); uint8_t z1 = SPI.transfer(0); uint8_t z0 = SPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);

  // Convert 24-bit readings to 32-bit signed integers
  int32_t x = (x2 << 16) | (x1 << 8) | x0;
  int32_t y = (y2 << 16) | (y1 << 8) | y0;
  
  if (x & 0x00800000) { x |= 0xFF000000; }
  if (y & 0x00800000) { y |= 0xFF000000; }

  // Convert raw counts to microTesla
  float x_uT = (float)x / gain;
  float y_uT = (float)y / gain;
  
  // --- HEADING CALCULATION ---

  // 1. Apply calibration offsets
  float calibrated_x = x_uT - mag_offset_x;
  float calibrated_y = y_uT - mag_offset_y;

  // 2. Calculate heading in radians using atan2
  float heading = atan2(calibrated_y, calibrated_x);

  // 3. Convert heading to degrees
  float headingDegrees = heading * 180.0 / M_PI;

  // 4. Normalize to a 0-360 degree range
  if (headingDegrees < 0) {
    headingDegrees += 360.0;
  }

  // --- Display the final result ---
  Serial.print("Compass Heading: ");
  Serial.print(headingDegrees);
  Serial.println(" degrees");

  delay(250); // Update heading 4 times per second
}

