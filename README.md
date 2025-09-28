ESP8266 Digital Compass with RM3100 Magnetometer

This project transforms an ESP8266 microcontroller and a PNI RM3100 magnetometer into a precise digital compass. The firmware reads the raw magnetic field data, calibrates it, and calculates a heading in degrees (0-360), which is then output over the serial port.

This is an ideal starting point for projects requiring orientation sensing, such as drones, robotics, or navigation systems.
Features

    High Precision: Utilizes the low-noise RM3100 sensor for stable readings.

    Simple Output: Provides a clear 0-360 degree heading.

    User Calibration: Includes simple-to-follow instructions for calibrating the sensor to counteract local magnetic interference.

    Lightweight: Runs on the resource-constrained ESP8266.

    Arduino IDE Compatible: Easily programmable using the familiar Arduino environment.

Hardware Required

    An ESP8266 Development Board (e.g., NodeMCU, Wemos D1 Mini)

    PNI RM3100 Magnetometer Breakout Board

    Breadboard and Jumper Wires

    Micro-USB Cable

Wiring Diagram

Connect the RM3100 sensor to the ESP8266 board as follows. The SPI pins are standard for the ESP8266.

RM3100 Pin           -                ESP8266 Pin              -            NodeMCU Label
 
VCC                -                     3.3V                -                 3V3

GND                  -                   GND                 -                 GND

CS                  -                   GPIO15              -                  D8
DRDY                -                   GPIO5                -                 D1

SCLK                   -                GPIO14                -                D5

MOSI                    -               GPIO13                 -               D7

MISO                   -                GPIO12                 -               D6
Software Setup

    Install Arduino IDE: Download and install the latest version from the official Arduino website.

    Add ESP8266 Board Manager:

        Go to File > Preferences.

        In "Additional Boards Manager URLs," add: http://arduino.esp8266.com/stable/package_esp8266com_index.json

    Install ESP8266 Core:

        Go to Tools > Board > Boards Manager....

        Search for "esp8266" and install the package by "ESP8266 Community."

    Select Board:

        Go to Tools > Board and select your specific ESP8266 board (e.g., "NodeMCU 1.0 (ESP-12E Module)").

    Copy Code: Copy the code from esp8266_rm3100.cpp into the Arduino IDE.

Calibration (Crucial Step)

Before use, you must calibrate the compass to get accurate readings.

    Temporarily Modify Code: In the loop() function, comment out the final Serial.print block and add the following lines to print the raw magnetic field values:

    // --- Temporarily change the end of your loop() to this ---
    Serial.print("X: ");
    Serial.print(x_uT);
    Serial.print("   Y: ");
    Serial.println(y_uT);
    delay(250);

    Collect Data: Upload this code. With the sensor held flat, open the Serial Monitor (baud rate 115200) and slowly rotate the sensor through a full 360 degrees.

    Find Min/Max: Watch the output and record the minimum and maximum values you see for both X and Y.

    Calculate Offsets: Use these formulas to find your unique offsets:

        mag_offset_x = (x_max + x_min) / 2

        mag_offset_y = (y_max + y_min) / 2

    Update Firmware: Enter your calculated offsets into these variables at the top of the main sketch:

    float mag_offset_x = /* your value here */;
    float mag_offset_y = /* your value here */;

    Re-upload: Restore the original code in the loop() function and upload the final, calibrated firmware to your ESP8266.

Usage

After uploading the calibrated firmware, open the Arduino Serial Monitor with the baud rate set to 115200. As you rotate the sensor, you will see the compass heading printed in degrees.

Compass Heading: 95.43 degrees
Compass Heading: 182.10 degrees
Compass Heading: 271.88 degrees
Compass Heading: 359.95 degrees

License

This project is licensed under the MIT License - see the LICENSE file for details.
