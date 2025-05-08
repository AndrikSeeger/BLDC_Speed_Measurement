# Efficient Measurement of Rotational Motor Speed 

This ultra-efficient and interrupt-based implementation measures the rotation speed (RPM) and direction of a brushless DC (BLDC) motor using only the built-in Hall effect sensors — **no additional hardware is required**. The system is implemented on a standard Arduino-compatible board and provides clean, high-speed serial output for live monitoring or logging.

## Features

* ⚡ **Ultra-Efficient:** Uses hardware interrupts and a timer for minimal CPU usage and maximum responsiveness.
* 🧠 **Clean and Modular Code:** Structured and well-commented for easy understanding and further development.
* ⏱️ **Interrupt-Based Sensing:** All signal transitions are captured via pin change interrupts to ensure timing precision.
* 📟 **Console Output:** Outputs real-time RPM and direction data via the serial interface at 921600 baud.
* 🧰 **No Additional Hardware Needed:** Only the Arduino board and the motor’s integrated Hall sensors are required.

## How It Works

The three digital signals from the Hall effect sensors (typically labeled H1, H2, H3) are connected to three GPIO pins on the Arduino. Each pin is monitored for state changes using hardware interrupts. Every transition is time-stamped using `micros()`, allowing precise measurement of signal frequency, from which RPM is calculated.

Direction is determined by evaluating the sequence of Hall sensor state transitions. A small state machine analyzes the pattern of changes to detect clockwise (CW) or counterclockwise (CCW) motion. The average RPM is then signed based on the detected direction.

A hardware timer triggers a callback every 100 ms to print the latest RPM values to the serial console.

## Serial Output Format

Every 100 ms, the Arduino prints the following five values to the serial console (comma-separated):

```
<RPM1>,<RPM2>,<RPM3>,<Latest RPM>,<Average RPM>
```

Where:

* `RPM1`, `RPM2`, `RPM3`: Instantaneous RPM values from each of the three Hall signals
* `Latest RPM`: The most recently updated signal’s RPM
* `Average RPM`: The mean RPM, signed based on the rotation direction (positive = CW, negative = CCW)

## Pin Configuration

| Signal        | Arduino Pin |
| ------------- | ----------- |
| Hall Sensor 1 | D27         |
| Hall Sensor 2 | D25         |
| Hall Sensor 3 | D26         |

These pins are configured as `INPUT_PULLUP` and are assumed to go LOW when active. The internal pull-up resistors help stabilize the signal and reduce noise.

## Requirements

* Arduino-compatible board (e.g., ESP32, STM32, or fast AVR)
* Motor with integrated 3-phase Hall sensors
* Arduino IDE or PlatformIO

## Installation

1. Connect the three Hall sensor outputs to digital pins 27, 25, and 26.
2. Upload the code to your Arduino board using the Arduino IDE.
3. Open the Serial Monitor or a terminal at **921600 baud** to view the output.

## Notes

* If no signal changes are detected within 1 second, the RPM values are automatically reset to zero.
* The implementation assumes a two-phase pulse per revolution for each Hall signal. Adjust the RPM formula as needed for different encoder configurations.
* For best results, ensure your Hall sensor signals are clean and debounce-free.

## License

This project is open-source and available for any use within the MIT-License. Contributions and improvements are welcome.
