# 🚰 Smart Soda Dispenser  

A user-friendly **automated soda dispenser** built with the ATmega328P microcontroller.  
This project integrates **RFID authentication**, **ultrasonic distance sensing**, and **relay-controlled pumps** to deliver secure and convenient beverage dispensing at the push of a button.  

## 📌 Features  
- **RFID Authentication** – Only authorized users can activate the dispenser.  
- **Cup Detection** – Liquid dispenses only when a cup is detected under the nozzle.  
- **Dual Pump System** – Two push buttons trigger independent pumps for multiple beverage options.  
- **Hardware Debouncing** – Reliable push button inputs using resistors and capacitors.  
- **Serial Debugging** – Real-time diagnostic messages via UART.  
- **Failsafe Control** – Relays reset each cycle to ensure clean operation.  

## 🛠️ Hardware Components  
- **ATmega328P** (Arduino Uno compatible)  
- **HC-SR04 Ultrasonic Sensor** – For cup presence detection  
- **RFID-RC522 Module** – For user authentication  
- **5V Single-Channel Relays (x2)** – For pump control  
- **5V Submersible Water Pumps (x2)** – Beverage dispensing  
- **Push Buttons (x2)** – User input with hardware debouncing  
- **Supporting Resistors & Capacitors** – For stable hardware operation  

## ⚙️ Software Implementation  
- **Main Loop** continuously monitors distance & RFID authentication.  
- **Ultrasonic Library** (`Ultrasonic.h`) – Measures distance with timer interrupts.  
- **RFID Library** (`mfrc522.h`, `spi.h`) – Handles UID scanning & authentication.  
- **Custom UART Driver** – Sends debug info to serial monitor.  
- **Interrupt Service Routines (ISR):**  
  - `INT0_vect` – Ultrasonic timing  
  - `WDT_vect` – Watchdog wake-up  

## 📋 System Flow  
1. Detects a cup using the ultrasonic sensor.  
2. Waits for valid RFID authentication.  
3. Enables push buttons.  
4. Dispenses liquid when a button is pressed.  
5. Resets relays at the end of each cycle.  

![System Flow Diagram](https://github.com/Adrian-A/SodaDispenser/blob/main/Tests/Screenshot%202025-08-25%20151232.png)

## 📊 Results  
- The dispenser successfully prevents dispensing without a cup.  
- RFID ensures secure access.  
- Liquid dispenses reliably upon authentication and button press.  
- Encountered minor **noise issues** from pumps affecting sensors, mitigated with layout adjustments.  

📹 [Full Demo Video]([https://iowa-my.sharepoint.com/:v:/g/personal/sebolen_uiowa_edu/Eca9Vk9n3-VHr41r5trK9NEB0kA0RxeAtCbYkjQcaoJH_g?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJPbmVEcml2ZUZvckJ1c2luZXNzIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXciLCJyZWZlcnJhbFZpZXciOiJNeUZpbGVzTGlua0NvcHkifX0&e=K41ZzT](https://github.com/Adrian-A/SodaDispenser/blob/main/Tests/7C41D827-79C2-4EBE-A6F9-6F0FE475BF0B.MOV))  

## 🔧 Future Improvements  
- Replace 5V pumps with **12V isolated power pumps** for reduced noise.  
- Upgrade ultrasonic sensor to **infrared (IR)** for more accurate cup detection.  
- Use a **robust waterproof enclosure** instead of cardboard.  
- Optimize pump height to prevent spillage.  

## 📂 Repository Structure 
- SodaDispenser
  - Include #Helper Functions
    - Header
      - Ultrasonic.h
      - mfrc522.h
      - mfrc522_cmd.h
      - mfrc522_reg.h
      - spi.h
      - spi_config.h
      - utils.h
    - Src
      - Ultrasonic.c
      - mfrc522.c
      - spi.c
  - Tests # Testing files used through the process
  - Main.c # Main source code
  - README.md

## 📚 References  
- [MIFARE RC522 RFID Library](https://github.com/asif-mahmud/MIFARE-RFID-with-AVR)  
- [HC-SR04 C Library](https://github.com/Ovidiu22/HC-SR04)  
- [ATmega328P Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061A.pdf)  
- [HC-SR04 Datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)  
- [Relay Module](https://diyables.io/products/relay-5v-1-channel)  
