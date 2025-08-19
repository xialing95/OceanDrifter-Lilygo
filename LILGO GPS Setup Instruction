# Ocean Drifter Challenge
# MIT-Portugal Program 2023 Robotics Summer Camp
Ocean Drifter Challenge

## Using LILYGO SIM7000G Cellular GPS tracker 
![image](https://github.com/xialing95/OceanDrifter-Lilygo/assets/9020926/07e87111-3bf0-42e0-b1af-615bc4868d23)
https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G
- Materials:
   - LILYGO SIM7000G development board (There is a new SIM7600G available now)
   - a SIM card suited for your region (SixFab Global LTE IoT)
   - USB-C Cable (**Beware the board only have the USB-C port connect in one direction, if the cable isn't working, flip it.**)
   - Other additional sensor (Barometer BMP390, 10k Thermistor)
- Software requirement: 
   - CP210xVCPDriver USB to UART for programming (https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
   - Arduino IDE
      - Arduino Board **Tools > Board**:
        - https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
         - Use ESP32 Wrover Module
         - <img width="510" alt="Screen Shot 2023-06-15 at 10 14 30 PM" src="https://github.com/xialing95/OceanDrifter-Lilygo/assets/9020926/7a60e887-554d-40ea-bd5c-7de77386f43f">
      - Arduino Library, **Sketch > Include Library > Manage Libraries**:
        - TinyGSM by Volodymyr Shymanskyy.
        - StreamDebugger by Volodymyr Shymanskyy.
       
# OceanDrifter-Lilygo: Instruction Page

Below is a general instruction page template for the OceanDrifter-Lilygo project. This guide is based on standard practices for LilyGO hardware and ESP32-based projects, adapted for similar repositories and devices[1][2].

## 1. Hardware Requirements

- **LilyGO/TTGO ESP32 board** (specific model as used in OceanDrifter-Lilygo)
- **USB cable** for programming
- **Sensors** (as specified in your project)
- **Computer** with internet access

## 2. Software Requirements

- **Arduino IDE** (version 1.8 or later) or **Visual Studio Code** with **PlatformIO**
- **ESP32 Board Support** for Arduino or PlatformIO
- **Required libraries** (see the `lib` or `libraries` folder in the repository)

## 3. Setup Instructions

### A. Arduino IDE

1. **Install Arduino IDE**  
   Download from the official Arduino website and install.

2. **Add ESP32 Board Support**
   - Go to `File > Preferences`.
   - In "Additional Board Manager URLs," add:  
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to `Tools > Board > Boards Manager`, search for "ESP32," and install.

3. **Install Required Libraries**
   - Go to `Sketch > Include Library > Manage Libraries`.
   - Search and install libraries as listed in the repository's documentation or `README.md`.

4. **Open the Project**
   - Download or clone the OceanDrifter-Lilygo repository.
   - Open the `.ino` file in Arduino IDE.

5. **Select Board and Port**
   - Go to `Tools > Board` and select your LilyGO/TTGO model (e.g., "ESP32 Dev Module").
   - Select the correct COM port under `Tools > Port`.

6. **Upload the Sketch**
   - Click the upload button to flash the firmware to your device.

### B. PlatformIO (Recommended for Advanced Users)

1. **Install Visual Studio Code and Python**
2. **Install PlatformIO Extension**
   - Open VSCode, go to Extensions, and search for "PlatformIO."
   - Install and restart VSCode.

3. **Open the Project Folder**
   - Use `File > Open Folder` to select the OceanDrifter-Lilygo directory.

4. **Install Dependencies**
   - PlatformIO will automatically install dependencies listed in `platformio.ini`.

5. **Build and Upload**
   - Click the checkmark (✔) in the lower bar to build.
   - Connect your board via USB.
   - Click the right arrow (→) to upload.

## 4. Configuration

- Edit configuration files (e.g., `config.h`, `platformio.ini`) to set WiFi credentials, sensor parameters, or other project-specific settings.
- Refer to comments in these files for guidance.

## 5. Running and Monitoring

- After uploading, open the Serial Monitor (baud rate: 115200 or as specified) to view output.
- For web-based interfaces, note the IP address printed in the Serial Monitor and access it via your browser if applicable.

## 6. Troubleshooting

- **Board not detected:** Check USB cable and drivers.
- **Compilation errors:** Ensure all libraries are installed and board support is up to date.
- **Upload fails:** Hold the BOOT button while pressing RESET, then try uploading again[2].

## 7. Additional Resources

- Check the project’s `README.md` for updates and specific instructions.
- Visit the LilyGO GitHub for more examples and documentation[3][1][2].

This template provides a comprehensive starting point for an instruction page. Adjust details as needed to match the exact hardware and software used in your OceanDrifter-Lilygo project.

[1] https://github.com/Xinyuan-LilyGO/LilyGo-HiGrow
[2] https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
[3] https://github.com/Xinyuan-LilyGO
[4] https://github.com/xialing95/OceanDrifter-Lilygo/tree/main
[5] https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library
[6] https://github.com/OceanParcels/drifter_trajectories_network/blob/master/constrain_drifterdata_to_northatlantic.py
[7] https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series
[8] https://github.com/Xinyuan-LilyGO/TTGO-T-Display
[9] https://github.com/LilyGO



https://github.com/user-attachments/assets/ac6ef01f-cbc1-4a8b-bc05-6667f3468ced


https://github.com/user-attachments/assets/68fc129f-a11f-4fcb-8bb2-79ae8c8a03e7


