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
   - Another other additional sensor (Barometer BMP390, 10k Thermistor)
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
