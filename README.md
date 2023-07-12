# Ocean Drifter Challenge
# MIT-Portugal Program 2023 Robotics Summer Camp
Ocean Drifter Challenge

## Using the LandAirSea Cellular GPS tracker
## Using LILYGO SIM7000G Cellular GPS tracker 
![image](https://github.com/xialing95/OceanDrifter-Lilygo/assets/9020926/07e87111-3bf0-42e0-b1af-615bc4868d23)
https://github.com/Xinyuan-LilyGO/LilyGO-T-SIM7000G
- Materials:
   - LILYGO SIM7000G development board (There is a new SIM7600G available now)
   - a SIM card suited for your region (SixFab Global LTE IoT)
   - USB-C Cable (**Beware the board only have the USB-C port connect in one direction, if the cable isn't working, flip it.**)
   - Another other additional sensor (Barometer BMP390, 10k Thermistor)
- Software requirement: 
   - Arduino IDE
        - Board
   - CP210xVCPDriver USB to UART for programming (https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
 
```
curl -L https://download.stereolabs.com/zedsdk/3.8/l4t32.6/jetsons -o installer.run # Replace link with https://download.stereolabs.com/zedsdk/3.8/cu117/ubuntu20 if using docker
chmod +x installer.run 
```
