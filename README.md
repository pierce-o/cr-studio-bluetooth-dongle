# CR Studio Color Kit Emulator
## About
This purpose of this project is to emulate the CR Lizard color kit bluetooth module which will allow you to use your phones camera to take photos to be used to generate textures for your scan.

## Requirements

 - VSCode 
 - Python3 
 - CP2102 ESP32 (DevKit3 ESP-WROOM-32) 

You will need access to a Linux environment if you want to be able to revert the hardware ID changes made to the CP2102.

## Building and Flashing
### Building
Open the project in VSCode and follow this quick start guide on how to install the necessary tools.
https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md

Build your project using the icon at the bottom of VSCode or via the command pallete.

### Flashing via VSCode
Plug your ESP-32 into your computer.

Select the COM port that your device is on, you can find this via the device manager.

Flash your device using the lightning bolt icon ( you may need to press the boot button depending on the version of your ESP32 devkit )

## Post Flashing
After you have flashed the program onto your ESP-32 you will need to update the hardware IDs so that it can be used by CR Studio.

### Windows
Note that once you have changed the hardware IDs using this method then you wont be able to revert them back to the default values unless you use linux and run the orignal.sh script. This is due to how the CP21xxCustomizationUtility software finds the device.
To change the hardware IDs you will need to download the [CP21xxCustomizationUtility](https://www.silabs.com/documents/public/example-code/AN721SW.zip) program found in the Silicon Labs AN721 software package.
Once you have opened the tool select your ESP-32 device and update the hardware IDs to the following. VID - **1a86** PID - **55d3**.
Click "program device" and wait. Verification will fail due to the hardware IDs being different to what the program is looking for but you can verify this yourself via the device manager looking for a device name containing "CH341".

### Linux
On linux you are able to use the cp210x-cfg project to update the hardware IDs.

	git clone https://github.com/DiUS/cp210x-cfg

	cd cp210x-cfg/
	sudo apt install -y libusb-1.0-0-dev
	make
	
	chmod +x ./original.sh
	chmod +x ./crstudio.sh
	
I have provided two scripts which will update the hardware IDs accordingly, these are original.sh and crstudio.sh.
Once cp210x-cfg is installed and setup you can execute the ./crstudio.sh script which will update your ESP-32s hardware IDs. You can revert the IDs by running ./original.sh and this will allow you to use the device as an ESP32 and flash it as a normal ESP-32. 

# Issues and Features
Currently there is no support for digital cameras however, the commmands have been implemented where three GPIO pins can be pulled HIGH or LOW. Currently I'm unsure of how this should work but I believe it should make use of a 3.5mm jack and the shutter input of digital cameras. I do not have an orignal CR Lizard bluetooth module or a digital camera so I will not be making any further progress in development of this feature.

# Other
Creality included three extras with the color kit and excluding the digital camera connectors connectors these are a dual mount for the tripod, phone clamp, and a photo studio. These can all be had from Amazon for less than £10 each or cheaper from aliexpress.