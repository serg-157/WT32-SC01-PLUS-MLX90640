# Portable thermal imaging camera based on MLX90640 sensor and WT32-SC01-PLUS touch screen with ESP32-S3 onboard controller

## Concept

A portable, battery-powered, low-cost thermal imaging camera with a touch screen that can take screenshots and has a built-in web server to download them.

## Hardware

- [Wireless-Tag WT32-SC01 PLUS multi-touch 3.5" parallel LCD screen with integrated ESP32-S3 devboard](https://shop.wireless-tag.com/products/wt32-sc01-plus-with-3-5-inch-lcd-screen)
- [Waveshare MLX90640 32×24 Thermal Imaging Camera](https://www.waveshare.com/mlx90640-d55-thermal-camera.htm)
- [Li-ion Battery Charger Module with 5V 2A boost converter](https://www.ebay.co.uk/itm/5V-2A-Type-C-USB-3-7V-18650-Lithium-Li-ion-Battery-Charging-Board-DIY-Power-Bank/124259709453)
- [505573 Li-Po Battery 3.7V 2500mAh](https://aliexpress.ru/item/32839426413.html)

## Libraries used

- [Patched TFT_eSPI Display Library](https://github.com/dkalliv/TFT_eSPI)
- [Patched Adafruit FT6206 Touch Screen Library](https://github.com/dkalliv/Adafruit_FT6206_Library)
- [Melexis MLX90640 Thermal Sensor Library](https://github.com/melexis/mlx90640-library)
- [ESPAsyncWebServer Library](https://github.com/me-no-dev/ESPAsyncWebServer.git)

## Implementation details

- IDE: VS Code [PlatformIO](https://platformio.org/), project config file: [platformoi.ini](https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/platformio.ini)

- The patched versions of the **TFT_eSPI** display and **Adafruit FT6206** touch libraries are used instead of the more popular **LovyanGFX** libraries to simplify UI/UX design and development (credit: @dkalliv). Here is the [discussion](https://github.com/Bodmer/TFT_eSPI/discussions/2319) why the original TFT_eSPI library cannot be used. Sprites are used to speed up the output of thermal images and parameters.

- [FreeRTOS multitasking](https://www.freertos.org/implementation/a00004.html) is used to process the touch screen events and button presses in a separate task, while the main loop is used to read the sensor data, process and output the image.

- [Linear interpolation](https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/src/interpolation.h) is used to transform 32×24 thermal matrix data into a 320×240 screen image, as well as a frame-to-frame softening algorithm. Both can be disabled from the user interface.

- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer.git) is used to provide user access to screenshots (.bmp files) stored on the onboard SD card. To download and manage screenshots, the user needs to connect to the board's Wi-Fi hotspot and navigate to the local IP address in the browser.

- EEPROM memory is used to store the boot counter and the measurable temperature range.


## Building and flashing

- Install VS Code and the [PlatformIO](https://platformio.org/) plugin.

- Create new project and select **esp32-s3-devkitc-1** board.

- Add dependencies:
  - [TFT_eSPI](https://github.com/dkalliv/TFT_eSPI)
  - [Adafruit FT6206](https://github.com/dkalliv/Adafruit_FT6206_Library)
  - [Melexis MLX90640 API](https://github.com/melexis/mlx90640-library)
  - [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer.git)

- Set the correct target board for **TFT_eSPI** library by editing the contents of [User_Setup_Select.h](https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/lib/User_Setup_Select.h) and [Setup214_WT32_SC01_Plus.h](https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/lib/Setup214_WT32_SC01_Plus.h) files.

- Replace **platformio.ini** file and the contents of your **src** directory with the contents of this repository's **src** [directory](https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/tree/main/src).

- Enable the console output if needed `#define CONSOLE_OUTPUT true`.

- Connect **MLX90640** thermal imaging sensor to [Extended I/O Interface](https://doc.riot-os.org/group__boards__esp32s3__wt32__sc01__plus.html) of your **WT32-SC01 PLUS** board.

- Connect the **WT32-SC01 PLUS** board to your computer using the USB-C data cable. To put the board into upload mode, it is necessary to ground pin 6 (boot, GPIO0) of the debug interface and press the reset button.

- Compile the project and upload it to the board

## Wiring

<img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/wiring.jpg" height="650" hspace="7"/>

## Modelling

<img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/modelling.jpg" height="450" hspace="7"/>

## Unit body and layout

The device is housed in a transparent two-layer case with a power switch and a USB-C interface for charging the battery. The sensor and battery are located on the top layer to minimize the effect of the ESP32-32's heat generation on sensor readings and battery performance.

<img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/unit_back.jpg" width="436" height="361" hspace="7"/><img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/unit_front.jpg" height="361"/>

## UI/UX

The screenshots below show examples of the interface with interpolation and softening on and off respectively. The thermal image is displayed in the top half of the screen, while the temperature range, controls, and parameters are displayed in the bottom half of the screen.

By touching the thermal image, the user can move the temperature measurement point (white circle) for the temperature, displayed in the center in yellow color. The temperature measurement range can also be changed by touching different zones of the color legend (rainbow ruler).

The current temperature range is displayed on the first line of the status bar at the bottom of the screen, along with the current fps value and center point coordinates. The second line displays access point connection information.

Pressing the Reboot button forces the ESP32-S3 board to reboot in the event of a sensor or other hardware failure.  Pressing the Save button will capture thermal and full-screen images and save them to the SD card. They can be downloaded later via a Wi-Fi connection.

<img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/screenshot_interpolated.jpg" width="300" hspace="7"/><img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/screenshot_not_interpolated.jpg" width="300"/>


## Web Server
Provides user access to screenshots stored on the onboard SD card. To download and manage screenshots, the user should connect to the board's Wi-Fi access point and follow **http://192.168.4.1/** url.

The main web page shows the list of all captured images on the SD card. Images are stored as bitmaps, the first number in the filename is the boot number, the second is the image number in the current boot. The filename is clickable and brings up the image preview page, the user can also download or delete the image. Pagination at the bottom of the main web page is provided for navigation.

<img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/webpage.jpg" width="300" hspace="7"/><img src="https://github.com/serg-157/WT32-SC01-PLUS-MLX90640/blob/main/assets/preview_page.jpg" width="300"/>
