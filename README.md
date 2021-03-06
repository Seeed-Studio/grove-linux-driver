Grove Linux Driver
==================

  Here is the Grove Linux Driver, currently it only support some grove devices on below platforms:
  - [PocketBeagle](https://beagleboard.org/pocket) with Pocket Beagle Grove Cape
  - [BeagleBone Green](https://www.seeedstudio.com/Grove-Cape-for-BeagleBone-Series-p-1718.html)
  - [BeagleBone Green Wireless](https://www.seeedstudio.com/BeagleBone-Green-Wireless-Development-Board-TI-AM335x-WiFi-B-p-2650.html)
  - SeeedStudio BeagleBone Green Gateway Baseboard
  - [Raspberry Pi](https://www.seeedstudio.com/category/Boards-c-17.html)



<br><br>
Installation
------------
  Below steps will prepare all grove device tree blob object(.dtbo) to
```/lib/firmware/``` on BB Series or ```/boot/overlays/``` on RPi,
and grove device kernel module(.ko) to ```/lib/modules/<kernel version>/``` or it's sub-folder.

1. Clone this repo

   ```bash
   cd
   git clone https://github.com/Seeed-Studio/grove-linux-driver.git
   ```

2. Begin install

   ```bash
   cd ~/grove-linux-driver
   sudo ./install.sh
   ```


<br><br>
Usage
-----
After installation, the driver is prepared well, then we need to enable specific
grove device in system configuration file.

#### BB Series

* Append below line to the configuration file
  [/boot/uEnv.txt](https://elinux.org/Beagleboard:BeagleBoneBlack_Debian#U-Boot_Overlays):

  ```bash

  uboot_overlay_addr<n>=/lib/firmware/<Device-Tree-Blob-Name>.dtbo
  
  ```
  ```<n>```: is in 0..7 which not be used in the uEnv.txt.  <br>
  ```<Device-Tree-Blob-Name>```: refer to [Grove Device Table](#grove-device-table).

  ***example:***


  ```bash
  # If you plugin Grove LED to slot 1057 of Pocket Beagle Grove Cape, 
  # and uboot_overlay_addr0 is unused, append the line:

  uboot_overlay_addr0=/lib/firmware/BB-GPIO-GROVE-LED.dtbo

  ```

#### RPi Series

* Append below line to the configuration file
  [/boot/config.txt](https://github.com/raspberrypi/linux/blob/rpi-4.14.y/arch/arm/boot/dts/overlays/README)

  ```bash
  
  dtoverlay=<Device-Tree-Blob-Name>,<param0>=<value0>,<param1>=<value1>,...
  
  ```
  ```<Device-Tree-Blob-Name>```,```<paramN>```, ```<valueN>```:
  refer to specific usage in [Grove Device Table](#grove-device-table)

  ***example:***


  ```bash
  # If you connect Grove Button to gpio 5 of Raspberry Pi, append the line:
  
  dtoverlay=gpio-key,gpio=5
  
  # The default keycode is KEY_POWER, so the pressing will result in a power off.
  ```


#### User interface
* refer to ```Specific Usage``` of [Grove Device Table](#grove-device-table).



<br><br>
Grove Device Table
------------------
<div>
  <table border="0">
    <tr align="center">
      <th>Grove Devices</th>
      <th>Device Tree Blob</th>
      <th>Device Driver</th>
      <th>Specific Usage</th>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Red-LED-p-1142.html">Red LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-Green-LED-p-1144.html">Green LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-Purple-LED-3m-p-1143.html">Purple LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-White-LED-p-1140.html">White LED</a>
      </td>
      <td>
        BB : <a href="dts/bbb/BB-GPIO-GROVE-LED.dts">BB-GPIO-GROVE-LED</a><br>
        RPi: <a href="dts/rpi/grove-led-overlay.dts">grove-led</a>
      </td>
      <td>inner <a href="https://github.com/beagleboard/linux/blob/master/drivers/leds/leds-gpio.c">drivers/leds/leds-gpio.c</a></td>
      <td><a href="src/grove-led/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/s/Grove-Button-p-766.html">Button</a></td>
      <td>
        BB : <a href="dts/bbb/BB-GPIO-GROVE-BUTTON.dts">BB-GPIO-GROVE-BUTTON</a><br>
        RPi: <a href="https://github.com/raspberrypi/linux/blob/rpi-4.14.y/arch/arm/boot/dts/overlays/gpio-key-overlay.dts">gpio-key</a>
      </td>
      <td>inner <a href="https://github.com/beagleboard/linux/blob/master/drivers/input/keyboard/gpio_keys.c">drivers/input/keyboard/gpio_keys.c</a></td>
      <td><a href="src/grove-button/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Ultrasonic-Ranger-p-960.html">Ultrasonic Ranger</a></td>
      <td><a href="dts/bbb/BB-GPIO-HCSR04.dts">BB-GPIO-HCSR04</a></td>
      <td><a href="src/hcsr04/hcsr04.c">src/hcsr04/hcsr04.c</a></td>
      <td><a href="src/hcsr04/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Temp-Humi-Barometer-Sensor-BME280-p-2653.html">Temp&Humi&Barometer Sensor (BME280)</a></td>
      <td><a href="dts/bbb/BB-I2C1-BME280.dts">BB-I2C1-BME280</a></td>
      <td>inner <a href="https://github.com/beagleboard/linux/blob/master/drivers/iio/pressure/bmp280-i2c.c">drivers/iio/pressure/bmp280-i2c.c</a></td>
      <td><a href="src/bme280/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-16-x-2-LCD-Black-on-Red-p-3197.html">16 x 2 LCD (Black on Red)</a><br>
        <a href="https://www.seeedstudio.com/Grove-16-x-2-LCD-Black-on-Yellow-p-3198.html">16 x 2 LCD (Black on Yellow)</a><br>
        <a href="https://www.seeedstudio.com/Grove-16-x-2-LCD-White-on-Blue-p-3196.html">16 x 2 LCD (White on Blue)</a>
      </td>
      <td><a href="dts/bbb/BB-I2C1-JHD1802.dts">BB-I2C1-JHD1802</a></td>
      <td><a href="src/hd44780/hd44780-i2c.c">src/hd44780/hd44780-i2c.c</a></td>
      <td><a href="src/hd44780/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-3-Axis-Digital-Accelerometer-16-p-1156.html">3-Axis Digital Accelerometer (±16g)</a></td>
      <td><a href="dts/bbb/BB-I2C2-ADXL34X.dts">BB-I2C2-ADXL34X</a></td>
      <td><a href="src/adxl34x/adxl34x-i2c.c">src/adxl34x/adxl34x-i2c.c</a></td>
      <td><a href="src/adxl34x/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-3-Axis-Digital-Accelerometer-400-p-1897.html">3-Axis Digital Accelerometer (±400g)</a></td>
      <td><a href="dts/bbb/BB-I2C2-LIS331DLH.dts">BB-I2C2-LIS331DLH</a></td>
      <td>inner <a href="https://github.com/beagleboard/linux/blob/master/drivers/iio/accel/st_accel_i2c.c">drivers/iio/accel/st_accel_i2c.c</a></td>
      <td><a href="src/lis3lv02d/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Temperature-Humidity-Sensor-SHT31-p-2655.html">Temperature & Humidity Sensor (SHT31)</a></td>
      <td><a href="dts/bbb/BB-I2C2-SHT3X.dts">BB-I2C2-SHT3X</a></td>
      <td><a href="src/sht3x/sht3x.c">src/sht3x/sht3x.c</a></td>
      <td><a href="src/sht3x/README.md">Click</a></td>
    </tr>
    <tr align="center">
      <td><a href="https://www.seeedstudio.com/Grove-Time-of-Flight-Distance-Sensor-VL53L0-p-3086.html">Time of Flight Distance Sensor (VL53L0X)</a></td>
      <td><a href="dts/bbb/BB-I2C2-VL53L0X.dts">BB-I2C2-VL53L0X</a></td>
      <td><a href="src/vl53l0x/vl53l0x.c">src/vl53l0x/vl53l0x.c</a></td>
      <td><a href="src/vl53l0x/Readme.md">Click</a></td>   
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Triple-Color-E-Ink-Display-1-54-p-2890.html">Triple Color E-Ink Display 1.54"</a><br>
        <a href="https://www.seeedstudio.com/Grove-Triple-Color-E-Ink-Display-2-13-p-2889.html">Triple Color E-Ink Display 2.13"</a><br>
      </td>
      <td><a href="dts/bbb/BB-UART4-E-INK.dts">BB-UART4-E-INK</a></td>
      <td><a href="src/e-ink/eink-tty.c">src/e-ink/eink-tty.c</a></td>
      <td></td>
    </tr>
    <tr align="center">
      <td>
        <a href="https://www.seeedstudio.com/Grove-Chainable-RGB-LED-p-850.html">Chainable RGB LED</a><br>
        <a href="https://www.seeedstudio.com/Grove-Chainable-RGB-Led-V2-0.html">Chainable RGB Led V2.0</a><br>
      </td>
      <td><a href="dts/bbb/BB-GPIO-P9813.dts">BB-GPIO-P9813</a></td>
      <td><a href="src/p9813/p9813.c">src/p9813/p9813.c</a></td>
      <td><a href="src/p9813/README.md">Click</a></td>
    </tr>
  </table>
</div>

**Note:**  
  ```inner``` means the device driver integreted into linux kernel image,  
no need driver source code in this repo.
