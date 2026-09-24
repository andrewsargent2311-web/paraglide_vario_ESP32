# Paraglide Vario / Flight Computer (ESP32-S3)

A DIY flight computer for paragliding and paramotoring that costs about **$60 USD** in parts. It has a sunlight-readable screen, a climb/sink beeper (vario), GPS, a barometer, airspace warnings, live ADS-B traffic, weather stations, FANET radio and IGC flight logging.

!\[The finished flight computer](Photos/20260919\_101505.jpg)

**You do not need any electronics or programming experience.** This guide walks through every step. If you can follow a recipe and are willing to learn to solder a few small parts (or find a friend who can), you can build this.

> ⚠️ \*\*Safety:\*\* This is a hobby project, not a certified flight instrument. Never rely on it as your only source of airspace, altitude or traffic information. Test everything on the ground before you fly with it.

\---

## Contents

1. [What it does](#1-what-it-does)
2. [What you need to buy](#2-what-you-need-to-buy)
3. [Tools you need](#3-tools-you-need)
4. [Step 1: Order the circuit board](#step-1-order-the-circuit-board)
5. [Step 2: 3D print the case](#step-2-3d-print-the-case)
6. [Step 3: Solder the circuit board](#step-3-solder-the-circuit-board)
7. [Step 4: Install the software tools](#step-4-install-the-software-tools)
8. [Step 5: Add your WiFi details](#step-5-add-your-wifi-details)
9. [Step 6: Upload the code](#step-6-upload-the-code)
10. [Step 7: Prepare the SD card](#step-7-prepare-the-sd-card)
11. [Step 8: Assemble everything](#step-8-assemble-everything)
12. [Step 9: First power-on check](#step-9-first-power-on-check)
13. [How to use it](#how-to-use-it)
14. [Optional extras](#optional-extras)
15. [Troubleshooting](#troubleshooting)
16. [Using it outside New Zealand](#using-it-outside-new-zealand)
17. [Credits](#credits)

\---

## 1\. What it does

* **Vario:** beeps faster and higher as you climb, lower tones when you sink
* **Main flight screen:** altitude, height above ground (AGL), ground speed, heading, QNH, battery
* **Airspace:** warns you about nearby controlled airspace from an OpenAIP file
* **ADS-B page:** shows nearby aircraft on a radar-style display
* **Weather page:** wind from nearby weather stations (Zephyr) or from FANET
* **FANET:** sends and receives FANET radio messages, with preset messages you can broadcast
* **IGC flight logging** to the SD card, downloadable over WiFi
* **Optional:** connects by Bluetooth to a separate engine meter (RPM, cylinder temperature) for paramotor use

**One button** controls everything (see [How to use it](#how-to-use-it)).

## 2\. What you need to buy

|Part|Approx. cost|Notes|
|-|-|-|
|[Waveshare ESP32-S3-RLCD-4.2](https://www.waveshare.com/esp32-s3-rlcd-4.2.htm)|$27|The "brain", screen, speaker driver, SD slot and battery holder all in one|
|[Adafruit BMP580](https://www.adafruit.com/product/6411)|$8|Barometer, measures altitude change for the vario|
|[Waveshare LC76G GNSS module](https://www.waveshare.com/lc76g-gnss-module.htm)|$15|GPS|
|Custom circuit board (PCB) from PCBWay|$5|See [Step 1](#step-1-order-the-circuit-board)|
|3D printer filament (PLA)|$5|Orange in the photos, any colour works|
|microSD card, 8–16 GB|a few $|Doesn't need to be bigger|
|**HT-RA62 LoRa radio module**|check price|Soldered directly onto the PCB, used for FANET. *Not included in the original $60 estimate.*|

**Small parts for the PCB** (all size 0805 surface-mount):

|Qty|Part|
|-|-|
|2|100 nF capacitor|
|2|10 µF capacitor|
|1|10 kΩ resistor|

You will also need some pin headers and/or sockets to connect the Waveshare board, BMP580 and GPS module to the PCB. The exact layout is shown in the [KiCad project](PCB%20files/ESP32%20vario_PCB_layout/).

**Total: roughly $60–80 USD** plus shipping, depending on the items in bold.

> \*\*Tip:\*\* Buy everything from the same order window if you can. PCBs from PCBWay usually arrive in a couple of weeks, so order that first.

## 3\. Tools you need

* **Soldering iron** with a fine tip, solder and flux (for the tiny 0805 parts and the LoRa module). A hot-air station makes this easier but is not required.
* **Tweezers** (for the small parts)
* **A 3D printer**, or use a print service / friend with one
* **A computer** (Windows, Mac or Linux) with a USB port
* **A microSD card reader**
* Small screwdrivers, and wire cutters

**Never soldered before?** The only genuinely fiddly parts are the 0805 capacitors/resistor and the HT-RA62 module. Watch a couple of "how to solder SMD 0805" videos and practise on a spare board first, or ask a local maker space or electronics-savvy friend to do just those parts.

**Time:** allow a weekend. The 3D printing (about 9.5 hours total) runs by itself.

\---

## Step 1: Order the circuit board

The circuit board (PCB) connects all the modules together so you don't have to wire anything by hand.

1. Go to [pcbway.com](https://www.pcbway.com) and choose **Instant Quote**.
2. Upload the file [`PCB files/Gerber Files V2/ESP32 vario-B\_PCB\_.zip`](PCB%20files/Gerber%20Files%20V2/). It contains all the Gerber and drill files in one zip.
3. Leave the options at their defaults unless you have a preference (it is a simple two-layer board), and place the order.

The original design files (KiCad) are in [`PCB files/ESP32 vario\_PCB\_layout/`](PCB%20files/ESP32%20vario_PCB_layout/) if you want to modify the board.

## Step 2: 3D print the case

All files are in [`3D print Files/`](3D%20print%20Files/).

|Part|STL file|Print time (0.15 mm PLA, Prusa MK3S)|
|-|-|-|
|Main case|`Flight computer main case.stl`|5 h 49 m|
|Back plate|`Flight computer back plate.stl`|2 h 11 m|
|Base plate|`Flight computer base plate.stl`|1 h 14 m|
|Button|`button.stl`|8 m|
|Button (short)|`button\_Short.stl`|6 m|
|Button (long)|`button\_Long.stl`|9 m|

* If you have a **Prusa MK3S**, you can print the ready-made G-code in the `Prusa slicer G code` folder.
* For **any other printer**, open the `.stl` files in your printer's slicing software (Cura, PrusaSlicer, Bambu Studio, etc.) and use normal PLA settings at 0.15 mm layer height.
* Print all three buttons; you'll pick whichever length feels best when you assemble it.

## Step 3: Solder the circuit board

!\[The PCB with the LoRa module fitted](Photos/20260916\_134729.jpg)

Work from the smallest parts to the biggest, and take your time. Solder joints should be shiny and cone-shaped, not blobs.

1. **Small surface-mount parts first:** the two 100 nF capacitors, two 10 µF capacitors and the 10 kΩ resistor. The reference labels (C1, C2, C3, C-GPS1, R1) are printed on the board. Use the [parts list](PCB%20files/ESP32%20vario_PCB_layout/ESP32%20vario.csv) to match them.
2. **HT-RA62 LoRa module (U1):** solder this onto its pads on the board. Check the orientation using the silkscreen and the photo above. Double-check no pins are bridged with solder.
3. **Headers:** solder the pin headers/sockets for the Waveshare board (U2), BMP580 (U3) and GPS (U4).
4. **Plug in the modules.** Make sure each one goes the right way round. Use the photos in [`Photos/`](Photos/) as a reference.

**Before going on, check:** with a magnifier, look at every solder joint for bridges (solder connecting two neighbouring pins) and for pins that missed the solder.

## Step 4: Install the software tools

This is the same every time you set up an ESP32 board, and it's easier than it sounds.

### 4.1 Install the Arduino IDE

Download **Arduino IDE 2** from [arduino.cc/en/software](https://www.arduino.cc/en/software) and install it.

### 4.2 Add ESP32 support

1. Open the Arduino IDE → **File → Preferences**.
2. In **Additional boards manager URLs**, paste: `https://espressif.github.io/arduino-esp32/package\_esp32\_index.json`
3. Click **OK**, then open **Tools → Board → Boards Manager**.
4. Search for **esp32** and install **"esp32 by Espressif Systems"**.

### 4.3 Download this project

On this GitHub page, click the green **Code** button → **Download ZIP**, then unzip it somewhere you can find it (for example your Documents folder).

### 4.4 Install the libraries

Libraries are add-on code the project uses. Install these from **Sketch → Include Library → Manage Libraries…** by searching for each name:

|Library|Author|
|-|-|
|Adafruit BMP5xx|Adafruit|
|Adafruit SHTC3|Adafruit|
|Adafruit Unified Sensor|Adafruit|
|TinyGPSPlus (shown as "TinyGPSPlus")|Mikal Hart|
|ArduinoJson|Benoit Blanchon|
|NimBLE-Arduino|h2zero|
|RadioLib|Jan Gromeš|
|PCF85063A (search "PCF85063A"; the clock library, by Soldered)|Soldered|

When the IDE asks *"Install all dependencies?"*, click **Install all**.

**The screen library (U8g2) needs special handling.** This project uses a version that supports the Waveshare screen:

1. Download Waveshare's repository as a ZIP: [github.com/waveshareteam/ESP32-S3-RLCD-4.2](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2) → **Code → Download ZIP**.
2. Unzip it and open the folder `01\_Arduino\_Libraries`.
3. Copy the folder named **`U8g2`** into your Arduino libraries folder. On Windows this is `Documents\\Arduino\\libraries`; on Mac it's `Documents/Arduino/libraries`.
4. If you previously installed "U8g2" from the Library Manager, delete or uninstall it first, so there's only one copy.
5. Restart the Arduino IDE.

> The other libraries in that Waveshare folder (SensorLib, LVGL) aren't needed for this project.

## Step 5: Add your WiFi details

The code expects a file called `secrets.h` containing your WiFi network names and passwords. It is deliberately not uploaded to GitHub (so passwords don't leak), so you have to create it.

1. In the project folder, open `paraglide\_vario\_ESP32/` (the folder with `paraglide\_vario\_ESP32.ino` in it).
2. Copy the file **`secrets.example.h`** and rename the copy to **`secrets.h`**.
3. Open `secrets.h` in any text editor and put in your details, for example:

```cpp
static const WifiNetwork WIFI\_NETWORKS\[WIFI\_NETWORK\_COUNT] = {
  { "MyHomeWiFi",     "my-home-password" },
  { "MyPhoneHotspot", "hotspot-password" },
  { "",               "" },
};
```

* Use a **2.4 GHz** network. The ESP32 can't connect to 5 GHz WiFi.
* Leave unused slots as empty quotes `""`.
* A **phone hotspot** is handy at the launch site, since that's how the ADS-B traffic and weather pages get their data in the field.

## Step 6: Upload the code

1. In the Arduino IDE, open **`paraglide\_vario\_ESP32/paraglide\_vario\_ESP32.ino`**.
2. Plug the Waveshare board into your computer with the USB-C cable.
3. Go to **Tools** and set these exactly (they match Waveshare's own recommended settings):

|Setting|Value|
|-|-|
|Board|**ESP32S3 Dev Module**|
|USB CDC On Boot|**Enabled**|
|Flash Mode|**QIO 80MHz**|
|Flash Size|**16MB (128Mb)**|
|Partition Scheme|**16M Flash (3MB APP/9.9MB FATFS)**|
|PSRAM|**OPI PSRAM**|
|Upload Speed|921600|

4. Under **Tools → Port**, choose the port that appeared when you plugged the board in (COM-something on Windows, `/dev/cu.usbmodem…` on Mac).
5. Click the **Upload** button (the right-pointing arrow). The first build takes a few minutes.
6. When it says **"Done uploading"**, the code is on your device.

If the upload doesn't start, hold the **BOOT** button on the board, tap **RESET**, then release BOOT, and try again.

## Step 7: Prepare the SD card

1. **Format** the microSD card as **FAT32**.
2. Copy the files you want from [`paraglide\_vario\_ESP32/Goes into SD card/`](paraglide_vario_ESP32/Goes%20into%20SD%20card/) into the **root** of the card (not inside a folder):

|File|What it's for|
|-|-|
|`AIRSPACE.txt`|Airspace data, used for warnings|
|`Lower\_North\_Island.ADEM`|Terrain height map, used to work out your height above ground (AGL). It's about 99 MB.|

3. Eject the card safely and put it into the microSD slot on the Waveshare board.

**Notes:**

* The vario still works without the SD card, but you won't get flight logs, maps or airspace.
* You can keep several `.ADEM` maps on the card and choose between them from the menu (**Map**).
* The included airspace and terrain files cover the lower North Island of New Zealand. See [Optional extras](#optional-extras) for other areas.
* An optional splash image can be shown at startup: a file called `ROY.BIN` in the card root. If it's missing, a plain text splash screen is shown instead, so you can skip this.

## Step 8: Assemble everything

!\[Assembled unit](Photos/20260919\_102321.jpg)

1. Fit the **18650 battery** into the holder on the Waveshare board (check the +/− direction printed on the holder).
2. Plug the **speaker** into its 2-pin socket.
3. Push the Waveshare board onto the PCB, making sure the pins are lined up.
4. Connect the GPS and BMP580 modules to the PCB, and attach the antenna to the LoRa module.
5. Fit the board sandwich into the 3D-printed **main case**, then close it with the **back plate** and **base plate**.
6. Push the printed **button** into place over the button on the board. Try the three button lengths and use whichever feels best.

The GPS "bump" on the case (marked "GPS") should face the sky when you hold the unit flat, so the GPS gets a clear view.

Look at the photos in the [`Photos/`](Photos/) folder if you're unsure how something fits.

## Step 9: First power-on check

Do this indoors first, then repeat outside.

1. Switch on / plug in the unit. The **splash screen** appears for about 3 seconds, then the main screen.
2. Move the unit up and down by about a metre. You should hear the **vario beep** and see the altitude change.
3. Take it **outside with a clear view of the sky**. The first GPS lock can take a few minutes. Once it locks, speed, heading and time appear.
4. To see what the device is doing while it boots, open **Tools → Serial Monitor** in the Arduino IDE (baud rate **115200**) while the unit is plugged into your computer. It prints messages showing whether the SD card, barometer, GPS and radio all started correctly.

\---

## How to use it

The unit has **one button**:

|Menu|Press|What happens|
|-|-|-|
|Closed|**Short press**|Switch to the next page (Glider → Wind → ADS-B)|
|Closed|**Double press**|Open the menu|
|Closed|**Hold \~3–4 seconds**|Mute / unmute the vario|
|Open|**Short press**|Move down to the next menu item|
|Open|**Hold \~2 seconds**|Select the highlighted item|

**Menu contents**

* **Config:** time zone, units (altitude / speed), vario pitch, volume, screen orientation
* **Connections:** WiFi on/off and network choice, Bluetooth (engine meter), FANET on/off, FANET preset messages
* **Map:** choose which terrain map to use
* **ADS-B settings:** alert distances, range rings, airspace bar
* **Weather settings:** update interval, number of stations, data source
* **Flight recordings:** recording on/off, **Export files** (see below)

**Flight logs:** the unit automatically starts recording an `.IGC` file when you start moving (above about 10 km/h for 10 seconds). To download logs, open **Flight recordings → Export files**. Once the unit is connected to WiFi it shows a web address (something like `http://192.168.x.x/`). Open that in a browser on a phone or computer on the same network and click the file to download.

\---

## Optional extras

### Get airspace for your own area

1. Go to [openaip.net](https://www.openaip.net/) and crop the airspace for your region.
2. Export it in **OpenAir `.txt`** format.
3. Rename it **`AIRSPACE.txt`** and put it in the root of the SD card, replacing the old one.
4. Very large files can cause problems. Crop to a sensible area around where you fly.

### Get more terrain maps

* Ready-made maps are in this shared folder: [Google Drive maps](https://drive.google.com/drive/folders/18hl1OCvvLns53WSxIf6_7ccbEHTFu314?usp=sharing). Download the `.ADEM` file you want and copy it to the SD card.
* To make your own from a New Zealand LINZ elevation download (GeoTIFF), use the Python script [`dem\_to\_agldem.py`](paraglide_vario_ESP32/dem_to_agldem.py). The script's comments explain how it works.

### Engine meter (paramotor)

The [`Engine\_meter/`](Engine_meter/) folder contains a separate project (nRF52840-based) that measures RPM and cylinder head temperature and sends them to the flight computer over Bluetooth. It is not needed for paragliding.

\---

## Troubleshooting

|Problem|Try this|
|-|-|
|**Can't find a COM port / upload won't start**|Use a USB cable that carries data (many are charge-only). Hold BOOT, tap RESET, release BOOT, then upload again.|
|**Error: `secrets.h: No such file or directory`**|You skipped [Step 5](#step-5-add-your-wifi-details). Copy `secrets.example.h` to `secrets.h`.|
|**Error about `PCF85063A.h`, `U8g2` or another missing file**|A library isn't installed. Go back to [Step 4.4](#44-install-the-libraries).|
|**Error mentioning `U8G2\_ST7305\_300X400…`**|The wrong U8g2 is installed. Use the copy from Waveshare's repo (see Step 4.4).|
|**Screen stays blank or code crashes on boot**|Check the Tools settings in Step 6, especially **PSRAM: OPI PSRAM** and **Flash Size: 16MB**.|
|**No sound**|Check the speaker is plugged in and the vario isn't muted (hold the button \~3 s). Check volume in **Config → Volume**.|
|**No GPS fix**|Go outside with a clear view of the sky and wait a few minutes. Check the GPS module's soldering and that the case doesn't block it.|
|**Altitude doesn't change / vario silent**|The BMP580 isn't being detected. Check its soldering and orientation. The Serial Monitor will report this on boot.|
|**SD card not detected**|Make sure it's FAT32, and files are in the root, not inside a folder. Try a different (smaller) card.|
|**WiFi won't connect**|2.4 GHz only. Check the name and password in `secrets.h` match exactly (they're case-sensitive), then re-upload.|
|**Menu doesn't open with a double press**|Press twice more quickly, or adjust the double-press timing in the code.|
|**No FANET / radio not working**|Check the HT-RA62 soldering and antenna.|

Still stuck? Open an **Issue** on this GitHub page, include what you were doing, what you expected and what happened, and copy in the messages from the Serial Monitor.

\---

## Using it outside New Zealand

The code is set up for New Zealand, so a few things need changing elsewhere:

* **FANET radio frequency** is set to 868.2 MHz (NZ). Other regions differ (for example Australia uses the 915–928 MHz band). It's set in `paraglide\_vario\_ESP32.ino` where `fanetRadio.begin(` is called. **Check your local rules before changing it.**
* **Weather stations** come from Zephyr (`api.zephyrapp.nz`), which only covers NZ. FANET weather still works anywhere.
* **Terrain maps** are built from NZ LINZ data. The conversion script would need different source data elsewhere.
* **Time zone** defaults to NZ (with daylight saving); you can set a fixed offset in **Config → Time**.
* **ADS-B** uses the worldwide adsb.fi service and airspace uses OpenAIP, so those work anywhere.

\---

## Credits

Designed and built by Andrew Sargent, including the PCB and case design. The code was written with help from Claude, GPT-5, Gemini and Copilot, plus code from other projects on GitHub. Thanks to everyone whose open-source work this builds on.

