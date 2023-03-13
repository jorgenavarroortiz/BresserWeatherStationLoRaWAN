# BresserWeatherSensorReceiver with Bresser 7-in-1 decoder

This repository is based on [matthias-bs' BresserWeatherSensorReceiver repo](https://github.com/matthias-bs/BresserWeatherSensorReceiver). Its main addition is to include the Bresser 7-in-1 decoder, since matthias-bs' repo only includes the decoders for Bresser 5-in-1/6-in-1.

Tested with the `BresserWeatherSensorBasic.ino` example and with the following equipment:
- [BRESSER 7-in-1 ClimateConnect Tuya Smart Home Weather Station](https://www.bresser.de/en/Weather-Time/BRESSER-7-in-1-ClimateConnect-Tuya-Smart-Home-Weather-Station.html)
- [BRESSER professional 7-in-1 Wi-Fi Weather Station with Light Intensity and UV Measurement Function](https://www.bresser.de/en/Weather-Time/Weather-Center/BRESSER-professional-7-in-1-Wi-Fi-Weather-Station-with-Light-Intensity-and-UV-Measurement-Function.html). Data available at [Weather Underground](https://www.wunderground.com/dashboard/pws/IGRANA86), [Weather Cloud](https://app.weathercloud.net/d4424986045#current) and [AWEKAS](https://stationsweb.awekas.at/index.php?id=27737).

These stations also includes a light sensor, compared to the 6-in-1 weather stations. The microcontroller was a [Heltec Wireless Stick](https://heltec.org/project/wireless-stick/), which has an SX1276 LoRa chip with the same pinout than the TTGO LoRa32-OLED v2.1.6 board (already supported by mattias-bs' repo and selected as the board in the Arduino IDE).

In order to include the Bresser 7-in-1 decoder, I employed the [decoder from RTL_433](https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_7in1.c) adapted to this repository (similar to the work done by mattias-bs for the Bresser 5-in-1 and 6-in-1 decoders).

**TO BE CHECKED**: The value for rain is incorrect in the RTL_433 tool. It shows 7.2 mm in a sunny day without rain (0.0 mm in the display). The Bresser 7-in-1 decoder employs 3 bytes, as the 6-in-1 decoder. However, the 5-in-1 decoder only employs 2 bytes. Using the first two bytes (msg[10] and msg[11]) the result is 0.0 mm, as it should be. This has to be tested once it is rainning (my weather station is not easily accessible).

## Example with a [BRESSER 7-in-1 ClimateConnect Tuya Smart Home Weather Station](https://www.bresser.de/en/Weather-Time/BRESSER-7-in-1-ClimateConnect-Tuya-Smart-Home-Weather-Station.html)

![image](https://user-images.githubusercontent.com/17797704/222829991-9b3a91fe-9dbc-4125-9b0a-901630880ca6.png)   <img src="https://user-images.githubusercontent.com/17797704/222539637-6bba56e6-e20b-474c-8229-e61a5b199a2c.png" width="384">

![image](https://user-images.githubusercontent.com/17797704/222539809-b47126ac-325f-493c-9a34-5310eac34d75.png)

## Example with a [BRESSER professional 7-in-1 Wi-Fi Weather Station with Light Intensity and UV Measurement Function](https://www.bresser.de/en/Weather-Time/Weather-Center/BRESSER-professional-7-in-1-Wi-Fi-Weather-Station-with-Light-Intensity-and-UV-Measurement-Function.html)

<img src="https://user-images.githubusercontent.com/17797704/222829202-84db7081-6eb8-423d-b3fc-88ba173424ed.png" width="384">   <img src="https://user-images.githubusercontent.com/17797704/222445129-3bb9ef83-785b-44b3-a460-8177400c067e.png" width="384">

![image](https://user-images.githubusercontent.com/17797704/222444791-03e4de29-71d9-454a-a23b-e7c39dc7e63d.png)

## Example showing TTN console and Arduino serial port

![image](https://user-images.githubusercontent.com/17797704/224826387-3b5e05c5-f5ff-4114-ac8c-e50dc92188d2.png)

