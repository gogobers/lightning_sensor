# Blitz- und Gewitterwarner auf Basis des AS3935

Verwendet wird
- ESP32-C3 Super mini: https://randomnerdtutorials.com/getting-started-esp32-c3-super-mini/
- Board: DFRobot Lightning Sensor V1.0 https://www.dfrobot.com/product-1828.html
- mit AS3935 Franklin lightning sensor IC and Coilcraft MA5532-AE
- Open Source Alternative zum Board: https://github.com/sparkfun/SparkFun_AS3935_Lightning_Detector
- Chip: https://www.sciosense.com/as3935-franklin-lightning-sensor-ic/
- Library: https://github.com/sparkfun/SparkFun_AS3935_Lightning_Detector_Arduino_Library

## Verdrahtung (ESP32‑C3 Super Mini, I²C)

- **ESP32‑C3 SDA (GPIO6) → AS3935 SDA**
- **ESP32‑C3 SCL (GPIO7) → AS3935 SCL**
- **3V3 ↔ VCC**, **GND ↔ GND**
- **AS3935 IRQ → ESP32‑C3 GPIO10** (im Code `PIN_AS3935_IRQ`)
- 4 LEDs (mit Vorwiderständen) an die Pins `3, 4, 5, 1` gegen GND

### Verbesserung der Antenne des ESP32-C3 

lohnt sich und kann so gemacht werden:
https://peterneufeld.wordpress.com/2025/03/04/esp32-c3-supermini-antenna-modification/?utm_source=chatgpt.com

Im Kern nimmt man hier 
- 1mm Kupferdraht, 31mm lang ($\lambda/4$)
- Am Fuß Schleife mit Durchmesser 5mm (mm 5mm-Bohrer wickeln), so, dass dieser Teil der Antenne genau auf die Keramik-Antenne passt
- geraden Teil hoch biegen (Schleife ist links, dann wird rechter Teil hoch gebogen)
- Einfach über die Keramikantenne klemmen und fest löten

## LED-Logik

AS3935 liefert folgende Distanzschätzung: 

| REG0x07[5:0] | Distance [km] | grün | grün | gelb | rot |
|--------------|---------------|------|------|------|-----|        
| 111111 | Out of range        |      |      |      |     |
| 101000 | 40                  |  on  |      |      |     |
| 100101 | 37                  |      | on   |      |     |
| 100010 | 34                  |  on  | on   |      |     |
| 011111 | 31                  |      |      | on   |     |
| 011011 | 27                  |  on  |      | on   |     |
| 011000 | 24                  |      | on   | on   |     |
| 010100 | 20                  |  on  | on   | on   |     |
| 010001 | 17                  |      |      |      | on  |
| 001110 | 14                  |  on  |      |      | on  |
| 001100 | 12                  |      | on   |      | on  |
| 001010 | 10                  |  on  | on   |      | on  |
| 001000 |  8                  |      |      | on   | on  |
| 000110 |  6                  |  on  |      | on   | on  |
| 000101 |  5                  |      | on   | on   | on  |
| 000001 | Storm is Overhead   |  on  | on   | on   | on  |
