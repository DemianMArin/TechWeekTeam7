# TechWeekTeam7

2 Hr Hardware Hackathon with 'Nucleo-G474RE NUG474RE$AT4' 

Hackathon Idea: Gesture control lock using a set of movements to unlock it. Using the gyroscope to measure the set of movements to unlock a door.

Hardware: 
- Microcontroller: NUCLEO-G474RE
- IMU: MPU-6050
- Servo: SG90
- Speaker: Adafruit STEMMA Speaker

## Pinout

| Signal | MCU Pin | Arduino Header | Peripheral |
|---|---|---|---|
| I2C1 SCL (MPU-6050) | PB8 | D15 | I2C1 |
| I2C1 SDA (MPU-6050) | PB9 | D14 | I2C1 |
| Servo PWM (SG90) | PA6 | D12 | TIM3 CH1 |
| Speaker Signal (STEMMA) | PA9 | D8 | TIM1 CH2 |
| System Reset Button | PC13 | B1 (User Button) | GPIO Input |

### Power
| Rail | Source | Connected To |
|---|---|---|
| 3.3V | Nucleo 3.3V pin | MPU-6050 VCC, STEMMA Speaker VCC |
| 5V | Nucleo 5V pin | SG90 VCC |
| GND | Nucleo GND | All peripherals |

### Notes
- MPU-6050 AD0 → GND (sets I2C address to 0x68)
- SG90 signal is 3.3V logic compatible despite 5V power supply
- STEMMA Speaker volume adjustable via onboard trim pot
