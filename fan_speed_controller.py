# import RPi.GPIO as GPIO
# import time
import time
import math
from machine import Pin,PWM,ADC,I2C
from utime import sleep
from pico_i2c_lcd import I2cLcd
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
I2C_ADDR = i2c.scan()[0]
lcd = I2cLcd(i2c, I2C_ADDR, 4, 20)
# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BOARD)
# GPIO.setup(28,GPIO.IN) // No inbuilt ADC for Pi
# GPIO.setup(15,GPIO.OUT)
# Define the pin connected to the NTC thermistor
ntc_pin = Pin(26)
# Create ADC object
adc = ADC(Pin(26)) # Use ADC 0 (0-65535)
LDR = ADC(28)
pwm = PWM(Pin(15))
# The frequency (pwm.freq) tells Raspberry Pi Pico how often to switch 
# the power between on and off for the LED.
pwm.freq(1000)

switch_pin = Pin(9, Pin.IN)
# Function to read temperature from NTC sensor
def read_temperature():
    # Read raw analog value from ADC
    adc_value = adc.read_u16()
    #print(adc)
    # Convert ADC value to voltage
    voltage = adc_value * 3.3 / 65535  # 3.3V reference voltage
    # Calculate resistance of NTC based on voltage divider
    resistor = 10000  # 10kOhm resistor
    ntc_resistance = resistor * (3.3 / voltage -1)
    # Calculate temperature using Steinhart-Hart equation
    # NTC parameters: resistance at 25°C (R25), B coefficient, reference temperature (Tref)
    R25 = 10000  # Resistance of NTC at 25°C
    B_coefficient = 3950  # B coefficient of NTC
    Tref = 298.15  # Reference temperature in Kelvin (25°C)
    # Calculate temperature in Kelvin
    temperature_kelvin = 1 / (1 / Tref + 1 / B_coefficient * math.log(ntc_resistance / R25))
    # Convert temperature from Kelvin to Celsius
    temperature_celsius = temperature_kelvin - 273.15
    return temperature_celsius
# Main loop
while True:
    # Read temperature from NTC sensor
    temperature = int(read_temperature())
    if temperature<=10:
        lcd.clear()
        lcd.putstr("Temperature : "+str(temperature)+" C  Fan speed: OFF State")
    elif 10<temperature<=20 :
        lcd.clear()
        lcd.putstr("Temperature : "+str(temperature)+" C  Fan speed: 25% of speed")
    elif 20<temperature<=25 :
        lcd.clear()
        lcd.putstr("Temperature : "+str(temperature)+" C  Fan speed: 50% of speed")
    elif 25<temperature<=30 :
        lcd.clear()
        lcd.putstr("Temperature : "+str(temperature)+" C  Fan speed: 75% of speed")
    else: 
        lcd.clear()
        lcd.putstr("Temperature : "+str(temperature)+" C  Fan speed: 100% of speed")
    # Print temperature
    #print("Temperature: {:.2f} °C".format(temperature))
    # Wait for a short period
    switchState=switch_pin.value()
    print(switchState)
    if switchState==1:
        value = LDR.read_u16()
        #print(value)
        pwm.duty_u16(value)
        if temperature <=10:
            pwm.duty_u16(0)
        while switchState==0 and temperature>10 :
            pwm.duty_u16(0)
    else:
        pwm.duty_u16(0)
    time.sleep(1)
# For Raspberry Pi Pico in MicroPython, this can range from 0 to 65025. 
# 65025 would be 100% of the time, so the LED would stay bright. 
# A value of around 32512 would indicate that it should be on for half the time.
# Have a play with the pwm.freq() values and the pwm.duty_u16 values, 
# as well as the length of time for the sleep, 
# to get a feel for how you can adjust the brightness and pace of the pulsing LED.
