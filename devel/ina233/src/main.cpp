#include <Arduino.h>
#include <Wire.h>

#define LED_PIN_0 PA0
#define LED_PIN_1 PA1

#define I2C_SCL PB10
#define I2C_SDA PB11

void setup()
{
	pinMode(LED_PIN_0, OUTPUT);
	pinMode(LED_PIN_1, OUTPUT);
	Wire.begin(I2C_SDA, I2C_SCL);
}

void loop()
{
	for (int addr = 0; addr <= 127; addr++) {

		static bool led_state = false;
		led_state = !led_state;

		// digitalWrite(LED_PIN_0, led_state);
		digitalWrite(LED_PIN_1, !led_state);
		delay(50);

		Wire.beginTransmission(addr);
		if (0 == Wire.endTransmission()) {
			while (1) {
				led_state = !led_state;
				digitalWrite(LED_PIN_0, led_state);
				delay(500);
			}
		}
	}
	delay (5000);
}