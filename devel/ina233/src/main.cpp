#include <Arduino.h>

#include "i2c_bus.h"
#include "ina233.h"

#define LED_PIN_0 PA0
#define LED_PIN_1 PA1

#define I2C_SCL PB10
#define I2C_SDA PB11

I2cBus Wire = I2cBus();
Ina233 ina233 = Ina233(&(Wire), (uint8_t)0x40);

void setup()
{
	pinMode(LED_PIN_0, OUTPUT);
	pinMode(LED_PIN_1, OUTPUT);
	Wire.setSCL(I2C_SCL);
	Wire.setSDA(I2C_SDA);
	Wire.begin();
}

void loop()
{
	for (int addr = 1; addr < 127; addr++) {

		static bool led_state = false;
		led_state = !led_state;

		// digitalWrite(LED_PIN_0, led_state);
		digitalWrite(LED_PIN_1, !led_state);
		delay(50);

		Wire.beginTransmission(addr);
		if (0 == Wire.endTransmission()) {

			uint8_t model[6] = {0};
			ina233.getModel(model);

			while (1) {
				led_state = !led_state;
				digitalWrite(LED_PIN_0, led_state);

				if (addr == 0x40)
					digitalWrite(LED_PIN_1, HIGH);
				else digitalWrite(LED_PIN_1, LOW);

				delay(500);
			}
		}
	}
	delay (5000);
}

