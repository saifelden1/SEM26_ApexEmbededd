#include "hall_sensor.h"

// Define the Hall sensor structs and initialize them
/*example layout
 * typedef struct {GPIO_PinState state ,GPIO_PinState filteredState ,uint16_t pin ,GPIO_TypeDef* port; } HallSensor;*/
HallSensor_t hallA = {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_3, GPIOB}; // Hall A on GPIOB
HallSensor_t hallB = {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_4, GPIOB}; // Hall B on GPIOB
HallSensor_t hallC = {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_5, GPIOB}; // Hall C on GPIOB

// Global variable to store the combined filtered value
uint8_t HallSensor_Combined = 0;  // Holds the combined value of the filtered Hall sensors

// Hall sensor initialization (handled by CubeMX, do not modify)
void HallSensor_Init(void) {
    // CubeMX handles the initialization of GPIO pins
    // No need to modify here as CubeMX configures the GPIO pins and initializes the peripherals
}

// Read Hall sensor by pin (using the struct)
uint8_t HallSensor_Read(HallSensor_t *sensor) {

	sensor->state = HAL_GPIO_ReadPin(sensor->port, sensor->pin);
    return sensor->state;
}




// Read each Hall sensor multiple times and apply a filter (majority vote)
uint8_t HallSensor_ReadWithFilter(HallSensor_t *sensor, uint8_t iterations) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < iterations; i++) {
        if (HAL_GPIO_ReadPin(sensor->port, sensor->pin) == GPIO_PIN_SET) {
            count++;
        }
    }

    // Apply majority vote filtering: if count > half of iterations, return 1, else return 0
    sensor->filteredState = (count > (iterations / 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return sensor->filteredState;
}

// Efficient function to combine the filtered Hall sensor states
uint8_t HallSensor_GetCombinedHallState(void) {
    // Filter each Hall sensor value (only once for each)
	HallSensor_ReadWithFilter(&hallA, FILTER_ITERATIONS);
	HallSensor_ReadWithFilter(&hallB, FILTER_ITERATIONS);
	HallSensor_ReadWithFilter(&hallC, FILTER_ITERATIONS);

    // Combine the Hall sensor states into a single variable (using bit shifting)
	HallSensor_Combined = (hallC.filteredState << 2) | (hallB.filteredState << 1) | hallA.filteredState;
    return HallSensor_Combined;
}
