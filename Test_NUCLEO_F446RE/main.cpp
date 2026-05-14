#include "mbed.h"

// Initialize the built-in LED (LD2 on NUCLEO-F446RE) as an output
DigitalOut led(LED1);

int main() {
    // Print a startup message to the serial console
    printf("NUCLEO-F446RE is alive!\n");
    
    while (true) {
        led = !led; // Toggle LED
        printf("LED is blinking...\n");
        ThisThread::sleep_for(500ms); // Sleep for 500 milliseconds
    }
}
