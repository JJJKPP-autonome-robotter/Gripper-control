#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"


int main()
{
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        char input_buffer[10]; // Input buffer

        if (fgets(input_buffer, sizeof(input_buffer), stdin)) {
            input_buffer[strcspn(input_buffer, "\r\n")] = 0; // Remove new line chars

            if (strcmp(input_buffer, "open") == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Toggle LED
                sleep_ms(1000);
                printf("open"); // Return status
                continue;
            }
            
            if (strcmp(input_buffer, "close") == 0) {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); // Toggle LED
                sleep_ms(1000);
                printf("closed"); // Return status
                continue;
            }

            printf("Unkown command");
        }
    }
}
