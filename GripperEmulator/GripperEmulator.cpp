#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"


int main()
{
    stdio_init_all();
    sleep_ms(2000);

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

        int ch = getchar_timeout_us(0);

        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == 'o') {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                sleep_ms(1000);
                printf("open\n");
            }

            if (ch == 'c') {
                cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
                sleep_ms(1000);
                printf("closed\n");
            }
        }

        sleep_ms(50);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(50);
    }   
}
