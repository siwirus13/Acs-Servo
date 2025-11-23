
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../include/servos.h"

void app_main(void)
{
    printf("Initializing servos...\n");

    if (!init_servos()) {
        printf("Servo initialization FAILED!\n");
        return;
    }

    printf("Servos initialized. Starting movement test...\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    canards_t pos;

    while (1) {

        pos.canard_angle1 = -30;
        pos.canard_angle2 = -30;
        pos.canard_angle3 = -30;
        pos.canard_angle4 = -30;
        update_canards(&pos);
        printf("Canards -> -30°\n");
        vTaskDelay(pdMS_TO_TICKS(1000));

        pos.canard_angle1 = 30;
        pos.canard_angle2 = 30;
        pos.canard_angle3 = 30;
        pos.canard_angle4 = 30;
        update_canards(&pos);
        printf("Canards -> +30°\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
