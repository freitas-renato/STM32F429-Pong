#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <FreeRTOS.h>
#include "queue.h"
#include "gpio.h"

xQueueHandle button_queue = 0;

Model::Model() : modelListener(0)
{
    button_queue = xQueueGenericCreate(5, sizeof(button_event_t), queueQUEUE_TYPE_BASE);

}

void Model::tick()
{
    button_event_t received_button_event;

    if (xQueueReceive(button_queue, &received_button_event, 0) == pdTRUE) {
        modelListener->buttonEventHandler(received_button_event.button_id, received_button_event.button_state);
    }



}
