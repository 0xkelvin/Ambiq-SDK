//*****************************************************************************
//
//! @file print_task.c
//!
//! @brief Simple task with a while() loop that implements a simple print queue
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk5p0p1-61912905f0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "freertos_sample.h"

//*****************************************************************************
//
// Global variables and templates
//
//*****************************************************************************
static char rcv_msg[SPRINTF_SIZE];

//*****************************************************************************
//
// print task handle.
//
//*****************************************************************************
TaskHandle_t print_task_handle;

//*****************************************************************************
//
// print queue handle.
//
//*****************************************************************************
xQueueHandle simple_print_queue;

//*****************************************************************************
//
// Perform initial setup for the print task.
//
//*****************************************************************************
void
PrintTaskSetup(void)
{
    simple_print_queue = xQueueCreate(PRINT_QUEUE_SIZE, SPRINTF_SIZE);

    if (NULL == simple_print_queue)  /* Queue not created */
    {
        am_util_stdio_printf("Print Queue Not Created\r\n");
    }
    else
    {
        am_util_stdio_printf("Print Queue Created\r\n");
    }
}

//*****************************************************************************
//
// PrintTask
//
//*****************************************************************************

void
PrintTask(void *pvParameters)
{
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS; /* 10m second delay */

    while (1)
    {
        //
        // See if there's a message in the queue (do not block)
        //
        if (xQueueReceive(simple_print_queue, (void *)&rcv_msg, (TickType_t)NULL) == pdTRUE)
        {
            am_util_stdio_printf(rcv_msg);
        }

        vTaskDelay(xDelay);
    }
}
