
///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, Murat Terzi
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "MCP_Advanced.h"
#define DEBUG_MotorListen
#define MCP_Addr 0x81 //address

static const char *MCP_TAG = "MCP_Controller";
static SemaphoreHandle_t MCP_Serial_Semaphore = NULL;
MCP_Advanced MCP_Controller(UART_NUM_1,5);

extern "C"{
void app_main();
}

static void MCP_Controller_Listen(void *parameter){
    bool valid = false;
    uint8_t status;
    uint32_t Motor1_current_velocity = 0;
    uint32_t Motor2_current_velocity = 0;
    uint32_t Motor1_current_position = 0;
    uint32_t Motor2_current_position = 0;
    int16_t Motor1_amper = 0;
    int16_t Motor2_amper = 0;
    while(1){
        //MOTOR 1 Example
        if(xSemaphoreTake(MCP_Serial_Semaphore, (TickType_t) 500) == pdTRUE){ // Try to take the MCP Serial Semaphore for 500ms
            valid=MCP_Controller.Get_Encoder1_Count(MCP_Addr,Motor1_current_position,status);
            if(!valid){
                ESP_LOGE(MCP_TAG,"get_encoder1_count failed");
            }
            valid=MCP_Controller.Get_Encoder1_Speed(MCP_Addr,Motor1_current_velocity,status);
            if(!valid){
                ESP_LOGE(MCP_TAG,"get_encoder1_speed failed");
                Motor1_current_velocity = 0;
            }
            valid=MCP_Controller.Get_Both_Currents(MCP_Addr,Motor1_amper,Motor2_amper);
            if(!valid){
                ESP_LOGE(MCP_TAG,"get_encoder_amper failed");
                Motor1_amper = 0;
            }
            if ( ( MCP_Serial_Semaphore ) != NULL ){
                xSemaphoreGive( ( MCP_Serial_Semaphore ) );  // Make the MCP Serial Port available for use, by "Giving" the Semaphore.
            }
        }
            ESP_LOGI(MCP_TAG,"Motor1 -> Position: %d, Velocity: %d, Amp: %.3f", Motor1_current_position,Motor1_current_velocity, float(Motor1_amper)/100); // read the documentation for unit conversations
    }
}
static void MCP_Controller_Drive(void *parameter){
    bool valid = false;
    int Motor1_desired_velocity = 0;
    while(1){
        if(xSemaphoreTake(MCP_Serial_Semaphore, (TickType_t) 500) == pdTRUE){ // Try to take the MCP Serial Semaphore for 500ms
            valid=MCP_Controller.Drive_M1_with_Vel(MCP_Addr, Motor1_desired_velocity);
            if(!valid){
                ESP_LOGE(MCP_TAG,"Tried %d but failed", MAXRETRY);
            }
            if(Motor1_desired_velocity<50000){
                Motor1_desired_velocity+=50;
            }
            if ( ( MCP_Serial_Semaphore ) != NULL ){
                xSemaphoreGive( ( MCP_Serial_Semaphore ) );  // Make the MCP Serial Port available for use, by "Giving" the Semaphore.
            }
        }
        
        vTaskDelay( 500 / portTICK_PERIOD_MS );
    }
}

void app_main(){
    if ( MCP_Serial_Semaphore == NULL )  // Check to confirm that the MCP_Serial_Semaphore has not already been created.
    {
        MCP_Serial_Semaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore, we will use to manage the MCP Serial
        if ( ( MCP_Serial_Semaphore ) != NULL )
        xSemaphoreGive( ( MCP_Serial_Semaphore ) );  // Make the MCP Serial available for use, by "Giving" the Semaphore.
    }
    MCP_Controller.start(1000000);//1Mbps
    xTaskCreate(&MCP_Controller_Listen, "MCP TASK", 4096, NULL, 5, NULL);
    xTaskCreate(&MCP_Controller_Drive, "MCP TASK", 4096, NULL, 5, NULL);
}