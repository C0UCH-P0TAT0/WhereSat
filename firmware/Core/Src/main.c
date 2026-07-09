/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with Full ADCS Pipeline Integration.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "host_interface.h"
#include "telemetry.h"
#include "camera_geometry.h"
#include "catalog_loader.h"
#include "triangle_builder.h"
#include "star_matcher.h"
#include "quaternion.h"
#include "quest.h"
#include "test_pipeline.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Redirects standard output (printf) to UART2 for debugging.
 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Ensure SPI Chip Select (PA4) starts HIGH (Inactive)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  printf("\r\n====================================\r\n");
  printf(" WHERESAT STAR TRACKER - WEEK 8\r\n");
  printf(" System Clock: %lu MHz\r\n", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
  printf(" Pipeline: StarID -> QUEST -> MEKF -> Control\r\n");
  printf("====================================\r\n");

  // Initialize the Star Catalog
  if (catalog_init() == false) {
      printf("CRITICAL ERROR: Database failed to load!\r\n");
  } else {
      printf("Database Loaded: %lu Stars, %lu Triangles\r\n", 
             catalog_get_num_stars(), catalog_get_num_triangles());
  }

  // ======================================================
  // RUN AUTOMATED REGRESSION TESTS (Yash's Week 8 Task)
  // ======================================================
  run_all_tests();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    FPGA_Packet_t current_packet;
    ObservedStar live_stars[MAX_CENTROIDS];
    ObservedTriangle triangles[MAX_OBSERVED_TRIANGLES];
    MatchedStar final_matches[MAX_CENTROIDS];
    
    QUEST_Input_t quest_data;
    Quaternion_t estimated_q;

    // Sockets for Aditya's Week 8 Tasks
    // float gyro_data[3] = {0.01f, -0.02f, 0.005f}; // Fake gyro data
    // float torque_cmd[3] = {0.0f, 0.0f, 0.0f};

    while (1)
    {
        // 1. INGEST: Get Centroid Data (Mocked for now, later real SPI)
        load_test_centroids(&current_packet);

        if (fpga_validate_packet(&current_packet)) {
            
            // 2. GEOMETRY: Pixels -> Body Vectors
            for (int i = 0; i < current_packet.count; i++) {
                Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
                live_stars[i].local_id = i;
                live_stars[i].x = vec.x;
                live_stars[i].y = vec.y;
                live_stars[i].z = vec.z;
            }

            // 3. STAR ID: Build Triangles & Match (Yash's Engine)
            uint16_t num_triangles = 0;
            build_triangles(live_stars, current_packet.count, triangles, &num_triangles);
            match_stars(triangles, num_triangles, current_packet.count, final_matches);

            // 4. QUEST: Prepare Data & Solve (Aditya's Engine)
            quest_data.count = 0;
            for (int i = 0; i < current_packet.count; i++) {
                if (final_matches[i].is_matched) {
                    quest_data.body_v[quest_data.count] = (Vector3_t){live_stars[i].x, live_stars[i].y, live_stars[i].z};
                    catalog_get_star_vector(final_matches[i].hip_id, &quest_data.reference_v[quest_data.count]);
                    quest_data.weights[quest_data.count] = 1.0f;
                    quest_data.count++;
                }
            }

            bool adcs_locked = false;
            if (quest_data.count >= 2) {
                estimated_q = quest_compute(&quest_data);
                adcs_locked = true;
            }

            // ==========================================================
            // 5. MEKF: Sensor Fusion (Aditya's Week 8 Task)
            // ==========================================================
            // mekf_predict(gyro_data, dt);
            // if (adcs_locked) {
            //     mekf_update(estimated_q);
            // }
            // Quaternion_t filtered_q = mekf_get_attitude();

            // ==========================================================
            // 6. CONTROLLER: Calculate Torque (Aditya's Week 8 Task)
            // ==========================================================
            // controller_compute_torque(filtered_q, target_q, gyro_data, torque_cmd);

            // 7. TELEMETRY: Clean, professional output
            if (adcs_locked) {
                printf("[TELEMETRY] Stars: %d | Locked: YES | Q: [%.4f, %.4f, %.4f, %.4f]\r\n", 
                       quest_data.count, estimated_q.q0, estimated_q.q1, estimated_q.q2, estimated_q.q3);
            } else {
                printf("[TELEMETRY] Stars: %d | Locked: NO  | Q: [N/A]\r\n", quest_data.count);
            }
        } else {
            printf("[TELEMETRY] SPI Packet Corrupted!\r\n");
        }

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Heartbeat LED
        HAL_Delay(1000); // 1Hz telemetry loop for testing
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = HAL_RCC_GetPCLK1Freq() > 45000000 ? RCC_HCLK_DIV4 : RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */