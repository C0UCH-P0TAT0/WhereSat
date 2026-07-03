/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with QUEST Integration
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "fpga_interface.h"
#include "camera_geometry.h"
#include "test_suite.h"
#include "mock_fpga.h"
#include "catalog_loader.h"
#include "triangle_builder.h"
#include "star_matcher.h"
#include "quaternion.h"      // Aditya Week 7
#include "quest.h"           // Aditya Week 7
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
 * @brief Redirects standard output (printf) to UART2.
 */
int __io_putchar(int ch) {
    extern UART_HandleTypeDef huart2;
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
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

  // System Boot Message
  printf("\r\n====================================\r\n");
  printf(" WHERESAT STAR TRACKER - WEEK 7\r\n");
  printf(" System Clock: %lu MHz\r\n", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
  printf(" Pipeline: StarID -> QUEST\r\n");
  printf("====================================\r\n");

  // Run Aditya's Geometry Verification
  test_camera_geometry();

  // Initialize the Star Catalog (Yash's Task)
  if (catalog_init() == false) {
      printf("CRITICAL ERROR: Database failed to load!\r\n");
  } else {
      printf("Database Loaded: %lu Stars, %lu Triangles\r\n",
             catalog_get_num_stars(), catalog_get_num_triangles());
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  FPGA_Packet_t current_packet;
  ObservedStar live_stars[MAX_CENTROIDS];
  ObservedTriangle triangles[MAX_OBSERVED_TRIANGLES];
  MatchedStar final_matches[MAX_CENTROIDS];

  QUEST_Input_t quest_data;    // Aditya Week 7
  Quaternion_t estimated_q;    // Aditya Week 7

  while (1)
  {
      // 1. Get Centroid Data (Mocked for Week 7 Integration)
      load_test_centroids(&current_packet);

      if (fpga_validate_packet(&current_packet)) {

          printf("\r\n--- PROCESSING NEW FRAME (%d Stars) ---\r\n", current_packet.count);

          // 2. Camera Geometry: Pixels -> Body Vectors
          for (int i = 0; i < current_packet.count; i++) {
              Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
              live_stars[i].local_id = i;
              live_stars[i].x = vec.x;
              live_stars[i].y = vec.y;
              live_stars[i].z = vec.z;
          }

          // 3. Star Identification: Build Triangles & Match
          uint16_t num_triangles = 0;
          build_triangles(live_stars, current_packet.count, triangles, &num_triangles);
          match_stars(triangles, num_triangles, current_packet.count, final_matches);

          // 4. QUEST Integration: Prepare Observation/Reference Pairs
          quest_data.count = 0;
          printf("Star ID Results:\r\n");

          for (int i = 0; i < current_packet.count; i++) {
              if (final_matches[i].is_matched) {
                  printf("  [%d] HIP %lu (Votes: %d)\r\n", i, final_matches[i].hip_id, final_matches[i].vote_count);

                  // Add to QUEST input
                  // Body vector is what we measured
                  quest_data.body_v[quest_data.count] = (Vector3_t){live_stars[i].x, live_stars[i].y, live_stars[i].z};

                  // Reference vector is retrieved from the catalog using the HIP ID
                  catalog_get_star_vector(final_matches[i].hip_id, &quest_data.reference_v[quest_data.count]);

                  quest_data.weights[quest_data.count] = 1.0f; // Equal weighting for now
                  quest_data.count++;
              }
          }

          // 5. Compute Attitude if we have enough stars (Minimum 2)
          if (quest_data.count >= 2) {
              estimated_q = quest_compute(&quest_data);

              printf("\r\n>>> ATTITUDE DETERMINED <<<\r\n");
              printf("Quaternion [q0, q1, q2, q3]:\r\n");
              printf("[ %.6f, %.6f, %.6f, %.6f ]\r\n",
                      estimated_q.q0, estimated_q.q1, estimated_q.q2, estimated_q.q3);
          } else {
              printf("\r\n>>> ATTITUDE FAILED: Insufficient matches for QUEST (%d/2) <<<\r\n", quest_data.count);
          }
      }

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Heartbeat
      HAL_Delay(5000); // 5 second update rate for debug
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
