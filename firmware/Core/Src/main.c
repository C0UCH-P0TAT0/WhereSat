/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with QUEST Integration & Validation
  ******************************************************************************
  * @author Aditya (WhereSat Team)
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
#include <math.h>
#include "fpga_interface.h"
#include "camera_geometry.h"
#include "test_suite.h"
#include "mock_fpga.h"
#include "catalog_loader.h"
#include "triangle_builder.h"
#include "star_matcher.h"
#include "quaternion.h"
#include "quest.h"
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

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
  */
int main(void)
{
  /* HAL and System Clock Initialization */
  HAL_Init();
  SystemClock_Config();

  /* Initialize Peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // SPI CS High

  printf("\r\n====================================\r\n");
  printf(" WHERESAT STAR TRACKER - WEEK 7\r\n");
  printf(" System Clock: %lu MHz\r\n", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
  printf(" Convention: Scalar-First (w, x, y, z)\r\n");
  printf("====================================\r\n");

  // 1. Verify Camera Geometry Math
  test_camera_geometry();

  // 2. Initialize Star Catalog
  if (catalog_init() == false) {
      printf("CRITICAL ERROR: Database failed to load!\r\n");
  } else {
      printf("Database Loaded: %lu Stars, %lu Triangles\r\n", 
             catalog_get_num_stars(), catalog_get_num_triangles());
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  FPGA_Packet_t current_packet;
  ObservedStar live_stars[MAX_CENTROIDS];
  ObservedTriangle triangles[MAX_OBSERVED_TRIANGLES];
  MatchedStar final_matches[MAX_CENTROIDS];

  QUEST_Input_t quest_data;
  Quaternion_t estimated_q;

  while (1)
  {
      // 3. Load Mock Centroids (Day 5 logic)
      load_test_centroids(&current_packet);

      if (fpga_validate_packet(&current_packet)) {

          printf("\r\n--- PROCESSING NEW FRAME (%d Stars) ---\r\n", current_packet.count);

          // 4. Camera Geometry: Pixels -> Body Vectors
          for (int i = 0; i < current_packet.count; i++) {
              Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
              live_stars[i].local_id = i;
              live_stars[i].x = vec.x;
              live_stars[i].y = vec.y;
              live_stars[i].z = vec.z;
          }

          // 5. Star Identification: Build Triangles & Match
          uint16_t num_triangles = 0;
          build_triangles(live_stars, current_packet.count, triangles, &num_triangles);
          match_stars(triangles, num_triangles, current_packet.count, final_matches);

          // 6. QUEST Integration: Pair Body Vectors with Catalog Reference Vectors
          quest_data.count = 0;
          for (int i = 0; i < current_packet.count; i++) {
              if (final_matches[i].is_matched) {
                  // Body vector (Measured)
                  quest_data.body_v[quest_data.count] = (Vector3_t){live_stars[i].x, live_stars[i].y, live_stars[i].z};

                  // Reference vector (Catalog)
                  catalog_get_star_vector(final_matches[i].hip_id, &quest_data.reference_v[quest_data.count]);

                  quest_data.weights[quest_data.count] = 1.0f;
                  quest_data.count++;
              }
          }

          // 7. Compute Attitude
          if (quest_data.count >= 2) {
              estimated_q = quest_compute(&quest_data);

              // Force Scalar-Positive (Canonical Form to match Python)
              if (estimated_q.q0 < 0) {
                  estimated_q.q0 *= -1.0f;
                  estimated_q.q1 *= -1.0f;
                  estimated_q.q2 *= -1.0f;
                  estimated_q.q3 *= -1.0f;
              }

              printf("\r\n>>> ATTITUDE DETERMINED <<<\r\n");
              printf("Quaternion [w, x, y, z]:\r\n");
              printf("[ %.6f, %.6f, %.6f, %.6f ]\r\n",
                      estimated_q.q0, estimated_q.q1, estimated_q.q2, estimated_q.q3);

              // 8. Direction Validation Test
              // Rotate the first reference vector by our estimate.
              // It should match the first body vector.
              Vector3_t check_vec = quat_rotate_vector(estimated_q, quest_data.reference_v[0]);
              printf("Direction Check (Rotated Ref vs Measured Body):\r\n");
              printf("  Expected: [%.3f, %.3f, %.3f]\r\n", quest_data.body_v[0].x, quest_data.body_v[0].y, quest_data.body_v[0].z);
              printf("  Actual:   [%.3f, %.3f, %.3f]\r\n", check_vec.x, check_vec.y, check_vec.z);

          } else {
              printf("\r\n>>> ATTITUDE FAILED: Insufficient matches (%d/2) <<<\r\n", quest_data.count);
          }
      }

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Heartbeat LED
      HAL_Delay(5000);
    /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration (180MHz)
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
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  HAL_PWREx_EnableOverDrive();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
