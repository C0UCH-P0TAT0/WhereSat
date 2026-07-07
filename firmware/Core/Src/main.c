/**
 * @file main.c
 * @brief Main program body with QUEST Integration and Convention Verification.
 *
 * This file implements the full Lost-in-Space pipeline:
 * Mock Centroids -> Body Vectors -> Star ID -> QUEST -> Attitude.
 * It includes a dual-direction check to verify if the quaternion represents
 * Reference-to-Body or Body-to-Reference using the [x, y, z, w] convention.
 *
 * @author Aditya (WhereSat Team)
 */

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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock to 180MHz */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  printf("\r\n====================================\r\n");
  printf(" WHERESAT STAR TRACKER - WEEK 7/8\r\n");
  printf(" System Clock: %lu MHz\r\n", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
  printf(" Convention: Scalar-Last [x, y, z, w]\r\n");
  printf("====================================\r\n");

  // Verify Aditya's geometry math
  test_camera_geometry();

  // Initialize the Star Catalog
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
  int matched_source_indices[MAX_CENTROIDS]; // Map QUEST input back to original star index

  while (1)
  {
      // 1. Get Centroid Data (Mocked)
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
          for (int i = 0; i < current_packet.count; i++) {
              if (final_matches[i].is_matched) {
                  // Store mapping for debug printing
                  matched_source_indices[quest_data.count] = i;

                  // Populate QUEST input arrays
                  quest_data.body_v[quest_data.count] = (Vector3_t){live_stars[i].x, live_stars[i].y, live_stars[i].z};
                  catalog_get_star_vector(final_matches[i].hip_id, &quest_data.reference_v[quest_data.count]);

                  quest_data.weights[quest_data.count] = 1.0f;
                  quest_data.count++;
              }
          }

          // 5. Compute Attitude and Verify Convention
          if (quest_data.count >= 2) {

              // Debug: Print Matched Vectors (Corrected Indexing)
              printf("\nQUEST Input Vectors:\n");
              for(int i=0; i < quest_data.count; i++) {
                  int original_idx = matched_source_indices[i];
                  printf("[%d] HIP ID: %lu\n", i, final_matches[original_idx].hip_id);
                  printf("  Ref : [% .6f, % .6f, % .6f]\n",
                         quest_data.reference_v[i].x, quest_data.reference_v[i].y, quest_data.reference_v[i].z);
                  printf("  Body: [% .6f, % .6f, % .6f]\n",
                         quest_data.body_v[i].x, quest_data.body_v[i].y, quest_data.body_v[i].z);
              }

              estimated_q = quest_compute(&quest_data);

              printf("\r\n>>> ATTITUDE DETERMINED <<<\r\n");
              printf("Quaternion [x, y, z, w]:\r\n");
              printf("[ %.6f, %.6f, %.6f, %.6f ]\r\n",
                     estimated_q.x, estimated_q.y, estimated_q.z, estimated_q.w);

              /* ---------- Direction Check ---------- */
              Vector3_t expected = quest_data.body_v[0];

              // Test Original Quaternion (q)
              Vector3_t actual = quat_rotate_vector(estimated_q, quest_data.reference_v[0]);

              // Test Conjugate (q_conj)
              Quaternion_t qc = quat_conjugate(estimated_q);
              Vector3_t actual_inv = quat_rotate_vector(qc, quest_data.reference_v[0]);

              printf("\r\nDirection Check (Rotated Ref vs Measured Body):\r\n");
              printf("Expected:        [% .3f, % .3f, % .3f]\r\n", expected.x, expected.y, expected.z);
              printf("Actual (q):      [% .3f, % .3f, % .3f]\r\n", actual.x, actual.y, actual.z);
              printf("Actual (q_conj): [% .3f, % .3f, % .3f]\r\n", actual_inv.x, actual_inv.y, actual_inv.z);

              // Calculate Errors (Optimized dx*dx)
              float dx = expected.x - actual.x;
              float dy = expected.y - actual.y;
              float dz = expected.z - actual.z;
              float err = sqrtf(dx*dx + dy*dy + dz*dz);

              float dx_inv = expected.x - actual_inv.x;
              float dy_inv = expected.y - actual_inv.y;
              float dz_inv = expected.z - actual_inv.z;
              float err_inv = sqrtf(dx_inv*dx_inv + dy_inv*dy_inv + dz_inv*dz_inv);

              printf("Direction Error (q)      = %.6f\r\n", err);
              printf("Direction Error (q_conj) = %.6f\r\n", err_inv);

          } else {
              printf("\r\n>>> ATTITUDE FAILED: Insufficient matches (%d/2) <<<\r\n", quest_data.count);
          }
      }

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(5000);
    /* USER CODE END WHILE */
  }
}

/**
  * @brief System Clock Configuration
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

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
