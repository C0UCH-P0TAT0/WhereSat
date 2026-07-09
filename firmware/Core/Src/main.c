/**
  * @file           : main.c
  * @brief          : Final ADCS Pipeline: StarID -> QUEST -> MEKF -> Control.
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
#include "mekf.h"        
#include "controller.h"  
#include "test_pipeline.h" 
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // <--- THIS PROTOTYPE FIXES THE ERROR
void Error_Handler(void);

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
  /* Hardware Initialization */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  printf("\r\n====================================\r\n");
  printf(" WHERESAT STAR TRACKER - WEEK 8\r\n");
  printf(" Pipeline: StarID -> QUEST -> MEKF -> Control\r\n");
  printf(" Convention: Scalar-Last [x, y, z, w]\r\n");
  printf("====================================\r\n");

  test_camera_geometry();
  if (catalog_init() == false) {
      printf("CRITICAL ERROR: Database failed to load!\r\n");
  }
  run_all_tests(); 

  // --- ADCS INITIALIZATION ---
  MEKF_t filter;
  mekf_init(&filter, (Quaternion_t){0, 0, 0, 1}); 

  PD_Controller_t adcs_controller = {
      .Kp = 0.5f,
      .Kd = 0.1f
  };

  Quaternion_t target_q = {0, 0, 0, 1};          
  Vector3_t gyro_data = {0.01f, -0.02f, 0.005f}; 
  Vector3_t torque_cmd = {0, 0, 0};
  float dt = 5.0f;                               
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
      load_test_centroids(&current_packet);

      if (fpga_validate_packet(&current_packet)) {
          printf("\r\n--- PROCESSING NEW FRAME (%d Stars) ---\r\n", current_packet.count);

          // 1. GEOMETRY
          for (int i = 0; i < current_packet.count; i++) {
              Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
              live_stars[i].local_id = i;
              live_stars[i].x = vec.x; live_stars[i].y = vec.y; live_stars[i].z = vec.z;
          }

          // 2. STAR ID
          uint16_t num_triangles = 0;
          build_triangles(live_stars, current_packet.count, triangles, &num_triangles);
          match_stars(triangles, num_triangles, current_packet.count, final_matches);

          // 3. QUEST PREP
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

          // 4. ATTITUDE DETERMINATION (QUEST)
          if (quest_data.count >= 2) {
              adcs_locked = true;
              estimated_q = quest_compute(&quest_data);
          }

          // 5. SENSOR FUSION (MEKF)
          mekf_predict(&filter, gyro_data, dt);
          
          if (adcs_locked) {
              mekf_update(&filter, estimated_q, 0.001f);
          }
          
          Quaternion_t filtered_q = filter.q;

          // 6. CONTROL
          torque_cmd = controller_compute_torque(&adcs_controller, filtered_q, target_q, gyro_data);

          // 7. TELEMETRY
          printf("\r\n==================================================\r\n");
          printf("[ADCS] Mode: %s | Stars Matched: %d\r\n", adcs_locked ? "LOCKED" : "SEARCHING", quest_data.count);
          printf("[MEKF] Att [x,y,z,w]: [%.4f, %.4f, %.4f, %.4f]\r\n", filtered_q.x, filtered_q.y, filtered_q.z, filtered_q.w);
          printf("[MEKF] Bias [rad/s]:  [%.5f, %.5f, %.5f]\r\n", filter.beta[0], filter.beta[1], filter.beta[2]);
          printf("[CTRL] Torque [Nm]:   [%.3f, %.3f, %.3f]\r\n", torque_cmd.x, torque_cmd.y, torque_cmd.z);
          printf("==================================================\r\n");

      }

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); 
      HAL_Delay(5000); 
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = HAL_RCC_GetPCLK1Freq() > 45000000 ? RCC_HCLK_DIV4 : RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}