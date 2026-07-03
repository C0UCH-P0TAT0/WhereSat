/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "fpga_interface.h"
#include "camera_geometry.h"
#include "test_suite.h"
#include "mock_fpga.h"
#include "catalog_loader.h"    // <-- ADDED
#include "triangle_builder.h"  // <-- ADDED
#include "star_matcher.h"      // <-- ADDED
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
 *
 * This function overrides the weak implementation in the syscalls file.
 * It allows us to use printf() to send debug messages to the PC terminal.
 *
 * @param ch The character to send
 * @return int The character sent
 */
int __io_putchar(int ch) {
    // External handle for UART2 (defined in usart.c)
    extern UART_HandleTypeDef huart2;

    // Transmit 1 byte over UART2 with a 10ms timeout
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

  // Day 1 Verification Message
  printf("\r\n====================================\r\n");
  printf(" WHERESAT STAR TRACKER - INITIALIZED\r\n");
  printf(" System Clock: %lu MHz\r\n", (unsigned long)(HAL_RCC_GetHCLKFreq() / 1000000));
  printf(" UART2: 115200 Baud - OK\r\n");
  printf(" SPI1: Master Mode - OK\r\n");
  printf("====================================\r\n");
  test_camera_geometry();

   // Initialize the Star Catalog
  if (catalog_init() == false) {
      printf("CRITICAL ERROR: Database failed to load!\r\n");
  } else {
      printf("Database Loaded: %lu Stars, %lu Triangles\r\n", 
             catalog_get_num_stars(), catalog_get_num_triangles());
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    FPGA_Packet_t current_packet;
    ObservedStar live_stars[MAX_CENTROIDS];
    ObservedTriangle triangles[MAX_OBSERVED_TRIANGLES];
    MatchedStar final_matches[MAX_CENTROIDS];

    while (1)
    {
        // --- OPTION B: MOCK DATA (Using the 5 stars from your Python script) ---
        load_test_centroids(&current_packet);
        HAL_StatusTypeDef status = HAL_OK;

        if (status == HAL_OK) {
            if (fpga_validate_packet(&current_packet)) {
                
                printf("\r\n--- PROCESSING NEW FRAME (%d Stars) ---\r\n", current_packet.count);

                // 1. Convert pixels to 3D Unit Vectors
                for (int i = 0; i < current_packet.count; i++) {
                    Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
                    live_stars[i].local_id = i;
                    live_stars[i].x = vec.x;
                    live_stars[i].y = vec.y;
                    live_stars[i].z = vec.z;
                }

                // 2. Build Triangles (YOUR Week 6 Code!)
                uint16_t num_triangles = 0;
                build_triangles(live_stars, current_packet.count, triangles, &num_triangles);
                printf("Built %d triangles.\r\n", num_triangles);

                // 3. Search Database and Vote (YOUR Week 6 Code!)
                match_stars(triangles, num_triangles, current_packet.count, final_matches);

                // 4. Print the results!
                for (int i = 0; i < current_packet.count; i++) {
                    if (final_matches[i].is_matched) {
                        printf("Star %d -> MATCHED HIP ID: %lu (Votes: %d)\r\n", 
                               i, final_matches[i].hip_id, final_matches[i].vote_count);
                    } else {
                        printf("Star %d -> No match found.\r\n", i);
                    }
                }
            } else {
                printf("Packet Validation Failed!\r\n");
            }
        }

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Blink LED
        HAL_Delay(5000); // 5 second delay so you can read the terminal easily
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
