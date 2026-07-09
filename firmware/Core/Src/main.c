/**
 * @file main.c
 * @brief Final ADCS Pipeline Integration for WhereSat.
 *
 * Pipeline Flow:
 * 1. Host Ingest (Blocking UART) - Receives Centroids + Gyro
 * 2. Camera Geometry (Pixel -> Unit Vector)
 * 3. Star Identification (Triangle Voting)
 * 4. QUEST (Attitude Determination)
 * 5. MEKF (Sensor Fusion & Bias Estimation)
 * 6. Controller (PD Torque Calculation)
 * 7. Telemetry (Binary State Export) - Sends Q, Omega, Torque, Status
 *
 * @author Aditya (WhereSat Team)
 */

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
#include "mekf.h"
#include "controller.h"
#include "test_suite.h"

/* Private Constants ---------------------------------------------------------*/
#define HIL_DT                  (0.1f)      // 10Hz Simulation Timestep
#define STAR_TRACKER_VARIANCE   (1.0e-3f)   // MEKF R-matrix diagonal
#define IDENTITY_QUATERNION     {0.0f, 0.0f, 0.0f, 1.0f}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

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
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();

    /* 1. Initialize Star Catalog */
    if (catalog_init() == false) {
        printf("CRITICAL ERROR: Catalog Load Failed\r\n");
        Error_Handler();
    }

    /* 2. Initialize ADCS State Modules */
    MEKF_t filter;
    mekf_init(&filter, (Quaternion_t)IDENTITY_QUATERNION);

    PD_Controller_t adcs_controller = {
        .Kp = 0.5f,
        .Kd = 0.1f
    };

    /* 3. Static Simulation Parameters */
    const float dt = HIL_DT;
    const Quaternion_t target_q = IDENTITY_QUATERNION;

    /* 4. Pipeline Buffers */
    FPGA_Packet_t current_packet;
    Vector3_t gyro_data;
    ObservedStar live_stars[MAX_CENTROIDS];
    ObservedTriangle triangles[MAX_OBSERVED_TRIANGLES];
    MatchedStar final_matches[MAX_CENTROIDS];
    QUEST_Input_t quest_data;
    Quaternion_t estimated_q = IDENTITY_QUATERNION;

    printf("\r\n====================================\r\n");
    printf(" WHERESAT ADCS ENGINE - ONLINE\r\n");
    printf(" Mode: HIL Slave (Python Master)\r\n");
    printf(" Convention: Scalar-Last [x, y, z, w]\r\n");
    printf("====================================\r\n");

    /* Infinite loop */
    while (1)
    {
        // --- STEP 1: INGEST ---
        // Blocks until Python HIL master sends a binary centroid + gyro packet
        if (host_receive_packet(&current_packet, &gyro_data) == HAL_OK)
        {
            // Defensive check on star count
            if (current_packet.count > MAX_CENTROIDS) {
                continue;
            }

            // Visual Profiling: LED ON indicates processing window
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

            // --- STEP 2: GEOMETRY ---
            for (int i = 0; i < current_packet.count; i++) {
                Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
                live_stars[i].local_id = i;
                live_stars[i].x = vec.x;
                live_stars[i].y = vec.y;
                live_stars[i].z = vec.z;
            }

            // --- STEP 3: STAR IDENTIFICATION ---
            uint16_t num_triangles = 0;
            build_triangles(live_stars, current_packet.count, triangles, &num_triangles);
            match_stars(triangles, num_triangles, current_packet.count, final_matches);

            // --- STEP 4: QUEST PREPARATION ---
            quest_data.count = 0;
            for (int i = 0; i < current_packet.count; i++) {
                if (final_matches[i].is_matched) {
                    quest_data.body_v[quest_data.count] = (Vector3_t){live_stars[i].x, live_stars[i].y, live_stars[i].z};
                    catalog_get_star_vector(final_matches[i].hip_id, &quest_data.reference_v[quest_data.count]);
                    quest_data.weights[quest_data.count] = 1.0f;
                    quest_data.count++;
                }
            }

            // --- STEP 5: ATTITUDE DETERMINATION ---
            bool adcs_locked = (quest_data.count >= 2);
            if (adcs_locked) {
                estimated_q = quest_compute(&quest_data);
            }

            // --- STEP 6: SENSOR FUSION (MEKF) ---
            // Predict forward using raw gyro rates (Dead Reckoning)
            mekf_predict(&filter, gyro_data, dt);

            // Correct using Star Tracker if a lock is achieved
            if (adcs_locked) {
                mekf_update(&filter, estimated_q, STAR_TRACKER_VARIANCE);
            }

            // --- STEP 7: ATTITUDE CONTROL ---
            // Use bias-corrected angular velocity for the D-term
            Vector3_t omega_corr = {
                gyro_data.x - filter.beta[0],
                gyro_data.y - filter.beta[1],
                gyro_data.z - filter.beta[2]
            };

            Vector3_t torque_cmd = controller_compute_torque(&adcs_controller, filter.q, target_q, omega_corr);

            // --- STEP 8: TELEMETRY ---
            // Export full state back to Python for logging and visualization
            telemetry_send(&filter.q, &omega_corr, &torque_cmd, adcs_locked, (uint8_t)quest_data.count);

            // Visual Profiling: LED OFF indicates processing complete
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = HAL_RCC_GetPCLK1Freq() > 45000000 ? RCC_HCLK_DIV4 : RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) { Error_Handler(); }
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
