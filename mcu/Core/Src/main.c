/**
 * @file main.c
 * @brief Final ADCS Pipeline - Robust HIL Mode with NaN Guards, Non-Blocking Trace, Frame-Drop Recovery, and Jackknife Fallback (3-Axis MEKF).
 * @author Aditya (WhereSat Team)
 */

#include "main.h"
#include "gpio.h"
#include "usart.h"
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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "core_cm4.h"

/* External UART handle for error recovery */
extern UART_HandleTypeDef huart2;

/* Private Constants ---------------------------------------------------------*/
#define HIL_DT                  0.1f
#define STAR_TRACKER_VARIANCE   (1.0e-3f)
#define IDENTITY_QUATERNION     {0.0f, 0.0f, 0.0f, 1.0f}
#define MEKF_LOCK_THRESHOLD     0.001f

/* Reject QUEST outputs that diverge from MEKF by more than ~16 degrees */
#define QUEST_INNOVATION_THRESHOLD 0.99f 
/* Let the MEKF initialize for the first 10 frames before gating */
#define WARMUP_FRAMES              10

/* TARGET: 45-degree rotation around the Y-axis [x, y, z, w] */
#define TARGET_QUATERNION       {-0.805684f, -0.290840f, -0.075647f, -0.510453f}

#define MAX_STARS_INTERNAL      12
#define MAX_TRIANGLES_INTERNAL  260 

/* Robustness Defines */
#define QUEST_MIN_STARS         3

/* Static Pipeline Buffers (Prevent Stack Overflow) --------------------------*/
static FPGA_Packet_t current_packet;
static Vector3_t gyro_data;
/* Renamed to observed_stars to match star_matcher.c external references */
static ObservedStar observed_stars[MAX_STARS_INTERNAL]; 
static ObservedTriangle triangles[MAX_TRIANGLES_INTERNAL];
static MatchedStar final_matches[MAX_STARS_INTERNAL];
static QUEST_Input_t quest_data;

/* Per-frame Star Blacklist */
static bool star_blacklisted[MAX_STARS_INTERNAL];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); 
void Error_Handler(void);
bool quat_is_valid(Quaternion_t q);
float quat_dot(Quaternion_t q1, Quaternion_t q2);
void build_quest_input(void);

/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && ((ITM->TER & 1UL) != 0UL)) {
        uint32_t timeout = 0xFFFF;
        while (ITM->PORT[0].u32 == 0UL && timeout--);
        if (timeout != 0) ITM->PORT[0].u8 = (uint8_t)ch;
    }
    return ch;
}

bool quat_is_valid(Quaternion_t q) {
    if (isnan(q.x) || isnan(q.y) || isnan(q.z) || isnan(q.w)) return false;
    if (isinf(q.x) || isinf(q.y) || isinf(q.z) || isinf(q.w)) return false;
    if (fabsf(q.x) < 1e-6f && fabsf(q.y) < 1e-6f && fabsf(q.z) < 1e-6f && fabsf(q.w) < 1e-6f) return false;
    return true;
}

float quat_dot(Quaternion_t q1, Quaternion_t q2) {
    return fabsf(q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w);
}

void sort_centroids_by_mass(FPGA_Packet_t *pkt) {
    for (int i = 0; i < pkt->count - 1; i++) {
        int max_idx = i;
        for (int j = i + 1; j < pkt->count; j++) {
            if (pkt->centroids[j].mass > pkt->centroids[max_idx].mass) max_idx = j;
        }
        Centroid_t temp = pkt->centroids[i];
        pkt->centroids[i] = pkt->centroids[max_idx];
        pkt->centroids[max_idx] = temp;
    }
}

/**
 * @brief Helper to build QUEST input array while respecting the star blacklist.
 */
void build_quest_input(void) {
    quest_data.count = 0;
    for (int i = 0; i < current_packet.count; i++) {
        if (final_matches[i].is_matched && !star_blacklisted[i]) {
            quest_data.body_v[quest_data.count] = (Vector3_t){observed_stars[i].x, observed_stars[i].y, observed_stars[i].z};
            catalog_get_star_vector(final_matches[i].hip_id, &quest_data.reference_v[quest_data.count]);
            quest_data.weights[quest_data.count] = 1.0f;
            quest_data.count++;
        }
    }
}
/* USER CODE END 0 */

int main(void) {
    HAL_Init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR = 0xC5ACCE55;
    ITM->TCR = ITM_TCR_ITMENA_Msk;
    ITM->TER = 1;

    SystemClock_Config();
    MX_GPIO_Init(); 
    MX_USART2_UART_Init();

    if (catalog_init() == false) Error_Handler();
    printf("Step 0: Catalog OK\r\n");
    
    MEKF_t filter;
    mekf_init(&filter, (Quaternion_t)IDENTITY_QUATERNION);
    printf("Step 0: MEKF OK\r\n");

    PD_Controller_t adcs_controller = {.Kp = 0.05f, .Kd = 0.1f};
    Quaternion_t target_q = TARGET_QUATERNION;
    Quaternion_t estimated_q = IDENTITY_QUATERNION;
    
    static uint32_t frame_count = 0;
    uint32_t missed_frames = 0;

    while (1) {
        HAL_StatusTypeDef rx_status = host_receive_packet(&current_packet, &gyro_data);

        if (rx_status != HAL_OK) {
            missed_frames++;
            printf("\r\n[WARNING] Frame dropped. Missed count: %lu\r\n", (unsigned long)missed_frames);
            
            if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
                __HAL_UART_CLEAR_OREFLAG(&huart2);
                huart2.ErrorCode = HAL_UART_ERROR_NONE;
                printf("[RECOVERY] Cleared UART Overrun Error.\r\n");
            }
            continue; 
        }

        frame_count++;
        float current_dt = HIL_DT + (missed_frames * HIL_DT);
        missed_frames = 0; 

        printf("\r\nStep 1: Ingest OK (dt = %.2fs)\r\n", current_dt);

        /* Reset per-frame star blacklist */
        memset(star_blacklisted, 0, sizeof(star_blacklisted));

        sort_centroids_by_mass(&current_packet);
        if (current_packet.count > MAX_STARS_INTERNAL) current_packet.count = MAX_STARS_INTERNAL;

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

        for (int i = 0; i < current_packet.count; i++) {
            Vector3_t vec = pixel_to_vector(current_packet.centroids[i]);
            observed_stars[i] = (ObservedStar){.local_id = i, .x = vec.x, .y = vec.y, .z = vec.z};
        }
        
        uint16_t num_triangles = 0;
        build_triangles(observed_stars, current_packet.count, triangles, &num_triangles);
        
        /* Initial Matching - match_stars now handles its own verbose logging internally */
        
        match_stars(observed_stars, triangles, num_triangles, current_packet.count, final_matches);

        bool locked = false;
        uint8_t final_star_count = 0;
        float innovation_dot = 0.0f; 

        /* Step 5: Robust QUEST with Jackknife Fallback */
        build_quest_input();

        if (quest_data.count >= QUEST_MIN_STARS) {
            estimated_q = quest_compute(&quest_data);
            
            /* Guard: Check validity BEFORE math */
            if (quat_is_valid(estimated_q)) {
                float dot_global = quat_dot(filter.q, estimated_q);
                innovation_dot = dot_global;

                if (frame_count <= WARMUP_FRAMES || dot_global >= QUEST_INNOVATION_THRESHOLD) {
                    locked = true;
                    final_star_count = quest_data.count;
                    printf("Step 5: QUEST OK (stars=%d, dot=%.3f)\r\n", quest_data.count, dot_global);
                } else {
                    /* Innovation too high - Start Jackknife */
                    printf("Step 5: Innovation High (dot=%.3f). Starting Jackknife...\r\n", dot_global);
                    
                    for (int i = 0; i < current_packet.count; i++) {
                        if (!final_matches[i].is_matched) continue;

                        star_blacklisted[i] = true;
                        build_quest_input();

                        if (quest_data.count >= QUEST_MIN_STARS) {
                            Quaternion_t q_sub = quest_compute(&quest_data);
                            
                            if (quat_is_valid(q_sub)) {
                                float dot_sub = quat_dot(filter.q, q_sub);
                                if (dot_sub >= QUEST_INNOVATION_THRESHOLD) {
                                    printf("Step 5: Jackknife Success (Dropped Star %d, dot=%.3f)\r\n", i, dot_sub);
                                    estimated_q = q_sub;
                                    innovation_dot = dot_sub;
                                    final_star_count = quest_data.count;
                                    locked = true;
                                    break; 
                                }
                            }
                        }
                        star_blacklisted[i] = false; // Restore
                    }
                }
            } else {
                printf("Step 5: QUEST Output NaN/Invalid.\r\n");
            }
        }

        /* Step 6: MEKF Update and Control */
        mekf_predict(&filter, gyro_data, current_dt);
        
        if (locked) {
            mekf_update(&filter, estimated_q, STAR_TRACKER_VARIANCE);
        } else {
            printf("Step 5: FALLBACK - Prediction only frame.\r\n");
        }

        /* Bias has been removed; pass raw gyro directly to controller */
        Vector3_t omega_corr = gyro_data; 
        Vector3_t torque = controller_compute_torque(&adcs_controller, filter.q, target_q, omega_corr);
        
        /* Updated telemetry function call to reflect bias removal */
        telemetry_send(&filter.q, &omega_corr, &torque, &estimated_q, &gyro_data, innovation_dot, locked, final_star_count);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
}

void SystemClock_Config(void) {
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
    RCC_ClkInitStruct.APB1CLKDivider = HAL_RCC_GetPCLK1Freq() > 45000000 ? RCC_HCLK_DIV4 : RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void) {
    __disable_irq();
    while (1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); HAL_Delay(100); }
}
