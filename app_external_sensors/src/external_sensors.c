// -- Debug Module ---

#define DEBUG_MODULE "EXTERNAL_SENSORS"

// --- Includes ---

#include <stdint.h>
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "stm32f4xx.h"
#include "log.h"
#include "i2cdev.h"
#include "bmi270_config_file.h"

// --- BMI270 Registers ---
#define BMI270_I2C_ADDR        0x68
#define BMI270_REG_CHIP_ID     0x00
#define BMI270_CHIP_ID_VAL     0x24
#define BMI270_REG_PWR_CONF    0x7C
#define BMI270_REG_PWR_CTRL    0x7D
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_ACC_CONF    0x40
#define BMI270_REG_ACC_DATA    0x0C
#define BMI270_REG_INIT_CTRL   0x59
#define BMI270_REG_INIT_DATA   0x5E

// --- PCA9546 I2C Multiplexer ---
#define PCA9546_ADDR 0x70

// --- DATA ---
static volatile uint16_t micValue;
static uint8_t currentChannel = 0;
int16_t ax0, ay0, az0;
int16_t ax1, ay1, az1;
int16_t ax2, ay2, az2;
int16_t ax3, ay3, az3;

// --- LOGGING ---
LOG_GROUP_START(microphone)
LOG_ADD(LOG_UINT16, adcValue, &micValue)
LOG_GROUP_STOP(microphone)

LOG_GROUP_START(accel0)
LOG_ADD(LOG_INT16, x, &ax0)
LOG_ADD(LOG_INT16, y, &ay0)
LOG_ADD(LOG_INT16, z, &az0)
LOG_GROUP_STOP(accel0)

LOG_GROUP_START(accel1)
LOG_ADD(LOG_INT16, x, &ax1)
LOG_ADD(LOG_INT16, y, &ay1)
LOG_ADD(LOG_INT16, z, &az1)
LOG_GROUP_STOP(accel1)

LOG_GROUP_START(accel2)
LOG_ADD(LOG_INT16, x, &ax2)
LOG_ADD(LOG_INT16, y, &ay2)
LOG_ADD(LOG_INT16, z, &az2)
LOG_GROUP_STOP(accel2)

LOG_GROUP_START(accel3)
LOG_ADD(LOG_INT16, x, &ax3)
LOG_ADD(LOG_INT16, y, &ay3)
LOG_ADD(LOG_INT16, z, &az3)
LOG_GROUP_STOP(accel3)

//---------------------------------------------------
// PCA9546 Multiplexer
//---------------------------------------------------

static void pca9546_select(uint8_t ch)
{
    uint8_t mask = 1 << ch;
    i2cdevWrite(I2C1_DEV, PCA9546_ADDR, 1, &mask);
}

//---------------------------------------------------
// Accelerometer
//---------------------------------------------------

static void bmi270Init(void) 
{

    // Check communication
    uint8_t id = 0;
    i2cdevReadReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_CHIP_ID, 1, &id);
    if (id != BMI270_CHIP_ID_VAL) {
        DEBUG_PRINT("BMI270 not found! ID = 0x%02X\n", id);
        return;
    }

    // Disable PWR_CONF
    uint8_t pwr_conf = 0x00; 
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_PWR_CONF, 1, &pwr_conf);

    // Wait for at least 450us
    vTaskDelay(M2T(5));

    // Prepare config load
    uint8_t ctrl = 0x00;
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_INIT_CTRL, 1, &ctrl);

    // Loading config file and save as array
    extern const uint8_t bmi270_config_file[];
    const uint32_t config_size = sizeof(bmi270_config_file);

    // Burst write to INIT_DATA
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_INIT_DATA, config_size, (uint8_t*)bmi270_config_file);

    // Complete config load
    ctrl = 0x01;
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_INIT_CTRL, 1, &ctrl);

    // Wait at least 20ms
    vTaskDelay(M2T(200));

    // Check internal status
    uint8_t status = 0;
    for (int i = 0; i < 200; i++) { 
        i2cdevReadReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_INTERNAL_STATUS, 1, &status);
        if ((status & 0x01) == 0x01) break;
        vTaskDelay(M2T(1));
    }

    // Enable acquisition of accelerometer data
    uint8_t pwr = 0x0E;
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_PWR_CTRL, 1, &pwr);

    // Enable ACC_FILTER
    uint8_t acc_conf = 0xA8;
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_ACC_CONF, 1, &acc_conf);

    // Disable adv_power_save
    uint8_t cmd_normal_mode = 0x02;
    i2cdevWriteReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_PWR_CONF, 1, &cmd_normal_mode);
}

static bool bmi270ReadAccel(void)
{
    // Read Data
    uint8_t data[6];
    if (!i2cdevReadReg8(I2C1_DEV, BMI270_I2C_ADDR, BMI270_REG_ACC_DATA, 6, data))
        return false;

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);

    // Assign Data To Specific Accelerometers
    switch (currentChannel) {
        case 0:
            ax0 = x; ay0 = y; az0 = z;
            // DEBUG_PRINT("Channel 0 - Accel X: %d, Y: %d, Z: %d\n", ax0, ay0, az0);
            break;
        case 1:
            ax1 = x; ay1 = y; az1 = z;
            // DEBUG_PRINT("Channel 1 - Accel X: %d, Y: %d, Z: %d\n", ax1, ay1, az1);
            break;
        case 2:
            ax2 = x; ay2 = y; az2 = z;
            // DEBUG_PRINT("Channel 2 - Accel X: %d, Y: %d, Z: %d\n", ax2, ay2, az2);
            break;
        case 3:
            ax3 = x; ay3 = y; az3 = z;
            // DEBUG_PRINT("Channel 3 - Accel X: %d, Y: %d, Z: %d\n", ax3, ay3, az3);
            break;
        default:
            DEBUG_PRINT("Invalid channel %d!\n", currentChannel);
            return false;
    }

    return true;
}

//---------------------------------------------------
// Microphone
//---------------------------------------------------

static void micAdcInit(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // Enable ADC1 clock

    GPIOA->MODER |= (3 << (2 * 2));        // Set PA2 to analog mode
    GPIOA->PUPDR &= ~(3 << (2 * 2));       // No pull-up/pull-down

    ADC1->SQR3 = 2;                        // Set ADC channel
    ADC1->SMPR2 |= (3 << (3 * 2));         // Sample time: 56 cycles
    ADC1->CR2 |= ADC_CR2_ADON;             // Enable ADC
}

static uint16_t readMicrophone(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;         // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));     // Wait until conversion complete
    return ADC1->DR;                      // Return ADC result
}

//---------------------------------------------------
// Main
//---------------------------------------------------

void appMain(void)
{
    // Microphone Initialization
    micAdcInit();
    vTaskDelay(M2T(200));

    // Accelerometer Initialization On Each Channel
    for (uint8_t ch = 0; ch < 4; ch++) {
        pca9546_select(ch);
        vTaskDelay(M2T(200));
        bmi270Init();
    }

    while (1) {
        vTaskDelay(M2T(100));

        // Accelerometer Reading
        for (currentChannel = 0; currentChannel < 4; currentChannel++) {
            pca9546_select(currentChannel);
            vTaskDelay(M2T(1));
            bmi270ReadAccel();
        }

        // Microphone Reading
        micValue = readMicrophone();
        // DEBUG_PRINT("Microphone ADC value: %d\n", micValue);
    }
}
