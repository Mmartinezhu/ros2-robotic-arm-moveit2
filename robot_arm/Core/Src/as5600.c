
#include "as5600.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"


AS5600_TypeDef as5600_instance;

// Inicialización sin retorno de estado
void AS5600_Init(AS5600_TypeDef *dev)
{
    // Ejemplo de configuración por defecto si el usuario no la estableció
    if (!dev->PositiveRotationDirection) {
        dev->PositiveRotationDirection = AS5600_DIR_CW;
    }
    if (!dev->LowPowerMode) {
        dev->LowPowerMode = AS5600_POWER_MODE_DEFAULT;
    }
    if (!dev->Hysteresis) {
        dev->Hysteresis = AS5600_HYSTERESIS_DEFAULT;
    }
    if (!dev->OutputMode) {
        dev->OutputMode = AS5600_OUTPUT_STAGE_DEFAULT;
    }
    if (!dev->PWMFrequency) {
        dev->PWMFrequency = AS5600_PWM_FREQUENCY_DEFAULT;
    }
    if (!dev->SlowFilter) {
        dev->SlowFilter = AS5600_SLOW_FILTER_DEFAULT;
    }
    if (!dev->FastFilterThreshold) {
        dev->FastFilterThreshold = AS5600_FAST_FILTER_DEFAULT;
    }
    if (!dev->WatchdogTimer) {
        dev->WatchdogTimer = AS5600_WATCHDOG_DEFAULT;
    }

    // Escribir al registro de configuración
    uint8_t txData[3];
    txData[0] = AS5600_REGISTER_CONF_HIGH;
    txData[1] = dev->confRegister[0];
    txData[2] = dev->confRegister[1];

    HAL_StatusTypeDef  status = HAL_I2C_Master_Transmit(
        dev->i2cHandle,
        dev->i2cAddr,  // dirección 8 bits (ej. (0x36 << 1))
        txData,
        3,
        HAL_MAX_DELAY
    );


}




// Función para obtener el ángulo
void AS5600_GetAngle(AS5600_TypeDef *dev, float *angle)
{
    uint8_t data[2] = {0};
    if (HAL_I2C_Mem_Read(dev->i2cHandle, dev->i2cAddr,AS5600_REGISTER_ANGLE_HIGH,I2C_MEMADD_SIZE_8BIT,data, 2, HAL_MAX_DELAY) == HAL_OK)
    {
        *angle = ((data[0] << 8) | data[1]) & 0x0FFF;  // Se aplica la máscara de 12 bits
        *angle = ((float) *angle / 4095.0f) * 360.0f;
    }
    else {
        *angle = 0;
    }
}




// Función para configurar el modo de salida
void AS5600_SetOutputMode(AS5600_TypeDef *dev, uint8_t mode, uint8_t freq)
{
    uint8_t pwm = 0;

    switch (mode) {
        case AS5600_OUTPUT_STAGE_FULL:
            dev->confRegister[1] &= ~((1UL << 5) | (1UL << 4));
            break;
        case AS5600_OUTPUT_STAGE_REDUCED:
            dev->confRegister[1] |= (1UL << 4);
            dev->confRegister[1] &= ~(1UL << 5);
            break;
        case AS5600_OUTPUT_STAGE_PWM:
            dev->confRegister[1] &= ~(1UL << 4);
            dev->confRegister[1] |= (1UL << 5);
            pwm = 1;
            break;
        default:
            break;
    }

    if (pwm) {
        switch (freq) {
            case AS5600_PWM_FREQUENCY_115HZ:
                dev->confRegister[1] &= ~((1UL << 7) | (1UL << 6));
                break;
            case AS5600_PWM_FREQUENCY_230HZ:
                dev->confRegister[1] |= (1UL << 6);
                dev->confRegister[1] &= ~(1UL << 7);
                break;
            case AS5600_PWM_FREQUENCY_460HZ:
                dev->confRegister[1] &= ~(1UL << 6);
                dev->confRegister[1] |= (1UL << 7);
                break;
            case AS5600_PWM_FREQUENCY_920HZ:
                dev->confRegister[1] |= ((1UL << 7) | (1UL << 6));
                break;
            default:
                break;
        }
    }
    HAL_I2C_Mem_Write_IT(dev->i2cHandle, dev->i2cAddr,
                         AS5600_REGISTER_CONF_HIGH,
                         I2C_MEMADD_SIZE_8BIT,
                         dev->confRegister, 2);
}
// Función para obtener el estado del imán

void AS5600_GetMagnetStatus(AS5600_TypeDef *dev, uint8_t *stat)
{
    HAL_I2C_Mem_Read(dev->i2cHandle, dev->i2cAddr,AS5600_REGISTER_STATUS,I2C_MEMADD_SIZE_8BIT,stat, 1, HAL_MAX_DELAY);
}









