/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "bmi270.h"
#include "spi.h"
#include "gpio.h"
#include "led.h"
#include "cli.h"

#ifdef USE_ACCGYRO_BMI270

// 10 MHz max SPI frequency
#define BMI270_MAX_SPI_CLK_HZ 10000000

#define BMI270_FIFO_FRAME_SIZE 6

#define BMI270_CONFIG_SIZE 328

// Declaration for the device config (microcode) that must be uploaded to the sensor
extern const uint8_t bmi270_maximum_fifo_config_file[BMI270_CONFIG_SIZE];

#define BMI270_CHIP_ID 0x24

// BMI270 registers (not the complete list)
typedef enum {
    BMI270_REG_CHIP_ID = 0x00,
    BMI270_REG_ERR_REG = 0x02,
    BMI270_REG_STATUS = 0x03,
    BMI270_REG_ACC_DATA_X_LSB = 0x0C,
    BMI270_REG_GYR_DATA_X_LSB = 0x12,
    BMI270_REG_SENSORTIME_0 = 0x18,
    BMI270_REG_SENSORTIME_1 = 0x19,
    BMI270_REG_SENSORTIME_2 = 0x1A,
    BMI270_REG_EVENT = 0x1B,
    BMI270_REG_INT_STATUS_0 = 0x1C,
    BMI270_REG_INT_STATUS_1 = 0x1D,
    BMI270_REG_INTERNAL_STATUS = 0x21,
    BMI270_REG_TEMPERATURE_LSB = 0x22,
    BMI270_REG_TEMPERATURE_MSB = 0x23,
    BMI270_REG_FIFO_LENGTH_LSB = 0x24,
    BMI270_REG_FIFO_LENGTH_MSB = 0x25,
    BMI270_REG_FIFO_DATA = 0x26,
    BMI270_REG_ACC_CONF = 0x40,
    BMI270_REG_ACC_RANGE = 0x41,
    BMI270_REG_GYRO_CONF = 0x42,
    BMI270_REG_GYRO_RANGE = 0x43,
    BMI270_REG_AUX_CONF = 0x44,
    BMI270_REG_FIFO_DOWNS = 0x45,
    BMI270_REG_FIFO_WTM_0 = 0x46,
    BMI270_REG_FIFO_WTM_1 = 0x47,
    BMI270_REG_FIFO_CONFIG_0 = 0x48,
    BMI270_REG_FIFO_CONFIG_1 = 0x49,
    BMI270_REG_SATURATION = 0x4A,
    BMI270_REG_INT1_IO_CTRL = 0x53,
    BMI270_REG_INT2_IO_CTRL = 0x54,
    BMI270_REG_INT_LATCH = 0x55,
    BMI270_REG_INT1_MAP_FEAT = 0x56,
    BMI270_REG_INT2_MAP_FEAT = 0x57,
    BMI270_REG_INT_MAP_DATA = 0x58,
    BMI270_REG_INIT_CTRL = 0x59,
    BMI270_REG_INIT_DATA = 0x5E,
    BMI270_REG_ACC_SELF_TEST = 0x6D,
    BMI270_REG_GYR_SELF_TEST_AXES = 0x6E,
    BMI270_REG_PWR_CONF = 0x7C,
    BMI270_REG_PWR_CTRL = 0x7D,
    BMI270_REG_CMD = 0x7E,
} bmi270Register_e;

// BMI270 register configuration values
typedef enum {
    BMI270_VAL_CMD_SOFTRESET = 0xB6,
    BMI270_VAL_CMD_FIFOFLUSH = 0xB0,
    BMI270_VAL_PWR_CTRL = 0x0E,              // enable gyro, acc and temp sensors
    BMI270_VAL_PWR_CONF = 0x02,              // disable advanced power save, enable FIFO self-wake
    BMI270_VAL_ACC_CONF_ODR800 = 0x0B,       // set acc sample rate to 800hz
    BMI270_VAL_ACC_CONF_ODR1600 = 0x0C,      // set acc sample rate to 1600hz
    BMI270_VAL_ACC_CONF_BWP = 0x02,          // set acc filter in normal mode
    BMI270_VAL_ACC_CONF_HP = 0x01,           // set acc in high performance mode
    BMI270_VAL_ACC_RANGE_8G = 0x02,          // set acc to 8G full scale
    BMI270_VAL_ACC_RANGE_16G = 0x03,         // set acc to 16G full scale
    BMI270_VAL_GYRO_CONF_ODR3200 = 0x0D,     // set gyro sample rate to 3200hz
    BMI270_VAL_GYRO_CONF_BWP_OSR4 = 0x00,    // set gyro filter in OSR4 mode
    BMI270_VAL_GYRO_CONF_BWP_OSR2 = 0x01,    // set gyro filter in OSR2 mode
    BMI270_VAL_GYRO_CONF_BWP_NORM = 0x02,    // set gyro filter in normal mode
    BMI270_VAL_GYRO_CONF_NOISE_PERF = 0x01,  // set gyro in high performance noise mode
    BMI270_VAL_GYRO_CONF_FILTER_PERF = 0x01, // set gyro in high performance filter mode

    BMI270_VAL_GYRO_RANGE_2000DPS = 0x08,    // set gyro to 2000dps full scale
                                             // for some reason you have to enable the ois_range bit (bit 3) for 2000dps as well
                                             // or else the gyro scale will be 250dps when in prefiltered FIFO mode (not documented in datasheet!)

    BMI270_VAL_INT_MAP_DATA_DRDY_INT1 = 0x04,// enable the data ready interrupt pin 1
    BMI270_VAL_INT_MAP_FIFO_WM_INT1 = 0x02,  // enable the FIFO watermark interrupt pin 1
    BMI270_VAL_INT1_IO_CTRL_PINMODE = 0x0A,  // active high, push-pull, output enabled, input disabled 
    BMI270_VAL_FIFO_CONFIG_0 = 0x00,         // don't stop when full, disable sensortime frame
    BMI270_VAL_FIFO_CONFIG_1 = 0x80,         // only gyro data in FIFO, use headerless mode
    BMI270_VAL_FIFO_DOWNS = 0x00,            // select unfiltered gyro data with no downsampling (6.4KHz samples)
    BMI270_VAL_FIFO_WTM_0 = 0x06,            // set the FIFO watermark level to 1 gyro sample (6 bytes)
    BMI270_VAL_FIFO_WTM_1 = 0x00,            // FIFO watermark MSB
} bmi270ConfigValues_e;

#define _PIN_DEF_CS 0
// Need to see at least this many interrupts during initialisation to confirm EXTI connectivity
#define GYRO_EXTI_DETECT_THRESHOLD 1000
static bool bmi270_Driver_Init(void);
static uint8_t spi_ch = _DEF_SPI1;
static uint8_t _buffer[21];

#ifdef _USE_HW_CLI
static void cliBmi270(cli_args_t *args);
#endif

bool bmi270_Init(sensor_Dev_t *p_driver)
{
    bool ret = true;
    bmi270_Driver_Init();
    p_driver->initFn = NULL;
    p_driver->gyro_readFn = bmi270SpiGyroRead;
    p_driver->acc_readFn = bmi270SpiAccRead;
    p_driver->setCallBack = NULL;
    p_driver->scale = GYRO_SCALE_2000DPS;
    #ifdef _USE_HW_CLI
     cliAdd("bmi270", cliBmi270);
    #endif

    return ret;
}

bool bmi270_Driver_Init(void)
{
    spiBegin(spi_ch);
    spiSetDataMode(spi_ch, SPI_MODE0);

    gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
    SPI_ByteWriteRead(_DEF_SPI1, BMI270_REG_CHIP_ID | 0x80, _buffer, 2);
    gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);
    if (_buffer[1] == BMI270_CHIP_ID)
    {
        return true;
    }
    return false;
}

bool bmi270SpiAccRead(void)
{
    return true;
}
bool bmi270SpiGyroRead(void)
{
    return true;
}

static void (*frameCallBack)(void) = NULL;

bool bmi270SetCallBack(void (*p_func)(void))
{
  frameCallBack = p_func;

  return true;
}

#ifdef _USE_HW_CLI
void cliBmi270(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 3 && args->isStr(0, "mem_read") == true)
  {
    uint8_t ch;
    uint8_t addr;
    uint8_t buffer[2] = {0, 0};
    HAL_StatusTypeDef status;

    ch   = (uint8_t)args->getData(1);
    addr = (uint8_t)args->getData(2);
    addr |= 0x80;

    gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
    status = SPI_ByteWriteRead(ch, addr, buffer, 2);
    gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);

    if(status == HAL_OK)
    {
        cliPrintf("bmi270 mem_read : ch (%d), addr (0x%X), data[0] : (0x%X), data[1] : (0x%X), status (%d)\n", ch, addr, buffer[0], buffer[1], status);
    }else
    {
        cliPrintf("bmi270 read - Fail(%d) \n", status);
    }
    ret = true;
  }
  if (ret != true)
  {
    cliPrintf("bmi270 mem_read ch0:1, addr \n");
  }
}
#endif



#endif // USE_ACCGYRO_BMI270
