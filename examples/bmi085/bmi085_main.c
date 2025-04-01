/****************************************************************************
 * apps/examples/bmi085/bmi085_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <inttypes.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <nuttx/clock.h>
#include <nuttx/signal.h>

#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/sensors/bmi085.h>

/****************************************************************************
* Pre-processor Definitions
****************************************************************************/

#define ACC_DEVPATH      "/dev/accel0"
#define I2C_DRIVER_PATH  "/dev/i2c0"

#define BMI085_ACCEL_ADDR  0x18  // Accelerometer I2C address
#define BMI085_GYRO_ADDR   0x68  // Gyroscope I2C address
#define BMI085_CHIP_ID_REG 0x00  // Register to read the chip ID

#define ACCEL_CHIP_ID           0x1F
#define ACCEL_RESET_CMD         0xB6
#define ACCEL_ENABLE_CMD        0x04
#define ACCEL_ACTIVE_MODE_CMD   0x00
#define ACCEL_DISABLE_CMD       0x00

#define ACCEL_CHIP_ID_ADDR          0x00
#define ACCEL_CHIP_ID_MASK          0xFF
#define ACCEL_CHIP_ID_POS           0

#define ACCEL_POS_SELF_TEST     0x0D
#define ACCEL_NEG_SELF_TEST     0x09
#define ACCEL_DIS_SELF_TEST     0x00

#define ACCEL_SOFT_RESET_ADDR       0x7E
#define ACCEL_SOFT_RESET_MASK       0xFF
#define ACCEL_SOFT_RESET_POS        0

#define ACCEL_SELF_TEST_ADDR        0x6D
#define ACCEL_SELF_TEST_MASK        0xFF
#define ACCEL_SELF_TEST_POS         0

#define ACCEL_PWR_CONF_ADDR         0x7C
#define ACCEL_PWR_CONF_MASK         0xFF
#define ACCEL_PWR_CONF_POS          0

#define ACCEL_PWR_CNTRL_ADDR        0x7D
#define ACCEL_PWR_CNTRL_MASK        0xFF
#define ACCEL_PWR_CNTRL_POS         0

#define ACCEL_ERR_CODE_ADDR         0x02
#define ACCEL_ERR_CODE_MASK         0x1C
#define ACCEL_ERR_CODE_POS          2

#define ACCEL_RANGE_2G 0x00
#define ACCEL_RANGE_4G 0x01
#define ACCEL_RANGE_8G 0x02
#define ACCEL_RANGE_16G 0x03

#define ACCEL_ODR_ADDR              0x40
#define ACCEL_ODR_MASK              0xFF
#define ACCEL_ODR_POS               0

#define ACCEL_RANGE_ADDR            0x41
#define ACCEL_RANGE_MASK            0x03
#define ACCEL_RANGE_POS             0

#define ACCEL_FATAL_ERR_ADDR        0x02
#define ACCEL_FATAL_ERR_MASK        0x01
#define ACCEL_FATAL_ERR_POS         0

#define BMI085_I2C_FREQ         400000

#define GET_FIELD(regname,value) ((value & regname##_MASK) >> regname##_POS)
#define	SET_FIELD(regval,regname,value) ((regval & ~regname##_MASK) | ((value << regname##_POS) & regname##_MASK))

/****************************************************************************
* Private Data
****************************************************************************/

static int fd;
static float accel_range_mss;
static float accel_mss[3];
const float G = 9.807f;

typedef enum {
  RANGE_2G = 0x00,
  RANGE_4G = 0x01,
  RANGE_8G = 0x02,
  RANGE_16G = 0x03
} Range;

typedef enum {
  ODR_1600HZ_BW_280HZ,
  ODR_1600HZ_BW_234HZ,
  ODR_1600HZ_BW_145HZ,
  ODR_800HZ_BW_230HZ,
  ODR_800HZ_BW_140HZ,
  ODR_800HZ_BW_80HZ,
  ODR_400HZ_BW_145HZ,
  ODR_400HZ_BW_75HZ,
  ODR_400HZ_BW_40HZ,
  ODR_200HZ_BW_80HZ,
  ODR_200HZ_BW_38HZ,
  ODR_200HZ_BW_20HZ,
  ODR_100HZ_BW_40HZ,
  ODR_100HZ_BW_19HZ,
  ODR_100HZ_BW_10HZ,
  ODR_50HZ_BW_20HZ,
  ODR_50HZ_BW_9HZ,
  ODR_50HZ_BW_5HZ,
  ODR_25HZ_BW_10HZ,
  ODR_25HZ_BW_5HZ,
  ODR_25HZ_BW_3HZ,
  ODR_12_5HZ_BW_5HZ,
  ODR_12_5HZ_BW_2HZ,
  ODR_12_5HZ_BW_1HZ
} Odr;

/****************************************************************************
* Public Functions
****************************************************************************/

int bmi085_i2c_read(uint8_t addr, uint8_t reg, uint8_t *regval, uint8_t count) {
  int ret;
  struct i2c_msg_s msg[2];
  struct  i2c_transfer_s  i2c_transfer;

  msg[0].frequency = BMI085_I2C_FREQ;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = BMI085_I2C_FREQ;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = count;

  i2c_transfer.msgv = msg;
  i2c_transfer.msgc = 2;

  printf("priv->addr = %d, regval = %d\n", addr, reg);

  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t)&i2c_transfer);
  if (ret < 0)
    {
      printf("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

int bmi085_i2c_write(uint8_t addr, uint8_t regaddr, uint8_t regval) {
  int ret;
  struct i2c_msg_s msg[2];
  struct  i2c_transfer_s  i2c_transfer;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = BMI085_I2C_FREQ;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  i2c_transfer.msgv = msg;
  i2c_transfer.msgc = 1;

  ret = ioctl(fd, I2CIOC_TRANSFER, (unsigned long)(uintptr_t)&i2c_transfer);
  if (ret < 0)
    {
      printf("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
* Private Functions
****************************************************************************/

bool setRange(Range range) {
  uint8_t writeReg = 0, readReg = 0;
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_RANGE_ADDR, &readReg, 1);
  writeReg = SET_FIELD(readReg,ACCEL_RANGE,range);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_RANGE_ADDR, writeReg);
  // delay 1 msecond
  nxsig_usleep(1000);
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_RANGE_ADDR, &readReg, 1);

  if (readReg == writeReg) {
    switch (range) {
      case ACCEL_RANGE_2G: {
        accel_range_mss = 2.0f * G;
        break;
      }
      case ACCEL_RANGE_4G: {
        accel_range_mss = 4.0f * G;
        break;
      }
      case ACCEL_RANGE_8G: {
        accel_range_mss = 8.0f * G;
        break;
      }
      case ACCEL_RANGE_16G: {
        accel_range_mss = 16.0f * G;
        break;
      }      
    }
    return true;
  } else {
    return false;
  }
}

bool setOdr(Odr odr) {
  uint8_t writeReg = 0, readReg = 0, value;
  switch (odr) {
    case ODR_1600HZ_BW_280HZ: {
      value = (0x0A << 4) | 0x0C;
      break;
    }
    case ODR_1600HZ_BW_234HZ: {
      value = (0x09 << 4) | 0x0C;
      break;      
    }
    case ODR_1600HZ_BW_145HZ: {
      value = (0x08 << 4) | 0x0C;
      break;       
    }
    case ODR_800HZ_BW_230HZ: {
      value = (0x0A << 4) | 0x0B;
      break;            
    }
    case ODR_800HZ_BW_140HZ: {
      value = (0x09 << 4) | 0x0B;
      break;      
    }
    case ODR_800HZ_BW_80HZ: {
      value = (0x08 << 4) | 0x0B;
      break;           
    }
    case ODR_400HZ_BW_145HZ: {
      value = (0x0A << 4) | 0x0A;
      break;      
    }
    case ODR_400HZ_BW_75HZ: {
      value = (0x09 << 4) | 0x0A;
      break;          
    }
    case ODR_400HZ_BW_40HZ: {
      value = (0x08 << 4) | 0x0A;
      break;    
    }
    case ODR_200HZ_BW_80HZ: {
      value = (0x0A << 4) | 0x09;
      break;       
    }
    case ODR_200HZ_BW_38HZ: {
      value = (0x09 << 4) | 0x09;
      break;      
    }
    case ODR_200HZ_BW_20HZ: {
      value = (0x08 << 4) | 0x09;
      break;      
    }
    case ODR_100HZ_BW_40HZ: {
      value = (0x0A << 4) | 0x08;
      break;      
    }
    case ODR_100HZ_BW_19HZ: {
      value = (0x09 << 4) | 0x08;
      break;      
    }
    case ODR_100HZ_BW_10HZ: {
      value = (0x08 << 4) | 0x08;
      break;      
    }
    case ODR_50HZ_BW_20HZ: {
      value = (0x0A << 4) | 0x07;
      break;      
    }
    case ODR_50HZ_BW_9HZ: {
      value = (0x09 << 4) | 0x07;
      break;      
    }
    case ODR_50HZ_BW_5HZ: {
      value = (0x08 << 4) | 0x07;
      break;            
    }
    case ODR_25HZ_BW_10HZ: {
      value = (0x0A << 4) | 0x06;
      break;            
    }
    case ODR_25HZ_BW_5HZ: {
      value = (0x09 << 4) | 0x06;
      break;         
    }
    case ODR_25HZ_BW_3HZ: {
      value = (0x08 << 4) | 0x06;
      break;         
    }
    case ODR_12_5HZ_BW_5HZ: {
      value = (0x0A << 4) | 0x05;
      break;         
    }
    case ODR_12_5HZ_BW_2HZ: {
      value = (0x09 << 4) | 0x05;
      break;         
    }
    case ODR_12_5HZ_BW_1HZ: {
      value = (0x08 << 4) | 0x05;
      break;         
    }
    default: {
      value = (0x0A << 4) | 0x0C;
      break;
    }
  }

  writeReg = SET_FIELD(writeReg,ACCEL_ODR,value);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_ODR_ADDR, writeReg);
  // delay 1 second
  nxsig_usleep(1000);
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_ODR_ADDR, &readReg, 1);
  return (readReg == writeReg) ? true : false;
}

void softReset() {
  uint8_t reg = 0;
  reg = SET_FIELD(reg,ACCEL_SOFT_RESET,ACCEL_RESET_CMD);
  // writeRegister(ACCEL_SOFT_RESET_ADDR,reg);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_SOFT_RESET_ADDR, reg);
  // delay 50s
  nxsig_usleep(50000);
}

bool setPower(bool enable) {
  uint8_t writeReg = 0, readReg = 0;
  uint8_t value = (enable) ? ACCEL_ENABLE_CMD : ACCEL_DISABLE_CMD;
  writeReg = SET_FIELD(writeReg,ACCEL_PWR_CNTRL,value);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_PWR_CNTRL_ADDR, writeReg);
  // 5 ms wait after power mode changes
  nxsig_usleep(5000);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_PWR_CNTRL_ADDR, readReg);
  return (readReg == writeReg) ? true : false;
}

bool selfTest() {
  uint8_t writeReg = 0;
  float accel_pos_mg[3], accel_neg_mg[3];
  /* set 24G range */
  setRange(RANGE_16G);
  /* set 1.6 kHz ODR, 4x oversampling */
  setOdr(ODR_1600HZ_BW_145HZ);
  /* wait >2 ms */
  nxsig_usleep(3000);
  /* enable self test, positive polarity */
  writeReg = SET_FIELD(writeReg,ACCEL_SELF_TEST,ACCEL_POS_SELF_TEST);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_SELF_TEST_ADDR, writeReg);

  /* wait >50 ms */
  nxsig_usleep(51000);
  /* read self test values */
  // readSensor();
  for (uint8_t i = 0; i < 3; i++) {
    accel_pos_mg[i] = accel_mss[i] / G * 1000.0f;
  }
  /* enable self test, negative polarity */
  writeReg = SET_FIELD(writeReg,ACCEL_SELF_TEST,ACCEL_NEG_SELF_TEST);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_SELF_TEST_ADDR, writeReg);

  /* wait >50 ms */
  nxsig_usleep(51000);
  /* read self test values */
  // readSensor();
  for (uint8_t i = 0; i < 3; i++) {
    accel_neg_mg[i] = accel_mss[i] / G * 1000.0f;
  }
  /* disable self test */
  writeReg = SET_FIELD(writeReg,ACCEL_SELF_TEST,ACCEL_DIS_SELF_TEST);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_SELF_TEST_ADDR, writeReg);

  /* wait >50 ms */
  nxsig_usleep(51000);
  /* check self test results */
  if ((fabs(accel_pos_mg[0] - accel_neg_mg[0]) >= 1000) && (fabs(accel_pos_mg[1] - accel_neg_mg[1]) >= 1000) && (fabs(accel_pos_mg[2] - accel_neg_mg[2]) >= 500)) {
    return true;
  } else {
    return false;
  }
}

bool setMode() {
  uint8_t writeReg = 0, readReg = 0;
  uint8_t value = ACCEL_ACTIVE_MODE_CMD;
  writeReg = SET_FIELD(writeReg,ACCEL_PWR_CONF,value);
  bmi085_i2c_write(BMI085_ACCEL_ADDR, ACCEL_PWR_CONF_ADDR, writeReg);
  // 5 ms wait after power mode changes
  nxsig_usleep(5000);
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_PWR_CONF_ADDR, &readReg, 1);
  return (readReg == writeReg) ? true : false;
}

bool isConfigErr() {
  uint8_t readReg = 0;
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_ERR_CODE_ADDR, &readReg, 1);
  return (GET_FIELD(ACCEL_ERR_CODE,readReg)) ? true : false;
}

bool isFatalErr()
{
  uint8_t readReg = 0;
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_FATAL_ERR_ADDR, &readReg, 1);
  return (GET_FIELD(ACCEL_FATAL_ERR,readReg)) ? true : false;
}

int accel_init() {
  /* check device id */
  uint8_t readReg = 0;
  bmi085_i2c_read(BMI085_ACCEL_ADDR, ACCEL_CHIP_ID_ADDR, &readReg, 1);
  if (GET_FIELD(ACCEL_CHIP_ID,readReg) != ACCEL_CHIP_ID) {
    printf("Wrong chip id: %d \n", GET_FIELD(ACCEL_CHIP_ID,readReg));
    return -1;
  } else {
    printf("Good chip id: %d \n", GET_FIELD(ACCEL_CHIP_ID,readReg));
  }

  /* Soft reset */
  softReset();

  /* Enable accelerometer */
  if (!setPower(true)) {
    printf("Enable accelerometer failed.\n");
    return -2;
  }

  /* Enter active mode */
  if (!setMode(true)) {
    return -3;
  }

  /* self test */
  // if (!selfTest()) {
  //   return -4;
  // }

  // /* soft reset */
  // softReset();

  // /* enable the accel */
  // if (!setPower(true)) {
  //   return -5;
  // }

  // /* enter active mode */
  // if (!setMode(true)) {
  //   return -6;
  // } 

  /* Set default range */
  if (!setRange(ACCEL_RANGE_16G)) {
    return -7;
  }

  /* Set default ODR */
  if (!setOdr(ODR_1600HZ_BW_280HZ)) {
    return -8;
  }

  /* Check config errors */
  if (isConfigErr()) {
    return -9;
  }

  /* Check fatal errors */
  if (isFatalErr()) {
    return -10;
  }

  return 0;
}


static int check_bmi085() {
  
  /* Initialize accelerometer and gyroscope */
  accel_init();

  return 0;
}

/****************************************************************************
* bmi085_main
****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct accel_gyro_st_s data;
  uint32_t prev;

  printf("Hello BMI085!\n");

  fd = open(I2C_DRIVER_PATH, O_RDWR);
  if (fd < 0) {
    printf("Error opening i2c driver path.\n");
    return -1;
  }

  if (check_bmi085() == 0) {
    printf("BMI085 communication successful!\n");
  } else {
    printf("BMI085 communication is faulty...\n");
    return -1;
  }

  close(fd);
  return 0;
}
