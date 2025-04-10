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
#include <stdlib.h>
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

#define BMI085_DEVPATH   "/dev/bmi085"
#define I2C_DRIVER_PATH  "/dev/i2c0"

#define LOOP 1

/****************************************************************************
* bmi085_main
****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd_bmi085;
  int loop = LOOP;
  struct accel_gyro_st_s data;
  uint32_t prev;

  /* Check input */
  if (argc >= 2)
    loop = atoi(argv[1]);

  printf("Usage: test_bmi085 <number_of_samples>\n");

  fd_bmi085 = open(BMI085_DEVPATH, O_RDONLY);
  if (fd_bmi085 < 0)
    {
      printf("Device %s open failure. %d\n\n", BMI085_DEVPATH, fd_bmi085);
      return -1;
    }

  /* Start reading data */
  prev = 0;
  while(loop--) {
    int ret;

    ret = read(fd_bmi085, &data, sizeof(struct accel_gyro_st_s));
    if (ret != sizeof(struct accel_gyro_st_s))
      {
        fprintf(stderr, "Read failed.\n");
        break;
      }

    /* If sensing time has been changed, show 6 axis data. */

    if (prev != data.sensor_time)
    {
      printf("[%" PRIu32 "] %d, %d, %d / %d, %d, %d\n",
              data.sensor_time,
              data.gyro.x, data.gyro.y, data.gyro.z,
              data.accel.x, data.accel.y, data.accel.z);
      printf("[Temperature] %d C\n", data.sensor_temp);
      fflush(stdout);
      prev = data.sensor_time;   
    }

    up_mdelay(50);
  }

  close(fd_bmi085);

  return 0;
}
