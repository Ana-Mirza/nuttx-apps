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

#include "StepCountingAlgo.h"
#include <nuttx/sensors/bmi085.h>

#define DATASET_PATH "DataSet/optimisation/data/_Patient_1/accelerometer.csv"
#define BMI085_DEVPATH   "/dev/bmi085"

void check_csv_file()
{
  initAlgo();

  char *line = NULL;
  size_t len = 0;
  size_t read;
  FILE *fp = fopen(DATASET_PATH, "r");

  if (fp == NULL)
  {
      perror("Error while opening the file.\n");
      exit(EXIT_FAILURE);
  }

  clock_t startTime = clock();
  while ((read = getline(&line, &len, fp)) != -1) {
      long time = strtol(strtok(line, ","), NULL, 10);
      strtok(NULL, ",");
      long x = (long) (strtof(strtok(NULL, ","), NULL) * 100000000);
      long y = (long) (strtof(strtok(NULL, ","), NULL) * 100000000);
      long z = (long) (strtof(strtok(NULL, ","), NULL) * 100000000);
      printf("%ld,%f,%f,%f\n",time,x,y,z);
      processSample(time, x, y, z);
  }

  clock_t endTime = clock();
  printf("steps=%i\nt=%fs\n", getSteps(), ((float)(endTime - startTime))/10000000);

  fclose(fp);

  if (line)
      free(line);
}

int main(int argc, FAR char *argv[])
{
  int ret;
  uint32_t prev;
  int fd_bmi085;
  struct accel_gyro_st_s data;

  int16_t accelVals[3];
  int16_t lastAccelSample[3];

  /* Initialize step counting algorithm */ 
  initAlgo();

  fd_bmi085 = open(BMI085_DEVPATH, O_RDONLY);
  if (fd_bmi085 < 0)
    {
      printf("Device %s open failure. %d\n\n", BMI085_DEVPATH, fd_bmi085);
      return -1;
    }

  /* Read initial data sample */
  ret = read(fd_bmi085, &data, sizeof(struct accel_gyro_st_s));
  if (ret != sizeof(struct accel_gyro_st_s))
    {
      fprintf(stderr, "Read failed.\n");
      return -1;
    }
    
  lastAccelSample[0] = data.accel.x;
  lastAccelSample[1] = data.accel.y;
  lastAccelSample[2] = data.accel.z;
      
  /* Start reading data */
  prev = 0;
  while(1) {
    up_mdelay(17);
    ret = read(fd_bmi085, &data, sizeof(struct accel_gyro_st_s));
    if (ret != sizeof(struct accel_gyro_st_s))
      {
        fprintf(stderr, "Read failed.\n");
        break;
      }

    accelVals[0] = (int16_t)(lastAccelSample[0] - data.accel.x);
    accelVals[1] = (int16_t)(lastAccelSample[1] - data.accel.y);
    accelVals[2] = (int16_t)(lastAccelSample[2] - data.accel.z);
    lastAccelSample[0] = data.accel.x;
    lastAccelSample[1] = data.accel.y;
    lastAccelSample[2] = data.accel.z;

    /* Convert µs to ms. */
    uint32_t timestamp_ms = data.sensor_time / 1000;

    /* Process sample with timestamp and accel data. */ 
    processSample(timestamp_ms, accelVals[0], accelVals[1], accelVals[2]);

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

    printf("Step count: %d\n", getSteps());
  }

  close(fd_bmi085);

  return 0;
}
