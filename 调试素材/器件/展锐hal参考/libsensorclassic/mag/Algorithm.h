#ifndef RUN_ALGORITHM_H
#define RUN_ALGORITHM_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
	float dRawMag[3];
	float mag_bias[3];
	float data_cali[3];
	float pitch,
	      roll,
          yaw;
	float acc[3];
}_QMC7983;

long long get_time_in_nanosec(void);
void mcal(long timestamp_ms);
int process(float* magData,long timestamp_ms);
int get_mag_bias(float* mag_bias);
char get_mag_accuracy(void);
int push2mcal(_QMC7983 *data);

#ifdef __cplusplus
}
#endif

#endif
