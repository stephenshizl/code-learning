
#ifndef __QST_MY_FILTER_H__
#define __QST_MY_FILTER_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	char   init;
	float  x_now;
	float  x_mid;
	float  p_mid;
	float  x_last;
	float  p_last;
	float  p_now;
	float  kg;
}MyFilter_t;


typedef struct
{
	char	init;
	float	alpha;
	float	last;
}LpfFilter_t;

#define AVG_ARRAY_SIZE		10
typedef struct
{
	char	init;
	float	array[AVG_ARRAY_SIZE];
	float	avg;
}AvgFilter_t;

typedef struct
{
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
}QST_Filter;

typedef struct
{
	float _delay_element_1;
	float _delay_element_2;
}QST_Filter_Buffer;


void InitMyFilter(MyFilter_t *filter);
float ExeMyFilter(MyFilter_t *filter, float in);

void InitLpfFilter(LpfFilter_t *filter);
float ExeLpfFilter(LpfFilter_t *filter, float in);

void InitAvgFilter(AvgFilter_t *filter);
float ExeAvgFilter(AvgFilter_t *filter, float in);

void InitBtwzFilter(void);
float ExeBtwzFilter(float in);

#ifdef __cplusplus
}
#endif

#endif

