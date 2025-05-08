
#include "stdafx.h"
#include "stdio.h"
#include "MyFilter.h"


#if 0
//#define Factor			0.05f
//#define R				0.00801f
#define Factor			0.025f
#define R				0.000000414975f
#else
//#define Factor			0.01f
//#define R				0.000000414975f

#define Factor			0.01f
#define R				0.00005f

#endif
#define Q				(R*Factor)

#define Pi 3.14159265358979f

static QST_Filter press_filter_parameter = {-0.747789085f, 0.272214949f, 0.131106466f, 0.262212932f, 0.131106466f};
static QST_Filter_Buffer press_buffer;


void InitMyFilter(MyFilter_t *filter)
{
	memset(filter, 0, sizeof(MyFilter_t));
}

float ExeMyFilter(MyFilter_t *filter, float in)
{
	float out;

	if(filter->init == 0)
	{
		filter->x_last = in;
		filter->init = 1;
	}
    filter->x_mid=filter->x_last; 
    filter->p_mid=filter->p_last + Q; 
    filter->kg=filter->p_mid/(filter->p_mid+R); 
    filter->x_now=filter->x_mid + filter->kg*(in -filter->x_mid); 
    filter->p_now=(1-filter->kg)*filter->p_mid;
    filter->p_last = filter->p_now; 
    filter->x_last = filter->x_now; 
    out = filter->x_now;

	return out;
}


void InitLpfFilter(LpfFilter_t *filter)
{
	filter->init = 0;
	filter->alpha = 0.85f;
	filter->last = 0.0f;
}

float ExeLpfFilter(LpfFilter_t *filter, float in)
{
	if(filter->init == 0)
	{
		filter->last = in;
		filter->init = 1;
	}
	filter->last = (filter->alpha*filter->last) + (1-filter->alpha)*in;

	return filter->last;
}


void InitAvgFilter(AvgFilter_t *filter)
{
	filter->init = 0;
}

float ExeAvgFilter(AvgFilter_t *filter, float in)
{
	int i;
	float sum = 0.0f;

	if(filter->init == 0)
	{
		for(i=0; i<AVG_ARRAY_SIZE; i++)
		{
			filter->array[i] = in;
		}
		filter->init = 1;
	}

	sum = 0.0f;;
	for(i=1; i<AVG_ARRAY_SIZE; i++)
	{
		sum += filter->array[i];
		filter->array[i-1] = filter->array[i];
	}
	filter->array[AVG_ARRAY_SIZE-1] = in;
	sum += in;
	filter->avg = sum/AVG_ARRAY_SIZE;

	return filter->avg;
}


void set_cutoff_frequency(float sample_freq, float cutoff_freq, QST_Filter *filter)
{	
	float fr = sample_freq / cutoff_freq;
	float ohm = tanf(Pi / fr);
	float c = 1.0f + 2.0f * cosf(Pi / 4.0f) * ohm + ohm * ohm;
	filter->b0 = ohm * ohm / c;
	filter->b1 = 2.0f * filter->b0;
	filter->b2 = filter->b0;
	filter->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	filter->a2 = (1.0f - 2.0f * cosf(Pi / 4.0f) * ohm + ohm * ohm) / c;
}

float Filter_Apply(float sample, QST_Filter_Buffer *buffer, QST_Filter *filter) 
{
	//do the filtering
	float delay_element_0 = sample - buffer->_delay_element_1 * filter->a1 - buffer->_delay_element_2 * filter->a2;

	const float output = delay_element_0 * filter->b0 + buffer->_delay_element_1 * filter->b1 + buffer->_delay_element_2 * filter->b2;

	buffer->_delay_element_2 = buffer->_delay_element_1;
	buffer->_delay_element_1 = delay_element_0;

	//return the value. Should be no need to check limits
	return output;
}

void InitBtwzFilter(void)
{
	set_cutoff_frequency(50, 5, &press_filter_parameter);
}

float ExeBtwzFilter(float in)
{
	return Filter_Apply(in, &press_buffer, &press_filter_parameter);
}