/*
 * filter_Lib.c
 *
 *  Created on: Nov 1, 2012
 *      Author: franz
 */
#include <filter_Lib.h>
#include <math.h>


void update_intFilter(intFilter *filter, int lowpassValue, int newMeasurement)
{
	filter->r = newMeasurement + filter->r - filter->filtered;
	filter->filtered = filter->r>>lowpassValue;
}

void initAvgValues(struct movingAverageValues *avg)
{
	int i = 0;
	avg->countValue = 0;
	for(i = 0;i<MA_VALUE;i++)
	{
		avg->avgArray[i] = 0;
	}
}

int calcMovingAvg(struct movingAverageValues *avg, int newValue)
{
	int i = 0, result = 0;

	avg->avgArray[avg->countValue] = newValue;
	for(i = 0;i<MA_VALUE;i++)
	{
		result += avg->avgArray[i];
	}
	avg->countValue++;
	if (avg->countValue >= MA_VALUE)
	{
		avg->countValue = 0;
	}
	return result/MA_VALUE;
}


void quickSort (int a[], int lo, int hi)
{
//  lo is the lower index, hi is the upper index
//  of the region of array a that is to be sorted
    int i=lo, j=hi, h;
    int x=a[(lo+hi)/2];

    //  partition
    do
    {
        while (a[i]<x) i++;
        while (a[j]>x) j--;
        if (i<=j)
        {
            h=a[i]; a[i]=a[j]; a[j]=h;
            i++; j--;
        }
    } while (i<=j);

    //  recursion
    if (lo<j) quickSort(a, lo, j);
    if (i<hi) quickSort(a, i, hi);
}



int calcStdDev (int a[], int index, int avgVal)
{
	int sum = 0, i = 0;

	for(i = 0; i<index;i++)
	{
		sum = (a[i]-avgVal)*(a[i]-avgVal);
	}
	return (sqrt(sum/(index-1)));
}


/*
 * SampleRate 250Hz cutoff: 5Hz 4th order Butterworth
 * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
 */
void floatFilterAcc1(floatFilter *value, int newValue)
{
	float *xv = &value->filter_xv[0];
	float *yv = &value->filter_yv[0];
	xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4];
	xv[4] = (float)newValue / 2.674241096e+06 /*GAIN*/;
	yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4];
	yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
				 + ( -0.8768965608 * yv[0]) + (  3.6227607596 * yv[1])
				 + ( -5.6145268496 * yv[2]) + (  3.8686566679 * yv[3]);
	value->currentValue = yv[4];
}

void floatFilterAcc2(floatFilter *value, int newValue)
{
	float *xv = &value->filter_xv[0];
	float *yv = &value->filter_yv[0];
	xv[0] = xv[1]; xv[1] = xv[2];
	xv[2] = (float)newValue / 2.555570536e+04 /*GAIN*/;
    yv[0] = yv[1]; yv[1] = yv[2];
    yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                 + ( -0.9823854506 * yv[0]) + (  1.9822289298 * yv[1]);
    value->currentValue = yv[2];
}

void floatFilterAcc(floatFilter *value, int newValue)
{
	float *xv = &value->filter_xv[0];
	float *yv = &value->filter_yv[0];
	xv[0] = xv[1]; xv[1] = xv[2];
	xv[2] = (float)newValue / 1.017716736e+05 /*GAIN*/;
    yv[0] = yv[1]; yv[1] = yv[2];
    yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                 + ( -0.9911535959 * yv[0]) + (  1.9911142922 * yv[1]);
    value->currentValue = yv[2];
}


/*
 * SampleRate 500Hz, cutoff 200Hz, 4th order Butterworth
 * http://www-users.cs.york.ac.uk/~fisher/mkfilter/trad.html
 */
void floatFilterGyro(floatFilter *value, int newValue)
{
	float *xv = &value->filter_xv[0];
	float *yv = &value->filter_yv[0];

	xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4];
	xv[4] = (float)newValue / 2.310287053e+00;
	yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4];
	yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
				 + ( -0.1873794924 * yv[0]) + ( -1.0546654059 * yv[1])
				 + ( -2.3139884144 * yv[2]) + ( -2.3695130072 * yv[3]);
	value->currentValue = yv[4];
}



