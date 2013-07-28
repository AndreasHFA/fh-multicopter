/*
 * filter_Lib.h
 *
 *  Created on: Nov 1, 2012
 *      Author: franz
 */

#ifndef FILTER_LIB_H_
#define FILTER_LIB_H_

#define MA_VALUE 5

struct movingAverageValues
{
	int avgArray[MA_VALUE];
	int countValue;
};

typedef struct _intFilter_
{
	int r;
	int filtered;
}intFilter;


#define NZEROS 4
#define NPOLES 4

typedef struct _floatFilter_
{
	float filter_xv[NZEROS+1];
	float filter_yv[NPOLES+1];
	float currentValue;
}floatFilter;


void update_intFilter(intFilter *filter, int lowpassValue, int newMeasurement);
void initAvgValues(struct movingAverageValues *avg);
int calcMovingAvg(struct movingAverageValues *avg, int newValue);
void quickSort (int a[], int lo, int hi);
int calcStdDev (int a[], int index, int avgVal);
void floatFilterAcc(floatFilter *value, int newValue);
void floatFilterGyro(floatFilter *value, int newValue);

#endif /* FILTER_LIB_H_ */
