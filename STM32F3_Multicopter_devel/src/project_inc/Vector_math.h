#ifndef _VECTOR_MATH_H_ 
#define _VECTOR_MATH_H_


float VectorDotProduct(float vector1[3],float vector2[3]);
void VectorCrossProduct(float vectorOut[3], float v1[3],float v2[3]);
void VectorScale(float vectorOut[3],float vectorIn[3], float scale2);
void VectorAdd(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]); 

void MatrixMultiply(float a[3][3], float b[3][3],float mat[3][3]);


#endif // #define _VECTOR_MATH_H_
