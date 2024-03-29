#include <Vector_math.h>


//Computes the dot product of two vectors 
float VectorDotProduct(float vector1[3],float vector2[3]) 
{   
	int c = 0;
	float op=0;      
	for( c = 0 ; c < 3 ; c++ )   
	{   
		op += vector1[c] * vector2[c];   
	}      
	return op;  
}  

//Computes the cross product of two vectors 
void VectorCrossProduct(float vectorOut[3], float v1[3],float v2[3]) 
{   
	vectorOut[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);   
	vectorOut[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);   
	vectorOut[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]); 
}  

//Multiply the vector by a scalar.  
void VectorScale(float vectorOut[3],float vectorIn[3], float scale2) 
{   
	int c = 0;
	for( c = 0 ; c < 3 ; c++ )   
	{    
		vectorOut[c] = vectorIn[c] * scale2;    
	} 
}  

void VectorAdd(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]) 
{
	int c = 0;
   
	for(c = 0 ; c < 3 ; c++ )   
	{      
		vectorOut[c] = vectorIn1[c] + vectorIn2[c];   
	} 
} 



//Multiply two 3x3 matrices. 
void MatrixMultiply(float a[3][3], float b[3][3],float mat[3][3]) 
{   
	float op[3];    
	int x = 0;
	int y = 0;
	int w = 0;
	//float test = 0;
	
	for(  x = 0 ; x < 3 ; x++ )   
	{     
		for(  y = 0 ; y < 3 ; y++ )     
		{       
			for(  w = 0 ; w < 3 ; w++ )       
			{        
				op[w] = a[x][w]*b[w][y];       
			}        
			mat[x][y] = 0;       
			mat[x][y] = op[0] + op[1] + op[2];              
			//test=mat[x][y];     
		}   
	} 
} 
