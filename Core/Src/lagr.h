/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/
#define LAGR_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>

//double x[8] = {-20*M_PI/180, 20*M_PI/180,     70*M_PI/180,     180*M_PI/180,    290*M_PI/180,    340*M_PI/180,    380*M_PI/180,    430*M_PI/180};
//double y[8] = {800, 1000,    1000,   50,    500,    800,    1000,   1000};
//double xx[128];
//double yy[128];
//double xxx[128];
//double yyy[128];

double Lagrange (double xp,double* R, double* Phi, uint8_t size)
{
 double p,yp;
    	 /* Implementing Lagrange Interpolation */
	 for(uint8_t i=0;	i<size;	i++ )
	 {
		  p=1;
		  for(uint8_t j=0;	j<size;	j++ )
		  {
			   if(i!=j)
			   {
			    	p = p*(xp - *(Phi+j) )/( *(Phi+i) - *(Phi+j));
			   }
		  }
		  yp = yp + p * (*(R+i));
	 }
    return yp;
}

//int main()
//{
// //   printf("Hello World");
//
//    double xp = 0;
//    double S = 0;
//    uint8_t i;
//    float step = 0.05;
//    while (xp < 2*M_PI)
//    {
//        xx[i] = xp;
//        yy[i] = Lagrange(xp);
//
////        if (yy[i]<0)yy[i] = 0;
//
//        S+= yy[i]*yy[i]*step/2/1000000;
//
//        xp+=step;
//        i++;
//    }
//
//    printf("%f \n",S);
//
//
//
//    return 0;
//}
