//#ifndef MATRIX_HPP
//#define MATRIX_HPP

#include <stdio.h>
#include <math.h>
//#include <iostream>

//#include "deftype.h"
// EB_ONERA
#include "Matrix_onera.hpp"
#include <stdlib.h>


namespace math
{


double inverse_matrice_pivot(  real (*mat),  real (*mat_inv), unsigned int M) /* détermine la matrice inverse d'une matrice carrée non nulle par la méthode du pivot*/
{
	/* matrice de départ de travail et matrice inverse mélangée */
	double determinant = 1.0;
    int *memoire;
	int i, j, k, indice;

	double pivot, diviseur;
	double max;
    int dim = M;
    
    real *mtr, *melange;
    
    mtr = (real*) malloc(M*M*sizeof(real));
    melange = (real*) malloc(M*M*sizeof(real));
    
        
    if((memoire = (int *) malloc(sizeof(int) * dim))==NULL) {return(0.0);}; /* tableau de correspondance entre la ligne et la colonne traitée */
    
 //   mtr = mat;
    for(i=0; i<dim;i++)
        for(j=0; j<dim;j++)
            mtr[i*dim+j]= mat[i*dim+j];
    
//    mat_inv.zero();
    for(i=0; i<dim;i++)
        for(j=0; j<dim;j++)
            mat_inv[i*dim+j]= 0.0;

//    melange.identity();
    for(i=0; i<dim;i++)
        for(j=0; j<dim;j++)
            melange[i*dim+j]= 0.0;
    
    for(i=0; i<dim;i++)
        melange[i*dim+i]= 1.0;



	/* inversion de la matrice par recherche dans chaque ligne du pivot le plus grand en absolu */

	for(i=0;i<dim;i++) /* on traite la ligne i */
	{
		indice = 0;

		/* recherche de l'indice de l'élément le plus grand en absolu 

		!!!! en prenant l'élément le plus grand, on ne traite plus la ligne correspondant
		 à la i, mais la ligne correspondant à l'indice de l'élément choisi. On construit donc la
		 matrice inverse avec des lignes mélangées. Il faut donc garder une trace de la 
		 correspondance entre la ligne traitée et la ligne correspondante dans la matrice
		 inverse finale pour par la suite recopier le tout dans le bon ordre.

		*/

		max = fabs(mtr[i*dim+indice]);

        j = 1;

		while(j<dim)
		{
			if(fabs(mtr[i*dim+j]) > max)
			{
				max = fabs(mtr[i*dim+j]);
				indice = j;
			}
			j++;
		}

		if(max == 0.0) /* a trouver une ligne dont tous les coéfficients sont nuls */
		{
//			free(mtr);
//			free(melange);
//			free(memoire);
			return(0.0); /* matrice n'est pas inversible car déterminant nul */
		}
		        
		pivot = mtr[i*dim+indice];

		*(memoire+i) = indice; /* la ligne i correspond à la ligne finale indice */

		/* calcule le déterminant */
		determinant*=pivot;

		/* divise la ligne des deux matrices par le pivot */
		for(j = 0; j<dim ; j++)
		{
			mtr[i*dim+j]/=real(pivot);
			melange[i*dim+j]/=real(pivot);
		}

		/* soustrait la ligne du pivot des autres lignes * le coéfficient de la colonne du pivot */

		for(j=0; j<i; j++)
		{
			diviseur = mtr[j*dim+indice];

			for(k=0; k<dim; k++)
			{
				mtr[j*dim+k]-= real(diviseur) * (mtr[i*dim+k]);
				melange[j*dim+k]-= real(diviseur) * (melange[i*dim+k]);
			}
		}

		for(j=i+1; j<dim; j++)
		{
			diviseur = mtr[j*dim+indice];

			for(k=0; k<dim; k++)
			{
				mtr[j*dim+k]-= real(diviseur) * (mtr[i*dim+k]);
				melange[j*dim+k]-= real(diviseur) * (melange[i*dim+k]);
			}
		}
    }
        
	/* copie des lignes de la matrice inverse mélangée à la bonne place dans la matrice
	inverse finale : ligne i de mélange = ligne *(memoire+i) de la matrice inverse  */

	for(i=0;i<dim;i++) /* on traite la ligne i */
	{
		for(j=0; j<dim; j++)
		{
			mat_inv[*(memoire+i)*dim+j] = melange[i*dim+j];
        }

	}
    
	free(mtr);
	free(melange);
	free(memoire);

	return(determinant);
}


void choldc1(real (*a), real (*p), unsigned int M) 
{
     int i,j,k;
     int n = M;
     real sum;

    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*M+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*M+k] * a[j*M+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    printf(" a is not positive definite!\n");
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*M+i] = sum / p[i];
            }
        }
    }
}


void choldcsl(const real (*A), real (*a), unsigned int M)
{
    int i,j,k;
    int n = M;
    real sum;
    real *p;
    
    p = (real*) malloc(M*sizeof(real));

    for (i = 0; i < n; i++) 
        for (j = 0; j < n; j++) 
            a[i*M+j] = A[i*M+j];
    
    choldc1(a, p, M);
    
    for (i = 0; i < n; i++) {
        a[i*M+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*M+k] * a[k*M+i];
            }
            a[j*M+i] = sum / p[j];
        }
    }
    
    free(p);
}


double inverse_matrice_cholesky( const real (*mat),   real (*mat_inv), unsigned int M) /* détermine la matrice inverse d'une matrice carrée non nulle par la méthode du pivot*/
{
	/* matrice de départ de travail et matrice inverse mélangée */
	unsigned int i, j, k;
	double  d = 1.0;      
    real *a;
    
    choldcsl(mat, mat_inv, M);  
    
   a = mat_inv;
    
    for (i = 0; i < M; i++) {
        for (j = i + 1; j < M; j++) {
            a[i*M+j] = 0.0;
        }
    }
    
    for (i = 0; i < M; i++) {
        a[i*M+i] *= a[i*M+i];
        for (k = i + 1; k < M; k++) {
            a[i*M+i] += a[k*M+i] * a[k*M+i];
        }
        for (j = i + 1; j < M; j++) {
            for (k = j; k < M; k++) {
                a[i*M+j] += a[k*M+i] * a[k*M+j];
            }
        }
    }
    for (i = 0; i < M; i++) {
        for (j = 0; j < i; j++) {
            a[i*M+j] = a[j*M+i];
        }
    }
    return (d);
}

//        //check if matrix A is positive definite (return 1)
//        //or not positive definite (return 0) 
//        int Check_Matrix(int n, MAT A) {
//          int i,j,k,result; double sum;
//          result=1;
//	  for (i=0; i<n; i++) {
//	    for (j = i; j<n; j++) {
//              sum = A[i][j];
//              for (k = i - 1; k>=0; k--)
//                sum -= A[i][k] * A[j][k];
//              if (i == j)
//                if (sum <= 0.0) result=0;
//	    }
//          }
//	  return result;
//	}

}

//#endif // MATRIX_HPP
