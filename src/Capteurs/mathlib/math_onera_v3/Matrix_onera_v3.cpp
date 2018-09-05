// #define __EXPORT   __declspec( dllexport )
//  
//  
// // #ifndef MATRIX_CPP
// // #define MATRIX_CPP
// 
// #include <iostream>
// 
// #include "deftype.h"
//  
//  
// #include "Vector_v3.hpp"
// #include "Matrixs_v3.hpp"
// #include "Matrix_v3.hpp"

// EB_ONERA
#include "Matrix_onera_v3.hpp"
#include "Matrixs_onera_v3.hpp"

#include <stdlib.h>


namespace math
{
    //         template <unsigned int M>
    //      	Vector<M> operator *(const Matrixs<M> &m, const Vector<M> &v)  {
    // 		math::Vector<M> res;
    //         
    //         register int i, k ;
    //         register double val ;
    //         register double *ptrm = res.data ;
    //         const double *ptr1 , *ptr1_base = m.data ;
    //         const double *ptr2 , *ptr2_base = v.data ;
    //         for( i = 1; i <= M; i++ ){
    //             ptr1 = ptr1_base ;
    //             ptr2 = ptr2_base;
    //             val = 0. ;
    //             for( k=1 ; k< i ; k++ ){
    //                 val += (*ptr1)*(*ptr2) ;
    //                 ptr1 += M - k ;
    //                 ptr2++ ;
    //             }
    //             for(k=i ; k<= M ; k++ ){
    //                 val += (*ptr1++)*(*ptr2 ) ;
    //                 ptr2++ ;
    //             }
    //             *ptrm++ = val  ;
    // 
    //             ptr1_base++  ;
    //         }
    // 
    //         return res;
    // 	}

    template <unsigned int M , unsigned int N>
    Matrix<M , N> operator * ( const Matrix<M , N> &m1 , const Matrixs<N> &m )
    {
        Matrix<M , N> res ;

        register int i = M , j , k ;
        register double val ;
        register double *ptrm = res.data ;
        const register double *ptr1 ;
        const register double *ptr2 ;
        const register double *ptr1_base = m1.data ;
        const register double *ptr2_base = m.data ;

        while ( i-- )
        {
            for ( j = 1 ; j <= int(N) ; j++ )
            {
                ptr1 = ptr1_base ;
                ptr2 = ptr2_base++ ;
                val = 0. ;
                for ( k = 1 ; k < j ; k++ )
                {
                    val += ( *ptr1++ )*( *ptr2 ) ;
                    ptr2 += N - k  ;
                }
                for ( k = j ; k <= int(N) ; k++ )
                {
                    val += ( *ptr1++ )*( *ptr2++ ) ;
                }
                *ptrm++ = val  ;

            }
            ptr1_base += N  ;
            ptr2_base = m.data ;
        }

        return res ;
    }

    template <unsigned int M , unsigned int N>
    Matrix<M , N> operator * ( const Matrixs<M> &m1 , const Matrix<M , N> &m )
    {
        Matrix<M , N> res ;

        register int i = M , j , k ;
        register double val ;
        register double *ptrm = res.data ;
        const register double *ptr1 ;
        const register double *ptr2 ;
        const register double *ptr1_base = m1.data ;
        const register double *ptr2_base = m.data ;


        for ( i = 1 ; i <= int(M ) ; i++ )
        {
            for ( j = 1 ; j <= int(N ) ; j++ )
            {
                ptr1 = ptr1_base ;
                ptr2 = ptr2_base++ ;
                val = 0. ;
                for ( k = 1 ; k < i ; k++ )
                {
                    val += ( *ptr1 )*( *ptr2 ) ;
                    ptr1 += M - k ;
                    ptr2 += N   ;
                }
                for ( k = i ; k <= int(M ) ; k++ )
                {
                    val += ( *ptr1++ )*( *ptr2 ) ;
                    ptr2 += N ;
                }
                *ptrm++ = val  ;

            }
            ptr1_base ++  ;
            ptr2_base = m.data ;
        }

        return res ;
    }

    template <unsigned int M , unsigned int N>
    Matrixs<M> mxft ( const Matrix<M , N> &m1 , const Matrix<M , N> &m2)
    {
        Matrixs<M> res ;

        register int i , j , k ;
        register double val ;
        register double *ptrm = res.data ;
        const register double *ptr1 , *ptr1_base = m1.data ;
        const register double *ptr2 , *ptr2_base = m2.data ;

        for ( i = 0 ; i < int(M) ; i++ )
        {
            ptr2_base = m2.data  + i*N ;

            for ( j = i ; j < int(M) ; j++ )
            {
                ptr1 = ptr1_base ;
                ptr2 = ptr2_base ;
                k = N ;
                val = 0. ;
                while ( k-- )
                {
                    val += ( *ptr1++ )*( *ptr2++ ) ;
                }

                *ptrm++ = val  ;
                ptr2_base += N ;
            }
            ptr1_base += N ;
        }

        return res ;
    }

    template <unsigned int M , unsigned int N>
    Matrixs<M> fxm ( const Matrix<M , N> &m1 , const Matrix<N , M> &m2)
    {
        Matrixs<M> res ;

        register int i , j , k ;
        register double val ;
        register double *ptrm = res.data ;
        const register double *ptr1 , *ptr1_base = m1.data ;
        const register double *ptr2 , *ptr2_base = m2.data ;

        for ( i = 0 ; i < int(M ) ; i++ )
        {
            ptr2_base = m2.data  + i ;

            for ( j = i ; j < int(M ) ; j++ )
            {
                ptr1 = ptr1_base ;
                ptr2 = ptr2_base ;
                k = N ;
                val = 0. ;
                while ( k-- )
                {
                    val += ( *ptr1++ )*( *ptr2 ) ;
                    ptr2 += M ;
                }

                *ptrm++ = val  ;
                ptr2_base++ ;
            }
            ptr1_base += N ;
        }

        return res ;
    }

    template <unsigned int M , unsigned int N>
    Matrixs<M> fxmxft ( const Matrix<M , N> &f , const Matrixs<N> &m)
    {
        Matrix<M , N> fm = f * m ;

        Matrixs<M> res = mxft (fm , f) ;

        return res ;
    }

    double inverse_matrice_pivot ( const double (*mat) ,   double (*mat_inv) , unsigned int M) /* d�termine la matrice inverse d'une matrice carr�e non nulle par la m�thode du pivot*/
    {
        /* matrice de d�part de travail et matrice inverse m�lang�e */
        double determinant = 1.0 ;
        int *memoire ;
        int i , j , k , indice ;

        double pivot , diviseur ;
        double max ;
        int dim = M ;

        double *mtr , *melange ;

        mtr = ( double* ) malloc (M * M * sizeof (double )) ;
        melange = ( double* ) malloc (M * M * sizeof (double )) ;


        if ( ( memoire = ( int * ) malloc (sizeof (int ) * dim) ) == NULL )
        {
            return (0.0 ) ;
        } ; /* tableau de correspondance entre la ligne et la colonne trait�e */

        //   mtr = mat;
        for ( i = 0 ; i < dim ; i++ )
            for ( j = 0 ; j < dim ; j++ )
                mtr[i * dim + j] = mat[i * dim + j] ;

        //    mat_inv.zero();
        for ( i = 0 ; i < dim ; i++ )
            for ( j = 0 ; j < dim ; j++ )
                mat_inv[i * dim + j] = 0.0 ;

        //    melange.identity();
        for ( i = 0 ; i < dim ; i++ )
            for ( j = 0 ; j < dim ; j++ )
                melange[i * dim + j] = 0.0 ;

        for ( i = 0 ; i < dim ; i++ )
            melange[i * dim + i] = 1.0 ;



        /* inversion de la matrice par recherche dans chaque ligne du pivot le plus grand en absolu */

        for ( i = 0 ; i < dim ; i++ ) /* on traite la ligne i */
        {
            indice = 0 ;

            /* recherche de l'indice de l'�l�ment le plus grand en absolu 

            !!!! en prenant l'�l�ment le plus grand, on ne traite plus la ligne correspondant
             � la i, mais la ligne correspondant � l'indice de l'�l�ment choisi. On construit donc la
             matrice inverse avec des lignes m�lang�es. Il faut donc garder une trace de la 
             correspondance entre la ligne trait�e et la ligne correspondante dans la matrice
             inverse finale pour par la suite recopier le tout dans le bon ordre.

             */

            max = fabs (mtr[i * dim + indice]) ;

            j = 1 ;

            while ( j < dim )
            {
                if ( fabs (mtr[i * dim + j]) > max )
                {
                    max = fabs (mtr[i * dim + j]) ;
                    indice = j ;
                }
                j++ ;
            }

            if ( max == 0.0 ) /* a trouver une ligne dont tous les co�fficients sont nuls */
            {
                //			free(mtr);
                //			free(melange);
                //			free(memoire);
                return (0.0 ) ; /* matrice n'est pas inversible car d�terminant nul */
            }

            pivot = mtr[i * dim + indice] ;

            *( memoire + i ) = indice ; /* la ligne i correspond � la ligne finale indice */

            /* calcule le d�terminant */
            determinant *= pivot ;

            /* divise la ligne des deux matrices par le pivot */
            for ( j = 0 ; j < dim ; j++ )
            {
                mtr[i * dim + j] /= double (pivot) ;
                melange[i * dim + j] /= double (pivot) ;
            }

            /* soustrait la ligne du pivot des autres lignes * le co�fficient de la colonne du pivot */

            for ( j = 0 ; j < i ; j++ )
            {
                diviseur = mtr[j * dim + indice] ;

                for ( k = 0 ; k < dim ; k++ )
                {
                    mtr[j * dim + k] -= double (diviseur) * ( mtr[i * dim + k] ) ;
                    melange[j * dim + k] -= double (diviseur) * ( melange[i * dim + k] ) ;
                }
            }

            for ( j = i + 1 ; j < dim ; j++ )
            {
                diviseur = mtr[j * dim + indice] ;

                for ( k = 0 ; k < dim ; k++ )
                {
                    mtr[j * dim + k] -= double (diviseur) * ( mtr[i * dim + k] ) ;
                    melange[j * dim + k] -= double (diviseur) * ( melange[i * dim + k] ) ;
                }
            }
        }

        /* copie des lignes de la matrice inverse m�lang�e � la bonne place dans la matrice
        inverse finale : ligne i de m�lange = ligne *(memoire+i) de la matrice inverse  */

        for ( i = 0 ; i < dim ; i++ ) /* on traite la ligne i */
        {
            for ( j = 0 ; j < dim ; j++ )
            {
                mat_inv[*( memoire + i ) * dim + j] = melange[i * dim + j] ;
            }

        }

        free (mtr) ;
        free (melange) ;
        free (memoire) ;

        return (determinant ) ;
    }

    void choldc1 (double (*a) , double (*p) , unsigned int M)
    {
        int i , j , k ;
        int n = M ;
        double sum ;

        for ( i = 0 ; i < n ; i++ )
        {
            for ( j = i ; j < n ; j++ )
            {
                sum = a[i * M + j] ;
                for ( k = i - 1 ; k >= 0 ; k-- )
                {
                    sum -= a[i * M + k] * a[j * M + k] ;
                }
                if ( i == j )
                {
                    if ( sum <= 0 )
                    {
                        printf (" a is not positive definite!\n") ;
                    }
                    p[i] = sqrt (sum) ;
                }
                else
                {
                    a[j * M + i] = sum / p[i] ;
                }
            }
        }
    }

    void choldcsl (const double (*A) , double (*a) , unsigned int M)
    {
        int i , j , k ;
        int n = M ;
        double sum ;
        double *p ;

        p = ( double* ) malloc (M * sizeof (double )) ;

        for ( i = 0 ; i < n ; i++ )
            for ( j = 0 ; j < n ; j++ )
                a[i * M + j] = A[i * M + j] ;

        choldc1 (a , p , M) ;

        for ( i = 0 ; i < n ; i++ )
        {
            a[i * M + i] = 1 / p[i] ;
            for ( j = i + 1 ; j < n ; j++ )
            {
                sum = 0 ;
                for ( k = i ; k < j ; k++ )
                {
                    sum -= a[j * M + k] * a[k * M + i] ;
                }
                a[j * M + i] = sum / p[j] ;
            }
        }

        free (p) ;
    }

    double inverse_matrice_cholesky ( const double (*mat) ,   double (*mat_inv) , unsigned int M) /* d�termine la matrice inverse d'une matrice carr�e non nulle par la m�thode du pivot*/
    {
        /* matrice de d�part de travail et matrice inverse m�lang�e */
        unsigned int i , j , k ;
        double  d = 1.0 ;
        double *a ;

        choldcsl (mat , mat_inv , M) ;

        a = mat_inv ;

        for ( i = 0 ; i < M ; i++ )
        {
            for ( j = i + 1 ; j < M ; j++ )
            {
                a[i * M + j] = 0.0 ;
            }
        }

        for ( i = 0 ; i < M ; i++ )
        {
            a[i * M + i] *= a[i * M + i] ;
            for ( k = i + 1 ; k < M ; k++ )
            {
                a[i * M + i] += a[k * M + i] * a[k * M + i] ;
            }
            for ( j = i + 1 ; j < M ; j++ )
            {
                for ( k = j ; k < M ; k++ )
                {
                    a[i * M + j] += a[k * M + i] * a[k * M + j] ;
                }
            }
        }
        for ( i = 0 ; i < M ; i++ )
        {
            for ( j = 0 ; j < i ; j++ )
            {
                a[i * M + j] = a[j * M + i] ;
            }
        }
        return (d ) ;
    }

}

//#endif // MATRIX_CPP
