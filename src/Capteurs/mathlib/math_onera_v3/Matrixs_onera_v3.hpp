
#ifndef MATRIXS_HPP
#define MATRIXS_HPP
#include <unistd.h>
#include <math.h>
#include <string.h>
//#include <stdio.h>
#include <iostream>

#define EPS 1.E-06
using namespace std ;
namespace math {


    // MxN matrix with double elements

    template <unsigned int M>
    class  Matrixs
    {
      public:
        /**
         * matrix data[row*col]
         */
        //	double *data;
        double data[M*(M + 1) / 2] ;

        /**
         * trivial ctor
         * note that this ctor will not initialize elements
         */
        Matrixs ()
        {
            //        printf("Constructor\n");
            //        data = new double[M*N];
        }

        ~ Matrixs ()
        {
            //        delete [] data;
        }

        /**
         * copyt ctor
         */
        // 	Matrix(const Matrix<M, N> &m) {
        // //        data = new double[M*N];
        // 		memcpy(data, m.data, M*N*sizeof(double));
        // 	}

        Matrixs (const double *d)
        {
            //        data = new double[M*N];
            memcpy (data , d , M * (M + 1) / 2 * sizeof (double)) ;
        }

        /**
         * set data
         */
        void set (const double *d)
        {
            memcpy (data , d , M * (M + 1) / 2 * sizeof (double)) ;
        }

        /**
         * access by index
         */
        double &operator () (const unsigned int row , const unsigned int col)
        {
            unsigned int m , n ;
            if ( col < row )
            {
                n = row ;
                m = col ;
            }
            else
            {
                m = row ;
                n = col ;
            }

            return data[(m - 1) * M - (m - 1)*(m - 2) / 2 + n - m] ;
        }

        /**
         * access by index
         */
        double operator () (const unsigned int row , const unsigned int col) const
        {
            unsigned int m , n ;
            if ( col < row )
            {
                n = row ;
                m = col ;
            }
            else
            {
                m = row ;
                n = col ;
            }
            return data[(m - 1) * M - (m - 1)*(m - 2) / 2 + n - m] ;
        }

        /**
         * get rows number
         */
        unsigned int get_rows () const
        {
            return M ;
        }

        /**
         * get columns number
         */
        unsigned int get_cols () const
        {
            return M ;
        }

        /**
         * test for equality
         */
        // 	bool operator ==(const Matrix<M, N> &m) const {
        // 		for (unsigned int i = 0; i < M*N; i++)
        // 				if (data[i] != m.data[i])
        // 					return false;
        // 
        // 		return true;
        // 	}

        /**
         * test for inequality
         */
        // 	bool operator !=(const Matrix<M, N> &m) const {
        // 		for (unsigned int i = 0; i < M*N; i++)
        // 				if (data[i] != m.data[i])
        // 					return true;
        // 
        // 		return false;
        // 	}

        /**
         * set to value
         */
        Matrixs<M> &operator = (const Matrixs<M> &m)
        {
            //		printf("Matrix=\n");
            memcpy (data , m.data , M * (M + 1) / 2 * sizeof (double)) ;

            return *this ;
        }

        /**
         * negation
         */
        Matrixs<M> operator - (void) const
        {
            Matrixs<M> res ;

            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                res.data[i] = - data[i] ;

            return res ;
        }

        /**
         * addition
         */
        Matrixs<M> operator + (const Matrixs<M> &m) const
        {
            Matrixs<M> res ;

            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                res.data[i] = data[i] + m.data[i] ;

            return res ;
        }

        Matrixs<M> &operator += (const Matrixs<M> &m)
        {
            //		 printf("Matrix+=\n");
            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                data[i] += m.data[i] ;

            return *this ;
        }

        /**
         * subtraction
         */
        Matrixs<M> operator - (const Matrixs<M> &m) const
        {
            Matrixs<M> res ;

            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                res.data[i] = data[i] - m.data[i] ;

            return res ;
        }

        Matrixs<M> &operator -= (const Matrixs<M> &m)
        {
            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                data[i] -= m.data[i] ;

            return *this ;
        }

        /**
         * uniform scaling
         */
        Matrixs<M> operator * (const double num) const
        {
            Matrixs<M> res ;

            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                res.data[i] = data[i] * num ;

            return res ;
        }

        Matrixs<M> &operator *= (const double num)
        {
            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                data[i] *= num ;

            return *this ;
        }

        Matrixs<M> operator / (const double num) const
        {
            Matrixs<M> res ;

            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                res[i] = data[i] / num ;

            return res ;
        }

        Matrixs<M> &operator /= (const double num)
        {
            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                data[i] /= num ;

            return *this ;
        }

        /**
         * multiplication by another matrix
         */
        //	template <unsigned int P>
        //	Matrix<M, P> operator *(const Matrix<N, P> &m) const {
        //		Matrix<M, P> res;
        //        unsigned int i, j, k;
        //        
        //        for (i=0; i<M ; i++) {
        //            for (j=0; j<P ; j++) {
        //                res.data[i*P+j] = 0.0f;
        //                for (k=0; k<N ; k++) 
        //                    res.data[i*P+j] = res.data[i*P+j] + data[i*N+k] * m.data [k*P+j];
        //            }
        //        }
        //		return res;
        //	}

        //     template <unsigned int P>
        // 	Matrix<M, P> operator *(  Matrix<N, P> &m)  {
        // 		Matrix<M, P> res;
        //         
        //         register int i = M , j, k ;
        //         register double val ;
        //         register double *ptrm = res.data ;
        //         register double *ptr1;
        //         register double *ptr2;
        //         register double *ptr1_base = data ;
        //         register double *ptr2_base = m.data ;
        //         
        //             while( i-- ){
        //             j = P ;
        //             while( j-- ){ 
        //                 ptr1 = ptr1_base ;
        //                 ptr2 = ptr2_base++ ;
        //                 k = N ;
        //                 val = 0. ;
        //                 while( k-- ) {
        //                 val += (*ptr1++)*(*ptr2) ;
        //                  ptr2 += P ;
        //                 }
        //                 *ptrm++ = val  ;
        // 
        //             }
        //             ptr1_base += N ;
        //             ptr2_base = m.data ;
        //             }
        //             return res;      
        // 	}

        //     template <unsigned int M, unsigned int N>
        // 	Matrix<M, N> operator *(  Matrixs<N> &m)  {
        // 		Matrix<M, N> res;
        //         
        //         register int i = M , j, k ;
        //         register double val ;
        //         register double *ptrm = res.data ;
        //         register double *ptr1;
        //         register double *ptr2;
        //         register double *ptr1_base = data ;
        //         register double *ptr2_base = m.data ;
        // 
        //         while( i-- ){
        //             for( j = 1; j<= N ; j++ ){
        //                 ptr1 = ptr1_base ;
        //                 ptr2 = ptr2_base++ ;
        //                 val = 0. ;
        //                 for( k=1 ; k< j ; k++ ){
        //                 val += (*ptr1++)*(*ptr2) ;
        //                 ptr2 += N - k  ;
        //                 }
        //                 for(k=j ; k<= N ; k++ ){
        //                 val += (*ptr1++)*(*ptr2++ ) ;
        //                 }
        //                 *ptrm++ = val  ;
        // 
        //             }
        //                 ptr1_base += N  ;
        //                 ptr2_base = m.data ;
        //             }
        //         return res;      
        // 	}

        /**
         * set zero matrix
         */
        void zero (void)
        {
            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                data[i] = 0.0 ;
        }

        /**
         * set identity matrix
         */
        void identity (void)
        {
            for (unsigned int i = 0 ; i < M * (M + 1) / 2 ; i ++)
                data[i] = 0.0 ;

            register int i = M , jcol = M ;
            register double *ptr = data ;
            while (i --)
            {
                *ptr = 1. ;
                ptr += jcol -- ;
            }
        }


        //     template <unsigned int P, unsigned int Q>
        //     void extr(Matrix<P,Q> &m, int i0, int j0)  {
        // 
        //         for (unsigned int i = 0; i < M; i++)
        //             for (unsigned int j = 0; j < N; j++)
        //                 data[i*N+j] = m.data[(i+i0-1)*Q+j+j0-1];
        //     }
        //     
        //     template <unsigned int P, unsigned int Q>
        //     void insr(Matrix<P,Q> &m, int i0, int j0)  {
        // 
        //         for (unsigned int i = 0; i < P; i++)
        //                 for (unsigned int j = 0; j < Q; j++)
        //                     data[(i+i0-1)*N+j+j0-1] = m.data[i*Q+j];
        //     }

        template <unsigned int P>
        void insrdiag (Vector<P> &v , int i0)
        {

            for (unsigned int i = 0 ; i < P ; i ++)
                data[(i + i0 - 1) * M - (i + i0 - 1)*(i + i0 - 2) / 2] = v.data[i] ;

            //         register int i = P ;
            //         register int jcol = M ;
            //         register double *ptrd = data ;
            //         register double *ptrv = v.data ;
            //             while( i-- ){ 
            //                 *ptrd = *ptrv++ ;
            //                  ptrd += jcol-- ;
            //             }
        }

        template <unsigned int P>
        void insrdiag2 (Vector<P> &v , int i0)
        {

            for (unsigned int i = 0 ; i < P ; i ++)
                data[(i + i0 - 1) * M - (i + i0 - 1)*(i + i0 - 2) / 2] = v.data[i] * v.data[i] ;

            //         register int i = P ;
            //         register int jcol = M ;
            //         register double *ptrd = data ;
            //         register double *ptrv = v.data ;
            //             while( i-- ){ 
            //                 *ptrd = *ptrv * *ptrv ;
            //                 ptrv++;
            //                  ptrd += jcol-- ;
            //             }
        }

        void print (const char *str)
        {

            FILE *pFile ;
            pFile = fopen ("/media/clef/matrixs.txt" , "w") ;
            fflush (stdin) ;
            fflush (stdout) ;



            printf ("%s\n " , str) ;
            printf ("[") ;
            fprintf (pFile , "[") ;
            for (unsigned int i = 0 ; i < M ; i ++)
            {
                for (unsigned int j = 0 ; j < i ; j ++)
                {
                    printf ("\t%.4f" , data[(j) * M - j * (j - 1) / 2 + i - j]) ;
                    fprintf (pFile , "\t%.4f " , data[(j) * M - j * (j - 1) / 2 + i - j]) ;
                }
                for (unsigned int j = i ; j < M ; j ++)
                {
                    printf ("\t%.4f" , data[(i) * M - i * (i - 1) / 2 + j - i]) ;
                    fprintf (pFile , "\t%.4f " , data[(i) * M - i * (i - 1) / 2 + j - i]) ;

                }
                fprintf (pFile , ";") ;
                printf ("\n") ;
            }

            printf (" ]\n") ;
            fprintf (pFile , "]") ;
            fclose (pFile) ; // je referme le fichier
        }

        /**
         * multiplication by a vector
         */
        //    friend Vector<M> operator *(const Matrixs<M> &m, const Vector<M> &v);

        Vector<M> operator * (const Vector<M> &v) const
        {
            Vector<M> res ;

            register int i , k ;
            register double val ;
            register double *ptrm = res.data ;
            const double *ptr1 , *ptr1_base = data ;
            const double *ptr2 , *ptr2_base = v.data ;
            for ( i = 1 ; i <= int(M) ; i ++ )
            {
                ptr1 = ptr1_base ;
                ptr2 = ptr2_base ;
                val = 0. ;
                for ( k = 1 ; k < i ; k ++ )
                {
                    val += (*ptr1)*(*ptr2) ;
                    ptr1 += M - k ;
                    ptr2 ++ ;
                }
                for (k = i ; k <= int(M) ; k ++ )
                {
                    val += (*ptr1 ++)*(*ptr2 ) ;
                    ptr2 ++ ;
                }
                *ptrm ++ = val  ;

                ptr1_base ++  ;
            }

            return res ;
        }

        /**
         * invert the matrix
         */
        Matrixs<M> inversc (void)
        {
            Matrixs<M> res = * this ;

            register int i , j , k ;
            double inf = * res.data , sup = 0.0 ;
            register double sa ;

            register int colbase = 1 ;
            register int colc ;
            register double *ptrjbase = res.data , *ptrj ;
            register double *ptribase = res.data , *ptri ;

            //  printf ("inversc\n") ;

            Matrixs<M> m = * this ;

            //m.print ("M") ;

            // extraction de la racine 
            for (j = 1 ; j <= int(M) ; j ++ , ptrjbase ++ )
            {
                ptribase = res.data ;
                for (i = 1 ; i <= j - 1 ; i ++ , ptribase ++ )
                {
                    ptri = ptribase ;
                    for (k = 1 , sa = 0. , ptrj = ptrjbase , colc = M - 1 ;
                            k <= i - 1 ; k ++ )
                    {
                        sa += * ptri * *ptrj ;
                        ptri += colc ;
                        ptrj += colc -- ;
                    }
                    *ptrj = (*ptrj - sa) / * ptri ;
                }

                for (k = 1 , sa = 0. , ptrj = ptrjbase , colc = M - 1  ;
                        k <= j - 1 ; k ++)
                {
                    sa += * ptrj * *ptrj ;
                    ptrj += colc --  ;
                }

                sa = * ptrj - sa ;

                if (sa > sup) sup = sa ;
                if (sa < inf) inf = sa ;
                if ((inf - EPS * sup) < 0)
                {
                    print ("inversc : erreur") ;
                    return 0 ;
                }
                *ptrj = sqrt ( sa ) ;
            }
            // inversion de la racine
            ptrjbase = res.data + M * (M + 1) / 2 - 1 ;

            for (j = M ; j >= 1 ; j --)
            {
                colbase =  M - j + 1  ; // passsage de (i,j) a (i,j-1)

                ptrjbase =  & res.data[(j - 1) * M - (j - 1)*(j - 2) / 2] ; // diagonale (j,j)

                ptrj = ptrjbase - colbase ; // pointeur (j-1, j)
                ptribase =  ptrj - 1 ; //  diagonale ( j-1, j-1 )

                *ptrjbase = 1. / * ptrjbase ; // diagonale (j,j)

                for (i = j - 1 ; i >= 1 ; i --)
                {
                    ptri = ptribase ; // diagonale (i,i)
                    for (k = j , sa = 0. , colc = colbase ;  k >= i + 1 ; k -- , colc -- )
                    { // ici ptrj = a(i,j) et ptri = a(i,i)
                        ++ ptri ;
                        ptrj += colc ;
                        sa += * ptri * *ptrj ;
                    } // ICI : ptrj = a(j,j)  et ptri = a(i,j)

                    *ptri = - sa / * ptribase  ; // a(i,j)

                    colbase ++ ;
                    ptribase -= (colbase + 1) ;
                    ptrj = ptri - colbase  ; // point suivant :(i-1, j)
                }
            }

            // inverse de la matrice
            ptribase =  res.data ;
            for (i = 1 ; i <= int(M) ; i ++)
            {
                ptrj = ptribase ; // (i,i)
                for (j = i ; j <= int(M) ; j ++ )
                {
                    ptri = ptribase ; // (i,j)
                    for ( k = j , sa = 0.  ; k <= int(M) ; k ++ )
                        sa += * ptri ++ * *ptrj ++ ;
                    *ptribase ++ = sa ;
                }
            }


            //res.print("RES");

            return res ;
        }

        Matrixs<M> invers3 (void) const
        {
            Matrixs<M> res ;

            if (M != 3)
            {
                printf ("Matrix size must be 3x3") ;
                res.zero () ;
                return res ;
            }

            const double *mat = data ;
            double *mat_inv = res.data ;
            double d ;

            //  Calcul du D�terminant
            d = mat[0] * mat[3] * mat[5] + 2 * mat[1] * mat[4] * mat[2] - mat[0] * mat[4] * mat[4] - mat[3] * mat[2] * mat[2] - mat[5] * mat[1] * mat[1] ;

            // D�terminant =0 = non inversible
            if (d == 0) ;

            // Calcul de l'inverse par les commatrices
            mat_inv[0] = mat[3] * mat[5] - mat[4] * mat[4] ;
            mat_inv[1] = mat[4] * mat[2] - mat[1] * mat[5] ;
            mat_inv[2] = mat[1] * mat[4] - mat[3] * mat[2] ;

            mat_inv[3] = mat[0] * mat[5] - mat[2] * mat[2] ;
            mat_inv[4] = mat[1] * mat[2] - mat[0] * mat[4] ;

            mat_inv[5] = mat[0] * mat[3] - mat[1] * mat[1] ;

            for (int i = 0 ; i < 6 ; i ++)
                mat_inv[i] /= d ;

            return res ;
        }


        //     friend Matrix operator *( Matrix&,  Matrixs&);    
        //     friend Matrix operator *( Matrixs&,  Matrix&);

    } ;

    //     template <unsigned int M>
    // 	Vector<M> Matrixs<M>::operator *(const Vector<M> &v) const {
    // 		Vector<M> res;
    //         
    //         register int i, k ;
    //         register double val ;
    //         register double *ptrm = res.data ;
    //         const double *ptr1 , *ptr1_base = data ;
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


}

#endif // MATRIXS_HPP

