
#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <stdio.h>
#include <unistd.h>
#include <string.h>
// EB_ONERA
#define M_PI_2_F  (3.14159265358979f/2.0f)

#include "Vector_onera_v3.hpp"

namespace math {

    double inverse_matrice_pivot ( const double* , double* , unsigned int) ;
    double inverse_matrice_cholesky ( const double* , double* , unsigned int) ;

    // MxN matrix with double elements

    template <unsigned int M , unsigned int N>
    class Matrix
    {
      public:
        /**
         * matrix data[row*col]
         */
        //	double *data;
        double data[M*N] ;

        /**
         * trivial ctor
         * note that this ctor will not initialize elements
         */
        Matrix ()
        {
            //        printf("Constructor\n");
            //        data = new double[M*N];
        }

        ~ Matrix ()
        {
            //        delete [] data;
        }

        /**
         * copyt ctor
         */
        Matrix (const Matrix<M , N> &m)
        {
            //        data = new double[M*N];
            memcpy (data , m.data , M * N * sizeof (double)) ;
        }

        Matrix (const double *d)
        {
            //        data = new double[M*N];
            memcpy (data , d , M * N * sizeof (double)) ;
        }

        Matrix (const double d[M][N])
        {
            //        data = new double[M*N];
            memcpy (data , d , M * N * sizeof (double)) ;
        }

        /**
         * set data
         */
        void set (const double *d)
        {
            memcpy (data , d , M * N * sizeof (double)) ;
        }

        /**
         * set data
         */
        void set (const double d[M][N])
        {
            memcpy (data , d , M * N * sizeof (double)) ;
        }

        /**
         * access by index
         */
        double &operator () (const unsigned int row , const unsigned int col)
        {
            return data[(row - 1) * N + col - 1] ;
        }

        /**
         * access by index
         */
        double operator () (const unsigned int row , const unsigned int col) const
        {
            return data[(row - 1) * N + col - 1] ;
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
            return N ;
        }

        /**
         * test for equality
         */
        bool operator == (const Matrix<M , N> &m) const
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                if (data[i] != m.data[i])
                    return false ;

            return true ;
        }

        /**
         * test for inequality
         */
        bool operator != (const Matrix<M , N> &m) const
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                if (data[i] != m.data[i])
                    return true ;

            return false ;
        }

        /**
         * set to value
         */
        Matrix<M , N> &operator = (const Matrix<M , N> &m)
        {
            //		printf("Matrix=\n");
            memcpy (data , m.data , M * N * sizeof (double)) ;

            return *this ;
        }

        /**
         * negation
         */
        Matrix<M , N> operator - (void) const
        {
            Matrix<M , N> res ;

            for (unsigned int i = 0 ; i < M * N ; i ++)
                res.data[i] = - data[i] ;

            return res ;
        }

        /**
         * addition
         */
        Matrix<M , N> operator + (const Matrix<M , N> &m) const
        {
            Matrix<M , N> res ;

            for (unsigned int i = 0 ; i < M * N ; i ++)
                res.data[i] = data[i] + m.data[i] ;

            return res ;
        }

        Matrix<M , N> &operator += (const Matrix<M , N> &m)
        {
            //		 printf("Matrix+=\n");
            for (unsigned int i = 0 ; i < M * N ; i ++)
                data[i] += m.data[i] ;

            return *this ;
        }

        /**
         * subtraction
         */
        Matrix<M , N> operator - (const Matrix<M , N> &m) const
        {
            Matrix<M , N> res ;

            for (unsigned int i = 0 ; i < M * N ; i ++)
                res.data[i] = data[i] - m.data[i] ;

            return res ;
        }

        Matrix<M , N> &operator -= (const Matrix<M , N> &m)
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                data[i] -= m.data[i] ;

            return *this ;
        }

        /**
         * uniform scaling
         */
        Matrix<M , N> operator * (const double num) const
        {
            Matrix<M , N> res ;

            for (unsigned int i = 0 ; i < M * N ; i ++)
                res.data[i] = data[i] * num ;

            return res ;
        }

        Matrix<M , N> &operator *= (const double num)
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                data[i] *= num ;

            return *this ;
        }

        Matrix<M , N> operator / (const double num) const
        {
            Matrix<M , N> res ;

            for (unsigned int i = 0 ; i < M * N ; i ++)
                res[i] = data[i] / num ;

            return res ;
        }

        Matrix<M , N> &operator /= (const double num)
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                data[i] /= num ;

            return *this ;
        }

        /**
         * multiplication by another matrix
         */

                /*	template <unsigned int P>
                Matrix<M, P> operator *(const Matrix<N, P> &m) const {
                        Matrix<M, P> res;
                unsigned int i, j, k;

                for (i=0; i<M ; i++) {
                    for (j=0; j<P ; j++) {
                        res.data[i*P+j] = 0.0f;
                        for (k=0; k<N ; k++)
                            res.data[i*P+j] = res.data[i*P+j] + data[i*N+k] * m.data [k*P+j];
                    }
                }
                        return res;
                }
         */
        template <unsigned int P>
        Matrix<M , P> operator * (const Matrix<N , P> &m) const
        {
            Matrix<M , P> res ;

            register int i = M , j , k ;
            register double val ;
            register double *ptrm = res.data ;
            const double *ptr1 ;
            const double *ptr2 ;
            const double *ptr1_base = data ;
            const double *ptr2_base = m.data ;

            while ( i -- )
            {
                j = P ;
                while ( j -- )
                {
                    ptr1 = ptr1_base ;
                    ptr2 = ptr2_base ++ ;
                    k = N ;
                    val = 0. ;
                    while ( k -- )
                    {
                        val += (*ptr1 ++)*(*ptr2) ;
                        ptr2 += P ;
                    }
                    *ptrm ++ = val  ;

                }
                ptr1_base += N ;
                ptr2_base = m.data ;
            }
            return res ;
        }

        /**
         * transpose the matrix
         */
        Matrix<N , M> transposed (void) const
        {
            Matrix<N , M> res ;
            unsigned int i , j ;

            for (i = 0 ; i < N ; i ++)
                for (j = 0 ; j < M ; j ++)
                    res.data[i * M + j] = data[j * N + i] ;

            return res ;
        }

        /**
         * invert the matrix
         */
        Matrix<N , M> invers (void)
        {
            Matrix<N , M> res ;
            double d ;

            d = inverse_matrice_pivot (this->data , res.data , M) ;
            return res ;
        }

        /**
         * invert the matrix
         */
        Matrix<N , M> inversc (void)
        {
            Matrix<N , M> res ;
            double d ;

            d = inverse_matrice_cholesky (this->data , res.data , M) ;
            return res ;
        }

        Matrix<M , N> invers3 (void) const
        {
            Matrix<M , N> res ;

            if (M != 3 || N != 3)
            {
                printf ("Matrix size must be 3x3") ;
                res.zero () ;
                return res ;
            }

            const double *mat = data ;
            double *mat_inv = res.data ;
            double d ;

            //  Calcul du D�terminant
            d = mat[0] * mat[4] * mat[8] - mat[0] * mat[5] * mat[7] - mat[1] * mat[8] * mat[3] + mat[1] * mat[5] * mat[6] + mat[2] * mat[3] * mat[7] - mat[2] * mat[4] * mat[6] ;

            // D�terminant =0 = non inversible
            if (d == 0) ;

            // Calcul de l'inverse par les commatrices
            mat_inv[0] = mat[4] * mat[8] - mat[5] * mat[7] ;
            mat_inv[1] = mat[7] * mat[2] - mat[1] * mat[8] ;
            mat_inv[2] = mat[1] * mat[5] - mat[2] * mat[4] ;

            mat_inv[3] = mat[6] * mat[5] - mat[3] * mat[8] ;
            mat_inv[4] = mat[0] * mat[8] - mat[2] * mat[6] ;
            mat_inv[5] = mat[3] * mat[2] - mat[0] * mat[5] ;

            mat_inv[6] = mat[3] * mat[7] - mat[6] * mat[4] ;
            mat_inv[7] = mat[1] * mat[6] - mat[0] * mat[7] ;
            mat_inv[8] = mat[0] * mat[4] - mat[1] * mat[3] ;

            for (int i = 0 ; i < 9 ; i ++)
                mat_inv[i] /= d ;

            return res ;
        }

        /**
         * set zero matrix
         */
        void zero (void)
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                data[i] = 0.0 ;
        }

        /**
         * set identity matrix
         */
        void identity (void)
        {
            for (unsigned int i = 0 ; i < M * N ; i ++)
                data[i] = 0.0 ;

            unsigned int n = (M < N) ? M : N ;

            for (unsigned int i = 0 ; i < n ; i ++)
                data[i * N + i] = 1 ;
        }

        template <unsigned int P , unsigned int Q>
        void extr (const Matrix<P , Q> &m , int i0 , int j0)
        {

            for (unsigned int i = 0 ; i < M ; i ++)
                for (unsigned int j = 0 ; j < N ; j ++)
                    data[i * N + j] = m.data[(i + i0 - 1) * Q + j + j0 - 1] ;
        }

        template <unsigned int P , unsigned int Q>
        void insr (const Matrix<P , Q> &m , int i0 , int j0)
        {

            for (unsigned int i = 0 ; i < P ; i ++)
                for (unsigned int j = 0 ; j < Q ; j ++)
                    data[(i + i0 - 1) * N + j + j0 - 1] = m.data[i * Q + j] ;
        }

        template <unsigned int P>
        void insrdiag (const Vector<P> &v , int i0)
        {

            for (unsigned int i = 0 ; i < P ; i ++)
                data[(i + i0 - 1) * N + i + i0 - 1] = v.data[i] ;
        }

        template <unsigned int P>
        void insrdiag2 (const Vector<P> &v , int i0)
        {

            for (unsigned int i = 0 ; i < P ; i ++)
                data[(i + i0 - 1) * N + i + i0 - 1] = v.data[i] * v.data[i] ;
        }

        void print (const char *str)
        {
            printf ("%s\n " , str) ;
            printf ("[") ;
            for (unsigned int i = 0 ; i < M ; i ++)
            {
                for (unsigned int j = 0 ; j < N ; j ++)
                    printf ("\t%.4f" , data[i * N + j]) ;

                printf ("\n") ;
            }
            printf (" ]\n") ;
        }

        /**
         * multiplication by a vector
         */
        Vector<M> operator * (const Vector<N> &v) const
        {
            Vector<M> res ;
            unsigned int i , j ;

            for (i = 0 ; i < M ; i ++)
            {
                res.data[i] = 0.0f ;
                for (j = 0 ; j < N ; j ++)
                    res.data[i] = res.data[i] + data[i * N + j] * v.data[j] ;
            }

            return res ;
        }

        /* create a rotation matrix from given euler angles
         * based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
         */
        void from_euler (double roll , double pitch , double yaw)
        {
            double cp = cosf (pitch) ;
            double sp = sinf (pitch) ;
            double sr = sinf (roll) ;
            double cr = cosf (roll) ;
            double sy = sinf (yaw) ;
            double cy = cosf (yaw) ;

            data[0] = cp * cy ;
            data[1] = (sr * sp * cy) - (cr * sy) ;
            data[2] = (cr * sp * cy) + (sr * sy) ;
            data[3] = cp * sy ;
            data[4] = (sr * sp * sy) + (cr * cy) ;
            data[5] = (cr * sp * sy) - (sr * cy) ;
            data[6] = - sp ;
            data[7] = sr * cp ;
            data[8] = cr * cp ;
        }

        /**
         * get euler angles from rotation matrix
         */

        Vector<3> to_euler(void) const { // Je commente car ca ne compile pas à cause de M_PI_2_F inconnu
                Vector<3> euler;
                euler.data[1] = asinf(-data[6]);

                if (fabsf(euler.data[1] - M_PI_2_F) < 1.0e-3f) {
                        euler.data[0] = 0.0f;
                        euler.data[2] = atan2f(data[5] - data[1], data[2] + data[4]) + euler.data[0];

                } else if (fabsf(euler.data[1] + M_PI_2_F) < 1.0e-3f) {
                        euler.data[0] = 0.0f;
                        euler.data[2] = atan2f(data[5] - data[1], data[2] + data[4]) - euler.data[0];

                } else {
                        euler.data[0] = atan2f(data[7], data[8]);
                        euler.data[2] = atan2f(data[3], data[0]);
                }

                return euler;
        }

        void antisym (Vector<3> V)
        {
            memset (data , 0 , sizeof (data)) ;

            data[1] = - V.data[2] ;
            data[3] =  V.data[2] ;
            data[2] =  V.data[1] ;
            data[6] = - V.data[1] ;
            data[5] = - V.data[0] ;
            data[7] =  V.data[0] ;
        }

        //     friend Matrix<M, N> operator *( Matrix<M, N>&,  Matrixs<N>&); 
        //     friend Matrix<M, N> operator *( Matrixs<M>&,  Matrix<M, N>&);

    } ;

}

#endif // MATRIX_HPP

