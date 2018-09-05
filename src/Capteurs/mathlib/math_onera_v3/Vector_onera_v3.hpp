
#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <math.h>
// EB_ONERA
//typedef double real ;


namespace math {

    template <unsigned int N>
    class  Vector
    {
      public:
        /**
         * vector data
         */
        //	double *data;
        double data[N] ;

        /**
         * trivial ctor
         * note that this ctor will not initialize elements
         */
        Vector ()
        {
            //        data = new double[N];
        }

        ~ Vector ()
        {
            //        delete [] data;
        }

        /**
         * copy ctor
         */
        Vector (const Vector<N> &v)
        {
            //        data = new double[N];
            memcpy (data , v.data , N * sizeof (double)) ;
        }

        /**
         * setting ctor
         */
        Vector (const double d)
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] = d ;
        }

        Vector (const double d[N])
        {
            //        data = new double[N];
            //        printf("Vector sizeof %d %d\n", sizeof(double), N);
            memcpy (data , d , N * sizeof (double)) ;
        }

        Vector (const double x , const double y)
        {
            //        data = new double[2];
            data[0] = x ;
            data[1] = y ;
        }

        Vector (const double x , const double y , const double z)
        {
            //        data = new double[3];
            data[0] = x ;
            data[1] = y ;
            data[2] = z ;
        }

        Vector (const double x0 , const double x1 , const double x2 , const double x3)
        {
            //		data = new double[4];
            data[0] = x0 ;
            data[1] = x1 ;
            data[2] = x2 ;
            data[3] = x3 ;
        }

        double operator % (const Vector<2> &v) const
        {
            return data[0] * v.data[1] - data[1] * v.data[0] ;
        }

        Vector<3> operator % (const Vector<3> &v) const
        {
            return Vector<3>(
                    data[1] * v.data[2] - data[2] * v.data[1] ,
                    data[2] * v.data[0] - data[0] * v.data[2] ,
                    data[0] * v.data[1] - data[1] * v.data[0]
                    ) ;
        }

        /**
         * set data
         */
        void set (const double d[N])
        {
            //        printf("Vector set\n");
            memcpy (data , d , N * sizeof (double)) ;
        }

        /**
         * access to elements by index
         */
        double &operator () (const unsigned int i)
        {
            return data[i - 1] ;
        }

        /**
         * access to elements by index
         */
        double operator () (const unsigned int i) const
        {
            return data[i - 1] ;
        }

        /**
         * get vector size
         */
        unsigned int get_size () const
        {
            return N ;
        }

        /**
         * test for equality
         */
        bool operator == (const Vector<N> &v) const
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                if (data[i] != v.data[i])
                    return false ;

            return true ;
        }

        /**
         * test for inequality
         */
        bool operator != (const Vector<N> &v) const
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                if (data[i] != v.data[i])
                    return true ;

            return false ;
        }

        /**
         * set to value
         */

        const Vector<N> &operator = (const Vector<N> &v)
        {
            //        printf("Vector=\n");

            memcpy (data , v.data , N * sizeof (double)) ;

            return *this ;
        }

        /**
         * negation
         */
        const Vector<N> operator - (void) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = - data[i] ;

            return res ;
        }

        /**
         * addition
         */
        const Vector<N> operator + (const Vector<N> &v) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = data[i] + v.data[i] ;

            return res ;
        }

        /**
         * subtraction
         */
        const Vector<N> operator - (const Vector<N> &v) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = data[i] - v.data[i] ;

            return res ;
        }

        /**
         * uniform scaling
         */
        const Vector<N> operator * (const double num) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = data[i] * num ;

            return res ;
        }

        /**
         * uniform scaling
         */
        const Vector<N> operator / (const double num) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = data[i] / num ;

            return res ;
        }

        /**
         * addition
         */
        const Vector<N> &operator += (const Vector<N> &v)
        {
            //        printf("Vector+=\n");
            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] += v.data[i] ;

            return *this ;
        }

        /**
         * subtraction
         */
        const Vector<N> &operator -= (const Vector<N> &v)
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] -= v.data[i] ;

            return *this ;
        }

        /**
         * uniform scaling
         */
        const Vector<N> &operator *= (const double num)
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] *= num ;

            return *this ;
        }

        /**
         * uniform scaling
         */
        const Vector<N> &operator /= (const double num)
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] /= num ;

            return *this ;
        }

        /**
         * dot product
         */
        double operator * (const Vector<N> &v) const
        {
            double res = 0.0f ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res += data[i] * v.data[i] ;

            return res ;
        }

        /**
         * element by element multiplication
         */
        const Vector<N> emult (const Vector<N> &v) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = data[i] * v.data[i] ;

            return res ;
        }

        /**
         * element by element division
         */
        const Vector<N> edivide (const Vector<N> &v) const
        {
            Vector<N> res ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res.data[i] = data[i] / v.data[i] ;

            return res ;
        }

        /**
         * gets the length of this vector squared
         */
        double length_squared () const
        {
            double res = 0.0f ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res += data[i] * data[i] ;

            return res ;
        }

        /**
         * gets the length of this vector
         */
        double length () const
        {
            double res = 0.0f ;

            for (unsigned int i = 0 ; i < N ; i ++)
                res += data[i] * data[i] ;

            return sqrtf (res) ;
        }

        /**
         * normalizes this vector
         */
        void normalize ()
        {
            *this /= length () ;
        }

        /**
         * returns the normalized version of this vector
         */
        Vector<N> normalized () const
        {
            return *this / length () ;
        }

        /**
         * set zero vector
         */
        void zero (void)
        {
            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] = 0.0 ;
        }

        template <unsigned int P>
        void extr (Vector<P> &v , int j)
        {

            for (unsigned int i = 0 ; i < N ; i ++)
                data[i] = v.data[i + j - 1] ;
        }

        template <unsigned int P>
        void insr (Vector<P> &v , int j)
        {

            for (unsigned int i = 0 ; i < P ; i ++)
                data[i + j - 1] = v.data[i] ;
        }

        void print (const char *str)
        {
            printf ("%s\n[ " , str) ;

            for (unsigned int i = 0 ; i < N ; i ++)
                printf ("%.4f\t" , data[i]) ;

            printf ("]\n") ;
        }

        //    friend Vector<N> operator *(const Matrixs<N>&, const Vector<N>&);
    } ;


}

#endif // VECTOR_HPP

