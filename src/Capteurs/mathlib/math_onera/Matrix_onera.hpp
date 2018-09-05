
#ifndef MATRIX_ONERA_HPP
#define MATRIX_ONERA_HPP

#include <stdio.h>
#include <stdlib.h>
// EB_ONERA
#include "Vector_onera.hpp"

//typedef float real;


namespace math
{

//double inverse_matrice_pivot(real*, real*, unsigned int);
//extern double inverse_matrice_pivot(  real (*mat),  real (*mat_inv), unsigned int M_size);
//double inverse_matrice_cholesky(real*, real*, unsigned int);
//extern double inverse_matrice_cholesky( const real (*mat),   real (*mat_inv), unsigned int M_size);

// MxN matrix with float elements
template <unsigned int M, unsigned int N>
class __EXPORT Matrix
{
public:
	/**
	 * matrix data[row][col]
	 */
//	real *data;
    real data[M*N];

	/**
	 * trivial ctor
	 * note that this ctor will not initialize elements
	 */
    Matrix() {
//        printf("Constructor\n");
//        data = new real[M*N];
	}
    
    ~Matrix() {
//        delete [] data;
    }

	/**
	 * copyt ctor
	 */
	Matrix(const Matrix<M, N> &m) {
//        data = new real[M*N];
		memcpy(data, m.data, M*N*sizeof(real));
	}

	Matrix(const real *d) {
//        data = new real[M*N];
		memcpy(data, d, M*N*sizeof(real));
	}

	Matrix(const real d[M][N]) {
//        data = new real[M*N];
		memcpy(data, d, M*N*sizeof(real));
	}


	// EB_ONERA #########

	//double inverse_matrice_pivot(real*, real*, unsigned int);
	//extern double inverse_matrice_pivot(  real (*mat),  real (*mat_inv), unsigned int M_size);
	//double inverse_matrice_cholesky(real*, real*, unsigned int);
	//extern double inverse_matrice_cholesky( const real (*mat),   real (*mat_inv), unsigned int M_size);


	double inverse_matrice_pivot(  real (*mat),  real (*mat_inv), unsigned int M_size) /* détermine la matrice inverse d'une matrice carrée non nulle par la méthode du pivot*/
	{
		/* matrice de départ de travail et matrice inverse mélangée */
		double determinant = 1.0;
	    int *memoire;
		int i, j, k, indice;

		double pivot, diviseur;
		double max;
	    int dim = M_size;

	    real *mtr, *melange;

	    mtr = (real*) malloc(M_size*M_size*sizeof(real));
	    melange = (real*) malloc(M_size*M_size*sizeof(real));


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


	void choldc1(real (*a), real (*p), unsigned int M_size)
	{
	     int i,j,k;
	     int n = M_size;
	     real sum;

	    for (i = 0; i < n; i++) {
	        for (j = i; j < n; j++) {
	            sum = a[i*M_size+j];
	            for (k = i - 1; k >= 0; k--) {
	                sum -= a[i*M_size+k] * a[j*M_size+k];
	            }
	            if (i == j) {
	                if (sum <= 0) {
	                    printf(" a is not positive definite!\n");
	                }
	                p[i] = sqrt(sum);
	            }
	            else {
	                a[j*M_size+i] = sum / p[i];
	            }
	        }
	    }
	}


	void choldcsl(const real (*A), real (*a), unsigned int M_size)
	{
	    int i,j,k;
	    int n = M_size;
	    real sum;
	    real *p;

	    p = (real*) malloc(M_size*sizeof(real));

	    for (i = 0; i < n; i++)
	        for (j = 0; j < n; j++)
	            a[i*M_size+j] = A[i*M_size+j];

	    choldc1(a, p, M_size);

	    for (i = 0; i < n; i++) {
	        a[i*M_size+i] = 1 / p[i];
	        for (j = i + 1; j < n; j++) {
	            sum = 0;
	            for (k = i; k < j; k++) {
	                sum -= a[j*M_size+k] * a[k*M_size+i];
	            }
	            a[j*M_size+i] = sum / p[j];
	        }
	    }

	    free(p);
	}


	double inverse_matrice_cholesky( const real (*mat),   real (*mat_inv), unsigned int M_size) /* détermine la matrice inverse d'une matrice carrée non nulle par la méthode du pivot*/
	{
		/* matrice de départ de travail et matrice inverse mélangée */
		unsigned int i, j, k;
		double  d = 1.0;
	    real *a;

	    choldcsl(mat, mat_inv, M_size);

	   a = mat_inv;

	    for (i = 0; i < M_size; i++) {
	        for (j = i + 1; j < M_size; j++) {
	            a[i*M_size+j] = 0.0;
	        }
	    }

	    for (i = 0; i < M_size; i++) {
	        a[i*M_size+i] *= a[i*M_size+i];
	        for (k = i + 1; k < M_size; k++) {
	            a[i*M_size+i] += a[k*M_size+i] * a[k*M_size+i];
	        }
	        for (j = i + 1; j < M_size; j++) {
	            for (k = j; k < M_size; k++) {
	                a[i*M_size+j] += a[k*M_size+i] * a[k*M_size+j];
	            }
	        }
	    }
	    for (i = 0; i < M_size; i++) {
	        for (j = 0; j < i; j++) {
	            a[i*M_size+j] = a[j*M_size+i];
	        }
	    }
	    return (d);
	}




	// EB_ONERA ####END####
	/**
	 * set data
	 */
	void set(const real *d) {
		memcpy(data, d, M*N*sizeof(real));
	}

	/**
	 * set data
	 */
	void set(const real d[M][N]) {
		memcpy(data, d, M*N*sizeof(real));
	}

	/**
	 * access by index
	 */
	real &operator()(const unsigned int row, const unsigned int col) {
		return data[(row-1)*N+col-1];
	}

	/**
	 * access by index
	 */
	real operator()(const unsigned int row, const unsigned int col) const {
		return data[(row-1)*N+col-1];
	}

	/**
	 * get rows number
	 */
	unsigned int get_rows() const {
		return M;
	}

	/**
	 * get columns number
	 */
	unsigned int get_cols() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M*N; i++)
				if (data[i] != m.data[i])
					return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M*N; i++)
				if (data[i] != m.data[i])
					return true;

		return false;
	}

	/**
	 * set to value
	 */
    Matrix<M, N> &operator =(const Matrix<M, N> &m) {
//		printf("Matrix=\n");
        memcpy(data, m.data, M*N*sizeof(real));
        
//        for (unsigned int i = 0; i < M*N; i++)
//				data[i] = m.data[i];

        return *this;
	}

    
	/**
	 * negation
	 */
	Matrix<M, N> operator -(void) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M*N; i++)
				res.data[i] = -data[i];

		return res;
	}

	/**
	 * addition
	 */
	Matrix<M, N> operator +(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M*N; i++)
				res.data[i] = data[i] + m.data[i];

		return res;
	}

	Matrix<M, N> &operator +=(const Matrix<M, N> &m) {
//		 printf("Matrix+=\n");
        for (unsigned int i = 0; i < M*N; i++)
				data[i] += m.data[i];

       return *this;
	}

	/**
	 * subtraction
	 */
	Matrix<M, N> operator -(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M*N; i++)
				res.data[i] = data[i] - m.data[i];

		return res;
	}

	Matrix<M, N> &operator -=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M*N; i++)
				data[i] -= m.data[i];

        return *this;
	}

	/**
	 * uniform scaling
	 */
	Matrix<M, N> operator *(const real num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M*N; i++)
				res.data[i] = data[i] * num;

		return res;
	}

	Matrix<M, N> &operator *=(const real num) {
		for (unsigned int i = 0; i < M*N; i++)
				data[i] *= num;

        return *this;
	}

	Matrix<M, N> operator /(const real num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M*N; i++)
				res[i] = data[i] / num;

		return res;
	}

	Matrix<M, N> &operator /=(const real num) {
		for (unsigned int i = 0; i < M*N; i++)
				data[i] /= num;

        return *this;      
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
	Matrix<M, P> operator *(const Matrix<N, P> &m) const  {
		Matrix<M, P> res;
        
        register int i = M , j, k ;
        register real val ;
        register real *ptrm = res.data ;
        const real *ptr1;
        const real *ptr2;
        const real *ptr1_base = data ;
        const real *ptr2_base = m.data ;
        
            while( i-- ){
            j = P ;
            while( j-- ){ 
                ptr1 = ptr1_base ;
                ptr2 = ptr2_base++ ;
                k = N ;
                val = 0. ;
                while( k-- ) {
                val += (*ptr1++)*(*ptr2) ;
                 ptr2 += P ;
                }
                *ptrm++ = val  ;

            }
            ptr1_base += N ;
            ptr2_base = m.data ;
            }
            return res;      
	}


	/**
	 * transpose the matrix
	 */
	Matrix<N, M> transposed(void) const {
		Matrix<N, M> res;
        unsigned int i, j;
        
        for (i=0; i<N ;i++)
            for (j=0; j<M ;j++)
                    res.data[i*M+j] = data[j*N+i];
            
		return res;
	}

	/**
	 * invert the matrix
	 */
	Matrix<N, M> inversed(void) {
		Matrix<N, M> res;
        double d;
        
        d = inverse_matrice_pivot(this->data, res.data, M);
		return res;
	}

    	/**
	 * invert the matrix
	 */
	Matrix<N, M> cholinversed(void) {
		Matrix<N, M> res;
        double d;
        
        d = inverse_matrice_cholesky(this->data, res.data, M);
		return res;
	}

	/**
	 * set zero matrix
	 */
	void zero(void) {
        for (unsigned int i = 0; i < M*N; i++)
			data[i] = 0.0;
	}

	/**
	 * set identity matrix
	 */
	void identity(void) {
        for (unsigned int i = 0; i < M*N; i++)
			data[i] = 0.0;
        
		unsigned int n = (M < N) ? M : N;

		for (unsigned int i = 0; i < n; i++)
			data[i*N+i] = 1;
	}
    
    
    template <unsigned int P, unsigned int Q>
    void extr(const Matrix<P,Q> &m, int i0, int j0)  {

        for (unsigned int i = 0; i < M; i++)
            for (unsigned int j = 0; j < N; j++)
                data[i*N+j] = m.data[(i+i0-1)*Q+j+j0-1];
    }
    
    template <unsigned int P, unsigned int Q>
    void insr(const Matrix<P,Q> &m, int i0, int j0)  {

        for (unsigned int i = 0; i < P; i++)
                for (unsigned int j = 0; j < Q; j++)
                    data[(i+i0-1)*N+j+j0-1] = m.data[i*Q+j];
    }
    
    template <unsigned int P>
    void insrdiag(const Vector<P> &v, int i0)  {

		for (unsigned int i = 0; i < P; i++)
                data[(i+i0-1)*N+i+i0-1] = v.data[i];
    }


	void print(char *str) const {
        printf("%s\n ", str);
		for (unsigned int i = 0; i < M; i++) {
			printf("[ ");

			for (unsigned int j = 0; j < N; j++)
				printf("%.4f\t", data[i*N+j]);

			printf(" ]\n");
		}
	}
    
    	/**
	 * multiplication by a vector
	 */
	Vector<M> operator *(const Vector<N> &v) const {
		Vector<M> res;
        unsigned int i, j;
        
        for (i=0; i<M ;i++) {
            res.data[i] = 0.0f;
            for (j=0; j<N ;j++)
                res.data[i] = res.data[i] + data[i*N+j] * v.data[j];
        }

        return res;
	}
    
	/* create a rotation matrix from given euler angles
	 * based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
	 */
	void from_euler(real roll, real pitch, real yaw) {
		real cp = cosf(pitch);
		real sp = sinf(pitch);
		real sr = sinf(roll);
		real cr = cosf(roll);
		real sy = sinf(yaw);
		real cy = cosf(yaw);

		data[0] = cp * cy;
		data[1] = (sr * sp * cy) - (cr * sy);
		data[2] = (cr * sp * cy) + (sr * sy);
		data[3] = cp * sy;
		data[4] = (sr * sp * sy) + (cr * cy);
		data[5] = (cr * sp * sy) - (sr * cy);
		data[6] = -sp;
		data[7] = sr * cp;
		data[8] = cr * cp;
	}

	/**
	 * get euler angles from rotation matrix
	 */
	Vector<3> to_euler(void) const {
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
    
    void antisym(Vector<3> V) {
        memset(data, 0, sizeof(data));
        
		data[1] = -V.data[2];
        data[3] =  V.data[2];
        data[2] =  V.data[1];
        data[6] = -V.data[1];
        data[5] = -V.data[0];
        data[7] =  V.data[0];
	}

};

// EB_ONERA ADD v


// EB_ONERA ADD ^


}	// namespace math


#endif // MATRIX_ONERA_HPP
