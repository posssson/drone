
#ifndef VECTOR_ONERA_HPP
#define VECTOR_ONERA_HPP

#include <stdio.h>
#include <math.h>

typedef float real;

namespace math
{

template <unsigned int N>
class __EXPORT Vector
{
public:
	/**
	 * vector data
	 */
//	real *data;
    real data[N];


	/**
	 * trivial ctor
	 * note that this ctor will not initialize elements
	 */
	Vector() {
//        data = new real[N];
	}
    
    ~Vector() {
//        delete [] data;
    }

	/**
	 * copy ctor
	 */
	Vector(const Vector<N> &v) {
//        data = new real[N];
		memcpy(data, v.data, N*sizeof(real));
	}

	/**
	 * setting ctor
	 */
    Vector(const real d) {
        for (unsigned int i = 0; i < N; i++)
			data[i] = d;
	}
    
	Vector(const real d[N]) {
//        data = new real[N];
//        printf("Vector sizeof %d %d\n", sizeof(real), N);
		memcpy(data, d, N*sizeof(real));
	}

    Vector(const real x, const real y) {
//        data = new real[2];
		data[0] = x;
		data[1] = y;
	}

    Vector(const real x, const real y, const real z) {
//        data = new real[3];
		data[0] = x;
		data[1] = y;
		data[2] = z;
	}

    Vector(const real x0, const real x1, const real x2, const real x3) {
//		data = new real[4];
        data[0] = x0;
		data[1] = x1;
		data[2] = x2;
		data[3] = x3;
	}
    
    real operator %(const Vector<2> &v) const {
		return data[0] * v.data[1] - data[1] * v.data[0];
	}

    Vector<3> operator %(const Vector<3> &v) const {
		return Vector<3>(
			       data[1] * v.data[2] - data[2] * v.data[1],
			       data[2] * v.data[0] - data[0] * v.data[2],
			       data[0] * v.data[1] - data[1] * v.data[0]
		       );
	}
        
	/**
	 * set data
	 */
	void set(const real d[N]) {
//        printf("Vector set\n");
		memcpy(data, d, N*sizeof(real));
	}

	/**
	 * access to elements by index
	 */
	real &operator()(const unsigned int i) {
		return data[i-1];
	}

	/**
	 * access to elements by index
	 */
	real operator()(const unsigned int i) const {
		return data[i-1];
	}

	/**
	 * get vector size
	 */
	unsigned int get_size() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Vector<N> &v) const {
		for (unsigned int i = 0; i < N; i++)
			if (data[i] != v.data[i])
				return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Vector<N> &v) const {
		for (unsigned int i = 0; i < N; i++)
			if (data[i] != v.data[i])
				return true;

		return false;
	}

	/**
	 * set to value
	 */
     
    const Vector<N> &operator =(const Vector<N> &v) {
//        printf("Vector=\n");
//		for (unsigned int i = 0; i < N; i++)
//			data[i] = v.data[i];

        memcpy(data, v.data, N*sizeof(real));
        
        return *this;
	}


	/**
	 * negation
	 */
	const Vector<N> operator -(void) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = -data[i];

		return res;
	}

	/**
	 * addition
	 */
	const Vector<N> operator +(const Vector<N> &v) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] + v.data[i];

		return res;
	}

	/**
	 * subtraction
	 */
	const Vector<N> operator -(const Vector<N> &v) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] - v.data[i];

		return res;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> operator *(const real num) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] * num;

		return res;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> operator /(const real num) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] / num;

		return res;
	}

	/**
	 * addition
	 */
	const Vector<N> &operator +=(const Vector<N> &v) {
//        printf("Vector+=\n");
		for (unsigned int i = 0; i < N; i++)
			data[i] += v.data[i];

        return *this;
	}

	/**
	 * subtraction
	 */
	const Vector<N> &operator -=(const Vector<N> &v) {
		for (unsigned int i = 0; i < N; i++)
			data[i] -= v.data[i];

        return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> &operator *=(const real num) {
		for (unsigned int i = 0; i < N; i++)
			data[i] *= num;

        return *this;
	}

	/**
	 * uniform scaling
	 */
	const Vector<N> &operator /=(const real num) {
		for (unsigned int i = 0; i < N; i++)
			data[i] /= num;

        return *this;
	}

	/**
	 * dot product
	 */
	real operator *(const Vector<N> &v) const {
		real res = 0.0f;

		for (unsigned int i = 0; i < N; i++)
			res += data[i] * v.data[i];

		return res;
	}

	/**
	 * element by element multiplication
	 */
	const Vector<N> emult(const Vector<N> &v) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] * v.data[i];

		return res;
	}

	/**
	 * element by element division
	 */
	const Vector<N> edivide(const Vector<N> &v) const {
		Vector<N> res;

		for (unsigned int i = 0; i < N; i++)
			res.data[i] = data[i] / v.data[i];

		return res;
	}

	/**
	 * gets the length of this vector squared
	 */
	real length_squared() const {
		real res = 0.0f;

		for (unsigned int i = 0; i < N; i++)
			res += data[i] * data[i];

		return res;
	}

	/**
	 * gets the length of this vector
	 */
	real length() const {
		real res = 0.0f;

		for (unsigned int i = 0; i < N; i++)
			res += data[i] * data[i];

		return sqrtf(res);
	}

	/**
	 * normalizes this vector
	 */
	void normalize() {
		*this /= length();
	}

	/**
	 * returns the normalized version of this vector
	 */
	Vector<N> normalized() const {
		return *this / length();
	}

	/**
	 * set zero vector
	 */
	void zero(void) {
        for (unsigned int i = 0; i < N; i++)
			data[i] = 0.0;
	}

    template <unsigned int P>
    void extr(Vector<P> &v, int j)  {

		for (unsigned int i = 0; i < N; i++)
            data[i] = v.data[i+j-1];
    }
    
    template <unsigned int P>
    void insr(Vector<P> &v, int j)  {

		for (unsigned int i = 0; i < P; i++)
            data[i+j-1] = v.data[i];
    }


	void print(char *str) {
		printf("%s\n[ ", str);

		for (unsigned int i = 0; i < N; i++)
			printf("%.3f\t", data[i]);

		printf("]\n");
	}
};

}

#endif // VECTOR_HPP
