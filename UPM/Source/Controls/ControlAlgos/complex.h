#ifndef EATON_PANDA_COMPLEX_H_
#define EATON_PANDA_COMPLEX_H_
// ******************************************************************************************************
// *            complex.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2012 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: complex.h
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Jonathan Brandmeyer
// *
// *    DATE: 8/29/2012
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#include <cmath>

using namespace std;

/*
 * A template class for floating-point complex types.  The C28x compiler's library does
 * not provide std::complex.  This template class should be compatible with
 * std::complex<T> in every way, although it only includes those operators
 * which we are known to be using.  It effectively provides syntactic sugar
 * for working with complex numbers.  Due to the simplicity of most functions,
 * most of them are inline.
 */
template <typename Float>
class complex
{
public:
	typedef Float value_type;

	// Default constructor explicitly default-constructs the members
	// For built-in floating-point types, it will be zero.
	complex(): realpart(), imagpart() {}

	// Construct from real and (optionally) imaginary members.  If the imaginary
	// part is not provided, it is zero.
	explicit complex(value_type real, value_type imag = value_type(0))
		: realpart(real), imagpart(imag) {}

	// Allow default copy-constructor, copy-assignment operator, and destructor

	// Member accessors
	value_type real() const { return realpart; }
	value_type imag() const { return imagpart; }

private:
	// Data members themselves are private
	value_type realpart;
	value_type imagpart;
};

template <typename Float>
complex<Float> operator*(complex<Float> lhs, complex<Float> rhs);

template<typename Float>
complex<Float> operator*(complex<Float> lhs, complex<Float> rhs)
{
	// out-of-line due to function length
	return complex<Float>(lhs.real()*rhs.real() - lhs.imag()*rhs.imag(),
			lhs.real()*rhs.imag() + rhs.real()*lhs.imag());
}

template <typename Float>
complex<Float> operator*(complex<Float> lhs, float rhs)
{
	return complex<Float>(lhs.real()*rhs, lhs.imag()*rhs);
}

template <typename Float>
complex<Float> operator*(float lhs, complex<Float> rhs)
{
	return complex<Float>(lhs*rhs.real(), lhs*rhs.imag());
}

template <typename Float>
complex<Float> exp(complex<Float> a)
{
	return exp(a.real())*complex<Float>(cos(a.imag()), sin(a.imag()));
}

template <typename Float>
complex<Float> conj(complex<Float> a)
{
	return complex<Float>(a.real(), -a.imag());
}

template <typename Float>
Float norm(complex<Float> a)
{
	return a.real()*a.real() + a.imag()*a.imag();
}

template <typename Float>
Float arg(complex<Float> c)
{
	return atan2(c.imag(), c.real());
}

template <typename Float>
Float abs(complex<Float> c)
{
	return sqrt(norm(c));
}


#endif /* COMPLEX_H_ */
