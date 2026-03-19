// ******************************************************************************************************
// *            Fir.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: Fir.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 3/29/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

#include <cstring>
using std::memset;

namespace
{
    const uint16_t FIR_37_SIZE = 37;
}

template<uint16_t Order>
class Fir
{
public:
    typedef uint16_t size_type;
    static const size_type size = Order; 
    
    Fir( float const* table )
    	: CoefficientTable(table)
    {
    	for (size_type i = 0; i < size; ++i)
    	{
    		xn[i] = 0.0f;
    	}
    }
    ~Fir()
    {
    }

    public:
        float Run( float xn );
        float xn[Order];
        
    protected:
        float const* CoefficientTable;
};

template <uint16_t Order>
inline float Fir<Order>::Run( float x0)
{
    float result = 0;

    float* src = &xn[ ( Order - 2 ) ];      // point to second to last
    float* dst = &xn[ ( Order - 1 ) ];      // point to last

    // shift samples up one delay slot
    for ( size_type i = 0; i < ( Order - 1 ); i++ )
    {
        *dst-- = *src--;
    }

    xn[0] = x0;

    for ( size_type i = 0; i < Order; i++ )
    {
        result += xn[i] * CoefficientTable[i];
    }

    return result;
}

// ******************************************************************************************************
// *            End of Fir.h
// ******************************************************************************************************
