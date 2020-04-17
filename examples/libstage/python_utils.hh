#ifndef _PYTHON_UTILS_H_
#define _PYTHON_UTILS_H_


#include <boost/python.hpp>
#include <numpy/ndarrayobject.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>


boost::python::object stdUnsignedCharVecToNumpyArray(std::vector<unsigned char> const& vec )
{
    npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    unsigned char * data = size ? const_cast<unsigned char *>(&vec[0])
                                : static_cast<unsigned char *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_UINT8, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}
boost::python::object stdDoubleVecToNumpyArray(std::vector<double> const& vec )
{
    npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    double * data = size ? const_cast<double *>(&vec[0])
                         : static_cast<double *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_DOUBLE, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}

boost::python::object stdFloatVecToNumpyArray( std::vector<float> const& vec )
{
    npy_intp size = vec.size();

    /* const_cast is rather horrible but we need a writable pointer
       in C++11, vec.data() will do the trick
       but you will still need to const_cast
     */

    float * data = size ? const_cast<float *>(&vec[0])
                        : static_cast<float *>(NULL);

    // create a PyObject * from pointer and data

    npy_intp dims[1];
    dims[0] = size;

    PyObject * pyObj = PyArray_SimpleNewFromData( 1, dims, NPY_FLOAT, data );

    boost::python::handle<> handle( pyObj );

    boost::python::numeric::array arr( handle );

    /* The problem of returning arr is twofold: firstly the user can modify
      the data which will betray the const-correctness
      Secondly the lifetime of the data is managed by the C++ API and not the
      lifetime of the numpy array whatsoever. But we have a simple solution..
     */

    return arr.copy(); // copy the object. numpy owns the copy now.
}


#endif
