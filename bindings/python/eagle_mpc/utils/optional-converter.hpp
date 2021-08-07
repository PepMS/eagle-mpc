#include <exception>

#include <boost/mpl/if.hpp>
#include <boost/optional.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/to_python_converter.hpp>
#include <boost/type_traits/integral_constant.hpp>
#include <boost/python/module.hpp>

namespace eagle_mpc
{
namespace python
{
namespace bp = boost::python;

// Custom exceptions
struct AttributeError : std::exception {
    const char* what() const throw() { return "Optional exception"; }
};

struct TypeError : std::exception {
    const char* what() const throw() { return "TypeError exception"; }
};

// Set python exceptions
void translate(const std::exception& e)
{
    if (dynamic_cast<const AttributeError*>(&e)) PyErr_SetString(PyExc_AttributeError, e.what());
    if (dynamic_cast<const TypeError*>(&e)) PyErr_SetString(PyExc_TypeError, e.what());
}

template <typename T>
struct to_python_optional {
    static PyObject* convert(const boost::optional<T>& obj)
    {
        if (obj) return bp::incref(bp::object(*obj).ptr());
        // raise AttributeError if any value hasn't been set yet
        else {
            return Py_None;
        }
    }
};

template <typename T>
struct is_optional : boost::false_type {
};

template <typename T>
struct is_optional<boost::optional<T> > : boost::true_type {
};

/// @brief Type used to provide meaningful compiler errors.
template <typename>
struct return_optional_requires_a_optional_return_type {
};

template <typename T>
struct from_python_optional {
    static void* convertible(PyObject* obj_ptr)
    {
        // try {
        // }
        // // Without try catch it still raises a TypeError exception
        // // But this enables to custom your error message
        // catch (...) {
        //   throw TypeError();
        // }
        if (!is_optional<T>::value) return 0;
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        const T value = typename bp::extract<T>::extract(obj_ptr);

        assert(value);

        void* storage = ((bp::converter::rvalue_from_python_storage<boost::optional<T> >*)data)->storage.bytes;

        new (storage) boost::optional<T>(value);

        data->convertible = storage;
    }

    from_python_optional()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<boost::optional<T> >());
    }
};

struct return_optional {
    template <class T>
    struct apply {
        // The to_python_optional ResultConverter only checks if T is convertible
        // at runtime.  However, the following MPL branch cause a compile time
        // error if T is not a boost::optional by providing a type that is not a
        // ResultConverter model.
        typedef typename boost::mpl::
            if_<is_optional<T>, to_python_optional<T>, return_optional_requires_a_optional_return_type<T> >::type type;
    };  // apply
};      // return_optional

}  // namespace python
}  // namespace eagle_mpc