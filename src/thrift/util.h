namespace apache {
namespace thrift {

template<class T> class scoped_array_self
{
private:

    T * px;

    scoped_array_self(scoped_array_self const &);
    scoped_array_self & operator=(scoped_array_self const &);

    typedef scoped_array_self<T> this_type;

    void operator==( scoped_array_self const& ) const;
    void operator!=( scoped_array_self const& ) const;

public:

    typedef T element_type;

    explicit scoped_array_self( T * p = 0 )  : px( p )
    {

    }

    ~scoped_array_self()
    {
        delete []px ;
    }

    void reset(T * p = 0)
    {
        this_type(p).swap(*this);
    }

    T & operator[](std::ptrdiff_t i) const
    {
        return px[i];
    }

    T * get() const
    {
        return px;
    }



    void swap(scoped_array_self & b)
    {
        T * tmp = b.px;
        b.px = px;
        px = tmp;
    }
};


template<class T> inline void swap(scoped_array_self<T> & a, scoped_array_self<T> & b)
{
    a.swap(b);
}




}}
