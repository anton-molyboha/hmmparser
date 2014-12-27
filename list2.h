#ifndef IMMUTABLE_LIST2_H
#define IMMUTABLE_LIST2_H 1

#include <immutable/list.hpp>

template<typename T, typename Allocator = std::allocator<T> >
class list2
{
private:
    typedef immutable::list<T, Allocator> list1;

public:
    typedef T value_type;
    typedef typename list1::reference reference;
    typedef typename list1::const_reference const_reference;
    typedef typename list1::pointer pointer;
    typedef typename list1::const_pointer const_pointer;
    typedef typename list1::iterator iterator;
    typedef typename list1::const_iterator const_iterator;
    typedef std::size_t size_type;
    typedef std::ptrdiff_t difference_type;
    typedef Allocator allocator_type;

    explicit list2(const allocator_type &a = allocator_type()); // nothrow
    explicit list2(size_type n, const_reference v = value_type(),
            const allocator_type &a = allocator_type());
    template<class InputIterator>
    list2(InputIterator first, InputIterator last,
            const allocator_type &a = allocator_type());
    template<typename...Args>
    explicit list2(Args... args);
    //list2(const list2 &x); // nothrow
    //~list2(); // nothrow
    //list2 &operator=(const list2 &x); // nothrow

    const_iterator begin() const; // strong //// nothrow
    const_iterator end() const; // strong //// nothrow
    const_iterator cbegin() const; // strong //// nothrow
    const_iterator cend() const; // strong //// nothrow

    bool empty() const; // nothrow
    size_type size() const; // nothrow
    size_type max_size() const; // nothrow
    void resize(size_type n, const_reference v = value_type()); // strong

    const_reference front() const; // nothrow

    void push_front(const_reference v); // strong
    void pop_front(); // strong //// nothrow
    void push_back(const_reference v); // strong
    void assign(size_type n, const_reference v); // strong
    template<class InputIterator>
    void assign(InputIterator first, InputIterator last); // strong
    void insert(const_iterator pos, const_reference v); // strong
    void insert(const_iterator pos, size_type n,
            const_reference v); // strong
    template<class InputIterator>
    void insert(const_iterator pos, InputIterator first,
            InputIterator last); // strong
    template<class InputIterator>
    void insert_reverse(const_iterator pos, InputIterator first,
            InputIterator last); // strong
    const_iterator replace(const_iterator pos, const_reference v); // strong
    const_iterator erase(const_iterator pos); // strong
    const_iterator erase(const_iterator first,
            const_iterator last); // strong
    void reverse(); // nothrow
    void swap(list2 &x); // nothrow
    void clear(); // nothrow

    allocator_type get_allocator() const; // nothrow

private:
    void init_list(const_reference first);

    template<typename...Args>
    void init_list(const_reference first, Args... rest);

    mutable list1 front_part;
    mutable list1 back_part;

    void normalize() const;
};



template<typename T, typename Allocator>
list2<T, Allocator>::list2(const allocator_type &a)
: front_part(a), back_part(a)
{}

template<typename T, typename Allocator>
list2<T, Allocator>::list2(size_type n, const_reference v, const allocator_type &a)
: front_part(n, v, a), back_part(a)
{}

template<typename T, typename Allocator>
template<class InputIterator>
list2<T, Allocator>::list2(InputIterator first, InputIterator last,
        const allocator_type &a)
: front_part(first, last, a), back_part(a)
{}

template<typename T, typename Allocator>
template<typename...Args>
list2<T, Allocator>::list2(Args... args)
{
    init_list(args...);
}


template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::begin() const
{
    normalize();
    return front_part.begin();
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::end() const
{
    normalize();
    return front_part.end();
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::cbegin() const
{
    normalize();
    return front_part.cbegin();
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::cend() const
{
    normalize();
    return front_part.cend();
}


template<typename T, typename Allocator>
bool list2<T, Allocator>::empty() const
{
    return front_part.empty() && back_part.empty();
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::size_type list2<T, Allocator>::size() const
{
    return front_part.size() + back_part.size();
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::size_type list2<T, Allocator>::max_size() const
{
    return front_part.max_size();
}

template<typename T, typename Allocator>
void list2<T, Allocator>::resize(size_type n, const_reference v)
{
    front_part.resize(n, v);
    back_part.clear();
}


template<typename T, typename Allocator>
typename list2<T, Allocator>::const_reference list2<T, Allocator>::front() const
{
    if( front_part.empty() )
    {
        normalize();
    }
    return front_part.front();
}


template<typename T, typename Allocator>
void list2<T, Allocator>::push_front(const_reference v)
{
    front_part.push_front(v);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::pop_front()
{
    if( front_part.empty() )
    {
        normalize();
    }
    front_part.pop_front();
}

template<typename T, typename Allocator>
void list2<T, Allocator>::push_back(const_reference v)
{
    back_part.push_front(v);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::assign(size_type n, const_reference v)
{
    front_part.assign(n, v);
    back_part.clear();
}

template<typename T, typename Allocator>
template<class InputIterator>
void list2<T, Allocator>::assign(InputIterator first, InputIterator last)
{
    front_part.assign(first, last);
    back_part.clear();
}

template<typename T, typename Allocator>
void list2<T, Allocator>::insert(const_iterator pos, const_reference v)
{
    front_part.insert(pos, v);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::insert(const_iterator pos, size_type n,
        const_reference v)
{
    front_part.insert(pos, n, v);
}

template<typename T, typename Allocator>
template<class InputIterator>
void list2<T, Allocator>::insert(const_iterator pos, InputIterator first,
        InputIterator last)
{
    front_part.insert(pos, first, last);
}

template<typename T, typename Allocator>
template<class InputIterator>
void list2<T, Allocator>::insert_reverse(const_iterator pos, InputIterator first,
        InputIterator last)
{
    front_part.insert_reverse(pos, first, last);
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::replace(const_iterator pos, const_reference v)
{
    return front_part.replace(pos, v);
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::erase(const_iterator pos)
{
    return front_part.erase(pos);
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::const_iterator list2<T, Allocator>::erase(const_iterator first,
        const_iterator last)
{
    return front_part.erase(first, last);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::reverse()
{
    front_part.swap(back_part);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::swap(list2 &x)
{
    front_part.swap(x.front_part);
    back_part.swap(x.back_part);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::clear()
{
    front_part.clear();
    back_part.clear();
}

template<typename T, typename Allocator>
typename list2<T, Allocator>::allocator_type list2<T, Allocator>::get_allocator() const
{
    return front_part.get_allocator();
}


template<typename T, typename Allocator>
void list2<T, Allocator>::init_list(const_reference first)
{
    push_front(first);
}

template<typename T, typename Allocator>
template<typename...Args>
void list2<T, Allocator>::init_list(const_reference first, Args... rest)
{
    init_list(rest...);
    push_front(first);
}

template<typename T, typename Allocator>
void list2<T, Allocator>::normalize() const
{
    if( ! back_part.empty() )
    {
        front_part.insert_reverse(front_part.end(), back_part.begin(), back_part.end());
        back_part.clear();
    }
}

#endif
