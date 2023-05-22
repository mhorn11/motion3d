#pragma once

#include <map>

#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** \brief Extension of <CODE>std::map</CODE> with more iterators and find functions. */
template<class Key, class T, class Compare = std::less<Key>, class Allocator = std::allocator<std::pair<const Key, T>>>
class ExtendedMap : public std::map<Key, T, Compare, Allocator>
{
 public:
  DEFINE_POINTERS(ExtendedMap);

  using std::map<Key, T, Compare, Allocator>::map;

  using iterator = typename std::map<Key, T, Compare, Allocator>::iterator;
  using const_iterator = typename std::map<Key, T, Compare, Allocator>::const_iterator;
  using reverse_iterator = typename std::map<Key, T, Compare, Allocator>::reverse_iterator;
  using const_reverse_iterator = typename std::map<Key, T, Compare, Allocator>::const_reverse_iterator;

  /** Constructs and initializes map from separate <I>keys</I> and <I>values</I>.
    * \throws std::invalid_argument if the sizes of <I>keys</I> and <I>values</I> are not equal.
    */
  template<class KeyIn, class TIn>
  ExtendedMap(const std::vector<KeyIn> &keys, const std::vector<TIn> &values, const bool sorted_data = false)
    : ExtendedMap()
  {
    using KeyIt = typename std::vector<KeyIn>::const_iterator;
    using TIt = typename std::vector<TIn>::const_iterator;

    // check sizes
    if (keys.size() != values.size())
    {
      throw std::invalid_argument("The sizes of keys and values must be equal");
    }

    // insert function
    std::function<void(const KeyIt&, const TIt&)> insert_fun;
    if (sorted_data)
    {
      insert_fun = [this](const KeyIt &key_it, const TIt &value_it) {
        this->insert(this->end(), {*key_it, *value_it});
      };
    }
    else
    {
      insert_fun = [this](const KeyIt &key_it, const TIt &value_it) {
        this->insert({*key_it, *value_it});
      };
    }

    // iterate data
    auto key_it = keys.cbegin();
    auto value_it = values.cbegin();
    for (; key_it != keys.cend(); ++key_it, ++value_it)
    {
      insert_fun(key_it, value_it);
    }
  }

  struct const_key_iterator : const_iterator
  {
    const_key_iterator() : const_iterator() {};
    explicit const_key_iterator(const_iterator it) : const_iterator(it) {};
    const Key* operator->() const { return &(const_iterator::operator->()->first); }
    const Key& operator*() const { return const_iterator::operator*().first; }
  };

  struct const_reverse_key_iterator : const_reverse_iterator
  {
    const_reverse_key_iterator() : const_iterator() {};
    explicit const_reverse_key_iterator(const_reverse_iterator it) : const_reverse_iterator(it) {};
    const Key* operator->() const { return &(const_reverse_iterator::operator->()->first); }
    const Key& operator*() const { return const_reverse_iterator::operator*().first; }
  };

  struct value_iterator : iterator
  {
    value_iterator() : iterator() {};
    explicit value_iterator(iterator it) : iterator(it) {};
    T* operator->() { return &(iterator::operator->()->second); }
    T& operator*() { return iterator::operator*().second; }
  };

  struct const_value_iterator : const_iterator
  {
    const_value_iterator() : const_iterator() {};
    explicit const_value_iterator(const_iterator it) : const_iterator(it) {};
    const T* operator->() const { return &(const_iterator::operator->()->second); }
    const T& operator*() const { return const_iterator::operator*().second; }
  };

  struct reverse_value_iterator : reverse_iterator
  {
    reverse_value_iterator() : reverse_iterator() {};
    explicit reverse_value_iterator(reverse_iterator it) : reverse_iterator(it) {};
    T* operator->() { return &(reverse_iterator::operator->()->second); }
    T& operator*() { return reverse_iterator::operator*().second; }
  };

  struct const_reverse_value_iterator : const_reverse_iterator
  {
    const_reverse_value_iterator() : const_reverse_iterator() {};
    explicit const_reverse_value_iterator(const_reverse_iterator it) : const_reverse_iterator(it) {};
    const T* operator->() const { return &(const_reverse_iterator::operator->()->second); }
    const T& operator*() const { return const_reverse_iterator::operator*().second; }
  };


  /** \returns a key iterator to the first element. */
  const_key_iterator cbegin_keys() const { return const_key_iterator(this->cbegin()); }

  /** \returns the past-the-end iterator corresponding to cbegin_keys(). */
  const_key_iterator cend_keys() const { return const_key_iterator(this->cend()); }

  /** \sa cbegin_keys() */
  const_key_iterator begin_keys() const { return cbegin_keys(); }

  /** \sa cend_keys() */
  const_key_iterator end_keys() const { return cend_keys(); }


  /** \returns a reverse key iterator to the first element. */
  const_reverse_key_iterator crbegin_keys() const { return const_reverse_key_iterator(this->crbegin()); }

  /** \returns the past-the-end iterator corresponding to crbegin_keys(). */
  const_reverse_key_iterator crend_keys() const { return const_reverse_key_iterator(this->crend()); }

  /** \sa crbegin_keys() */
  const_reverse_key_iterator rbegin_keys() const { return crbegin_keys(); }

  /** \sa crend_keys() */
  const_reverse_key_iterator rend_keys() const { return crend_keys(); }


  /** \returns a value iterator to the first element. */
  value_iterator begin_values() { return value_iterator(this->begin()); }

  /** \returns the past-the-end iterator corresponding to begin_values(). */
  value_iterator end_values() { return value_iterator(this->end()); }

  /** \sa begin_values() */
  const_value_iterator cbegin_values() const { return const_value_iterator(this->cbegin()); }

  /** \sa end_values() */
  const_value_iterator cend_values() const { return const_value_iterator(this->cend()); }

  /** \sa begin_values() */
  const_value_iterator begin_values() const { return cbegin_values(); }

  /** \sa end_values() */
  const_value_iterator end_values() const { return cend_values(); }


  /** \returns a reverse value iterator to the first element. */
  reverse_value_iterator rbegin_values() { return reverse_value_iterator(this->rbegin()); }

  /** \returns the past-the-end iterator corresponding to rbegin_values(). */
  reverse_value_iterator rend_values() { return reverse_value_iterator(this->rend()); }

  /** \sa rbegin_values() */
  const_reverse_value_iterator crbegin_values() const { return const_reverse_value_iterator(this->crbegin()); }

  /** \sa rend_values() */
  const_reverse_value_iterator crend_values() const { return const_reverse_value_iterator(this->crend()); }

  /** \sa rbegin_values() */
  const_reverse_value_iterator rbegin_values() const { return crbegin_values(); }

  /** \sa rend_values() */
  const_reverse_value_iterator rend_values() const { return crend_values(); }


  /** \returns an iterator to the element at <I>key</I> or the end() iterator if no element was found. */
  iterator find_eq(const Key &key) { return this->find(key); }

  /** \sa find_eq(const Key &key) */
  const_iterator cfind_eq(const Key &key) const { return this->find(key); }

  /** \sa find_eq(const Key &key) */
  const_iterator find_eq(const Key &key) const { return cfind_eq(key); }


  /** \returns an iterator to the first element with key >= <I>key</I> or the end() iterator if no element was found. */
  iterator find_ge(const Key &key) { return this->lower_bound(key); }

  /** \sa find_ge(const Key &key) */
  const_iterator cfind_ge(const Key &key) const { return this->lower_bound(key); }

  /** \sa find_ge(const Key &key) */
  const_iterator find_ge(const Key &key) const { return cfind_ge(key); }


  /** \returns an iterator to the first element with key > <I>key</I> or the end() iterator if no element was found. */
  iterator find_gt(const Key &key) { return this->upper_bound(key); }

  /** \sa find_gt(const Key &key) */
  const_iterator cfind_gt(const Key &key) const { return this->upper_bound(key); }

  /** \sa find_gt(const Key &key) */
  const_iterator find_gt(const Key &key) const { return cfind_gt(key); }


  /** \returns an iterator to the last element with time < <I>key</I> or the end() iterator if no element was found. */
  iterator find_le(const Key &key)
  {
    if (this->begin() == this->end())
    {
      return this->end();
    }
    auto tmp = this->upper_bound(key);
    if (tmp == this->begin())
    {
      return this->end();
    }
    return --tmp;
  }

  /** \sa find_le(const Key &key) */
  const_iterator cfind_le(const Key &key) const
  {
    if (this->cbegin() == this->cend())
    {
      return this->cend();
    }
    auto tmp = this->upper_bound(key);
    if (tmp == this->cbegin())
    {
      return this->cend();
    }
    return --tmp;
  }

  /** \sa find_le(const Key &key) */
  const_iterator find_le(const Key &key) const { return cfind_le(key); }


  /** \returns an iterator to the last element with time <= <I>key</I> or the end() iterator if no element was found. */
  iterator find_lt(const Key &key)
  {
    if (this->begin() == this->end())
    {
      return this->end();
    }
    auto tmp = this->lower_bound(key);
    if (tmp == this->begin())
    {
      return this->end();
    }
    return --tmp;
  }

  /** \sa find_lt(const Key &key) */
  const_iterator cfind_lt(const Key &key) const
  {
    if (this->cbegin() == this->cend())
    {
      return this->cend();
    }
    auto tmp = this->lower_bound(key);
    if (tmp == this->cbegin())
    {
      return this->cend();
    }
    return --tmp;
  }

  /** \sa find_lt(const Key &key) */
  const_iterator find_lt(const Key &key) const { return cfind_lt(key); }

}; // class ExtendedMap

} // namespace motion3d
