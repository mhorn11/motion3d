#pragma once

#include <string>
#include <vector>

#include <motion3d/containers/extended_map.hpp>
#include <motion3d/transforms/base.hpp>
#include <motion3d/transforms/implementation_/operators.hpp>
#include <motion3d/utils/templating.hpp>

namespace motion3d
{

/** Exception thrown by TransformContainer in case motions and poses are mixed. */
struct TransformContainerException : public MessageException
{
  explicit TransformContainerException(std::string msg) : MessageException(std::move(msg)) {}
};

// forward declarations
class M3DReader;
class M3DWriter;

/** \brief Container class to store, process and convert stamped and unstamped transformations for either motions or
  * poses.
  *
  * This container makes it possible to store either stamped and unstamped data and enables element access without
  * requiring the user to know, if the data is really stamped or unstamped.
  * Further, the container provides unstamped- or stamped-specific functions, e.g., for searching elements at specific
  * stamps.
  * An invididual transform of this container is referred to as \f$T_i\f$.
  * Chaining two transforms \f$T_\mathrm{a}\f$ and \f$T_\mathrm{b}\f$ is denoted as
  * \f$T_\mathrm{a} \circ T_\mathrm{b}\f$.
  *
  * In case stamped functions are called on unstamped data or vice versa, or in case motions and poses are mixed,
  * a TransformContainerException is thrown.
  */
class TransformContainer
{
  friend class M3DReader;

 public:
  DEFINE_POINTERS(TransformContainer);

  using StampType = Time;
  using DataType = TransformInterface::Ptr;
  using DataTypeConst = TransformInterface::ConstPtr;
  using VectorType = std::vector<DataType>;
  using MapType = ExtendedMap<StampType, DataType>;


  /////////////////////////////////////////////////////////////////////////////
  /// Iterator Definitions
  /////////////////////////////////////////////////////////////////////////////

  struct iterator 
  {
    friend class TransformContainer;

    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = DataType;
    using pointer           = DataType;
    using reference         = DataType&;

    explicit iterator(VectorType::iterator vector_it);
    explicit iterator(MapType::iterator map_it);

    reference operator*() const;
    pointer operator->() const;

    // Prefix increment
    iterator& operator++();
    // Postfix increment
    iterator operator++(int);

    friend bool operator==(const iterator &a, const iterator &b);
    friend bool operator!=(const iterator &a, const iterator &b);

   private:
    VectorType::iterator vector_it_;
    MapType::iterator map_it_;
    bool is_map_;
  };

  struct const_iterator 
  {
    friend class TransformContainer;

    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using value_type        = DataTypeConst;
    using pointer           = DataTypeConst;
    using reference         = DataTypeConst;

    explicit const_iterator(VectorType::const_iterator vector_it);
    explicit const_iterator(MapType::const_iterator map_it);

    reference operator*() const;
    pointer operator->() const;

    // Prefix increment
    const_iterator& operator++();
    // Postfix increment
    const_iterator operator++(int);

    friend bool operator==(const const_iterator &a, const const_iterator &b);
    friend bool operator!=(const const_iterator &a, const const_iterator &b);

   private:
    VectorType::const_iterator vector_it_;
    MapType::const_iterator map_it_;
    bool is_map_;
  };

  struct const_item_iterator
  {
    friend class TransformContainer;

    using iterator_category = std::forward_iterator_tag;
    using difference_type   = std::ptrdiff_t;
    using key_type          = StampType;
    using value_type        = std::pair<const StampType, DataTypeConst>;
    using pointer           = const value_type*;
    using reference         = value_type;

    explicit const_item_iterator(MapType::const_iterator map_it);

    reference operator*() const;
    pointer operator->() const;

    // Prefix increment
    const_item_iterator& operator++();
    // Postfix increment
    const_item_iterator operator++(int);

    friend bool operator==(const const_item_iterator &a, const const_item_iterator &b);
    friend bool operator!=(const const_item_iterator &a, const const_item_iterator &b);

   private:
    MapType::const_iterator map_it_;
  };

  using const_stamp_iterator = typename MapType::const_key_iterator;
  using item_iterator = typename MapType::iterator;


  /////////////////////////////////////////////////////////////////////////////
  /// Constructors
  /////////////////////////////////////////////////////////////////////////////

  /** Constructs and initializes an empty TransformContainer. */
  TransformContainer(bool has_stamps, bool has_poses);

  /** Constructs and initializes a TransformContainer from unstamped <I>transforms</I>.
    * \throws std::invalid_argument if null pointers are added.
    */
  template<class T>
  TransformContainer(const std::vector<T> &transforms, bool has_poses);

  /** Constructs and initializes a TransformContainer from stamped <I>transforms</I>.
    * \throws std::invalid_argument if null pointers are added.
    */
  template<class T>
  TransformContainer(const std::map<StampType, T> &transforms, bool has_poses);

  /** Constructs and initializes a TransformContainer from separate <I>stamps</I> and <I>transforms</I>.
    * \throws std::invalid_argument if the sizes of <I>stamps</I> and <I>data</I> are not equal.
    * \throws std::invalid_argument if null pointers are added.
    */
  template<class T>
  TransformContainer(
    const std::vector<StampType> &stamps,
    const std::vector<T> &transforms, 
    bool has_poses,
    bool sorted_data = false);

  /** Copy constructor to copy all transforms. */
  TransformContainer(const TransformContainer &other);

  /** Delete copy assignment operator. */
  TransformContainer& operator=(const TransformContainer &other) = delete;

  /** Default move constructor. */
  TransformContainer(TransformContainer&& other) = default;

  /** Default move assignment operator. */
  TransformContainer& operator=(TransformContainer&& other) = default;

  /** Default destructor. */
  virtual ~TransformContainer() = default;


  /////////////////////////////////////////////////////////////////////////////
  /// Data Access
  /////////////////////////////////////////////////////////////////////////////

  /** \returns <CODE>true</CODE> if the container has stamped data. */
  bool hasStamps() const { return has_stamps_; }

  /** \returns <CODE>true</CODE> if the container has poses. */
  bool hasPoses() const { return has_poses_; }

  /** \returns <CODE>true</CODE> if the container has motions. */
  bool hasMotions() const { return !has_poses_; }

  /** \returns the number of elements. */
  std::size_t size() const;

  /** \returns <CODE>true</CODE> if the container has no elements. */
  bool empty() const;

  /** \returns <CODE>true</CODE> if the container contains the given <I>stamp</I>.
    * \throws TransformContainerException if the data is unstamped.
    */
  bool hasStamp(const StampType &stamp) const { return this->cfind_eq(stamp) != this->cend_items(); }

  /** \returns the element at <I>index</I>.
    * \throws std::out_of_range if <I>index</I> is out of range.
    */
  DataType& at(const std::size_t &index);

  /** \sa at(const std::size_t&) */
  DataTypeConst at(const std::size_t &index) const;

  /** \returns the element at <I>stamp</I>.
    * \throws TransformContainerException if the data is unstamped.
    * \throws std::out_of_range if <I>stamp</I> does not exist.
    */
  DataType& at_stamp(const StampType &stamp);

  /** \sa at_stamp(const StampType&) */
  DataTypeConst at_stamp(const StampType &stamp) const;

  /** \returns the stamp for the element at at <I>index</I>.
    * \throws TransformContainerException if the data is unstamped.
    * \throws std::out_of_range if <I>index</I> is out of range.
    */
  StampType stamp_at(const std::size_t &index) const;

  /** \returns the pair of stamp and element at <I>index</I>.
    * \throws TransformContainerException if the data is unstamped.
    * \throws std::out_of_range if <I>index</I> is out of range.
    */
  std::pair<const StampType, DataType>& item_at(const std::size_t &index);

  /** \sa item_at(const std::size_t&) */
  std::pair<const StampType, DataTypeConst> item_at(const std::size_t &index) const;

  /** Appends a new element.
    * \throws TransformContainerException if the data is stamped.
    */
  void append(const DataTypeConst &transform);

  /** Inserts a new element with <I>stamp</I> with the end as hint.
    * This is more efficient than insert(const StampType&, const DataTypeConst&, const bool),
    * when <I>stamp</I> is larger than all previously inserted timestamps and the data is really
    * inserted at the end.
    * However, it is is less efficient when the data is inserted somewhere else.
    * Existing stamps are not overwritten.
    * \throws TransformContainerException if the data is unstamped.
    */
  void append(const StampType &stamp, const DataTypeConst &transform);

  /** Inserts a new element at <I>index</I>.
    * \throws TransformContainerException if the data is stamped.
    */
  void insert(const std::size_t &index, const DataTypeConst &transform);

  /** Inserts a new element at <I>stamp</I>.
    * Use append(const StampType&, const DataTypeConst&) if the the new <I>stamp</I> is larger than all
    * previously inserted timestamps.
    * \returns if the data was inserted.
    * \throws TransformContainerException if the data is unstamped.
    */
  bool insert(const StampType &stamp, const DataTypeConst &transform, bool overwrite = false);

  /** Appends unstamped data <I>other</I> to <CODE>*this</CODE>.
    * \throws TransformContainerException if the data is stamped.
    */
  template<class T>
  void extend(const std::vector<T> &other);

  /** Inserts stamped data <I>other</I> to <CODE>*this</CODE>.
    * \throws TransformContainerException if the data is unstamped.
    */
  template<class T>
  void extend(const std::map<StampType, T> &other, bool overwrite = false);
  
  /** Extends <CODE>*this</CODE> with transforms from <CODE>other</CODE>.
    * \throws TransformContainerException if hasStamps() of <CODE>*this</CODE> and <CODE>other</CODE> do not match or if
    *   motions and poses are mixed.
    */
  void extend(const TransformContainer &other, bool overwrite = false);

  /** \sa extend(const TransformContainer&, bool) */
  void extend(const TransformContainer::ConstPtr &other, bool overwrite = false) { extend(*other, overwrite); }

  /** Removes the element at <I>index</I>. */
  void erase(const std::size_t &index);

  /** Removes the element at <I>stamp</I>.
    * \throws TransformContainerException if the data is unstamped.
    */
  void erase(const StampType &stamp);

  /** Removes the element corresponding to the <I>position</I> iterator.
    * \throws TransformContainerException if the data is unstamped.
    */
  void erase(iterator position);

  /** Removes all elements. */
  void clear();


  /////////////////////////////////////////////////////////////////////////////
  /// Iterator Access
  /////////////////////////////////////////////////////////////////////////////

  /** \returns an iterator to the first element. */
  iterator begin();

  /** \sa begin() */
  const_iterator cbegin() const;

  /** \sa begin() */
  const_iterator begin() const { return cbegin(); }


  /** \returns the past-the-end iterator corresponding to begin(). */
  iterator end();

  /** \sa end() */
  const_iterator cend() const;

  /** \sa end() */
  const_iterator end() const { return cend(); }


  /** \returns an iterator to the first stamp.
    * \throws TransformContainerException if the data is unstamped.
    */
  const_stamp_iterator cbegin_stamps() const;

  /** \sa cbegin_stamps() */
  const_stamp_iterator begin_stamps() const { return cbegin_stamps(); }


  /** \returns the past-the-end iterator corresponding to cbegin_stamps().
    * \throws TransformContainerException if the data is unstamped.
    */
  const_stamp_iterator cend_stamps() const;

  /** \sa cend_stamps() */
  const_stamp_iterator end_stamps() const { return cend_stamps(); }


  /** \returns an iterator to the first pair of stamp and element.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator begin_items();

  /** \sa begin_items() */
  const_item_iterator cbegin_items() const;

  /** \sa begin_items() */
  const_item_iterator begin_items() const { return cbegin_items(); }


  /** \returns the past-the-end iterator corresponding to begin_items().
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator end_items();

  /** \sa end_items() */
  const_item_iterator cend_items() const;

  /** \sa end_items() */
  const_item_iterator end_items() const { return cend_items(); }


  /////////////////////////////////////////////////////////////////////////////
  /// Find Methods
  /////////////////////////////////////////////////////////////////////////////

  /** \returns an iterator to the element at <I>stamp</I> or the end() iterator if no element was found.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator find_eq(const StampType &stamp);

  /** \sa find_eq(const StampType&) */
  const_item_iterator cfind_eq(const StampType &stamp) const;

  /** \sa find_eq(const StampType&) */
  const_item_iterator find_eq(const StampType &stamp) const { return cfind_eq(stamp); }


  /** \returns an iterator to the first element with time >= <I>stamp</I> or the end() iterator if no element was found.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator find_ge(const StampType &stamp);

  /** \sa find_ge(const StampType&) */
  const_item_iterator cfind_ge(const StampType &stamp) const;

  /** \sa find_ge(const StampType&) */
  const_item_iterator find_ge(const StampType &stamp) const { return cfind_ge(stamp); }


  /** \returns an iterator to the first element with time > <I>stamp</I> or the end() iterator if no element was found.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator find_gt(const StampType &stamp);

  /** \sa find_gt(const StampType&) */
  const_item_iterator cfind_gt(const StampType &stamp) const;

  /** \sa find_gt(const StampType&) */
  const_item_iterator find_gt(const StampType &stamp) const { return cfind_gt(stamp); }


  /** \returns an iterator to the last element with time <= <I>stamp</I> or the end() iterator if no element was found.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator find_le(const StampType &stamp);

  /** \sa find_le(const StampType&) */
  const_item_iterator cfind_le(const StampType &stamp) const;

  /** \sa find_le(const StampType&) */
  const_item_iterator find_le(const StampType &stamp) const { return cfind_le(stamp); }


  /** \returns an iterator to the last element with time < <I>stamp</I> or the end() iterator if no element was found.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator find_lt(const StampType &stamp);

  /** \sa find_lt(const StampType&) */
  const_item_iterator cfind_lt(const StampType &stamp) const;

  /** \sa find_lt(const StampType&) */
  const_item_iterator find_lt(const StampType &stamp) const { return cfind_lt(stamp); }


  /** \returns an iterator to the closest element to <I>stamp</I> or the end() iterator if the container is empty.
    * \throws TransformContainerException if the data is unstamped.
    */
  item_iterator find_closest(const StampType &stamp);

  /** \sa find_closest(const StampType&) */
  const_item_iterator cfind_closest(const StampType &stamp) const;

  /** \sa find_closest(const StampType&) */
  const_item_iterator find_closest(const StampType &stamp) const { return cfind_closest(stamp); }


  /////////////////////////////////////////////////////////////////////////////
  /// Transformations
  /////////////////////////////////////////////////////////////////////////////

  /** Inplace variant of removeStamps(). */
  TransformContainer& removeStamps_();

  /** \returns a TransformContainer with unstamped transforms. */
  TransformContainer removeStamps() const;

  /** Inplace variant of addStamps(). */
  TransformContainer& addStamps_(const std::vector<StampType> &stamps);

  /** \returns a TransformContainer with <I>stamps</I> added to the transforms.
    * \throws TransformContainerException if the container already has stamps or if the number of stamps does not match
    *   the number of transforms.
    */
  TransformContainer addStamps(const std::vector<StampType> &stamps) const;

  /** Inplace variant of asType() const. */
  template<class TransformClass>
  TransformContainer& asType_();

  /** \returns a TransformContainer with all transforms converted to the given <I>TransformClass</I>.
    * \tparam TransformClass the target transform class
    */
  template<class TransformClass>
  TransformContainer asType() const;

  /** Inplace variant of asType(const TransformType&) const. */
  TransformContainer& asType_(const TransformType &type);

  /** \returns a TransformContainer with all transforms converted to the given transform <I>type</I>.*/
  TransformContainer asType(const TransformType &type) const;

  /** Inplace variant of asPoses() const. */
  TransformContainer& asPoses_();

  /** \returns a TransformContainer with poses for unstamped data.
    * \li If <CODE>*this</CODE> is empty, an empty container is returned.
    * \li If <CODE>*this</CODE> contains \f$n\f$ motions, all motions are converted to \f$n+1\f$ poses and the identity
    *   transform is used as first pose.
    * \li If <CODE>*this</CODE> contains poses, the initial pose is retained.
    * \throws TransformContainerException for stamped data, since the timestamp for the additional transform would be
    *   unknown.
    * \sa asPoses(const DataTypeConst&) const
    */
  TransformContainer asPoses() const;

  /** Inplace variant of asPoses(const DataTypeConst&) const. */
  TransformContainer& asPoses_(const DataTypeConst &initial_pose);

  /** \returns a TransformContainer with poses and <I>initial_pose</I> as first pose for unstamped data.
    * \li If <CODE>*this</CODE> is empty, an empty container is returned.
    * \li If <CODE>*this</CODE> contains \f$n\f$ motions, all motions are converted to \f$n+1\f$ poses and
    *   <I>initial_pose</I> is used as first pose.
    * \li If <CODE>*this</CODE> contains poses, all transforms are adjusted so the first pose is <I>initial_pose</I>.
    * \throws TransformContainerException for stamped data, since the timestamp for the additional transform would be
    *   unknown.
    */
  TransformContainer asPoses(const DataTypeConst &initial_pose) const;

  /** Inplace variant of asMotions(). */
  TransformContainer& asMotions_();

  /** \returns a TransformContainer with motions for unstamped data.
    * \li If <CODE>*this</CODE> is empty or contains only a single pose, an empty container is returned.
    * \li If <CODE>*this</CODE> contains \f$n\f$ poses, \f$n-1\f$ motions are returned.
    * \li If <CODE>*this</CODE> contains motions, nothing is changed.
    * \throws TransformContainerException for stamped data, since it would be unclear, which timestamps should be
    *   retained.
    */
  TransformContainer asMotions() const;
  
  /** Inplace variant of inverse(). */
  TransformContainer& inverse_();

  /** Inverts all transforms. */
  TransformContainer inverse() const;

  /** Inplace variant of normalized(). */
  TransformContainer& normalized_();

  /** Normalizes all transforms. */
  TransformContainer normalized() const;

  /** Inplace variant of scaleTranslation(). */
  TransformContainer& scaleTranslation_(double factor);

  /** Scales translation of all transforms by <I>factor</I>. */
  TransformContainer scaleTranslation(double factor) const;

  /** Inplace variant of applyPre(). */
  template<class T>
  TransformContainer& applyPre_(const T &transform);

  /** Applies <I>transform</I> \f$T_\mathrm{pre}\f$ before each transform \f$T_{i}\f$ of <CODE>*this</CODE>:
    * \f[T_\mathrm{pre} \circ T_{i}\f]
    */
  template<class T>
  TransformContainer applyPre(const T &transform) const;

  /** Inplace variant of applyPost(). */
  template<class T>
  TransformContainer& applyPost_(const T &transform);

  /** Applies <I>transform</I> \f$T_\mathrm{post}\f$ after each transform \f$T_{i}\f$ of <CODE>*this</CODE>:
    * \f[T_{i} \circ T_\mathrm{post}\f]
    */
  template<class T>
  TransformContainer applyPost(const T &transform) const;

  /** Inplace variant of apply(). */
  template<class T1, class T2>
  TransformContainer& apply_(const T1 &transform_pre, const T2 &transform_post);

  /** Applies <I>transform_pre</I> \f$T_\mathrm{pre}\f$ and <I>transform_post</I> \f$T_\mathrm{post}\f$
    * before and after each transform \f$T_{i}\f$ of <CODE>*this</CODE>, respectively:
    * \f[T_\mathrm{pre} \circ T_{i} \circ T_\mathrm{post}\f]
    */
  template<class T1, class T2>
  TransformContainer apply(const T1 &transform_pre, const T2 &transform_post) const;

  /** Inplace variant of applyFunc(). */
  TransformContainer& applyFunc_(const std::function<DataType(const DataType&)> &func);

  /** Applies <I>func</I> on each transform. */
  TransformContainer applyFunc(const std::function<DataType(const DataTypeConst&)> &func) const;

  /** Inplace variant of applyIndexFunc(). */
  TransformContainer& applyIndexFunc_(const std::function<DataType(std::size_t, const DataType&)> &func);

  /** Applies <I>func</I> on each transform. */
  TransformContainer applyIndexFunc(const std::function<DataType(std::size_t, const DataTypeConst&)> &func) const;

  /** Inplace variant of applyStampFunc(). */
  TransformContainer& applyStampFunc_(const std::function<DataType(const StampType&, const DataType&)> &func);

  /** Applies <I>func</I> on each transform.
    * \throws TransformContainerException for unstamped data
    */
  TransformContainer applyStampFunc(const std::function<DataType(const StampType&, const DataTypeConst&)> &func) const;

  /** Inplace variant of changeFrame(). */
  template<class T>
  TransformContainer& changeFrame_(const T &transform);

  /** Changes the coordinate frame of the transforms using the given frame-forward <I>transform</I>
    * \f$T_\mathrm{frame}\f$.
    * \f[T_\mathrm{frame}^{-1} \circ T_{i} \circ T_\mathrm{frame}\f]
    */
  template<class T>
  TransformContainer changeFrame(const T &transform) const;


  /////////////////////////////////////////////////////////////////////////////
  /// Conversions
  /////////////////////////////////////////////////////////////////////////////

  /** \returns a vector with all transforms converted to the given <I>TransformClass</I>.
    * \tparam TransformClass the target transform class
    */
  template<class TransformClass>
  std::vector<TransformClass> toVector() const;

  /** \returns a vector with copies of all transforms in their original transform type.*/
  std::vector<DataType> toVector() const;

  /** \returns a vector with all transforms converted to the given transform <I>type</I>.*/
  std::vector<DataType> toVector(const TransformType &type) const;

  /** \returns an Eigen vector of all transforms converted to the given <I>TransformClass</I>
    *   where each row represents a single transformation.
    * \tparam TransformClass the target transform class
    */
  template<class TransformClass>
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> toEigenVector() const;

  /** \returns an Eigen vector of all transforms converted to the given transform <I>type</I> where each row represents
    *   a single transformation.
    */
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> toEigenVector(const TransformType &type) const;

  /** \returns a description of <CODE>*this</CODE>. */
  std::string desc() const { return streamToString(*this); }
  
  /** Inserts a description of <CODE>*this</CODE>. */
  friend std::ostream& operator<<(std::ostream& os, const TransformContainer& container);

 private:
  /** Converts motions to poses inplace, using <I>initial_pose</I> as first pose.
    * If <I>initial_pose</I> is a <CODE>nullptr</CODE>, the identity transform is used as first pose.
    * \throws TransformContainerException for stamped data, since the timestamp for the additional transform would be
    *   unknown.
    */
  void motionsToPoses(const DataTypeConst &initial_pose);

  VectorType& vector() { return vector_; };
  const VectorType& vector() const { return vector_; };

  MapType& map() { return map_; };
  const MapType& map() const { return map_; };

  VectorType vector_;  ///< Container for unstamped data.
  MapType map_;        ///< Container for stamped data.
  bool has_stamps_;    ///< Store if <CODE>*this</CODE> contains stamped or unstamped data.
  bool has_poses_;     ///< Store if <CODE>*this</CODE> contains motions or poses.

}; // class TransformContainer

} // namespace motion3d
