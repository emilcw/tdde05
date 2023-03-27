#include <cassert>
#include <mutex>
#include <vector>

template<typename _T_, int _SubSize_ = 1000>
class extensible_grid
{
    template<typename _Mutex>
    class releasable_lock_guard
    {
    public:
      typedef _Mutex mutex_type;

      explicit releasable_lock_guard(mutex_type& __m) : _M_device(__m)
      { _M_device.lock(); }

      releasable_lock_guard(mutex_type& __m, std::adopt_lock_t) : _M_device(__m)
      { } // calling thread owns mutex

      ~releasable_lock_guard()
      { release(); }

      void release()
      {
        if(not _M_released)
        {
          _M_device.unlock();
          _M_released = true;
        }
      }
      
      releasable_lock_guard(const releasable_lock_guard&) = delete;
      releasable_lock_guard& operator=(const releasable_lock_guard&) = delete;

    private:
      mutex_type&  _M_device;
      bool _M_released = false;
    };

public:
  /**
   * Iterator used to iterate over the subgrids
   */
  template<typename _TT_>
  struct subgrid_iterator_impl
  {
    subgrid_iterator_impl(_TT_ _values, int _left, int _top, int _rows) :
     m_values(_values), m_left(_left), m_top(_top), m_rows(_rows)
     {}
  public:
    /**
     * Access cell of the subgrid.
     * The coordinate _x and _y are given in the frame of the subgrid
     */
    _T_& operator()(int _x, int _y) { return (*m_values)[_x + _y * _SubSize_]; }
    bool operator==(const subgrid_iterator_impl<_TT_>& _rhs) { return m_values == _rhs.m_values; }
    bool operator!=(const subgrid_iterator_impl<_TT_>& _rhs) { return m_values != _rhs.m_values; }
    /**
     * Move to the next subgrid
     */
    subgrid_iterator_impl<_TT_>& operator++()
    {
      ++m_values;
      ++m_left;
      if(m_left >= m_rows)
      {
        m_left = 0;
        ++m_top;
      }
      return *this;
    };
  public:
    /**
     * @return true if it correspond to a valid subgrid
     */
    bool is_valid() const { return *m_values; }
    /**
     * @return the top coordinate (in cell)
     */
    int get_top() const { return m_top * _SubSize_; }
    /**
     * @return the left coordinate (in cell)
     */
    int get_left() const { return m_left * _SubSize_; }
    /**
     * @return the size of the subgrid
     */
    int get_size() const { return _SubSize_; }
  private:
    _TT_ m_values;
    int m_left, m_top, m_rows;
  };
  typedef subgrid_iterator_impl<_T_**> subgrid_iterator;
  typedef subgrid_iterator_impl<_T_* const *> subgrid_const_iterator;
public:
  /**
   * @param _resolution grid resolution in m
   */
  extensible_grid(double _resolution) : m_resolution(_resolution), m_x(0), m_y(0), m_width(0), m_height(0) {}
  ~extensible_grid()
  {
    for(int i = 0; i < m_width * m_height; ++i)
    {
      delete[] m_subgrids[i];
    }
  }
  /**
   * @return a reference to the cell that contains coordinates (_x, _y)
   */
  _T_& get_value_ref(double _x, double _y)
  {
    int ix = index(_x);
    int iy = index(_y);
    int ox, oy;
    return subgrid(ix, iy, &ox, &oy)[ox + oy * _SubSize_];
  }
  subgrid_iterator begin()
  {
    return subgrid_iterator(&(*m_subgrids.begin()), 0, 0, m_width);
  }
  subgrid_iterator end()
  {
    return subgrid_iterator(&(*m_subgrids.end()), 0, m_height, m_width);
  }
  subgrid_const_iterator cbegin() const
  {
    return subgrid_const_iterator(&(*m_subgrids.cbegin()), 0, 0, m_width);
  }
  subgrid_const_iterator cend() const
  {
    return subgrid_const_iterator(&(*m_subgrids.cend()), 0, m_height, m_width);
  }
  /**
   * @return the origin of the grid, in m
   */
  double get_origin_x() const
  {
    return m_x * m_resolution * _SubSize_;
  }
  /**
   * @return the origin of the grid, in m
   */
  double get_origin_y() const
  {
    return m_y * m_resolution * _SubSize_;
  }
  /**
   * @return the number of rows of the grid, in cells
   */
  int get_rows_count() const
  {
    return m_width * _SubSize_;
  }
  /**
   * @return the number of column of the grid, in cells
   */
  int get_columns_count() const
  {
    return m_height * _SubSize_;
  }
private:
  int index(double _v) const
  {
    return _v / m_resolution;
  }
  _T_* subgrid(int _ix, int _iy, int* _ox, int* _oy)
  {
    releasable_lock_guard<std::mutex> l(m_mutex);
    int si = _ix / _SubSize_ - m_x;
    int sj = _iy / _SubSize_ - m_y;
    if(_ix < 0) si -= 1;
    if(_iy < 0) sj -= 1;

    if(si < 0 or sj < 0 or si >= m_width or sj >= m_height)
    {
      int nx = m_x;
      int ny = m_y;
      int nw = m_width;
      int nh = m_height;
      if(si < 0)
      {
        nx = nx + si;
        nw = nw - si;
      }
      if(sj < 0)
      {
        ny = ny + sj;
        nh = nh - sj;
      }
      if(si >= m_width)
      {
        nw = si + 1;
      }
      if(sj >= m_height)
      {
        nh = sj + 1;
      }
      
      std::vector<_T_*> nsubs(nw*nh, nullptr);
      for(int k = 0; k < m_subgrids.size(); ++k)
      {
        if(m_subgrids[k])
        {
          int s_i = (k % m_width) + m_x - nx;
          int s_j = (k % m_height) + m_y - ny;
          nsubs[s_i + s_j * nw] = m_subgrids[k];
        }
      }
      m_x = nx;
      m_y = ny;
      m_width = nw;
      m_height = nh;
      std::swap(m_subgrids, nsubs);
      l.release();
      return subgrid(_ix, _iy, _ox, _oy);
    } else {
      *_ox = _ix - (si + m_x) * _SubSize_;
      *_oy = _iy - (sj + m_y) * _SubSize_;
      
      assert(si + sj*m_width < m_subgrids.size());
      _T_*& sg = m_subgrids[si + sj*m_width];

      if(sg == nullptr)
      {
        sg = new _T_[_SubSize_ * _SubSize_];
      }
      return sg;
    }
  }
private:
  std::mutex m_mutex;
  double m_resolution;
  int m_x, m_y, m_width, m_height;
  std::vector<_T_*> m_subgrids;
};
