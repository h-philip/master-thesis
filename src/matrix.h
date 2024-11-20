#pragma once

#include <functional>
#include <numeric>
#include <vector>
#include <stdexcept>

namespace dynamic_programming {

  template <typename T>
  class matrix {
  public:
    matrix(const long dim0, const size_t dim1, const size_t dim2, const size_t dim3, const size_t dim4, const size_t dim5, const size_t dim6) :
      m_dim0(dim1* dim2* dim3* dim4* dim5* dim6),
      m_dim1(dim2* dim3* dim4* dim5* dim6),
      m_dim2(dim3* dim4* dim5* dim6),
      m_dim3(dim4* dim5* dim6),
      m_dim4(dim5* dim6),
      m_dim5(dim6),
      m_dim6(1),
      m_nelem(dim0* dim1* dim2* dim3* dim4* dim5* dim6)
    {
      if (dim0 <= 0)
        throw std::invalid_argument("dim0 can't be 0 or smaller");
      m_data = new T[m_nelem];
    }

    ~matrix()
    {
      delete[] m_data;
    }

    const T& at(int i) const
    {
      return m_data[i];
    }

    T& at(int i)
    {
      return m_data[i];
    }

    const T& at(const long dim0, const size_t dim1, const size_t dim2, const size_t dim3, const size_t dim4, const size_t dim5, const size_t dim6) const
    {
      size_t flat_index = m_dim0 * dim0 + m_dim1 * dim1 + m_dim2 * dim2 + m_dim3 * dim3 + m_dim4 * dim4 + m_dim5 * dim5 + m_dim6 * dim6;
      return m_data[flat_index];
    }

    T& at(const long dim0, const size_t dim1, const size_t dim2, const size_t dim3, const size_t dim4, const size_t dim5, const size_t dim6)
    {
      size_t flat_index = m_dim0 * dim0 + m_dim1 * dim1 + m_dim2 * dim2 + m_dim3 * dim3 + m_dim4 * dim4 + m_dim5 * dim5 + m_dim6 * dim6;
      return m_data[flat_index];
    }

    size_t nelem() const
    {
      return m_nelem;
    }

  private:
    size_t m_nelem;
    size_t m_dim0;
    size_t m_dim1;
    size_t m_dim2;
    size_t m_dim3;
    size_t m_dim4;
    size_t m_dim5;
    size_t m_dim6;
    T* m_data;
  };
}