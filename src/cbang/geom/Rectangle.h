/******************************************************************************\

          This file is part of the C! library.  A.K.A the cbang library.

                Copyright (c) 2003-2023, Cauldron Development LLC
                               All rights reserved.

         The C! library is free software: you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public License
       as published by the Free Software Foundation, either version 2.1 of
               the License, or (at your option) any later version.

        The C! library is distributed in the hope that it will be useful,
          but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
                 Lesser General Public License for more details.

         You should have received a copy of the GNU Lesser General Public
                 License along with the C! library.  If not, see
                         <http://www.gnu.org/licenses/>.

        In addition, BSD licensing may be granted on a case by case basis
        by written permission from at least one of the copyright holders.
           You may request written permission by emailing the authors.

                  For information regarding this software email:
                                 Joseph Coffland
                          joseph@cauldrondevelopment.com

\******************************************************************************/

#pragma once

#include "Vector.h"
#include "Segment.h"

#include <algorithm>
#include <limits>

#if defined(_WIN32) && defined(max)
#undef max
#endif
#if defined(_WIN32) && defined(min)
#undef min
#endif


namespace cb {
  /// Really an axis aligned box 实际上为轴对齐框，而非平面、空间内任意方向。
  template <const unsigned DIM, typename T>
  class Rectangle {
  public:
    Vector<DIM, T> rmin; // 对角顶点：低点
    Vector<DIM, T> rmax; // 对角顶点：高点。高点和地点需要自行保证大小关系。

    Rectangle() : rmin(std::numeric_limits<T>::max()), // 实例化，低点和高点是负无穷大和正无穷大。
                  rmax(-std::numeric_limits<T>::max()) {}

    Rectangle(const Rectangle<DIM, T> &o) : rmin(o.rmin), rmax(o.rmax) {} // 实例化，以另一个为准，即复制一份。
    Rectangle(const Vector<DIM, T> &p1, const Vector<DIM, T> &p2) : // 实例化，以高低两个顶点为准。
      rmin(p1), rmax(p2) {
      for (unsigned i = 0; i < DIM; i++)
        if (rmax[i] < rmin[i]) std::swap(rmax[i], rmin[i]);
    }

    const Vector<DIM, T> &getMin() const {return rmin;} // 获得低点
    const Vector<DIM, T> &getMax() const {return rmax;} // 获得高点

    void setMin(const Vector<DIM, T> &rmin) {this->rmin = rmin;} // 设置低点
    void setMax(const Vector<DIM, T> &rmax) {this->rmax = rmax;} // 设置高点

    T getWidth() const {return rmax[0] - rmin[0];} // 获得宽度，也即高点0维数据减低点0维数据。
    T getLength() const { // 获得长度，也即高点1维数据减低点1维数据。维度小于2时报错。
      if (DIM < 2)
        CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
      return rmax[1] - rmin[1];
    }
    T getHeight() const { // 获得高度，也即高点2维数据减低点2维数据。维度小于3时报错。
      if (DIM < 3)
        CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
      return rmax[2] - rmin[2];
    }

    T getVolume() const { // 获得体积，也即每个维度的积。
      T volume = 1;
      for (unsigned i = 0; i < DIM; i++) volume *= getDimension(i);
      return volume;
    }


    T getDimension(unsigned i) const {return rmax[i] - rmin[i];} // 获得指定维度的差值。

    Vector<DIM, T> getDimensions() const { // 获得每个维度差值的向量。
      Vector<DIM, T> d;
      for (unsigned i = 0; i < DIM; i++) d[i] = getDimension(i);
      return d;
    }

    Vector<DIM, T> getCenter() const {return (rmin + rmax) / 2.0;} // 求矩形中心坐标向量。

    Vector<DIM, T> closestPoint(const Vector<DIM, T> &p) const { // 求矩形表面距离指定点最近的点坐标。p在矩形外。
      Vector<DIM, T> mid = getCenter();
      Vector<DIM, T> half = mid - rmin;
      Vector<DIM, T> closest;

      for (unsigned i = 0; i < DIM; i++)
        closest[i] =
          std::min(std::max(p[i], mid[i] - half[i]), mid[i] + half[i]);

      return closest;
    }

    Vector<DIM, T> closestPointOnSurface(const Vector<DIM, T> &p) const { // 求矩形表面距离指定点最近的点坐标。p在矩形内。
      if (!contains(p)) return closestPoint(p);

      // Find closest point on surface, given that p is inside

      // First find the closest face
      float faceDist[DIM * 2];
      for (unsigned i = 0; i < DIM; i++) {
        faceDist[i * 2] = p[i] - rmin[i];
        faceDist[i * 2 + 1] = rmax[i] - p[i];
      }

      unsigned face = 0;
      for (unsigned i = 1; i < DIM * 2; i++)
        if (faceDist[i] < faceDist[face]) face = i;

      // The closest point is on the closest face
      Vector<DIM, T> closest = p;
      if (face & 1) closest[face / 2] = rmax[face / 2];
      else closest[face / 2] = rmin[face / 2];

      return closest;
    }


    bool contains(const Vector<DIM, T> &p) const { // 判断p点是否在矩形内。
      for (unsigned i = 0; i < DIM; i++)
        if ((p[i] < rmin[i]) || (rmax[i] < p[i])) return false;
      return true;
    }

    bool contains(const Rectangle<DIM, T> &r) const { // 判断r矩形是否在当前矩形内，也即判断r的高低点是否在当前高低点范围内。
      return contains(r.rmin) && contains(r.rmax);
    }

    bool contains(const Segment<DIM, T> &s) const { // 判断s线段是否在当前矩形内，也即判断s的两端是否在高低点范围内。
      return contains(s.p1) && contains(s.p2);
    }


    Rectangle<DIM, T> intersection(const Rectangle<DIM, T> &o) const { // 求两个矩形的相交矩形。
      Rectangle<DIM, T> r;

      for (unsigned i = 0; i < DIM; i++) {
        r.rmax[i] = std::min(rmax[i], o.rmax[i]);
        r.rmin[i] = std::max(rmin[i], o.rmin[i]);
      }

      return r;
    }


    bool intersects(const Rectangle<DIM, T> &o) const { // 判断两个矩形是否相交。
      for (unsigned i = 0; i < DIM; i++)
        if ((o.rmax[i] < rmin[i]) || (rmax[i] < o.rmin[i])) return false;
      return true;
    }


    bool intersects(const Segment<DIM, T> &s) const { // 判断当前矩形是否与指定线段相交。
      if (contains(s.p1) || contains(s.p2)) return true;

      if (DIM == 1) return false; // 若维度为1，则不认为相交。

      else if (DIM == 2) { // 若维度为2，则判断四个角是否将线段包含其中。
        Vector<2, T> corners[4] = {
          Vector<2, T>(rmin.x(), rmin.y()),
          Vector<2, T>(rmax.x(), rmin.y()),
          Vector<2, T>(rmin.x(), rmax.y()),
          Vector<2, T>(rmax.x(), rmax.y()),
        };

        return
          s.intersects(Segment<2, T>(corners[0], corners[1])) ||
          s.intersects(Segment<2, T>(corners[1], corners[2])) ||
          s.intersects(Segment<2, T>(corners[2], corners[3])) ||
          s.intersects(Segment<2, T>(corners[3], corners[1]));

      }

      // NOTE Not sure this works for dimensions higher than 3
      // 注意！不确定维度超过3时是否还起作用。
      Vector<DIM - 1, T> rminl;
      Vector<DIM - 1, T> rmaxl;
      Segment<DIM - 1, T> sl;

      for (int i = 0; i < DIM; i++) {
        int k = 0;

        for (int j = 0; j < DIM; j++) {
          if (j == i) continue;
          rminl[k] = rmin[j];
          rmaxl[k] = rmax[j];
          sl.p1[k] = s.p1[j];
          sl.p2[k] = s.p2[j];
          k++;
        }

        // Intersect in lower dimension 递归计算低纬度是否相交。
        if (!Rectangle<DIM - 1, T>(rminl, rmaxl).intersects(sl))
          return false;
      }

      return true;
    }


    bool intersects(const Segment<DIM, T> &s, Vector<DIM, T> &p1, // 判断是否与线段s相交，若是，则求交点p1和p2。
                    Vector<DIM, T> &p2) const {
      // TODO Currently only implemented for the 2D case
      if (DIM != 2)
        CBANG_THROW("Invalid operation for Vector of dimension " << DIM);

      // NOTE This does not work if the segment is wholly inside the square.
      //   Of course then there are no intersection points either.
      // 注意，此方法在线段整体包含在矩形内时不起作用。当然，此时也没有交点。
      Vector<DIM, T> nil(std::numeric_limits<T>::max(),
                         std::numeric_limits<T>::max());
      p1 = p2 = nil;

      Segment<DIM, T> segs[4] = {
        Segment<DIM, T>(Vector<DIM, T>(rmin.x(), rmin.y()),
                        Vector<DIM, T>(rmax.x(), rmin.y())),
        Segment<DIM, T>(Vector<DIM, T>(rmax.x(), rmin.y()),
                        Vector<DIM, T>(rmax.x(), rmax.y())),
        Segment<DIM, T>(Vector<DIM, T>(rmax.x(), rmax.y()),
                        Vector<DIM, T>(rmin.x(), rmax.y())),
        Segment<DIM, T>(Vector<DIM, T>(rmin.x(), rmax.y()),
                        Vector<DIM, T>(rmin.x(), rmin.y())),
      };

      Vector<DIM, T> x;
      for (int i = 0; i < 4; i++)
        if (segs[i].intersection(s, x))
          if (x != p1 && x != p2) {
            if (p1 == nil) p1 = x;
            else if (p2 == nil) p2 = x;
            else break;
          }

      return p1 != nil;
    }


    void add(const Vector<DIM, T> &p) { // 累加指定点。
      for (unsigned i = 0; i < DIM; i++) {
        if (rmax[i] < p[i]) rmax[i] = p[i]; // 若高点某个维度小于指定点对应维度，则更新。
        if (p[i] < rmin[i]) rmin[i] = p[i]; // 若低点某个维度大于指定点对应维度，则更新。
      }
    }

    void add(const Rectangle<DIM, T> &r) { // 累加指定矩形。
      for (unsigned i = 0; i < DIM; i++) {
        if (r.rmin[i] < rmin[i]) rmin[i] = r.rmin[i]; // 若当前矩形高点某个维度小于指定矩形高点对应维度，则更新。
        if (rmax[i] < r.rmax[i]) rmax[i] = r.rmax[i]; // 若当前矩形高点某个维度大于指定矩形高点对应维度，则更新。
      }
    }


    template <unsigned LEN>
    Rectangle<LEN, T> slice(unsigned start = 0) const { // 从start维度开始取部分纬度值的高低点组成新的矩形。（降维）
      return Rectangle<LEN, T>(rmin.template slice<LEN>(start),
                               rmax.template slice<LEN>(start));
    }


    Rectangle<DIM, T> grow(const Vector<DIM, T> &amount) const { // 扩张，高点和地点扩张指定幅度amount。
      return Rectangle<DIM, T>(rmin - amount, rmax + amount);
    }


    Rectangle<DIM, T> shrink(const Vector<DIM, T> &amount) const { // 缩小，高点和地点缩小指定幅度amount。
      return Rectangle<DIM, T>(rmin + amount, rmax - amount);
    }

    const Vector<DIM, T> &operator[](unsigned i) const { // 运算符方括号重载：取地点或高点。如果i大于1则报错。
      if (1 < i) CBANG_THROW("Invalid Rectangle index" << i);
      return i ? rmax : rmin;
    }

    Vector<DIM, T> &operator[](unsigned i) { // 运算符方括号重载：取地点或高点。如果i大于1则报错。
      if (1 < i) CBANG_THROW("Invalid Rectangle index " << i);
      return i ? rmax : rmin;
    }

    // Assign
    Rectangle<DIM, T> &operator=(const Rectangle<DIM, T> &r) { // 运算符等号重载。即赋值两个端点。
      rmin = r.rmin;
      rmax = r.rmax;
      return *this;
    }

    // Compare
    bool operator==(const Rectangle<DIM, T> &r) const { // 比较运算符判相等重载。两个端点相等即视为相等，否则不等。
      return rmin == r.rmin && rmax == r.rmax;
    }

    bool operator!=(const Rectangle<DIM, T> &r) const { // 比较运算符判不等重载。两个端点相等即视为相等，否则不等。
      return rmin != r.rmin || rmax != r.rmax;
    }

    // Arithmetic
    Rectangle<DIM, T> operator+(const Rectangle<DIM, T> &r) const { // 运算符重载：加号。两个矩形的两个端点各自相加组成新的矩形。
      return Rectangle<DIM, T>(getMin() + r.getMin(), getMax() + r.getMax());
    }

    Rectangle<DIM, T> operator-(const Rectangle<DIM, T> &r) const { // 运算符重载：减号。两个矩形的两个端点各自相减组成新的矩形。
      return Rectangle<DIM, T>(getMin() - r.getMin(), getMax() - r.getMax());
    }

    Rectangle<DIM, T> operator*(const Rectangle<DIM, T> &r) const { // 运算符重载：乘号。两个矩形的两个端点各自相乘组成新的矩形。
      return Rectangle<DIM, T>(getMin() * r.getMin(), getMax() * r.getMax());
    }

    Rectangle<DIM, T> operator/(const Rectangle<DIM, T> &r) const { // 运算符重载：除号。两个矩形的两个端点各自相除组成新的矩形。
      return Rectangle<DIM, T>(getMin() / r.getMin(), getMax() / r.getMax());
    }


    Rectangle<DIM, T> operator+(const Vector<DIM, T> &v) const { // 运算符重载：加号。两个端点累加指定幅度形成新的矩形。
      return Rectangle<DIM, T>(getMin() + v, getMax() + v);
    }

    Rectangle<DIM, T> operator-(const Vector<DIM, T> &v) const { // 运算符重载：减号。两个端点累减指定幅度形成新的矩形。
      return Rectangle<DIM, T>(getMin() - v, getMax() - v);
    }

    Rectangle<DIM, T> operator*(const Vector<DIM, T> &v) const { // 运算符重载：乘号。两个端点累乘指定幅度形成新的矩形。
      return Rectangle<DIM, T>(getMin() * v, getMax() * v);
    }

    Rectangle<DIM, T> operator/(const Vector<DIM, T> &v) const { // 运算符重载：除号。两个端点累除指定幅度形成新的矩形。
      return Rectangle<DIM, T>(getMin() / v, getMax() / v);
    }


    Rectangle<DIM, T> operator+(T x) const { // 运算符重载：加号。两个端点累加指定值形成新的矩形。
      return Rectangle<DIM, T>(getMin() + x, getMax() + x);
    }

    Rectangle<DIM, T> operator-(T x) const { // 运算符重载：减号。两个端点累减指定值形成新的矩形。
      return Rectangle<DIM, T>(getMin() - x, getMax() - x);
    }

    Rectangle<DIM, T> operator*(T x) const { // 运算符重载：乘号。两个端点累乘指定值形成新的矩形。
      return Rectangle<DIM, T>(getMin() * x, getMax() * x);
    }

    Rectangle<DIM, T> operator/(T x) const { // 运算符重载：除号。两个端点累除指定值形成新的矩形。
      return Rectangle<DIM, T>(getMin() / x, getMax() / x);
    }

    // Math
    bool isReal() const {return rmin.isReal() || rmax.isReal();} // 判断矩形是否为有限体积，也即判定两端的每个维度是否为具体值。

    bool isEmpty() const { // 判断矩形是否为空，也即判断两个端点是否相同。
      for (unsigned i = 0; i < DIM; i++)
        if (rmin[i] == rmax[i]) return true;
      return false;
    }

    bool isValid() const { // 判断矩形是否有效。如果当前矩形为无限体积，则为无效。其次，若两个端点不满足严格小于，也视为无效。
      if (!isReal()) return false;
      for (unsigned i = 0; i < DIM; i++)
        if (rmax[i] < rmin[i]) return false;
      return true;
    }

    // Cast
    template <typename U>
    operator Rectangle<DIM, U>() const { // 类型转换，即转换为维度相同，数据类型不同的另一个矩形。
      return Rectangle<DIM, U>(rmin, rmax);
    }

    std::ostream &print(std::ostream &stream) const // 输出到输出流，以括号包裹，逗号分隔两个端点。
    {return stream << '(' << rmin << ", " << rmax << ')';}


    void read(const cb::JSON::Value &value) { // 读取JSON内容。
      rmin.read(value.getList("min"));
      rmax.read(value.getList("max"));
    }


    void write(JSON::Sink &sink) const { // 写入JSON。
      sink.beginDict();
      sink.beginInsert("min");
      rmin.write(sink);
      sink.beginInsert("max");
      rmax.write(sink);
      sink.endDict();
    }
  };


  template<const unsigned DIM, typename T> static inline
  std::ostream &operator<<(std::ostream &stream, const Rectangle<DIM, T> &r) { // 运算符<<重载，输出到指定输出流。
    return r.print(stream);
  }


  typedef Rectangle<2, int> Rectangle2I; // 平面整数坐标矩形。
  typedef Rectangle<2, unsigned> Rectangle2U; // 平面非负整数坐标矩形。
  typedef Rectangle<2, double> Rectangle2D; // 平面双精度浮点数坐标矩形。
  typedef Rectangle<2, float> Rectangle2F; // 平面单精度浮点数坐标矩形。

  typedef Rectangle<3, int> Rectangle3I; // 立体整数坐标矩形。
  typedef Rectangle<3, unsigned> Rectangle3U; // 立体非负整数坐标矩形。
  typedef Rectangle<3, double> Rectangle3D; // 立体双精度浮点数坐标矩形。
  typedef Rectangle<3, float> Rectangle3F; // 立体单精度浮点数坐标矩形。
}
