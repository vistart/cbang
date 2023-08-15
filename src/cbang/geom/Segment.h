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


namespace cb {
    template <const unsigned DIM, typename T>
    class Segment : public Vector<2, Vector<DIM, T> > { // 线段定义为2个同维度同元素类型的向量构成。
    public:
        typedef Vector<2, Vector<DIM, T> > Super_T; // 将线段类型重定义为Super_T。
        using Super_T::data;

        Segment() : Super_T(Vector<DIM, T>()) {} // 实例化，空内容。
        Segment(const Super_T &v) : Super_T(v) {} // 实例化，以另一个线段为准，即复制一份。
        Segment(const Vector<DIM, T> &p1, const Vector<DIM, T> &p2) : // 实例化，以线段两个端点向量为准。
                Vector<2, Vector<DIM, T> >(p1, p2) {}

        const Vector<DIM, T> &getStart() const {return data[0];} // 获得线段起点，即维度0数据。
        Vector<DIM, T> &getStart() {return data[0];}
        const Vector<DIM, T> &getEnd() const {return data[1];} // 获得线段中点，即维度1数据。
        Vector<DIM, T> &getEnd() {return data[1];}

        T length() const {return data[0].distance(data[1]);} // 获得线段长度，也即两个向量的距离。
        T lengthSquared() const {return data[0].distanceSquared(data[1]);} // 获得线段长度平方和，也即两个向量每个维度值的平方和。


        bool intersection(const Segment<DIM, T> &s, Vector<DIM, T> &p) const { // 求俩线段的交点，目前只实现了二维情况，非二维情况会报错。p为交点结果。
            // TODO Currently only implemented for the 2D case
            if (DIM != 2)
                CBANG_THROW("Invalid operation for Segment of dimension " << DIM);

            T d = (s.data[1][1] - s.data[0][1]) * (data[1][0] - data[0][0]) -
                  (s.data[1][0] - s.data[0][0]) * (data[1][1] - data[0][1]); // 计算两个向量的距离

            if (!d) return false; // Parallel 平行即为不想交。

            d = 1.0 / d; // 取倒数。

            T ua = ((s.data[1][0] - s.data[0][0]) * (data[0][1] - s.data[0][1]) -
                    (s.data[1][1] - s.data[0][1]) * (data[0][0] - s.data[0][0])) * d;
            if (ua <= 0 || 1 <= ua) return false;

            T ub = ((data[1][0] - data[0][0]) * (data[0][1] - s.data[0][1]) -
                    (data[1][1] - data[0][1]) * (data[0][0] - s.data[0][0])) * d;
            if (ub <= 0 || 1 <= ub) return false;

            p[0] = data[0][0] + ua * (data[1][0] - data[0][0]);
            p[1] = data[0][1] + ua * (data[1][1] - data[0][1]);

            return true;
        }


        bool intersects(const Segment<DIM, T> &s) const { // 判断俩线段是否相交。
            Vector<DIM, T> p;
            return intersection(s, p);
        }


        T distance(const Vector<DIM, T> &p) const { // 求当前线段到指定点的距离。
            return p.distance(closest(p));
        }

        T distance(const Vector<DIM, T> &p, Vector<DIM, T> &c) const { // 求当前线段到指定点的距离，并得出最近点。
            return p.distance(c = closest(p));
        }

        Vector<DIM, T> closest(const Vector<DIM, T> &p) const { // 求当前线段上与指定点距离最近的点。
            T len2 = lengthSquared();
            if (!len2) return data[0];

            T t = (p - data[0]).dot(data[1] - data[0]) / len2;

            if (t <= 0) return data[0];
            if (1 <= t) return data[1];

            return data[0] + (data[1] - data[0]) * t;
        }

        Vector<DIM, T> closest(const Vector<DIM, T> &p, T &dist) const { // 求当前线段上与指定点距离最近的点的值。
            Vector<DIM, T> c = closest(p);
            dist = p.distance(c);
            return c;
        }


        // Cast
        template <typename U>
        operator Segment<DIM, U>() const {return Segment<DIM, U>(data[0], data[1]);} // 类型转换，也即转换为维度相同数据类型不同的另一个类的实例。
    };


    typedef Segment<2, int> Segment2I; // 二维整数向量线段
    typedef Segment<2, unsigned> Segment2U; // 二维非负整数向量线段，即线段只出现在第一象限。
    typedef Segment<2, double> Segment2D; // 二维双精度浮点数向量线段
    typedef Segment<2, float> Segment2F; // 二维单精度浮点数向量线段

    typedef Segment<3, int> Segment3I;  // 三维整数向量线段
    typedef Segment<3, unsigned> Segment3U; // 三维非负整数向量线段，即线段只出现在第一卦限。
    typedef Segment<3, double> Segment3D; // 三维双精度浮点数向量线段
    typedef Segment<3, float> Segment3F; // 三维单精度浮点数向量线段
}
