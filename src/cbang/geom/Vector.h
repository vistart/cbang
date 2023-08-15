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

#include <cbang/Exception.h>
#include <cbang/String.h>
#include <cbang/Math.h>

#include <cbang/json/List.h>
#include <cbang/json/Sink.h>

#include <string>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <limits>

#if defined(_WIN32) && defined(max)
#undef max
#endif
#if defined(_WIN32) && defined(min)
#undef min
#endif


namespace cb {
    template <const unsigned DIM, typename T> // DIM为维度，类实例化后维度是固定的，不同维度的类不是同一个类。T为节点对应的数据类型。数据类型一般为浮点型，可以是float或double，也可以是其他复杂类型。
    class Vector {
    public:
        T data[DIM]; // 内部是一个数组，保存每个维度的数据。

        Vector(T v = 0) {for (unsigned i = 0; i < DIM; i++) data[i] = v;} // 实例化，初始每个维度的数据均为0.

        Vector(const T data[DIM])
        {for (unsigned i = 0; i < DIM; i++) this->data[i] = data[i];} // 实例化，初始每个维度的数据为指定的维度数据。

        Vector(const Vector<DIM, T> &v)
        {for (unsigned i = 0; i < DIM; i++) data[i] = v.data[i];} // 实例化，初始每个维度的数据为另一个向量。也即复制一份v。

        Vector(const Vector<DIM - 1, T> &v, T x) { // 实例化，初始每个维度的数据为另一个向量v，只有最后一个维度的数据为单独指定的x，因此维度必须大于等于1.
            if (DIM < 1)
                CBANG_THROW("Invalid constructor for Vector of dimension " << DIM);
            for (unsigned i = 0; i < DIM - 1; i++) data[i] = v.data[i];
            data[DIM - 1] = x;
        }

        Vector(const Vector<DIM - 2, T> &v, T x, T y) { // 实例化，初始每个维度的数据为另一个向量v，只有最后两个维度的数据为单独指定的x和y，因此维度必须大于等于2.
            if (DIM < 2)
                CBANG_THROW("Invalid constructor for Vector of dimension " << DIM);
            for (unsigned i = 0; i < DIM - 2; i++) data[i] = v.data[i];
            data[DIM - 2] = x;
            data[DIM - 1] = y;
        }

        Vector(const Vector<DIM - 3, T> &v, T x, T y, T z) { // 实例化，初始每个维度的数据为另一个向量v，只有最后三个维度的数据为单独指定的x、y和z，因此维度必须大于等于3.
            if (DIM < 3)
                CBANG_THROW("Invalid constructor for Vector of dimension " << DIM);
            for (unsigned i = 0; i < DIM - 3; i++) data[i] = v.data[i];
            data[DIM - 3] = x;
            data[DIM - 2] = y;
            data[DIM - 1] = z;
        }

        explicit Vector(const std::string &s) { // 实例化，初始每个维度的数据用字符串表示。
            std::vector<std::string> tokens; // 保存分割后的结果
            String::tokenize(s, tokens, "(,; \t\n\r)"); // 字符串分隔符为“(.;空格、制表符、回车、换行)"。
            if (tokens.size() != DIM) // 分割后的结果如果跟维度不一致，则报错。
                CBANG_THROW("Invalid Vector<" << DIM << "> string '" << s << "'");

            for (unsigned i = 0; i < DIM; i++) // 遍历每个维度，将字符串识别为双精度浮点数，并赋值。
                data[i] = (T)String::parseDouble(tokens[i]);
        }

        Vector(T x, T y) { // 实例化，初始两个维度数据x和y，因此维度数不为2时报错。
            if (DIM != 2)
                CBANG_THROW("Invalid constructor for Vector of dimension " << DIM);
            data[0] = x;
            data[1] = y;
        }

        Vector(T x, T y, T z) { // 实例化，初始三个维度数据x、y和z，因此维度数不为3时报错。
            if (DIM != 3)
                CBANG_THROW("Invalid constructor for Vector of dimension " << DIM);
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        Vector(T x, T y, T z, T a) { // 实例化，初始四个维度数据x、y、z和a，因此维度数不为4时报错。
            if (DIM != 4)
                CBANG_THROW("Invalid constructor for Vector of dimension " << DIM);
            data[0] = x;
            data[1] = y;
            data[2] = z;
            data[3] = a;
        }


        static unsigned getSize() {return DIM;} // 获得当前向量的维度数。


        T &operator[](unsigned i) {return data[i];} // 运算符方括号重载，返回的是第i个维度的数据。
        const T &operator[](unsigned i) const {return data[i];} // 运算符方括号重载，返回的是第i个维度的数据，且为常量。

        T &x() {return data[0];} // 返回x轴数据，也即第0维度。
        T x() const {return data[0];} // 返回x轴数据，也即第0维度。
        T &y() { // 返回y轴数据，也即第1维度。若维度小于2，则报错。
            if (DIM < 2)
                CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
            return data[1];
        }
        T y() const { // 返回y轴数据，也即第1维度。若维度小于2，则报错。
            if (DIM < 2)
                CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
            return data[1];
        }
        T &z() { // 返回z轴数据，也即第2维度。若维度小于3，则报错。
            if (DIM < 3)
                CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
            return data[2];
        }
        T z() const { // 返回z轴数据，也即第2维度。若维度小于3，则报错。
            if (DIM < 3)
                CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
            return data[2];
        }


        template <unsigned LEN>
        Vector<LEN, T> slice(unsigned start = 0) const { // 取维度片段，从start维度开始。维度片段长度为LEN。
            Vector<LEN, T> result;

            for (unsigned i = 0; i < DIM - start && i < LEN; i++)
                result[i] = data[start + i];

            return result;
        }

        void clear() {for (unsigned i = 0; i < DIM; i++) data[i] = 0;} // 清空维度数据，也即每个维度的数据置为0.

        void reverse() { // 维度数据翻转。
            for (unsigned i = 0; i < DIM / 2; i++)
                std::swap(data[i], data[DIM - i - 1]);
        }


        T sum() const { // 维度数据求和。
            T result = 0;
            for (unsigned i = 0; i < DIM; i++) result += data[i];
            return result;
        }


        T lengthSquared() const { // 求每个维度数值的平方和。
            T result = 0;
            for (unsigned i = 0; i < DIM; i++) result += data[i] * data[i];
            return result;
        }

        T length() const {return sqrt(lengthSquared());} // 求维度的长度，也即维度数值平方和的开方。


        Vector<DIM, T> normalize() const { // 维度规格化，也即每个维度数值除以维度的长度。
            return *this / length();
        }


        T distanceSquared(const Vector<DIM, T> &v) const { // 求两个向量的距离平方和，也即对应每个维度差的平方和。
            T d = 0;
            for (unsigned i = 0; i < DIM; i++)
                d += (data[i] - v.data[i]) * (data[i] - v.data[i]);
            return d;
        }

        T distance(const Vector<DIM, T> &v) const { // 求两个向量的距离，即两个向量距离平方和的开方。
            return sqrt(distanceSquared(v));
        }


        Vector<DIM, T> crossProduct(const Vector<DIM, T> &v) const { // 求两个向量的向量积，该方法仅适用于维度为3的情况，不为3时会报错。
            if (DIM != 3)
                CBANG_THROW("Invalid operation for Vector of dimension " << DIM);
            return Vector<DIM, T>(data[1] * v.data[2] - data[2] * v.data[1], // (a,b,c) X (d,e,f) = (bf-ec,cd-af,ae-bd)
                                  data[2] * v.data[0] - data[0] * v.data[2],
                                  data[0] * v.data[1] - data[1] * v.data[0]);
        }

        Vector<DIM, T> cross(const Vector<DIM, T> &v) const { // 求两个向量的向量积
            return crossProduct(v);
        }

        T dotProduct(const Vector<DIM, T> &v) const { // 求两个向量的数量积，即每个维度对应值的乘积之和。
            T result = 0;
            for (unsigned i = 0; i < DIM; i++) result += data[i] * v.data[i];
            return result;
        }

        T dot(const Vector<DIM, T> &v) const {return dotProduct(v);} // 求两个向量的数量积.


        T angleBetween(const Vector<DIM, T> &v) const { // 求两个向量的夹角。
            if (DIM == 2) { // 如果维度为2，即平面向量，则夹角为两个维度反正切的差值。
                T angle = atan2(v.y(), v.x()) - atan2(y(), x());
                if (angle < 0) angle += 2 * M_PI;

                return fmod(2 * M_PI - angle, 2 * M_PI); // 保证值区间在 0~2π之间。
            }

            return acos(normalize().dotProduct(v.normalize())); // 计算两个向量规格化之后的反余弦。
        }


        Vector<DIM, T> intersect(const Vector<DIM, T> &v, T distance) const { // 求与另一个向量每个维度差乘以距离后的新向量。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++)
                result[i] = data[i] + distance * (v.data[i] - data[i]);
            return result;
        }


        unsigned findLargest() const { // 找到每个维度的最大值，返回其坐标。
            unsigned index = 0;
            T value = data[0];
            for (unsigned i = 1; i < DIM; i++)
                if (value < data[i]) {
                    value = data[i];
                    index = i;
                }

            return index;
        }

        unsigned findSmallest() const { // 找到每个维度的最小值，返回其坐标。
            unsigned index = 0;
            T value = data[0];
            for (unsigned i = 1; i < DIM; i++)
                if (data[i] < value) {
                    value = data[i];
                    index = i;
                }

            return index;
        }


        // Cast
        template <typename U>
        operator Vector<DIM, U>() const { // 运算符实例化重载，也即重新复制一份。该方法适用于另一个实例化的类的数据类型与当前类型不同，也即不同数据类型的转换（cast）。
            Vector<DIM, U> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i];
            return result;
        }

        // Uniary
        Vector<DIM, T> operator-() const { // 单目运算：运算符负号重载。返回每个维度值的相反数的向量。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = -data[i];
            return result;
        }

        bool operator!() const { // 单目运算：运算符叹号重载。若某个维度为0，则返回假，否则为真。
            for (unsigned i = 0; i < DIM; i++) if (data[i]) return false;
            return true;
        }


        // Arithmetic
        Vector<DIM, T> operator+(const Vector<DIM, T> &v) const { //算术运算符：运算符加号重载。取两个向量每个维度值之和。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] + v.data[i];
            return result;
        }

        Vector<DIM, T> operator-(const Vector<DIM, T> &v) const { //算术运算符：运算符减号重载。取两个向量每个维度值之差。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] - v.data[i];
            return result;
        }

        Vector<DIM, T> operator*(const Vector<DIM, T> &v) const { //算术运算符：运算符乘号重载。取两个向量每个维度值之积。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] * v.data[i];
            return result;
        }

        Vector<DIM, T> operator/(const Vector<DIM, T> &v) const { //算术运算符：运算符除号重载。取两个向量每个维度值之商。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] / v.data[i];
            return result;
        }


        Vector<DIM, T> operator+(T v) const { //算术运算符：运算符加号重载。当前向量每个维度加相同的值。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] + v;
            return result;
        }

        Vector<DIM, T> operator-(T v) const { //算术运算符：运算符加号重载。当前向量每个维度减相同的值。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] - v;
            return result;
        }

        Vector<DIM, T> operator*(T v) const { //算术运算符：运算符加号重载。当前向量每个维度乘以相同的值。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] * v;
            return result;
        }

        Vector<DIM, T> operator/(T v) const { //算术运算符：运算符加号重载。当前向量每个维度除以相同的值。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = data[i] / v;
            return result;
        }


        // Assignment
        Vector<DIM, T> &operator=(const Vector<DIM, T> &v) { // 赋值运算符：运算符等号重载。将当前向量每个维度赋给目标向量对应维度。
            for (unsigned i = 0; i < DIM; i++) data[i] = v.data[i];
            return *this;
        }

        Vector<DIM, T> &operator+=(const Vector<DIM, T> &v) { // 赋值运算符：运算符加等号重载。将当前向量每个维度与目标向量对应维度的值累加。
            for (unsigned i = 0; i < DIM; i++) data[i] += v.data[i];
            return *this;
        }

        Vector<DIM, T> &operator-=(const Vector<DIM, T> &v) { // 赋值运算符：运算符减等号重载。将当前向量每个维度与目标向量对应维度的值累减。
            for (unsigned i = 0; i < DIM; i++) data[i] -= v.data[i];
            return *this;
        }

        Vector<DIM, T> &operator*=(const Vector<DIM, T> &v) { // 赋值运算符：运算符乘等号重载。将当前向量每个维度与目标向量对应维度的值累乘。
            for (unsigned i = 0; i < DIM; i++) data[i] *= v.data[i];
            return *this;
        }

        Vector<DIM, T> &operator/=(const Vector<DIM, T> &v) { // 赋值运算符：运算符除等号重载。将当前向量每个维度与目标向量对应维度的值累除。
            for (unsigned i = 0; i < DIM; i++) data[i] /= v.data[i];
            return *this;
        }


        Vector<DIM, T> &operator+=(T v) { // 赋值运算符：运算符加等号重载。将当前向量每个维度累加相同的值。
            for (unsigned i = 0; i < DIM; i++) data[i] += v;
            return *this;
        }

        Vector<DIM, T> &operator-=(T v) { // 赋值运算符：运算符减等号重载。将当前向量每个维度累减相同的值。
            for (unsigned i = 0; i < DIM; i++) data[i] -= v;
            return *this;
        }

        Vector<DIM, T> &operator*=(T v) { // 赋值运算符：运算符乘等号重载。将当前向量每个维度累乘相同的值。
            for (unsigned i = 0; i < DIM; i++) data[i] *= v;
            return *this;
        }

        Vector<DIM, T> &operator/=(T v) { // 赋值运算符：运算符除等号重载。将当前向量每个维度累除相同的值。
            for (unsigned i = 0; i < DIM; i++) data[i] /= v;
            return *this;
        }


        // Comparison
        bool operator==(const Vector<DIM, T> &v) const { // 比较运算符：运算符判像等重载。只有当前向量每个维度的值都与目标向量对应维度的值相等时，才认为相等，否则认为不等。
            for (unsigned i = 0; i < DIM; i++) if (data[i] != v.data[i]) return false;
            return true;
        }

        bool operator!=(const Vector<DIM, T> &v) const { // 比较运算符：运算符判不等重载。只有当前向量每个维度的值都与目标向量对应维度的值相等时，才认为相等，否则认为不等。也即判相等取反。
            return !(*this == v);
        }

        bool operator<(const Vector<DIM, T> &v) const { // 比较运算符：运算符判小于重载。只有当前向量每个维度的值都小于目标向量对应维度的值时，才认为小于，否则认为大于等于。
            for (unsigned i = 0; i < DIM; i++)
                if (data[i] < v.data[i]) return true;
                else if (v.data[i] < data[i]) return false;

            return false;
        }

        bool operator<=(const Vector<DIM, T> &v) const { // 比较运算符：运算符判小于等于重载。只有当前向量每个维度的值都小于等于目标向量对应维度的值时，才认为小于等于，否则认为大于。
            for (unsigned i = 0; i < DIM; i++)
                if (data[i] < v.data[i]) return true;
                else if (v.data[i] < data[i]) return false;

            return true;
        }

        bool operator>(const Vector<DIM, T> &v) const { // 比较运算符：运算符判大于重载。也即目标是否小于等于自己。
            return v <= *this;
        }

        bool operator>=(const Vector<DIM, T> &v) const { // 比较运算符：运算符判大于等于重载。也即目标是否小于自己。
            return v < *this;
        }


        // Math
        template <T (*F)(T)>
        Vector<DIM, T> apply() const { // 应用指定函数。为每个维度分别执行一遍指定函数，并返回结果向量。
            Vector<DIM, T> result;
            for (unsigned i = 0; i < DIM; i++) result[i] = F(data[i]);
            return result;
        }

        Vector<DIM, T> abs() const {return apply<std::abs>();} // 求向量绝对值。
        Vector<DIM, T> ceil() const {return apply<std::ceil>();} // 求向量向上取整。
        Vector<DIM, T> floor() const {return apply<std::floor>();} // 求向量向下取整。

        T min() const { // 找出向量维度中的最小值。
            T value = std::numeric_limits<T>::max();
            for (unsigned i = 0; i < DIM; i++) if (data[i] < value) value = data[i];
            return value;
        }

        T max() const { // 找出向量维度中的最大值。
            T value = std::numeric_limits<T>::min();
            for (unsigned i = 0; i < DIM; i++) if (value < data[i]) value = data[i];
            return value;
        }

        bool isReal() const { // 判断向量是否为实数：只要有一个维度是无穷量，则为false。
            for (unsigned i = 0; i < DIM; i++)
                if (!cb::Math::isfinite(data[i])) return false;
            return true;
        }

        // JSON
        cb::SmartPointer<cb::JSON::Value> getJSON() const { // 获取当前维度对应的JSON表达式指针。格式为每个维度值按顺序列出。
            SmartPointer<JSON::Value> list = new JSON::List;
            for (unsigned i = 0; i < DIM; i++) list->append(data[i]);
            return list;
        }

        void loadJSON(const cb::JSON::Value &value) {read(value);} // 加载JSON值。

        void read(const cb::JSON::Value &value) { // 读取JSON值，若列表长度与维度数不等，则报异常。
            auto &list = value.getList();

            if (list.size() != DIM)
                CBANG_THROW("Vector<" << DIM << "> expected list of length " << DIM);
            for (unsigned i = 0; i < DIM; i++) data[i] = list[i]->getNumber();
        }

        void write(JSON::Sink &sink) const { // 写入JSON值。也即写入每个维度列表。
            sink.beginList(true);
            for (unsigned i = 0; i < DIM; i++) sink.append(data[i]);
            sink.endList();
        }


        std::ostream &print(std::ostream &stream) const { // 输出到输出流。打印格式为小括号包裹，每个维度值逗号分割。
            for (unsigned i = 0; i < DIM; i++)
                stream << (i ? ',' : '(') << data[i];
            return stream << ')';
        }


        std::string toString() const { // 输出为字符串。
            std::ostringstream s;
            print(s);
            return s.str();
        }
    };


    template<const unsigned DIM, typename T> static inline
    std::ostream &operator<<(std::ostream &stream, const Vector<DIM, T> &v) { // 重载 << 运算符，与输出运算符一致。
        return v.print(stream);
    }


    typedef Vector<2, int> Vector2I; // 二维整数向量
    typedef Vector<2, unsigned> Vector2U; // 二维非负整数向量，即只有第一象限
    typedef Vector<2, double> Vector2D; // 二维双精度浮点向量
    typedef Vector<2, float> Vector2F; // 二维单精度浮点向量

    typedef Vector<3, int> Vector3I; // 三维整数向量
    typedef Vector<3, unsigned> Vector3U; // 三维非负整数向量，即只有第一卦限
    typedef Vector<3, double> Vector3D; // 三维双精度浮点数向量
    typedef Vector<3, float> Vector3F; // 三维单精度浮点数向量

    typedef Vector<4, int> Vector4I; // 四维整数向量
    typedef Vector<4, unsigned> Vector4U; // 四维非负整数向量
    typedef Vector<4, double> Vector4D; // 四维双精度浮点数向量
    typedef Vector<4, float> Vector4F; // 四维单精度浮点数向量
}
