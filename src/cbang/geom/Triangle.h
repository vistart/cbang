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
  typedef Vector<3, Vector2I> Triangle2I; // 平面整数坐标三角形。
  typedef Vector<3, Vector2U> Triangle2U; // 平面非负整数坐标三角形。即三角只出现在第一象限。
  typedef Vector<3, Vector2D> Triangle2D; // 平面双精度浮点数坐标三角形。
  typedef Vector<3, Vector2F> Triangle2F; // 平面单精度浮点数坐标三角形。

  typedef Vector<3, Vector3I> Triangle3I; // 立体整数坐标三角形。
  typedef Vector<3, Vector3U> Triangle3U; // 立体非负整数坐标三角形。即三角只出现在第一卦限。
  typedef Vector<3, Vector3D> Triangle3D; // 立体双精度浮点数坐标三角形。
  typedef Vector<3, Vector3F> Triangle3F; // 立体单精度浮点数坐标三角形。
}
