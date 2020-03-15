// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <open_chisel/geometry/Plane.h>

namespace chisel
{

    Plane::Plane()
    {

    }

    Plane::Plane(const Vec4& params) :
        normal(Vec3(params(0), params(1), params(2))), distance(params(3))
    {

    }

    Plane::Plane(const Vec3& _normal, float _distance) :
            normal(_normal), distance()
    {

    }

    Plane::Plane(const Vec3& a, const Vec3& b, const Vec3& c)
    {
        Vec3 ab = b - a;
        Vec3 ac = c - a;

        Vec3 cross = ab.cross(ac);
        normal = cross.normalized();// 法线确定一组平面，注意方向！（法线指向椎体内部？）
        distance = -(cross.dot(a));// 原点到平面法线的投影，是个负数！？  （用cross计算应该不是距离，错了？）
        // 原点在视锥外所以dot是负数，变为正数
    }

    Plane::Plane(float a, float b, float c, float d) :
                normal(a, b, c), distance(d)
    {

    }

    float Plane::GetSignedDistance(const Vec3& point) const
    {
        return point.dot(normal) + distance;
    }

    Plane::~Plane()
    {

    }

} // namespace chisel 
