/*
 * viewer/lib/colormap.cpp
 */

// local
#include "colormap.hpp"

namespace osg
{

namespace ColorMap
{

osg::Vec4 Mapper::at(uint8_t value, Type type)
{
    return type == Type::AUTUMN ? Autumn::at(value) :
           type == Type::COOL   ? Cool::at(value)   :
           type == Type::NONE   ? None::at(value)    :
           type == Type::JET    ? Jet::at(value)    :
           type == Type::RED    ? Red::at(value)    :
           type == Type::SPRING ? Spring::at(value) :
           type == Type::SUMMER ? Summer::at(value) :
           type == Type::WINTER ? Winter::at(value) :
            osg::Vec4(0, 0, 0, 1);
}

constexpr float Autumn::r[];
constexpr float Autumn::g[];
constexpr float Autumn::b[];

osg::Vec4 Autumn::at(uint8_t value)
{
    // 64 elements
    return osg::Vec4(r[value >> 2], g[value >> 2], b[value >> 2], 1);
}

constexpr float Cool::r[];
constexpr float Cool::g[];
constexpr float Cool::b[];

osg::Vec4 Cool::at(uint8_t value)
{
    // 64 elements
    return osg::Vec4(r[value >> 2], g[value >> 2], b[value >> 2], 1);
}

constexpr float Jet::r[];
constexpr float Jet::g[];
constexpr float Jet::b[];

osg::Vec4 None::at(uint8_t value)
{
    return osg::Vec4(1, 0, 0, 1);
}

osg::Vec4 Jet::at(uint8_t value)
{
    // 256 elements
    return osg::Vec4(r[value], g[value], b[value], 1);
}

osg::Vec4 Red::at(uint8_t value)
{
    // red is min, white is max
    double n_value = ((double) value) / 255;
    return osg::Vec4(0.75, n_value, n_value, 1);
}

constexpr float Spring::r[];
constexpr float Spring::g[];
constexpr float Spring::b[];

osg::Vec4 Spring::at(uint8_t value)
{
    // 64 elements
    return osg::Vec4(r[value >> 2], g[value >> 2], b[value >> 2], 1);
}

constexpr float Summer::r[];
constexpr float Summer::g[];
constexpr float Summer::b[];

osg::Vec4 Summer::at(uint8_t value)
{
    // 64 elements
    return osg::Vec4(r[value >> 2], g[value >> 2], b[value >> 2], 1);
}

constexpr float Winter::r[];
constexpr float Winter::g[];
constexpr float Winter::b[];

osg::Vec4 Winter::at(uint8_t value)
{
    // 16 elements
    return osg::Vec4(r[value >> 4], g[value >> 4], b[value >> 4], 1);
}

} // namespace ColorMap

} // namespace osg
