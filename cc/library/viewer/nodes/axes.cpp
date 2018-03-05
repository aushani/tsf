/*
 * viewer/lib/nodes/axes.cpp
 */

// OSG
#include <osgDB/ReadFile>

// local headers
#include "library/viewer/util/utils.hpp"

#include "axes.hpp"

namespace osg
{

Axes::Axes() :
        osg::MatrixTransform()
{
    // read axes file into osg::Node ptr
    osg::ref_ptr<osg::Node> axes =
            osgDB::readNodeFile(ViewerUtils::get_src_dir() + _k_axes_file);

    // TODO: throw an exception
    if (axes == nullptr) {
        std::cerr << "error reading axes file" << std::endl;
    }

    osg::Matrixd H;
    H.makeScale(_k_scale, _k_scale, _k_scale);
    setMatrix(H);

    addChild(axes);
}

} // namespace osg
