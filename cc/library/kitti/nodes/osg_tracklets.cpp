#include <osg/PolygonMode>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>

#include <iostream>

#include "osg_tracklets.hpp"

#include "library/viewer/nodes/colorful_box.hpp"
#include "library/viewer/nodes/composite_shape_group.hpp"

// perls
#include "thirdparty/perls-math/so3.h"

namespace osg
{

OsgTracklets::OsgTracklets() :
    osg::Group()
{
    _tracklets = NULL;
}

OsgTracklets::~OsgTracklets()
{

}

void
OsgTracklets::set_tracklets(Tracklets *t)
{
    _tracklets = t;
}

void
OsgTracklets::set_frame(int frame)
{
    if (_tracklets==NULL)
        return;

    printf("Set frame to %d\n", frame);

    // Render active tracks
    int n = _tracklets->numberOfTracklets();

    Tracklets::tPose *pose;
    Tracklets::tTracklet *t;

    osg::ref_ptr<osg::CompositeShapeGroup> csg = new osg::CompositeShapeGroup();

    osg::ref_ptr<osg::StateSet> ss = csg->getOrCreateStateSet();
    osg::ref_ptr<osg::PolygonMode> pm = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(8.0);
    ss->setAttributeAndModes(pm, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    ss->setAttributeAndModes(lw, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    for (int i=0; i<n; i++) {
        if (_tracklets->getPose(i, frame, pose)) {

            t = _tracklets->getTracklet(i);

            osg::Vec3 pos(pose->tx, pose->ty, pose->tz+0.8); //offset?
            osg::Quat quat(pose->rz, osg::Vec3(0, 0, 1)); // rotation only in heading

            osg::ref_ptr<osg::Box> box = new osg::Box(pos, t->l, t->w, t->h);
            box->setRotation(quat);

            osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(box);

            std::string type = t->objectType;
            //std::cout << "type: " << type << std::endl;

            if (type == "Car") {
                shape->setColor(osg::Vec4(0, 1, 0, 0.5));
            } else if (type == "Pedestrian" || type == "Person") {
                shape->setColor(osg::Vec4(1, 1, 0, 0.5));
            } else if (type == "Cyclist") {
                shape->setColor(osg::Vec4(1, 0, 0, 0.5));
            } else {
                shape->setColor(osg::Vec4(1, 1, 1, 0.5));
            }

            csg->addChild(shape);

            //printf("Track %d is active at frame %d\n", i, n);

        }
    }

    // rmeove old children
    while (getNumChildren() > 0)
        removeChild(0, 1);

    addChild(csg);

}

} // namespace osg
