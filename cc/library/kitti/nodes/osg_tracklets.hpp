#ifndef _OSG_TRACKLETS_H_
#define _OSG_TRACKLETS_H_

// OSG
#include <osg/MatrixTransform>

#include "library/kitti/tracklets.hpp"

namespace osg
{

class OsgTracklets : public osg::Group
{

  public:

    OsgTracklets();
    ~OsgTracklets();

    void set_tracklets(Tracklets *t);
    void set_frame(int frame);

  private:

    Tracklets *_tracklets;

};

} // namespace osg

#endif
