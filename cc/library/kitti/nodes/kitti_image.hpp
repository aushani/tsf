#ifndef _KITTI_IMAGE_NODE_H_
#define _KITTI_IMAGE_NODE_H_

// OSG
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Projection>

#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>
#include <osgUtil/Optimizer>
#include <osgViewer/ViewerEventHandlers>

// Eigen
#include <Eigen/Core>

namespace osg
{

class KittiImage : public osg::Group, public osgGA::GUIEventHandler
{

  public:

    KittiImage(int width, int height, double x0=0, double y0=0, double scale=1.0);
    ~KittiImage();

    void set_image(osg::ref_ptr<osg::Image> img);

    void update_window_size(int w, int h);

    bool handle(
            const osgGA::GUIEventAdapter&,
            osgGA::GUIActionAdapter&,
            osg::Object*,
            osg::NodeVisitor*
            );

  protected:

    bool add_to_hud(osg::Node *n);

    void set_up_texture(osg::Texture2D *texture, osg::Geode *geode, double x0, double y0, double width, double height, int bin_num=11);

    osg::ref_ptr<const osg::Image> _image = NULL;

    int _width;
    int _height;
    double _scale;

  private:

    osg::ref_ptr<osg::MatrixTransform> _camera_view_matrix;
    osg::ref_ptr<osg::Projection> _camera_proj_matrix;

    osg::ref_ptr<osg::Geode> _camera;

    osg::ref_ptr<osg::Texture2D> _camera_texture;
};

}

#endif
