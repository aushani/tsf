#include <iostream>

#include "kitti_image.hpp"

#include "library/viewer/nodes/composite_shape_group.hpp"

namespace osg
{

KittiImage::KittiImage(int width, int height, double x0, double y0, double scale):
    osg::Group(),
    _camera_view_matrix(new osg::MatrixTransform()),
    _camera_proj_matrix(new osg::Projection()),
    _camera(new osg::Geode()),
    _camera_texture(new osg::Texture2D())
{
    _width = width;
    _height = height;
    _scale = scale;

    osg::Matrix m = osg::Matrix::identity();
    m.makeScale(scale, scale, scale);
    m.postMultTranslate(osg::Vec3d(x0, y0, 0));
    _camera_view_matrix->setMatrix(m);

    _camera_view_matrix->setReferenceFrame(osg::Transform::ABSOLUTE_RF);

    addChild(_camera_proj_matrix);
    _camera_proj_matrix->addChild(_camera_view_matrix);

    //_camera_view_matrix->addChild(_camera);
    add_to_hud(_camera);

    set_up_texture(_camera_texture, _camera, 0, 0, _width, _height, 11);
}

KittiImage::~KittiImage()
{
}

void
KittiImage::set_up_texture(osg::Texture2D *texture, osg::Geode *geode, double x0, double y0, double width, double height, int bin_num)
{
    osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
    osg::Vec3Array* vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(x0, y0, -1));
    vertices->push_back(osg::Vec3(x0+width, y0, -1));
    vertices->push_back(osg::Vec3(x0+width, y0+height, -1));
    vertices->push_back(osg::Vec3(x0, y0+height, -1));

    osg::DrawElementsUInt* background_indices = new osg::DrawElementsUInt(osg::PrimitiveSet::POLYGON, 0);
    background_indices->push_back(0);
    background_indices->push_back(1);
    background_indices->push_back(2);
    background_indices->push_back(3);

    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 0.9f));

    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f,0.0f);
    (*texcoords)[1].set(1.0f,0.0f);
    (*texcoords)[2].set(1.0f,1.0f);
    (*texcoords)[3].set(0.0f,1.0f);

    geometry->setTexCoordArray(0,texcoords);
    texture->setDataVariance(osg::Object::DYNAMIC);
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));
    geometry->setNormalArray(normals);
    geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
    geometry->addPrimitiveSet(background_indices);
    geometry->setVertexArray(vertices);
    geometry->setColorArray(colors);
    geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

    geode->addDrawable(geometry);

    // Create and set up a state set using the texture from above:
    osg::ref_ptr<osg::StateSet> state_set = new osg::StateSet();
    geode->setStateSet(state_set);
    state_set->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

    // For this state set, turn blending on (so alpha texture looks right)
    state_set->setMode(GL_BLEND,osg::StateAttribute::ON);

    // Disable depth testing so geometry is draw regardless of depth values
    // of geometry already draw.
    state_set->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
    state_set->setRenderingHint( osg::StateSet::TRANSPARENT_BIN );

    // Need to make sure this geometry is draw last. RenderBins are handled
    // in numerical order so set bin number to 11 by default
    state_set->setRenderBinDetails( bin_num, "RenderBin");
}

void
KittiImage::set_image(osg::ref_ptr<osg::Image> img)
{
    _image = img;
    _camera_texture->setImage(img);
}

void
KittiImage::update_window_size(int w, int h)
{
    _camera_proj_matrix->setMatrix(osg::Matrix::ortho2D(0, w, 0, h));
}

bool
KittiImage::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&,
        osg::Object*, osg::NodeVisitor*)
{
    if (ea.getEventType() == osgGA::GUIEventAdapter::RESIZE) {
        //std::cerr << "Resize event, time = " << ea.getTime() << "\n";
        //printf("Window size; %d x %d\n", ea.getWindowWidth(), ea.getWindowHeight());

        update_window_size(ea.getWindowWidth(), ea.getWindowHeight());
    }

    return false;
}

bool
KittiImage::add_to_hud(osg::Node *n)
{
    return _camera_view_matrix->addChild(n);
}

}
