#include "occ_grid.hpp"

#include "library/viewer/nodes/colorful_box.hpp"
#include "library/viewer/nodes/composite_shape_group.hpp"

namespace osg
{

OccGrid::OccGrid() :
    osg::Group(),
    _cb(new osg::OccGridCallback())
{
    setUpdateCallback(_cb);

    pthread_mutex_init(&_mutex, nullptr);

    occ_grid = NULL;

    _color_occu = osg::Vec4(1, 0, 0, 0.5);
}

OccGrid::OccGrid(const OccGrid &og) : osg::Group(og)
{
    if (occ_grid)
        occ_grid = sparse_occ_grid_copy(occ_grid, 1, 0);

}

OccGrid::~OccGrid()
{
    if (occ_grid)
        sparse_occ_grid_destroy(occ_grid);
}

void
OccGrid::lock()
{
    pthread_mutex_lock(&_mutex);
}

void
OccGrid::unlock()
{
    pthread_mutex_unlock(&_mutex);
}

void
OccGrid::set_sparse_occ_grid(sparse_occ_grid_t *og)
{
    lock();
    if (occ_grid)
        sparse_occ_grid_destroy(occ_grid);
    occ_grid = sparse_occ_grid_copy(og, 1, 0);
    rendered = false;
    unlock();
}

void
OccGrid::set_color(osg::Vec4 color)
{
    lock();
    _color_occu = color;
    unlock();
}

osg::Vec4
OccGrid::get_color()
{
    return _color_occu;
}

void
OccGrid::set_selected_position(double x, double y)
{
    lock();
    sel_x = x;
    sel_y = y;
    sel_active = true;
    unlock();
}

void
OccGrid::clear_selected_position()
{
    lock();
    sel_active = false;
    unlock();
}

OccGridCallback::OccGridCallback() :
    osg::Callback()
{
}

bool
OccGridCallback::run(osg::Object *object, osg::Object *data)
{
    osg::OccGrid *og = (osg::OccGrid*) object;
    if (og == NULL)
        return true;

    og->lock();

    if (og->init && og->rendered == false && og->occ_grid != NULL) {

        osg::ref_ptr<osg::CompositeShapeGroup> csg = new osg::CompositeShapeGroup();
        csg->get_sdrawable()->setColor(og->get_color());

        osg::ref_ptr<osg::CompositeShapeGroup> csg_sel = new osg::CompositeShapeGroup();
        csg_sel->get_sdrawable()->setColor(osg::Vec4(1, 0, 0, 0.8));

        while (og->getNumChildren() > 0)
            og->removeChild(0, 1);

        // Iterate over occ grid and add occupied cells
        sparse_occ_grid_it_t git;
        sparse_occ_grid_it_init(og->occ_grid, &git, 0);
        sparse_occ_grid_it_use_filter(&git, 1);

        int64_t key;
        float val;
        double x, y, z;

        float scale = og->occ_grid->res * 0.75;

        while (sparse_occ_grid_it_next(&git, &key, &val)) {

            if (val<=0)
                continue;

            sparse_occ_grid_get_xyz(og->occ_grid, key, &x, &y, &z);

            csg->get_cshape()->addChild(new osg::Box(osg::Vec3(x, y, z), scale));
        }

        if (og->sel_active) {
            // Render selected position
            for (int i=-0; i<=0; i++) for (int j=-0; j<=0; j++) {
                double res = og->occ_grid->res;
                sparse_occ_grid_center_xyz(og->occ_grid, og->sel_x+i*res, og->sel_y+j*res, 0, &x, &y, &z);
                double z_max = og->occ_grid->res * og->occ_grid->n_dim[2]/2;
                double z_min = -z_max;
                for (double z=z_min; z<z_max; z+=og->occ_grid->res)
                    csg_sel->get_cshape()->addChild(new osg::Box(osg::Vec3(x, y, z), scale*1.25));
            }
        }

        og->rendered = true;

        og->addChild(csg);
        og->addChild(csg_sel);
    }


    og->unlock();

    return true;
}

OccGridCollection::OccGridCollection() :
    osg::Group()
{
}

OccGridCollection::~OccGridCollection()
{
}

void
OccGridCollection::set_sparse_occ_grids(std::vector<sparse_occ_grid_t*> ogs)
{
    // Remove al children
    OccGrid *og;
    while (getNumChildren() > 0) {

        og = dynamic_cast<OccGrid*>(getChild(0));

        if (og) og->lock();
        removeChild(0, 1);
        if (og) og->unlock();
    }

    // Add a bunch of new children for the new occ grids
    for (sparse_occ_grid_t *sog : ogs) {

        osg::ref_ptr<OccGrid> og = new OccGrid();

        // Can set color here
        // og->set_color();

        og->set_sparse_occ_grid(sog);

        addChild(og);

        og->init = true;
        og->rendered = false;
    }

    // Now we're done!
}

} // namespace osg
