#ifndef _FLOW_OCC_GRID_H
#define _FLOW_OCC_GRID_H

// OSG
#include <osg/MatrixTransform>

#include <vector>

#include "library/ray_tracing/sparse_occ_grid.h"

namespace osg
{

class OccGridCallback : public osg::Callback
{

  public:

    OccGridCallback();

    bool run(osg::Object *object, osg::Object *data) override;

  protected:

    virtual ~OccGridCallback() = default;
};


class OccGrid : public osg::Group
{

  public:

    OccGrid();
    OccGrid(const OccGrid &og);
    ~OccGrid();

    void set_sparse_occ_grid(sparse_occ_grid_t *og);

    void set_color(osg::Vec4 color);
    osg::Vec4 get_color();

    void set_selected_position(double x, double y);
    void clear_selected_position();

    void lock();
    void unlock();

    sparse_occ_grid_t *occ_grid;
    bool rendered = false;

    bool init = false;

    double sel_x, sel_y;
    bool sel_active = false;

  private:

    osg::ref_ptr<osg::OccGridCallback> _cb;

    pthread_mutex_t _mutex;

    osg::Vec4 _color_occu;
};

class OccGridCollection : public osg::Group
{

  public:

      OccGridCollection();
      ~OccGridCollection();

      void set_sparse_occ_grids(std::vector<sparse_occ_grid_t*> ogs);
};

} // namespace osg

#endif
