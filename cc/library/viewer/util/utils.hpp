#ifndef _VIEWER_LIB_UTILS_HPP_
#define _VIEWER_LIB_UTILS_HPP_

// C, C++
#include <string>

// OSG
#include <osg/Matrixd>
#include <osgViewer/ViewerBase>

class ViewerUtils
{

  public:

    // returns directory of source file
    // used to find osgt files (e.g for model of axes)
    static std::string get_src_dir();

    static osgViewer::ViewerBase::ThreadingModel
    setup_qt_threading(osg::ArgumentParser& args);

    // set mat to pose at translation t and quaternion q
    static void xyz_quat_to_matrixd(osg::Matrixd& mat,
                                    const double* t,
                                    const double* q);

    // set mat to pose with rotation matrix R, translation t
    static void Rt_to_matrixd(osg::Matrixd& mat,
                              const double* R,
                              const double* t);

    // set mat to pose with 4x4 rbt
    static void rbt_to_matrixd(osg::Matrixd& mat,
                               const double* H);

    // 6DOF pose [x,y,z,r,p,h]
    static void pose_to_matrixd(osg::Matrixd& mat,
                                const double* x);

    // where R[m x n] represents an m x n matrix
    // and C is allocated of the same size as R
    static void row_to_column_major(double* C,
                                    const double* R,
                                    unsigned int m,
                                    unsigned int n);

    // where R[m x n] represents an m x n matrix
    static void column_to_row_major(double* R,
                                    const double* C,
                                    unsigned int m,
                                    unsigned int n);
    // Matrixd print
    static void print_matrixd(osg::Matrixd& mat);

    // computes a 0-255 intensity vector from a height vector and min/max height
    static void double_to_uint8(uint8_t* out,
                                const double* in,
                                unsigned int num,
                                double min = 0,
                                double max = 10);

};

#endif // _VIEWER_LIB_UTILS_HPP_
