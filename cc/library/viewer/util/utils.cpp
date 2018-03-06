/*
 * viewer/lib/utils.cpp
 */

// C, C++
#include <cstdio>
#include <iostream>

// Qt
#include <QApplication>

// OSG
#include <osg/ArgumentParser>

// perls
#include "thirdparty/perls-math/so3.h"

// local
#include "utils.hpp"

std::string ViewerUtils::get_src_dir()
{
    std::string src_filepath(__FILE__);
    size_t found;
    found = src_filepath.find_last_of("/\\");
    return src_filepath.substr(0, found);
}

osgViewer::ViewerBase::ThreadingModel
ViewerUtils::setup_qt_threading(osg::ArgumentParser& args)
{
#if QT_VERSION >= 0x050000
    // Qt5 is currently crashing and reporting "Cannot make QOpenGLContext
    // current in a different thread" when the viewer is run multi-threaded,
    // this is regression from Qt4
    osgViewer::ViewerBase::ThreadingModel tm =
            osgViewer::ViewerBase::SingleThreaded;
#else
    osgViewer::ViewerBase::ThreadingModel tm =
            osgViewer::ViewerBase::CullDrawThreadPerContext;
#endif

    while (args.read("--SingleThreaded"))
        tm = osgViewer::ViewerBase::SingleThreaded;
    while (args.read("--CullDrawThreadPerContext"))
        tm = osgViewer::ViewerBase::CullDrawThreadPerContext;
    while (args.read("--DrawThreadPerContext"))
        tm = osgViewer::ViewerBase::DrawThreadPerContext;
    while (args.read("--CullThreadPerCameraDrawThreadPerContext"))
        tm = osgViewer::ViewerBase::CullThreadPerCameraDrawThreadPerContext;

#if QT_VERSION >= 0x040800
    // Required for multithreaded QGLWidget on Linux/X11, see
    // http://blog.qt.io/blog/2011/06/03/threaded-opengl-in-4-8/
    if (tm != osgViewer::ViewerBase::SingleThreaded) {
        QApplication::setAttribute(Qt::AA_X11InitThreads);
    }
#endif

    return tm;
}

void ViewerUtils::xyz_quat_to_matrixd(osg::Matrixd& mat,
                                      const double* t,
                                      const double *q)
{
    double R[9];
    so3_quat2rot(q, R);
    Rt_to_matrixd(mat, R, t);
}

void ViewerUtils::Rt_to_matrixd(osg::Matrixd& mat,
                                const double* R,
                                const double* t)
{
    double P[16];

    P[0] = R[0];
    P[4] = R[1];
    P[8] = R[2];
    P[12] = t[0];

    P[1] = R[3];
    P[5] = R[4];
    P[9] = R[5];
    P[13] = t[1];

    P[2] = R[6];
    P[6] = R[7];
    P[10] = R[8];
    P[14] = t[2];

    P[3] = 0;
    P[7] = 0;
    P[11] = 0;
    P[15] = 1;

    mat.set(P);
}

void ViewerUtils::rbt_to_matrixd(osg::Matrixd& mat,
                                const double* H)
{
    double P[16];
    row_to_column_major(P, H, 4, 4);
    mat.set(P);
}

void ViewerUtils::pose_to_matrixd(osg::Matrixd& mat,
                                  const double* x)
{
    double R[9];
    so3_rotxyz(R, x + 3); // convert rph to rotation matrix
    Rt_to_matrixd(mat, R, x);
}

void ViewerUtils::row_to_column_major(double* C,
                                      const double* R,
                                      unsigned int m,
                                      unsigned int n)
{
    for (unsigned int r = 0; r < m; r++) {
        for (unsigned int c = 0; c < n; c++) {
            // R at row r and column c
            // C at row c and column r
            C[m*c+r] = R[n*r+c];
        }
    }
}

void ViewerUtils::column_to_row_major(double* R,
                                      const double* C,
                                      unsigned int m,
                                      unsigned int n)
{
    for (unsigned int c = 0; c < n; c++) {
        for (unsigned int r = 0; r < m; r++) {
            // C at row c and column r
            // R at row r and column c
            R[n*r+c] = C[m*c+r];
        }
    }
}

void ViewerUtils::print_matrixd(osg::Matrixd& mat)
{
    printf("[ %10.5g, %10.5g, %10.5g, %10.5g ]\n"
           "[ %10.5g, %10.5g, %10.5g, %10.5g ]\n"
           "[ %10.5g, %10.5g, %10.5g, %10.5g ]\n"
           "[ %10.5g, %10.5g, %10.5g, %10.5g ]\n",
           mat(0,0), mat(1,0), mat(2,0), mat(3,0),
           mat(0,1), mat(1,1), mat(2,1), mat(3,1),
           mat(0,2), mat(1,2), mat(2,2), mat(3,2),
           mat(0,3), mat(1,3), mat(2,3), mat(3,3));
}

void ViewerUtils::double_to_uint8(uint8_t* out,
                                  const double* in,
                                  unsigned int num,
                                  double min,
                                  double max)
{
    const double k_multiplier = 255.0 / (max - min);
    for (unsigned int i = 0; i < num; i++) {
        if (in[i] < min) { out[i] = 0; }
        else if (in[i] > max) { out[i] = 255; }
        else { out[i] = (uint8_t)((in[i] - min)*k_multiplier); }
    }
}

