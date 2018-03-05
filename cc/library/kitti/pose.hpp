#ifndef _FLOW_POSE_H_
#define _FLOW_POSE_H_

#include <iostream>
#include <vector>
#include <string>

class Pose
{
  public:

    Pose();
    ~Pose();

    double x, y, z, r, p, h;

    double v_f, v_h;

    static std::vector<Pose>
    load_all_poses(std::string kitti_log_dir);

    static std::vector<Pose>
    load_all_poses(std::string kitti_log_dir, std::string kitti_log_date, int log_num);

  private:

    struct oxt_t
    {
        float lat;
        float lon;
        float alt;
        float roll;
        float pitch;
        float yaw;
        float vn;
        float ve;
        float vf;
        float vl;
        float vu;
        float ax;
        float ay;
        float az;
        float af;
        float al;
        float au;
        float wx;
        float wy;
        float wz;
        float wf;
        float wl;
        float wu;
        float posacc;
        float velacc;
        int navstat;
        int numsats;
        int posmode;
        int velmode;
        int orimode;
    };

    static oxt_t
    read_oxt_file(FILE *fp);

    static void
    lat_lon_to_mercator(double lat, double lon, double scale, double *x, double *y);

};

#endif
