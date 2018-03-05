// Adapted from perls/src/segway/slam/velodyne.h
#ifndef __SENSORS__VELODYNE_H__
#define __SENSORS__VELODYNE_H__

#include <stdint.h>

#define VELODYNE_DATA_PORT          2368
#define VELODYNE_DATA_PACKET_LEN    1206
#define VELODYNE_POSE_PORT          8308
#define VELODYNE_POSE_PACKET_LEN    512

#ifdef __cplusplus
extern "C" {
#endif


typedef struct _velodyne_returns_t velodyne_returns_t;
typedef struct _velodyne_calib_t velodyne_calib_t;
typedef struct _velodyne_intensity_calib_t velodyne_intensity_calib_t;


struct _velodyne_returns_t {
    int num_returns;       // the number of returns
    int num_allocated;     // number allocated below
    double *xyz;           // xyz location in sensor frame, size num_returns*3, row-major
    double *range;         // range normalized [0, 1]
    double *theta;         // theta (horizontal/yaw angle) **always [0, 2*PI)**
    double *phi;           // phi (veritcle/pitch angle)
    uint8_t *intensity;    // intensity [0, 255]
    int *firing;           // firing laser number (0-31 lower, 32-63 upper)
    int *sorted;           // sorted laser number (in order of increasing pitch)
    int64_t utime;
};

// point pre filtering callback funcitons
// returns true if point should be decoded
// can be used for self hit mask, random subsampling, ect
typedef uint8_t (*velodyne_prefilt_f)(int firing, int sorted,
                                      double theta, double range,
                                      void *user);

// decode a data packet
// returns an allocated velodyne_return_t that the user must free
// void *user is passed to the pre filter function
velodyne_returns_t *
velodyne_decode_packet (velodyne_calib_t *calib, uint8_t *data, int data_len,
                        velodyne_prefilt_f pf, void *user);

velodyne_returns_t *
velodyne_returns_create (int num_allocated);

velodyne_returns_t *
velodyne_returns_copy (const velodyne_returns_t *in);

void
velodyne_returns_append (velodyne_returns_t *vel, const velodyne_returns_t *to_append);

void
velodyne_returns_destroy (velodyne_returns_t *r);



// calibration ------------------------------

struct _velodyne_calib_t {
    int num_lasers;     // the number of lasers
    int *firing2sorted; // mapping firing laser index to sorted laser index
    int *sorted2firing; // mapping sorted laser index to firing laser index
    double *phi;        // vertical angle of laser (firing laser index)
    double *phi_sin;    // cached sin of vertical angle of laser (firing laser index)
    double *phi_cos;    // cached cos of vertical angle of laser (firing laser index)

    velodyne_intensity_calib_t *intensity_calib;
};

struct _velodyne_intensity_calib_t {
    int num_lasers;                 // number of lasers
    uint8_t *intensity_mapping;     // 2d array (laser x intensity, row major)
};

velodyne_calib_t*
velodyne_calib_create (void);

velodyne_calib_t*
velodyne_calib_load (const char *intensity_calib_filename);

// currently assumes factory default for HDL32E
void
velodyne_calib_destroy (velodyne_calib_t *c);


velodyne_intensity_calib_t *
velodyne_intensity_calib_create (void);

velodyne_intensity_calib_t *
velodyne_intensity_calib_load (const char *filename);

velodyne_intensity_calib_t *
velodyne_intensity_calib_copy (const velodyne_intensity_calib_t *in);

void
velodyne_intensity_calib_save (velodyne_intensity_calib_t * icalib, const char *filename);

void
velodyne_intensity_calib_destroy (velodyne_intensity_calib_t * icalib);

#ifdef __cplusplus
}
#endif

#endif // __SENSORS__VELODYNE_H__
