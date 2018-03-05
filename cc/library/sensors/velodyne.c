// Adapted from perls/src/segway/slam/velodyne.c
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <assert.h>

//#include "perls-math/fasttrig.h"

#include "velodyne.h"

#define NUM_LASERS 32

#define UPPER_MAGIC 0xeeff
#define LOWER_MAGIC 0xddff

#define RADIANS_PER_LSB 0.00017453293
#define METERS_PER_LSB  0.002
#define SPIN_RATE       0.00006283185 //10 Hz = 3600/1e6*DTOR rads/usec

#define NUM_FIRING_PER_PACKET      (12)              // Number of firings per packet
#define NUM_LASERS_PER_FIRING      (32)              // Number of laser returns per firing
#define NUM_RETURNS_PER_PACKET     (384)             // Number of returns per packet
#define DATA_FIRING_START(i_f)     (100*i_f)         // Index of the start of firing given the firing ind (i_f=0...11)
#define DATA_LASER_START(i_f, i_l) (100*i_f+4+i_l*3) // Index of the start of laser give the

#define GET_MAGIC(data, start) (data[start] + (data[start+1]<<8))
#define GET_THETA(data, start) ((data[start+2] + (data[start+3]<<8)) * RADIANS_PER_LSB)
#define GET_RANGE(data, start) ((data[start] + (data[start+1]<<8)) * METERS_PER_LSB)
#define GET_INTENSITY(data, start) (data[start+2])

// Get the offset, in microseconds, from the first firing in the packet
// given the firing index (i_f=0...11) and laser index (i_l=0..31)
// packet theta is recorded at start of packet
// see velodyne 32 manual pg 24
#define HDL32_LASER_FIRING_OFFSET_USEC(if, il) ((i_f*46.08) + (i_l*1.152))
#define HDL32_MICROSECS_PER_LASER_FIRING 1.152

velodyne_returns_t *
velodyne_returns_create (int num_allocated) {
    velodyne_returns_t *r = malloc (sizeof (*r));

    r->num_returns = 0;
    r->num_allocated = num_allocated;
    r->xyz = malloc(num_allocated * 3 * sizeof (double));
    r->range = malloc(num_allocated * sizeof (double));
    r->theta = malloc(num_allocated * sizeof (double));
    r->phi = malloc(num_allocated * sizeof (double));
    r->intensity = malloc(num_allocated * sizeof (uint8_t));
    r->firing = malloc(num_allocated * sizeof (int));
    r->sorted = malloc(num_allocated * sizeof (int));

    return r;
}

void
velodyne_returns_destroy (velodyne_returns_t *r) {
    if (r->xyz != NULL) free (r->xyz);
    if (r->range != NULL) free (r->range);
    if (r->theta != NULL) free (r->theta);
    if (r->phi != NULL) free (r->phi);
    if (r->intensity != NULL) free (r->intensity);
    if (r->firing != NULL) free (r->firing);
    if (r->sorted != NULL) free (r->sorted);
    free (r);
}

velodyne_returns_t*
velodyne_decode_packet (velodyne_calib_t *c, uint8_t *data, int data_len,
                        velodyne_prefilt_f pf, void *user) {

    velodyne_returns_t *r = velodyne_returns_create (NUM_RETURNS_PER_PACKET);

    if (data_len == VELODYNE_POSE_PACKET_LEN) {
        r->num_returns = 0;
        return r;
    }

    int cnt = 0;

    assert (data_len == VELODYNE_DATA_PACKET_LEN);

    // in a packet there are 12 firing blocks each with 32 laser returns
    // loop over each firing in this packet (0..11)
    for (int i_f = 0; i_f<NUM_FIRING_PER_PACKET ; i_f++) {

        assert (UPPER_MAGIC == GET_MAGIC (data, DATA_FIRING_START (i_f)));

        double ctheta = GET_THETA (data, DATA_FIRING_START (i_f));

        if (ctheta >= 2*M_PI)
            ctheta = 0;

        // loop over each laser in this firing (0..31)
        for (int i_l = 0; i_l<NUM_LASERS_PER_FIRING; i_l++) {

            double range = GET_RANGE (data, DATA_LASER_START (i_f, i_l));
            // compensate for intershot theta change
            // at 100m this can be up to 1/4 of a meter of correction
            double theta = ctheta + SPIN_RATE*i_l*HDL32_MICROSECS_PER_LASER_FIRING;
            if (theta >= 2*M_PI)
                theta -= 2*M_PI;

            if (pf != NULL) {
                if (!(*pf)(i_l, c->firing2sorted[i_l], theta, range, user))
                    continue;
            } else {
                if (range < 0.1)
                    continue;
            }

            r->range[cnt] = range;
            r->theta[cnt] = theta;
            r->phi[cnt] = c->phi[i_l];
            r->firing[cnt] = i_l;
            r->sorted[cnt] = c->firing2sorted[i_l];
            r->intensity[cnt] = GET_INTENSITY (data, DATA_LASER_START (i_f, i_l));
            // check to do remap using intensity calibration data
            r->intensity[cnt] = (c->intensity_calib == NULL) ? r->intensity[cnt] :
                c->intensity_calib->intensity_mapping[r->sorted[cnt]*256 + r->intensity[cnt]];

            double sin_theta, cos_theta;
            //fsincos (theta, &sin_theta, &cos_theta);
            //XXX fix this
            sin_theta = sin(theta);
            cos_theta = cos(theta);
            double sin_phi = c->phi_sin[i_l];
            double cos_phi = c->phi_cos[i_l];
            r->xyz[cnt*3 + 0] = range * cos_theta * cos_phi;
            r->xyz[cnt*3 + 1] = -range * sin_theta * cos_phi;
            r->xyz[cnt*3 + 2] = range * sin_phi;

            cnt++;
        }
    }

    r->num_returns = cnt;
    return r;
}

velodyne_returns_t *
velodyne_returns_copy (const velodyne_returns_t *in)
{
    velodyne_returns_t *out = (velodyne_returns_t *) malloc (sizeof(*out));

    out->num_returns = in->num_returns;
    out->num_allocated = in->num_allocated;
    out->xyz = (double *) malloc (3 * out->num_allocated * sizeof(*out->xyz));
    out->range = (double *) malloc (out->num_allocated * sizeof(*out->range));
    out->theta = (double *) malloc (out->num_allocated * sizeof(*out->theta));
    out->phi = (double *) malloc (out->num_allocated * sizeof(*out->phi));
    out->intensity = (uint8_t *) malloc (out->num_allocated * sizeof(*out->intensity));
    out->firing = (int *) malloc (out->num_allocated * sizeof(*out->firing));
    out->sorted = (int *) malloc (out->num_allocated * sizeof(*out->sorted));

    memcpy (out->xyz, in->xyz, 3 * out->num_returns * sizeof(*out->xyz));
    memcpy (out->range, in->range, out->num_returns * sizeof(*out->range));
    memcpy (out->theta, in->theta, out->num_returns * sizeof(*out->theta));
    memcpy (out->phi, in->phi, out->num_returns * sizeof(*out->phi));
    memcpy (out->intensity, in->intensity, out->num_returns * sizeof(*out->intensity));
    memcpy (out->firing, in->firing, out->num_returns * sizeof(*out->firing));
    memcpy (out->sorted, in->sorted, out->num_returns * sizeof(*out->sorted));

    return out;
}

void
velodyne_returns_append (velodyne_returns_t *vel, const velodyne_returns_t *to_append)
{
    int new_size = vel->num_returns + to_append->num_returns;

    if (new_size > vel->num_allocated) {
        // reallocate for appended stuff
        vel->xyz = realloc (vel->xyz, 3 * new_size * sizeof(*vel->xyz));
        vel->range = realloc (vel->range, new_size * sizeof(*vel->range));
        vel->theta = realloc (vel->theta, new_size * sizeof(*vel->theta));
        vel->phi = realloc (vel->phi, new_size * sizeof(*vel->phi));
        vel->intensity = realloc (vel->intensity, new_size * sizeof(*vel->intensity));
        vel->firing = realloc (vel->firing, new_size * sizeof(*vel->firing));
        vel->sorted = realloc (vel->sorted, new_size * sizeof(*vel->sorted));
        vel->num_allocated = new_size;
    }

    // copy over appended stuff
    memcpy (vel->xyz + 3 * vel->num_returns, to_append->xyz, 3 * to_append->num_returns * sizeof(*vel->xyz));
    memcpy (vel->range + vel->num_returns, to_append->range, to_append->num_returns * sizeof(*vel->range));
    memcpy (vel->theta + vel->num_returns, to_append->theta, to_append->num_returns * sizeof(*vel->theta));
    memcpy (vel->phi + vel->num_returns, to_append->phi, to_append->num_returns * sizeof(*vel->phi));
    memcpy (vel->intensity + vel->num_returns, to_append->intensity, to_append->num_returns * sizeof(*vel->intensity));
    memcpy (vel->firing + vel->num_returns, to_append->firing, to_append->num_returns * sizeof(*vel->firing));
    memcpy (vel->sorted + vel->num_returns, to_append->sorted, to_append->num_returns * sizeof(*vel->sorted));
    vel->num_returns = new_size;
}

// calibration -----------------------------------------------------

// needed for precompute sort
static velodyne_calib_t *__c = NULL;

static int
phi_compare (const void *_a, const void *_b) {
    int a = *((int*) _a);
    int b = *((int*) _b);

    if (__c->phi[a] < __c->phi[b]) {
        return -1;
    }
    return 1;
}

// NOT REENTRANT
void
precompute (velodyne_calib_t* c) {
    assert (!__c); // check for reentrancy...

    __c = c;

    for (int i = 0; i < c->num_lasers; i++)
        c->sorted2firing[i] = i;

    qsort (c->sorted2firing, c->num_lasers, sizeof (int), phi_compare);

    for (int sorted = 0; sorted < c->num_lasers; sorted++)
        c->firing2sorted[c->sorted2firing[sorted]] = sorted;

    for (int firing = 0; firing < c->num_lasers; firing++)
        sincos (c->phi[firing], &c->phi_sin[firing], &c->phi_cos[firing]);

    __c = NULL;
}

velodyne_calib_t*
hdl32_stock_calib_create (void) {

    velodyne_calib_t *c = malloc (sizeof (*c));
    c->num_lasers = NUM_LASERS;
    c->firing2sorted = malloc (NUM_LASERS * sizeof (int));
    c->sorted2firing = malloc (NUM_LASERS * sizeof (int));
    c->phi = malloc (NUM_LASERS * sizeof (double));
    c->phi_sin = malloc (NUM_LASERS * sizeof (double));
    c->phi_cos = malloc (NUM_LASERS * sizeof (double));

    double phi_tmp[NUM_LASERS] = { -0.535292,  // laser 0
                                  -0.162839,  // laser 1
                                  -0.511905,  // laser 2
                                  -0.139626,  // laser 3
                                  -0.488692,  // laser 4
                                  -0.116239,  // laser 5
                                  -0.465305,  // laser 6
                                  -0.093026,  // laser 7
                                  -0.442092,  // laser 8
                                  -0.069813,  // laser 9
                                  -0.418879,  // laser 10
                                  -0.046600,  // laser 11
                                  -0.395666,  // laser 12
                                  -0.023213,  // laser 13
                                  -0.372279,  // laser 14
                                   0.000000,  // laser 15
                                  -0.349066,  // laser 16
                                   0.023213,  // laser 17
                                  -0.325853,  // laser 18
                                   0.046600,  // laser 19
                                  -0.302466,  // laser 20
                                   0.069813,  // laser 21
                                  -0.279253,  // laser 22
                                   0.093026,  // laser 23
                                  -0.256040,  // laser 24
                                   0.116413,  // laser 25
                                  -0.232652,  // laser 26
                                   0.139626,  // laser 27
                                  -0.209440,  // laser 28
                                   0.162839,  // laser 29
                                  -0.186227,  // laser 30
                                   0.186227}; // laser 31
    memcpy (c->phi, phi_tmp, NUM_LASERS * sizeof (double));

    precompute (c);

    return c;
}

velodyne_calib_t*
velodyne_calib_create (void) {
    velodyne_calib_t *ret = hdl32_stock_calib_create();
    ret->intensity_calib = NULL;
    return ret;
}

velodyne_calib_t*
velodyne_calib_load (const char *intensity_calib_filename) {
    velodyne_calib_t *ret = hdl32_stock_calib_create();
    ret->intensity_calib = velodyne_intensity_calib_load (intensity_calib_filename);
    return ret;
}

void
velodyne_calib_destroy (velodyne_calib_t *c) {
    free (c->firing2sorted);
    free (c->sorted2firing);
    free (c->phi);
    free (c->phi_sin);
    free (c->phi_cos);
    free (c);
    if (c->intensity_calib) velodyne_intensity_calib_destroy (c->intensity_calib);
}


velodyne_intensity_calib_t *
velodyne_intensity_calib_create (void)
{
    velodyne_intensity_calib_t *icalib = (velodyne_intensity_calib_t *) malloc (sizeof (*icalib));

    icalib->num_lasers = NUM_LASERS;
    icalib->intensity_mapping = (uint8_t *) malloc (256 * NUM_LASERS * sizeof(uint8_t));

    return icalib;
}

velodyne_intensity_calib_t *
velodyne_intensity_calib_load (const char *filename)
{
    velodyne_intensity_calib_t *icalib = (velodyne_intensity_calib_t *) malloc (sizeof (*icalib));

    // LOAD AS PLAIN TEXT
    FILE *f = fopen (filename, "r");
    if (fscanf (f, "%d\n", &icalib->num_lasers) != 1) {
        fprintf (stderr, "Cannot load number of lasers from calibration file %s\n", filename);
        exit (EXIT_FAILURE);
    }
    int tmp;
    if (fscanf (f, "%d\n", &tmp) != 1) {
        fprintf (stderr, "Cannot load from calibration file %s\n", filename);
        exit (EXIT_FAILURE);
    }

    icalib->intensity_mapping = (uint8_t *) malloc (256 * icalib->num_lasers * sizeof(uint8_t));

    for (int l = 0; l < icalib->num_lasers; ++l) {
        for (int i = 0; i < 256; ++i) {
            if (fscanf (f, "%d ", &tmp) != 1) {
                fprintf (stderr, "Cannot load [%d, %d] from calibration file %s\n", l, i, filename);
                exit (EXIT_FAILURE);
            }

            icalib->intensity_mapping[l*256 + i] = (uint8_t) tmp;
        }
    }

    fclose (f);

    return icalib;
}

velodyne_intensity_calib_t *
velodyne_intensity_calib_copy (const velodyne_intensity_calib_t *in)
{
    velodyne_intensity_calib_t *out = velodyne_intensity_calib_create ();
    memcpy (out->intensity_mapping, in->intensity_mapping, 256 * NUM_LASERS * sizeof(uint8_t));
    return out;
}

void
velodyne_intensity_calib_save (velodyne_intensity_calib_t * icalib, const char *filename)
{
    // SAVE AS PLAIN TEXT
    FILE *f = fopen (filename, "w");
    fprintf (f, "%d\n", icalib->num_lasers);
    fprintf (f, "%d\n", 256);

    for (int l = 0; l < icalib->num_lasers; ++l) {
        for (int i = 0; i < 256; ++i) {
            fprintf (f, "%d ", icalib->intensity_mapping[l*256 + i]);
        }

        fprintf (f, "\n");
    }

    fclose (f);
}

void
velodyne_intensity_calib_destroy (velodyne_intensity_calib_t * icalib)
{
    free (icalib->intensity_mapping);
    free (icalib);
}
