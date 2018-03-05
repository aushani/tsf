/*
 * viewer/viewer/main.cpp
 *
 * based loosely on OpenSceneGraph/examples/osgviewerQt
 */

// C, C++
#include <iostream>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <vector>

// OSG
#include <osg/ArgumentParser>
#include <osgDB/ReadFile>

// local
#include "library/sensors/velodyne.h"

#include "library/kitti/tracklets.hpp"
#include "library/kitti/pose.hpp"
#include "library/kitti/util.hpp"

#include "app/flow/viewer.hpp"
#include "library/flow/pose_solver.hpp"

#include "library/flow/state_filter.hpp"

std::vector<velodyne_returns_t*>
load_all_velodyne(std::string kitti_log_dir, std::string kitti_log_date, int log_num)
{
    std::vector<velodyne_returns_t*> vrs;

    char filename[1000];

    int i = 0;
    while (1) {

        // Load Velodyne
        sprintf(filename, "%s/%s/%s_drive_%04d_sync/velodyne_points/data/%010d.bin",
                kitti_log_dir.c_str(), kitti_log_date.c_str(), kitti_log_date.c_str(), log_num, i);
        FILE *fp = fopen(filename, "r");

        if (fp == NULL) {
            break;
        }

        velodyne_returns_t *vr = read_kitti_file(fp, i);
        vrs.push_back(vr);
        fclose(fp);

        i++;
    }

    return vrs;
}

std::vector<Pose>
get_all_scan_matched_poses(std::string kitti_log_dir, std::string kitti_log_date, int log_num, std::vector<velodyne_returns_t*> vrs)
{
    // First check to see if we've already generated this result
    char filename[1000];
    sprintf(filename, "%s/%s/%s_drive_%04d_sync/sm_poses.txt",
            kitti_log_dir.c_str(), kitti_log_date.c_str(), kitti_log_date.c_str(), log_num);
    FILE *fp = fopen(filename, "r");

    if (fp != NULL) {

        std::vector<Pose> poses(vrs.size());

        for (size_t i=0; i<vrs.size(); i++) {

            Pose p;
            int res = fscanf(fp, "%lf %lf %lf %lf %lf %lf", &p.x, &p.y, &p.z, &p.r, &p.p, &p.h);
            if (res!=6)
                printf("Warning: couldn't read pose %ld, only read %d elements!\n", i, res);

            poses[i] = p;
        }

        fclose(fp);

        return poses;
    }

    // Otherwise just compute the scan matching result and save it for later

    printf("No scan matched poses, running scan matching now...\n");

    std::vector<Pose> poses = Pose::load_all_poses(kitti_log_dir, kitti_log_date, log_num);

    PoseSolver ps(vrs, poses);
    poses = ps.solve();

    // Write it out
    fp = fopen(filename, "w");
    for (Pose p : poses) {
        fprintf(fp, "%f %f %f %f %f %f\n", p.x, p.y, p.z, p.r, p.p, p.h);
    }
    fclose(fp);

    return poses;
}

std::vector<osg::ref_ptr<osg::Image> >
load_kitti_images(std::string kitti_log_dir, std::string kitti_log_date, int log_num)
{
    std::vector<osg::ref_ptr<osg::Image> > imgs;

    char filename[1000];

    int i = 0;
    while (1) {

        sprintf(filename, "%s/%s/%s_drive_%04d_sync/image_02/data/%010d.png",
                kitti_log_dir.c_str(), kitti_log_date.c_str(), kitti_log_date.c_str(), log_num, i);

        osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(filename);

        if (image == NULL)
            break;

        imgs.push_back(image);

        i++;
    }

    return imgs;
}

int main(int argc, char** argv)
{
    osg::ArgumentParser args(&argc, argv);
    osg::ApplicationUsage* au = args.getApplicationUsage();

    // report any errors if they have occurred when parsing the program arguments.
    if (args.errors()) {
        args.writeErrorMessages(std::cout);
        au->write(std::cout, osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return EXIT_FAILURE;
    }

    au->setApplicationName(args.getApplicationName());
    au->setCommandLineUsage(args.getApplicationName() + " [options]");
    au->setDescription(args.getApplicationName() +
                       " displays kitti sensor data.");
    au->addCommandLineOption("--kitti-log-dir <dirname>", "KITTI data directory", "~/data/kittidata/extracted/");
    au->addCommandLineOption("--kitti-log-date <dirname>", "KITTI date", "2011_09_26");
    au->addCommandLineOption("--log-num <num>", "KITTI log number", "18");
    //au->addCommandLineOption("--calib-dir <dirname>", "KITTI calib directory", "");
    au->addCommandLineOption("--params-dir <dirname>", "Params directory", "~/data/icra2017/kitti_eval/bg-final/");
    au->addCommandLineOption("--no-gui", "Run headless", "");
    au->addCommandLineOption("--save-tracklets <dirname>", "Save tracklets to files", "");
    au->addCommandLineOption("--save-matches <dirname>", "Save matches to files", "");
    au->addCommandLineOption("--save-eval <dirname>", "Save evaluation to files", "");
    au->addCommandLineOption("--test-ekf-flow", "Test EKF for Flow", "");
    au->addCommandLineOption("--test-ekf-pose", "Test EKF for Pose", "");

    // handle help text
    // call AFTER constructing viewer so keyboard shortcuts have been assigned
    unsigned int helpType = 0;
    if ((helpType = args.readHelpType())) {
        au->write(std::cout, helpType);
        return EXIT_SUCCESS;
    }

    if (args.read(std::string("--test-ekf-flow"))) {
        printf("Testing EKF...\n");

        StateFilter sf;

        printf("Initialize\n");
        sf.init(0, 1, 0, 1);
        std::cout << sf.get_mu() << std::endl;
        std::cout << sf.get_sigma() << std::endl;

        for (int i=0; i<100; i++) {

            printf("\n--- Step %d ---\n", i);

            printf("\nRun Process Model\n");
            sf.run_process_model(1);
            std::cout << sf.get_mu() << std::endl;
            std::cout << sf.get_sigma() << std::endl;

            printf("\nRun Observation Model\n");
            StateFilter::FlowObs z;
            z(0, 0) = sin(i*0.1);
            z(1, 0) = cos(i*0.1);
            sf.flow_observation(z);

            std::cout << "  Observation: " << z.transpose() << std::endl;
            std::cout << sf.get_mu() << std::endl;
            std::cout << sf.get_sigma() << std::endl;
        }

        return EXIT_SUCCESS;
    }

    // Now load all relevant data
    std::string home_dir = getenv("HOME");
    std::string kitti_log_dir = home_dir + "/data/kittidata/extracted/";
    if (!args.read(std::string("--kitti-log-dir"), kitti_log_dir)) {
        printf("Using default KITTI log dir: %s\n", kitti_log_dir.c_str());
        //printf("Need KITTI data\n");
        //exit(EXIT_FAILURE);
    }

    std::string kitti_log_date = "2011_09_26";
    if (!args.read(std::string("--kitti-log-date"), kitti_log_date)) {
        printf("Using default KITTI data: %s\n", kitti_log_date.c_str());
        //printf("Need KITTI date\n");
        //exit(EXIT_FAILURE);
    }

    int log_num = 18;
    if (!args.read(std::string("--log-num"), log_num)) {
        printf("Using default KITTI log number: %d\n", log_num);
        //printf("Need KITTI log number\n");
        //exit(EXIT_FAILURE);
    }

    std::string params_dir = home_dir + "/data/icra2017/kitti_eval/bg-final/";
    if (!args.read(std::string("--params-dir"), params_dir)) {
        printf("Using default KITTI params dir: %s\n", params_dir.c_str());
        //printf("Need Parameter data\n");
        //exit(EXIT_FAILURE);
    }

    std::string calib_dir =  home_dir + "/data/kittidata/extracted/2011_09_26";
    if (!args.read("--kitti-log-dir", calib_dir)) {
        printf("Using default KITTI calib dir: %s\n", calib_dir.c_str());
    }

    printf("Loading KITTI Log Directory: %s...\n", kitti_log_dir.c_str());

    std::vector<Pose> raw_poses = Pose::load_all_poses(kitti_log_dir, kitti_log_date, log_num);

    if (args.read(std::string("--test-ekf-pose"))) {
        printf("Testing EKF...\n");

        StateFilter sf;

        printf("Initialize\n");

        sf.init(raw_poses[0]);
        std::cout << sf.get_mu() << std::endl;
        std::cout << sf.get_sigma() << std::endl;

        for (size_t i=1; i<raw_poses.size(); i++) {

            printf("\n--- Pose %ld ---\n", i);
            Pose p = raw_poses[i];

            printf("\nRun Process Model\n");
            sf.run_process_model(0.1);
            std::cout << sf.get_mu() << std::endl;
            std::cout << sf.get_sigma() << std::endl;

            printf("\nRun Observation Model\n");
            sf.pose_observation(p);

            std::cout << sf.get_mu() << std::endl;
            std::cout << sf.get_sigma() << std::endl;

            std::cout << "Press Enter to continue...";
            std::cin.ignore();
        }

        return EXIT_SUCCESS;
    }

    printf("Loading velodyne...\n");
    std::vector<velodyne_returns_t*> vrs = load_all_velodyne(kitti_log_dir, kitti_log_date, log_num);
    printf("Loading pose...\n");
    std::vector<Pose> poses = get_all_scan_matched_poses(kitti_log_dir, kitti_log_date, log_num, vrs);
    printf("Loading images...\n");
    std::vector<osg::ref_ptr<osg::Image> > imgs = load_kitti_images(kitti_log_dir, kitti_log_date, log_num);

    // Load tracklets
    Tracklets t;
    char filename[1000];
    sprintf(filename, "%s/%s/%s_drive_%04d_sync/tracklet_labels.xml",
            kitti_log_dir.c_str(), kitti_log_date.c_str(), kitti_log_date.c_str(), log_num);
    std::string tracklets_dir(filename);
    if (t.loadFromFile(tracklets_dir)) {
        printf("Loaded %d tracklets from %s\n", t.numberOfTracklets(), tracklets_dir.c_str());
    } else {
        printf("Tracklets file %s not found\n", tracklets_dir.c_str());
    }

    printf("Making flow solver...\n");
    FlowApp fs;
    fs.load_bg_filter(params_dir);
    fs.set_velodyne_returns(vrs);
    fs.set_poses(poses);
    fs.set_raw_poses(raw_poses);
    fs.set_tracklets(&t);
    fs.set_images(imgs);

    fs.load_calibration(calib_dir);

    fs.set_ready(true);

    std::string save_dir;
    if (args.read("--save-tracklets", save_dir)) {
        printf("Saving tracklets\n");

        bool append = false;

        // iterate through scans but don't need to compute flow
        for (; fs.scan_at() < fs.n_scans() - 2; fs.next_scan(false)) {
            printf("Saving tracklets for scan %d of %d\n", fs.scan_at(), fs.n_scans());
            fs.save_tracklets(save_dir, append);
            append = true;
        }
    }

    if (args.read("--save-matches", save_dir)) {
        printf("Saving matches\n");

        bool append = false;

        // iterate through scans but don't need to compute flow
        for (fs.update_scan(0, false) ; fs.scan_at() < fs.n_scans() - 2; fs.next_scan(false)) {
            printf("Saving matches and mismatches for scan %d of %d\n", fs.scan_at(), fs.n_scans());
            fs.save_matches(save_dir, append);
            append = true;
        }
    }

    if (args.read("--save-eval", save_dir)) {
        printf("Saving evaluation\n");

        bool append = false;

        // iterate through scans
        for (fs.update_scan(0) ; fs.scan_at() < fs.n_scans() - 2; fs.next_scan()) {
            printf("Saving evaluation for scan %d of %d\n", fs.scan_at(), fs.n_scans());
            fs.save_eval(save_dir, append);
            append = true;
        }
    }

    // run viewer (only leaves when window is closed)
    if (!args.read("--no-gui")) {

        // Now make viewer
        FlowViewer v(args);
        v.set_flow_app(&fs);

        v.start();
    }

    printf("\n\nDone\n\n");

    return EXIT_SUCCESS;
}
