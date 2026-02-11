#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"

#include "System.h"


static int LoadImages(
    const std::string &strAssociationFilename,
    std::vector<std::string> &vImageFilenamesRGB,
    std::vector<std::string> &vImageFilenamesD,
    std::vector<double> &vTimestamps
)
{
    std::ifstream fAssociation(strAssociationFilename);
    if (!fAssociation.is_open()) {
        std::cerr << "Open association file " << strAssociationFilename << " failed" << std::endl;
        return -1;
    }

    while(!fAssociation.eof()) {
        std::string rgbFile, depthFile;
        double t1 = 0., t2 = 0.;

        fAssociation >> t1 >> rgbFile >> t2 >> depthFile;
        if (rgbFile.empty() || depthFile.empty() || t1 == 0. || t2 == 0.) {
            continue;
        }

        vImageFilenamesRGB.emplace_back(rgbFile);
        vImageFilenamesD.emplace_back(depthFile);
        vTimestamps.emplace_back(t1);
    }

    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("rgbd_tum");

    /* Declear parameters */
    node->declare_parameter("voc_file", "ORBvoc.bin");
    node->declare_parameter("setting_file", "TUM.yaml");
    node->declare_parameter("sequence_dir", "fr1_xyz");
    node->declare_parameter("association_file", "fr1_xyz.txt");
    node->declare_parameter("oneshot", true);

    // Retrieve paths to images
    std::vector<std::string> vstrImageFilenamesRGB;
    std::vector<std::string> vstrImageFilenamesD;
    std::vector<double> vTimestamps;
    RCLCPP_INFO(
        node->get_logger(),
        "Load images from %s with association file %s",
        node->get_parameter("sequence_dir").as_string().c_str(),
        node->get_parameter("association_file").as_string().c_str()
    );
    int ret = LoadImages(
        node->get_parameter("association_file").as_string(),
        vstrImageFilenamesRGB,
        vstrImageFilenamesD,
        vTimestamps
    );
    if (ret) {
        std::cerr << "Load images failed" << std::endl;
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Loaded %ld frames", vstrImageFilenamesRGB.size());

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty()) {
        std::cerr << std::endl << "No images found in provided path." << std::endl;
        return -1;
    } else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()) {
        std::cerr << std::endl << "Different number of images for rgb and depth." << std::endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    RCLCPP_INFO(
        node->get_logger(),
        "Init SLAM with vocabulary %s and setting %s",
        node->get_parameter("voc_file").as_string().c_str(),
        node->get_parameter("setting_file").as_string().c_str()
    );
    ORB_SLAM3::System SLAM(
        node->get_parameter("voc_file").as_string(),
        node->get_parameter("setting_file").as_string(),
        ORB_SLAM3::System::RGBD,
        true
    );
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    std::vector<int64_t> vTimesTrack;

    // Main loop
    cv::Mat imRGB, imD;
    for(int ni = 0; ni < nImages; ni++) {
        // Read image and depthmap from file
        imRGB = cv::imread(
            node->get_parameter("sequence_dir").as_string() + "/" + vstrImageFilenamesRGB[ni],
            cv::IMREAD_UNCHANGED
        );
        imD = cv::imread(
            node->get_parameter("sequence_dir").as_string() + "/" + vstrImageFilenamesD[ni],
            cv::IMREAD_UNCHANGED
        );
        double tframe = vTimestamps[ni];

        if(imRGB.empty()) {
            std::cerr << std::endl << "Failed to load image at: "
                      << vstrImageFilenamesRGB[ni] << std::endl;
            continue;
        }

        if(imageScale != 1.f) {
            int width = imRGB.cols * imageScale;
            int height = imRGB.rows * imageScale;
            cv::resize(imRGB, imRGB, cv::Size(width, height));
            cv::resize(imD, imD, cv::Size(width, height));
        }

        // Pass the image to the SLAM system
        auto t1 = std::chrono::high_resolution_clock::now();
        SLAM.TrackRGBD(imRGB, imD, tframe);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        vTimesTrack.emplace_back(dt);

        // Wait to load the next frame
        double T = 0;
        if(ni < nImages - 1) {
            T = vTimestamps[ni+1] - tframe;
        } else if(ni > 0) {
            T = tframe-vTimestamps[ni-1];
        }

        if(dt < T) {
            ::usleep((T - dt) * 1e6);
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    std::sort(vTimesTrack.begin(), vTimesTrack.end());
    int64_t totaltime = std::accumulate(vTimesTrack.begin(), vTimesTrack.end(), 0);

    RCLCPP_INFO(node->get_logger(), "Median tracking time: %.2f ms", vTimesTrack[nImages / 2] / 1000.f);
    RCLCPP_INFO(node->get_logger(), "Mean tracking time: %.2f ms", totaltime / nImages / 1000.f);

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}
