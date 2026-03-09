#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "aruco_detect/GenerateFractalMarker.h"

using namespace cv;
using namespace cv::aruco;

/* ---------------- Dictionary helper ---------------- */

Ptr<Dictionary> getDictionary(const std::string& name)
{
    static std::map<std::string, PREDEFINED_DICTIONARY_NAME> dict_map = {
        {"DICT_4X4_50", DICT_4X4_50},
        {"DICT_4X4_100", DICT_4X4_100},
        {"DICT_4X4_250", DICT_4X4_250},
        {"DICT_4X4_1000", DICT_4X4_1000},
        {"DICT_5X5_50", DICT_5X5_50},
        {"DICT_5X5_100", DICT_5X5_100},
        {"DICT_5X5_250", DICT_5X5_250},
        {"DICT_5X5_1000", DICT_5X5_1000},
        {"DICT_6X6_50", DICT_6X6_50},
        {"DICT_6X6_100", DICT_6X6_100},
        {"DICT_6X6_250", DICT_6X6_250},
        {"DICT_6X6_1000", DICT_6X6_1000},
        {"DICT_7X7_50", DICT_7X7_50},
        {"DICT_7X7_100", DICT_7X7_100},
        {"DICT_7X7_250", DICT_7X7_250},
        {"DICT_7X7_1000", DICT_7X7_1000},
        {"DICT_ARUCO_ORIGINAL", DICT_ARUCO_ORIGINAL}
    };

    auto it = dict_map.find(name);
    if (it == dict_map.end())
    {
        ROS_WARN("Unknown dictionary '%s', defaulting to DICT_4X4_50", name.c_str());
        return getPredefinedDictionary(DICT_4X4_50);
    }
    return getPredefinedDictionary(it->second);
}

/* ---------------- Service callback ---------------- */

bool generateMarker(
    aruco_detect::GenerateFractalMarker::Request& req,
    aruco_detect::GenerateFractalMarker::Response& res)
{
    ros::NodeHandle nh("~");

    std::string output_dir;
    int pixel_size, margin, border_bits;

    nh.param("output_dir", output_dir, std::string("/tmp"));
    nh.param("pixel_size", pixel_size, 1000);
    nh.param("margin", margin, 10);
    nh.param("border_bits", border_bits, 1);

    std::string output_path =
        output_dir + "/" + req.name + ".png";

    ROS_INFO_STREAM("Generating fractal marker:");
    ROS_INFO_STREAM("  Name       : " << req.name);
    ROS_INFO_STREAM("  Dictionary : " << req.dictionary);
    ROS_INFO_STREAM("  Levels     : " << req.levels);
    ROS_INFO_STREAM("  Scale      : " << req.scale);
    ROS_INFO_STREAM("  Output     : " << output_path);

    Ptr<Dictionary> dictionary = getDictionary(req.dictionary);

    Ptr<FractalMarkerSet> fractal =
        FractalMarkerSet::create(
            dictionary,
            req.levels,
            req.scale,
            1.0f,
            1
        );

    Mat image;
    fractal->draw(
        Size(pixel_size, pixel_size),
        image,
        margin,
        border_bits
    );

    if (!imwrite(output_path, image))
    {
        res.success = false;
        res.message = "Failed to write PNG";
        return true;
    }

    res.success = true;
    res.message = "Fractal marker generated successfully";
    res.output_path = output_path;
    return true;
}

/* ---------------- Main ---------------- */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fractal_marker_server");
    ros::NodeHandle nh;

    ros::ServiceServer service =
        nh.advertiseService("generate_fractal_marker", generateMarker);

    ROS_INFO("Fractal marker service ready.");
    ros::spin();
    return 0;
}
