#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

namespace st
{
    constexpr int min_offset = 1;
    constexpr int match_width = 20;
    constexpr int background_filter = 1;

    int find_offset(cv::Mat const&);
    cv::Mat calculate_depth_map(cv::Mat const&, int const);
    pcl::PointCloud<pcl::PointXYZ> as_points_cloud(cv::Mat const&);
}