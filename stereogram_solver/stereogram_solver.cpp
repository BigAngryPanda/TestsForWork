#include "stereogram_solver.hpp"

#include <bits/stdint-uintn.h>
#include <climits>
#include <cstddef>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <pcl/impl/point_types.hpp>

namespace st
{
    int overlap_diff(cv::Mat const& stereogram, int const offset)
    {
        int result = 0;
        int const n_channels = stereogram.channels();

        for (int i = 0; i < stereogram.rows; ++i)
        {
            for (int j = 0; j < stereogram.cols - offset; ++j)
            {
                for (int n = 0; n < n_channels; ++n)
                {
                    result += std::abs(
                        stereogram.at<uint8_t>(i, j*n_channels + n) -
                        stereogram.at<uint8_t>(i, (j + offset)*n_channels + n)
                    );
                }
            }
        }

        return result;
    }

    int find_offset(cv::Mat const& stereogram)
    {
        int result = min_offset;
        int min_diff = INT_MAX;

        for (int i = stereogram.cols/20; i < stereogram.cols/4; ++i)
        {
            int diff = overlap_diff(stereogram, i);

            if (diff < min_diff)
            {
                min_diff = diff;
                result = i;
            }
        }

        return result;
    }

    int find_depth(cv::Mat const& stereogram, int const offset, int const x, int const y)
    {
        int result = INT_MAX;
        double best_diff = DBL_MAX;

        // Block after offset
        cv::Rect block_after(x + offset, y, match_width, 1);

        for (int depth = 0; depth < offset / 2; ++depth)
        {
            double curr_diff = 0.0;

            for (int n = 0; n < stereogram.channels(); ++n)
            {
                cv::Mat diff_mat;
                cv::absdiff
                (
                    stereogram(cv::Rect(x + depth, y, match_width, 1)),
                    stereogram(block_after),
                    diff_mat
                );

                cv::Scalar diff = cv::sum(diff_mat);

                for (int i = 0; i < stereogram.channels(); ++i)
                {
                    curr_diff += diff[i];
                }
            }

            if (curr_diff < best_diff)
            {
                best_diff = curr_diff;
                result = depth;
            }
        }

        return result;
    }

    cv::Mat calculate_depth_map(cv::Mat const& stereogram, int const offset)
    {
        // Depth map
        cv::Mat result = cv::Mat(
            cv::Size(stereogram.cols - offset, stereogram.rows),
            CV_32SC1, cv::Scalar::all(0)
        );

        for (int row = 0; row < stereogram.rows; ++row)
        {
            for (int col = 0; col < (stereogram.cols - offset - match_width); ++col)
            {
                result.at<int>(row, col) = find_depth(stereogram, offset, col, row);
            }
        }

        result *= 255 / (offset / 4);

        return result;
    }

    pcl::PointCloud<pcl::PointXYZ> as_points_cloud(cv::Mat const& data)
    {
        pcl::PointCloud<pcl::PointXYZ> result;

        for (int row = 0; row < data.rows; ++row)
        {
            for (int col = 0; col < data.cols; ++col)
            {
                if (data.at<int>(row, col) > background_filter)
                {
                    result.push_back(pcl::PointXYZ(row, col, data.at<int>(row, col)));
                }
            }
        }

        return result;
    }
}