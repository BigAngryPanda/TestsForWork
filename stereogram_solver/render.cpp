#include "render.hpp"

#include <cstdlib>
#include <iostream>
#include <mutex>
#include <pcl/visualization/cloud_viewer.h>

#include <log.hpp>

namespace graphics
{
    void render::run()
    {
        while (is_active())
        {
            trace::stdout("[Thread 1] Read image " + m_img_name);

            cv::Mat image;
            image = imread(m_img_name, cv::IMREAD_COLOR);

            if (image.data == NULL)
            {
                throw std::runtime_error(
                    "Failed to open "
                    + m_img_name
                );
            }

            trace::stdout("[Thread 1] Calculating offset...");

            int offset = st::find_offset(image);

            trace::stdout("[Thread 1] Calculating depth map. It may take a long time...");

            cv::Mat depth_map = st::calculate_depth_map(image, offset);

            m_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
            *m_cloud_ptr = st::as_points_cloud(depth_map);

            trace::stdout("[Thread 1] Display image");

            m_viewer.showCloud(m_cloud_ptr);

            m_sync_flag = false;

            trace::stdout("[Thread 1] Done");
            std::unique_lock<std::mutex> lk(m_draw_mtx);
            m_cv.wait(lk, [&]{return m_sync_flag;});
        }
    }

    void render::update_image(char const* path)
    {
        m_img_name = path;
        m_sync_flag = true;
        m_cv.notify_all();
        trace::stdout("[Thread 0] Image updated");
    }

    void render::stop()
    {
        m_active = false;
        m_sync_flag = true;
        m_cv.notify_all();
    }

    bool render::is_busy()
    {
        return m_sync_flag;
    }

    bool render::is_active()
    {
        return !m_viewer.wasStopped() && m_active;
    }
}