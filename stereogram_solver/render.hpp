#pragma once

#include <string>
#include <condition_variable>

#include "stereogram_solver.hpp"

namespace graphics
{
    class render
    {
    public:
        render()  = default;
        ~render() = default;
        void run();
        void update_image(char const* path);
        void stop();
        bool is_busy(); // return true for working, false for idle state
        bool is_active();
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_ptr{nullptr};
        pcl::visualization::CloudViewer m_viewer{"Cloud Viewer"};
        bool m_active{true};
        bool m_sync_flag{true}; // true for working, false for idle state
        std::mutex m_draw_mtx;
        std::string m_img_name;
        std::condition_variable m_cv;
    };
}