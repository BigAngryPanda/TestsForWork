#include "file_list.hpp"

#include <algorithm>
#include <dirent.h>
#include <errno.h>

#include <log.hpp>
#include <stdexcept>
#include <string>

namespace dir
{
    bool match_ext(std::string const& src, std::string const& ext)
    {
        if (ext.size() > src.size())
        {
           return false;
        }

        return 0 == src.compare(src.length() - ext.length(), ext.length(), ext);
    }

    void file_list::open(char const* directory)
    {
        DIR* dirp = opendir(directory);

        if (!dirp)
        {
            throw std::runtime_error(
                "Failed to open "
                + std::string(directory)
                + " with error code "
                + std::to_string(errno)
            );
        }

        m_path = directory;

        if (m_path.back() != '/')
        {
            m_path += '/';
        }

        struct dirent* dp;

        while ((dp = readdir(dirp)) != NULL)
        {
            if (match_ext(dp->d_name, ".jpg") || match_ext(dp->d_name, ".jpeg"))
            {
                trace::stdout("Found " + std::string(dp->d_name));
                m_files.push_back(dp->d_name);
            }
        }

        closedir(dirp);

        std::sort(m_files.begin(), m_files.end());
    }

    dir_iterator::dir_iterator(file_list const& data) :
        m_length(data.m_files.size()),
        m_list(data)
    {}

    dir_iterator& dir_iterator::operator++()
    {
        m_counter = (m_counter + 1) % m_length;

        return *this;
    }

    dir_iterator& dir_iterator::operator--()
    {
        m_counter = (m_counter + m_length - 1) % m_length;

        return *this;
    }

    std::string dir_iterator::operator*() const
    {
        return m_list.m_path + m_list.m_files[m_counter];
    }
}