#pragma once

#include <string>
#include <vector>

namespace dir
{
    bool match_ext(std::string const&, std::string const&);

    // Collection of filenames
    // Selects only .jpg or .jpeg images
    class file_list
    {
    public:
        file_list()  = default;
        ~file_list() = default;

        void open(char const*);
    private:
        std::vector<std::string> m_files;
        std::string m_path;
    friend class dir_iterator;
    };

    // Of course it is not *true* iterator
    // Just simple useful class to handle work with files
    // This "iterator" has no end
    // In other words it performs cyclic iterations
    class dir_iterator
    {
    public:
        dir_iterator(file_list const&);
        ~dir_iterator() = default;
        std::string operator*() const;
        dir_iterator& operator++();
        dir_iterator& operator--();
    private:
        size_t const m_length{0};
        size_t m_counter{0};
        file_list const& m_list;
    };
}