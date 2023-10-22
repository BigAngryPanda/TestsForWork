#ifndef ARCH_HPP
#define ARCH_HPP

namespace arch
{

#if defined(_WIN32) || defined(_WIN64)
constexpr char root_path[] = "C:\\";
#else
constexpr char root_path[] = "/";
#endif

}
#endif
