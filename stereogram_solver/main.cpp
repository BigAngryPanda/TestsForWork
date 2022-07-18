#include <bits/types/FILE.h>
#include <cstddef>
#include <cstdio>
#include <string>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <asm/ioctls.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "render.hpp"
#include "file_list.hpp"
#include "log.hpp"

constexpr size_t STD_IN = 0;

enum keycode : int
{
    ESC_KEY = 27,
    LEFT_ARROW = 4479771,
    RIGHT_ARROW = 4414235,
};

int main(int argc, char const* argv[])
{
    termios term;
    tcgetattr(0, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term);

    if (argc != 2)
    {
        std::cerr << "Wrong number of arguments. Expected 2, found " << argc << std::endl;

        return 1;
    }

    dir::file_list files;

    try
    {
        files.open(argv[1]);
    }
    catch (std::runtime_error const& e)
    {
        std::cerr << e.what() << std::endl;

        return 2;
    }

    dir::dir_iterator it(files);

    graphics::render rnd;
    std::thread render_thread(&graphics::render::run, &rnd);

    rnd.update_image((*it).data());

    bool is_running = true;

    while (is_running && rnd.is_active())
    {
        trace::stdout("[Thread 0] Waiting for input");
        int keycode = 0;
        read(STD_IN, &keycode, sizeof(int));

        trace::stdout("[Thread 0] Receive key " + std::to_string(keycode));

        if (rnd.is_busy())
        {
            trace::stdout("[Thread 0] Key was ignored");
            continue;
        }

        switch (keycode)
        {
            case ESC_KEY:
            {
                rnd.stop();
                is_running = false;
                break;
            }
            case LEFT_ARROW:
            {
                --it;
                rnd.update_image((*it).data());
                break;
            }
            case RIGHT_ARROW:
            {
                ++it;
                rnd.update_image((*it).data());
                break;
            }
        }
    }

    render_thread.join();

    return 0;
}