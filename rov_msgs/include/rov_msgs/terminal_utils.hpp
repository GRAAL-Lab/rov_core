#ifndef ROV_CTRL_TERMINAL_UTILS_HPP
#define ROV_CTRL_TERMINAL_UTILS_HPP

#include <ctime>
#include <cstdio>
#include <string>

namespace rov {

namespace tc {
const char* const none = "\033[0m";
const char* const black = "\033[0;30m";
const char* const grayD = "\033[1;30m";
const char* const red = "\033[0;31m";
const char* const redL = "\033[1;31m";
const char* const green = "\033[0;32m";
const char* const greenL = "\033[1;32m";
const char* const brown = "\033[0;33m";
const char* const yellow = "\033[1;33m";
const char* const blu = "\033[0;34m";
const char* const bluL = "\033[1;34m";
const char* const mag = "\033[0;35m";
const char* const magL = "\033[1;35m";
const char* const cyan = "\033[0;36m";
const char* const cyanL = "\033[1;36m";
const char* const grayL = "\033[0;37m";
const char* const white = "\033[1;37m";
}

struct Spinner {
    Spinner(int frequency)
        : freq(frequency)
        , spinIndex(0)
        , spin_chars("/-\\|")
    {
        clock_gettime(CLOCK_MONOTONIC, &last);
        period = 1 / static_cast<double>(freq + 1E-6);
        //std::cout << "period: " << period << "s" << std::endl;
    }

    void operator()(void)
    {
        clock_gettime(CLOCK_MONOTONIC, &now);
        double timeElapsed = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1E9;

        //std::cout << "timeElapsed: " << timeElapsed << std::endl;
        if (period - timeElapsed < 1E-3) {
            //std::cout << "fabs(freq - timeElapsed): " << fabs(freq - timeElapsed) << std::endl;
            //printf("\e[?25l"); /* hide the cursor */
            putchar(' ');
            putchar(spin_chars[spinIndex % spin_chars.length()]);
            putchar(' ');
            fflush(stdout);
            putchar('\r');
            spinIndex++;
            last = now;
        }
    }

private:
    struct timespec last, now;
    int freq;
    double period;
    unsigned long spinIndex;
    std::string spin_chars;
};

}

#endif // ROV_CTRL_TERMINAL UTILS_HPP
