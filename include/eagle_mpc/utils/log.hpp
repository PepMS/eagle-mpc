///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_UTILS_LOG_HPP_
#define EAGLE_MPC_UTILS_LOG_HPP_

#include <stdio.h>
#include <utility>

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m \n"

namespace eagle_mpc
{
void log();

template <typename First, typename... Rest>
void log(First&& first, Rest&&... rest)
{
    std::cout << std::forward<First>(first);
    log(std::forward<Rest>(rest)...);
}

#if VERBOSE_LEVEL == 1
#define EMPC_ERROR(...)                  \
    {                                    \
        printf(ANSI_COLOR_RED);          \
        printf("[EAGLE_MPC | ERROR]: "); \
        log(__VA_ARGS__);                \
        printf(ANSI_COLOR_RESET);        \
    }
#define EMPC_WARN(...)                  \
    {                                   \
        printf(ANSI_COLOR_YELLOW);      \
        printf("[EAGLE_MPC | WARN]: "); \
        log(__VA_ARGS__);               \
        printf(ANSI_COLOR_RESET);       \
    }
#define EMPC_INFO(...) \
    {                  \
    }
#elif VERBOSE_LEVEL == 2
#define EMPC_ERROR(...)                  \
    {                                    \
        printf(ANSI_COLOR_RED);          \
        printf("[EAGLE_MPC | ERROR]: "); \
        log(__VA_ARGS__);                \
        printf(ANSI_COLOR_RESET);        \
    }
#define EMPC_WARN(...)                  \
    {                                   \
        printf(ANSI_COLOR_YELLOW);      \
        printf("[EAGLE_MPC | WARN]: "); \
        log(__VA_ARGS__);               \
        printf(ANSI_COLOR_RESET);       \
    }
#define EMPC_INFO(...)                  \
    {                                   \
        printf(ANSI_COLOR_CYAN);        \
        printf("[EAGLE_MPC | INFO]: "); \
        log(__VA_ARGS__);               \
        printf(ANSI_COLOR_RESET);       \
    }
#else
#define EMPC_ERROR(...)         \
    {                           \
        printf(ANSI_COLOR_RED); \
        printf("[EAGLE_MPC | ERROR]: ");
log(__VA_ARGS__);
printf(ANSI_COLOR_RESET);
}
#define EMPC_WARN(...) \
    {                  \
    }
#define EMPC_INFO(...) \
    {                  \
    }
#endif

#if VERBOSE_DEBUG == 1
#define EMPC_DEBUG(...)           \
    {                             \
        printf("[EAGLE_MPC | DEBUG]: ");
        log(__VA_ARGS__);         \
        printf(ANSI_COLOR_RESET); \
    }
#else
#define EMPC_DEBUG(...) \
    {                   \
    }
#endif

}  // namespace eagle_mpc
#endif