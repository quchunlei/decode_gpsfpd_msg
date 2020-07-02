/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2019, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Localization Group
 * Version: 0.2.0
 * Date: 2019
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#ifndef PROJECTS_PROMPT_H
#define PROJECTS_PROMPT_H

namespace robosense
{
#define RESET "\033[0m"
#define BLACK "\033[30m"              /* Black */
#define RED "\033[31m"                /* Red */
#define GREEN "\033[32m"              /* Green */
#define YELLOW "\033[33m"             /* Yellow */
#define BLUE "\033[34m"               /* Blue */
#define MAGENTA "\033[35m"            /* Magenta */
#define CYAN "\033[36m"               /* Cyan */
#define WHITE "\033[37m"              /* White */
#define BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m"   /* Bold White */

#include <iostream>
#define COUT (std::cout)
#define INFO (std::cout << GREEN)
#define EXPLAIN (std::cout << BOLDBLUE)
#define WARNING (std::cout << BOLDYELLOW)
#define ERROR (std::cout << RED)
#define DEBUG (std::cout << BOLDBLACK)
#define END (std::endl)
#define REND "\033[0m" << std::endl

// cursor movement
#define DELETE_LINE "\033[2K"
#define MOVE_UP(X) (std::string("\033[") + std::to_string(X) + std::string("A").c_str())

#define FLUSHCOUT (std::cerr << GREEN)
#define FLUSHWCOUT (std::cerr << BOLDYELLOW)
#define FLUSHICOUT (std::cerr << GREEN)
#define FLUSHEND "\033[0m\r" << std::flush

#define SETW_0 std::setw(0) << "    "
#define SETW_5 std::setw(5) << "    "
#define SETW_15 std::setw(15) << "    "
#define SETP_0 std::setprecision(0)
#define SETP_5 std::setprecision(5)
#define SETP_10 std::setprecision(10)
#define SETP_15 std::setprecision(15)
#define SETP_20 std::setprecision(20)

struct alignas(8) Point2D
{
  float x;
  float y;
};

struct alignas(8) Point3D
{
  float x;
  float y;
  float z;
};
}  // namespace robosense
#endif  // PROJECTS_PROMPT_H
