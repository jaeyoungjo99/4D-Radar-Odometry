/**
 * Module:      function_print.hpp
 * Description: print util functions for AutoKU
 * 
 * Authors: Jiwon Seok (pauljiwon96@gmail.com)
 *          Yuseung Na (ys.na0220@gmail.com)
 * 
 * Revision History
 *      Feb. 13, 2022: Jiwon Seok - Created.
 *      July 20, 2023: Yuseung Na - Update for AutoKU.
 */

#ifndef __FUNCTION_PRINT_HPP__
#define __FUNCTION_PRINT_HPP__
#pragma once

// STD header
#include <iostream>
#include <string>
#include <chrono>

using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

namespace autoku_functions {
    enum Color{ RESET,
                BLACK,
                RED,
                GREEN,
                YELLOW,
                BLUE,
                MAGENTA,
                CYAN,
                WHITE,
                BOLDBLACK,
                BOLDRED,
                BOLDGREEN,
                BOLDYELLOW,
                BOLDBLUE,
                BOLDMAGENTA,
                BOLDCYAN,
                BOLDWHITE,
                BOLDRESET};

    static const char *enum_str[] ={"\033[0m",          /* Reset */
                                    "\033[30m",         /* Black */
                                    "\033[31m",         /* Red */
                                    "\033[32m",         /* Green */
                                    "\033[33m",         /* Yellow */
                                    "\033[34m",         /* Blue */
                                    "\033[35m",         /* Magenta */
                                    "\033[36m",         /* Cyan */
                                    "\033[37m",         /* White */
                                    "\033[1m\033[30m",  /* Bold Black */
                                    "\033[1m\033[31m",  /* Bold Red */
                                    "\033[1m\033[32m",  /* Bold Green */
                                    "\033[1m\033[33m",  /* Bold Yellow */
                                    "\033[1m\033[34m",  /* Bold Blue */
                                    "\033[1m\033[35m",  /* Bold Magenta */
                                    "\033[1m\033[36m",  /* Bold Cyan */
                                    "\033[1m\033[37m"   /* Bold White */
                                    "\033[1m\033[0m"    /* Bold Reset */
                                    };

    template <typename TYPE>
    inline void DebugPrintValueInfo(const std::string& debug_info, const TYPE& value, const Color& color=RESET) {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout<< enum_str[color] << "[ INFO]" << "[" << time  <<"] " << debug_info << " " << value << enum_str[RESET] << std::endl;
    }

    template <typename TYPE>
    inline void DebugPrintValueWarn(const std::string& debug_info, const TYPE& value, const Color& color=YELLOW) {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[ WARN]" << "[" << time  <<"] " << debug_info << " " << value << enum_str[RESET] << std::endl;        
    }

    template <typename TYPE>
    inline void DebugPrintValueError(const std::string& debug_info, const TYPE& value, const Color& color=RED) {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[ERROR]" << "[" << time  <<"] " << debug_info << " " << value << enum_str[RESET] << std::endl;
    }
    
    inline void DebugPrintInfo(const std::string& debug_info, const Color& color=RESET) {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout<< enum_str[color] << "[ INFO]" << "[" << time  <<"] " << debug_info << enum_str[RESET] << std::endl;
    }

    inline void DebugPrintWarn(const std::string& debug_info, const Color& color=YELLOW) {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[ WARN]" << "[" << time  <<"] " << debug_info << enum_str[RESET] << std::endl;        
    }

    inline void DebugPrintError(const std::string& debug_info, const Color& color=RED) {
        std::string time = std::to_string(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count()/1000.);
        std::cout << enum_str[color] << "[ERROR]" << "[" << time  <<"] " << debug_info << enum_str[RESET] << std::endl;
    }
}

#endif  // __FUNCTION_PRINT_HPP__