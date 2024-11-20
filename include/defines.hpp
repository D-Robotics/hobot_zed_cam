// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <stdint.h>
#include <string>
#include <string.h>
#include <cxxabi.h>
#include <iostream>
#include <vector>
#include <chrono>

#if defined _WIN32
#if defined(SL_OC_COMPIL)
#define SL_OC_EXPORT __declspec(dllexport)
#else
#define SL_OC_EXPORT __declspec(dllimport)
#endif
#elif __GNUC__
#define SL_OC_EXPORT __attribute__((visibility("default")))
#if defined(__arm__) || defined(__aarch64__)
#define _SL_JETSON_
#endif
#endif

//// SDK VERSION NUMBER
#define ZED_OC_MAJOR_VERSION 0
#define ZED_OC_MINOR_VERSION 6
#define ZED_OC_PATCH_VERSION 0

#define ZED_OC_VERSION_ATTRIBUTE private: uint32_t mMajorVer = ZED_OC_MAJOR_VERSION, mMinorVer = ZED_OC_MINOR_VERSION, mPatchVer = ZED_OC_PATCH_VERSION

// Debug output
#define INFO_OUT(lvl,msg) { int status_dem_0; if (lvl>=3) std::cout << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] INFO: " << msg << std::endl; }
#define WARNING_OUT(lvl,msg) { int status_dem_0; if (lvl>=2) std::cerr << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] WARNING: " << msg << std::endl; }
#define ERROR_OUT(lvl,msg) { int status_dem_0; if (lvl>=1) std::cerr << "[" << abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status_dem_0) << "] ERROR: " << msg << std::endl; }



/*!
 * \brief Get the current system clock as steady clock, so with no jumps even if the system time changes
 * \return the current steady system clock in nanoseconds
 */
inline uint64_t getSteadyTimestamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}

/*!
 * \brief Get the current system clock as wall clock (it can have jumps if the system clock is updated by a sync service)
 * \return the current wall system clock in nanoseconds
 */
inline uint64_t getWallTimestamp() {return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();}

namespace sl_oc {

static const uint16_t SL_USB_VENDOR = 0x2b03;           //!< Stereolabs Vendor ID

static const uint16_t SL_USB_PROD_ZED_REVA = 0xf580;         //!< Old ZED firmware Product ID
static const uint16_t SL_USB_PROD_ZED_M_REVA = 0xf680;       //!< Old ZED-M binary modified firmware Product ID
static const uint16_t SL_USB_PROD_ZED_REVB = 0xf582;     //!< CBS ZED Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_M_REVB = 0xf682;   //!< CBS ZED-M Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_2_REVB = 0xf780;   //!< CBS ZED 2 Firmware Product ID
static const uint16_t SL_USB_PROD_ZED_2i = 0xf880;   //!< CBS ZED 2i Firmware Product ID
static const uint16_t SL_USB_PROD_MCU_ZEDM_REVA= 0xf681; //!< MCU sensor device for ZED-M
static const uint16_t SL_USB_PROD_MCU_ZED2_REVA = 0xf781; //!< MCU sensor device for ZED2
static const uint16_t SL_USB_PROD_MCU_ZED2i_REVA = 0xf881; //!< MCU sensor device for ZED2i

enum VERBOSITY {
    NONE = 0,
    ERROR = 1,
    WARNING = 2,
    INFO = 3
};

}

#endif //DEFINES_HPP
