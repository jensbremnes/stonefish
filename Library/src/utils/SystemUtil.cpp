/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  SystemUtil.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 11/28/12.
//  Copyright (c) 2012-2026 Patryk Cieslak. All rights reserved.
//

// This translation unit intentionally includes no Stonefish headers. The platform headers
// pulled in below (in particular <windows.h>) define macros such as ERROR, TRANSPARENT,
// near and far, which collide with identifiers used throughout the library.

#include <thread>

#if defined(_WIN32)
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
    #include <vector>
#elif defined(__APPLE__)
    #include <sys/sysctl.h>
#elif defined(__linux__)
    #include <fstream>
    #include <string>
    #include <set>
#endif

namespace sf
{

unsigned int GetPhysicalCores()
{
#if defined(_WIN32)
    // Windows: Use GetLogicalProcessorInformation
    DWORD length = 0;
    GetLogicalProcessorInformation(nullptr, &length);
    std::vector<SYSTEM_LOGICAL_PROCESSOR_INFORMATION> buffer(length / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION));

    if (GetLogicalProcessorInformation(buffer.data(), &length)) {
        unsigned int cores = 0;
        for (const auto& info : buffer) 
        {
            if (info.Relationship == RelationProcessorCore) 
                cores++;
        }
        return cores > 0 ? cores : 1;
    }
#elif defined(__APPLE__)
    // macOS: Use sysctlbyname
    int cores = 0;
    size_t size = sizeof(cores);
    if (sysctlbyname("hw.physicalcpu", &cores, &size, nullptr, 0) == 0) 
    {
        return cores;
    }
#elif defined(__linux__)
    // Linux: Read /proc/cpuinfo and count unique core IDs per physical ID
    std::ifstream file("/proc/cpuinfo");
    if (file.is_open()) 
    {
        std::string line;
        std::set<std::string> uniqueCores;
        std::string currentPhysId = "0";
        std::string currentCoreId = "";

        while (std::getline(file, line)) 
        {
            if (line.rfind("physical id", 0) == 0) 
            {
                currentPhysId = line.substr(line.find(':') + 1);
            } 
            else if (line.rfind("core id", 0) == 0) 
            {
                currentCoreId = line.substr(line.find(':') + 1);
                // Pair them up to handle multi-socket machines correctly
                uniqueCores.insert(currentPhysId + "_" + currentCoreId);
            }
        }
        if (!uniqueCores.empty()) 
        {
            return uniqueCores.size();
        }
    }
#endif

    // Fallback if OS-specific queries fail
    unsigned int logical = std::thread::hardware_concurrency();
    return logical > 0 ? logical : 1; 
}

}
