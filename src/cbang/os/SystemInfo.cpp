/******************************************************************************\

          This file is part of the C! library.  A.K.A the cbang library.

                Copyright (c) 2003-2023, Cauldron Development LLC
                               All rights reserved.

         The C! library is free software: you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public License
       as published by the Free Software Foundation, either version 2.1 of
               the License, or (at your option) any later version.

        The C! library is distributed in the hope that it will be useful,
          but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
                 Lesser General Public License for more details.

         You should have received a copy of the GNU Lesser General Public
                 License along with the C! library.  If not, see
                         <http://www.gnu.org/licenses/>.

        In addition, BSD licensing may be granted on a case by case basis
        by written permission from at least one of the copyright holders.
           You may request written permission by emailing the authors.

                  For information regarding this software email:
                                 Joseph Coffland
                          joseph@cauldrondevelopment.com

\******************************************************************************/

#include "SystemInfo.h"
#include "PowerManagement.h"
#include "CPUInfo.h"

#include <cbang/Info.h>
#include <cbang/SStream.h>
#include <cbang/String.h>

#include <cbang/os/Thread.h>
#include <cbang/os/SystemUtilities.h>

#include <cbang/log/Logger.h>
#include <cbang/util/HumanSize.h>

#include <cbang/boost/StartInclude.h>
#include <boost/filesystem/operations.hpp>
#include <cbang/boost/EndInclude.h>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN // Avoid including winsock.h
#include <windows.h>
#include <sysinfoapi.h>

#elif defined(__APPLE__)
#include <sys/types.h>
#include <sys/param.h>
#include <sys/sysctl.h>

#include <mach/vm_statistics.h>
#include <mach/mach_types.h>
#include <mach/mach_init.h>
#include <mach/mach_host.h>

#include <CoreFoundation/CoreFoundation.h>
#include <CoreServices/CoreServices.h>

#include "MacOSUtilities.h"

#elif defined(__FreeBSD__)
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <sys/sysctl.h>
#include <sys/ucred.h>
#include <unistd.h>

#else // !_MSC_VER && !__APPLE__
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#endif

#ifndef _WIN32
#include <unistd.h>
#endif

using namespace cb;
using namespace std;

namespace fs = boost::filesystem;


SystemInfo::SystemInfo(Inaccessible) {detectThreads();}


uint32_t SystemInfo::getCPUCount() const {
#if defined(_WIN32)
  // Count active CPU threads
  DWORD len = 0;

  if (!GetLogicalProcessorInformationEx(RelationGroup, 0, &len)) {
    SmartPointer<uint8_t>::Array buffer = new uint8_t[len];
    auto info = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)buffer.get();

    if (!GetLogicalProcessorInformationEx(RelationGroup, info, &len)) {
      uint32_t cpus = 0;

      for (unsigned i = 0; i < len;) {
        info = (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)&buffer[i];

        for (unsigned j = 0; j < info->Group.ActiveGroupCount; j++) {
          cpus += info->Group.GroupInfo[j].ActiveProcessorCount;

          i += info->Size;
        }
      }

      if (cpus) return cpus;
    }
  }

  // Fallback to old method
  SYSTEM_INFO sysInfo;
  GetSystemInfo(&sysInfo);
  return sysInfo.dwNumberOfProcessors;

#elif defined(__APPLE__)
  int nm[2];
  size_t length = 4;
  uint32_t count = 0;

  nm[0] = CTL_HW; nm[1] = HW_AVAILCPU;
  sysctl(nm, 2, &count, &length, 0, 0);

  if (!count) {
    nm[1] = HW_NCPU;
    sysctl(nm, 2, &count, &length, 0, 0);
  }

  return count ? count : 1;

#else
  long cpus = sysconf(_SC_NPROCESSORS_ONLN);
  return cpus < 1 ? 1 : cpus;
#endif
}


uint32_t SystemInfo::getPerformanceCPUCount() const {
#if defined(__APPLE__)
  int32_t pcount = 0;
  size_t len = sizeof(pcount);

  // macOS 12+
  if (!::sysctlbyname("hw.perflevel0.logicalcpu", &pcount, &len, 0, 0))
    if (0 < pcount) return pcount;

#ifdef __aarch64__
  // macOS 11 ARM; only four possibilities
  string brand = CPUInfo::create()->getBrand();

  if (brand == "Apple M1")     return 4;
  if (brand == "Apple M1 Max") return 8;
  if (brand == "Apple M1 Pro") return getCPUCount() - 2; // 6 or 8
#endif // __aarch64__
#endif // __APPLE__

  // TODO Windows and Linux support

  return 0;
}


uint64_t SystemInfo::getMemoryInfo(memory_info_t type) const {
#if defined(_WIN32)
  MEMORYSTATUSEX info;

  info.dwLength = sizeof(MEMORYSTATUSEX);
  GlobalMemoryStatusEx(&info);

  switch (type) {
  case MEM_INFO_TOTAL: return (uint64_t)info.ullTotalPhys;
  case MEM_INFO_FREE: return (uint64_t)info.ullAvailPhys;
  case MEM_INFO_SWAP: return (uint64_t)info.ullAvailPageFile;
  case MEM_INFO_USABLE:
    return (uint64_t)(info.ullAvailPhys + info.ullAvailPageFile);
  }

#elif defined(__APPLE__)
  if (type == MEM_INFO_TOTAL) {
    int64_t memory;
    int mib[2];
    size_t length = 2;

    mib[0] = CTL_HW;
    mib[1] = HW_MEMSIZE;
    length = sizeof(int64_t);

    if (!sysctl(mib, 2, &memory, &length, 0, 0)) return memory;

  } else {
    vm_size_t page;
    vm_statistics_data_t stats;
    mach_msg_type_number_t count = sizeof(stats) / sizeof(natural_t);
    mach_port_t port = mach_host_self();

    if (host_page_size(port, &page) == KERN_SUCCESS &&
        host_statistics(port, HOST_VM_INFO, (host_info_t)&stats, &count) ==
        KERN_SUCCESS)
      switch (type) {
      case MEM_INFO_TOTAL: break; // Handled above
      case MEM_INFO_FREE: return (uint64_t)(stats.free_count * page);
      case MEM_INFO_SWAP: return 0; // Not sure how to get this on OSX
      case MEM_INFO_USABLE:
        // An IBM article says this should be free + inactive + file-backed
        // pages.  No idea how to get at file-backed pages.
        return (uint64_t)((stats.free_count + stats.inactive_count) * page);
      }
  }

#else
  struct sysinfo info;
  if (!sysinfo(&info))
    switch (type) {
    case MEM_INFO_TOTAL: return (uint64_t)(info.totalram * info.mem_unit);
    case MEM_INFO_FREE: return (uint64_t)(info.freeram * info.mem_unit);
    case MEM_INFO_SWAP: return (uint64_t)(info.freeswap * info.mem_unit);
    case MEM_INFO_USABLE:
      return (uint64_t)((info.freeram + info.bufferram + info.freeswap) *
                        info.mem_unit);
    }
#endif

  return 0;
}


uint64_t SystemInfo::getFreeDiskSpace(const string &path) {
  fs::space_info si;

  try {
    si = fs::space(path);

  } catch (const fs::filesystem_error &e) {
    THROW("Could not get disk space at '" << path << "': " << e.what());
  }

  return si.available;
}


Version SystemInfo::getOSVersion() const {
#if defined(_WIN32)
  OSVERSIONINFO info;
  ZeroMemory(&info, sizeof(info));
  info.dwOSVersionInfoSize = sizeof(info);
  if (!GetVersionEx(&info)) THROW("Failed to get Windows version");

  return Version((uint8_t)info.dwMajorVersion, (uint8_t)info.dwMinorVersion);

#elif defined(__APPLE__)
  return MacOSUtilities::getMacOSVersion();

#else
  struct utsname i;

  uname(&i);
  string release = i.release;
  size_t dot = release.find('.');
  uint8_t major = String::parseU32(release.substr(0, dot));
  uint8_t minor = String::parseU32(release.substr(dot + 1));

  return Version(major, minor);
#endif
}


void SystemInfo::add(Info &info) {
  const char *category = "System";
  auto cpuInfo = CPUInfo::create();

  info.add(category, "CPU", cpuInfo->getBrand());
  info.add(category, "CPU ID", SSTR(
             cpuInfo->getVendor()
             << " Family "   << cpuInfo->getFamily()
             << " Model "    << cpuInfo->getModel()
             << " Stepping " << cpuInfo->getStepping()));
  info.add(category, "CPUs", String(getCPUCount()));

  info.add(category, "Memory", HumanSize(getTotalMemory()).toString() + "B");
  info.add(category, "Free Memory",
           HumanSize(getFreeMemory()).toString() + "B");
  info.add(category, "Threads", getThreadsType().toString());

  Version osVersion = getOSVersion();
  info.add(category, "OS Version", osVersion.toString());

  info.add(category, "Has Battery",
           String(PowerManagement::instance().hasBattery()));
  info.add(category, "On Battery",
           String(PowerManagement::instance().onBattery()));
}




void SystemInfo::detectThreads() {
  // Threads type
#ifdef _WIN32
  threadsType = ThreadsType::WINDOWS_THREADS;

#elif __linux__
  // Test for LinuxThreads which has a different PID for each thread
  struct TestThread : public Thread {
    unsigned pid;
    TestThread() : pid(0) {}
    void run() override {pid = SystemUtilities::getPID();}
  };

  TestThread testThread;
  testThread.start();
  testThread.join();

  if (testThread.pid != SystemUtilities::getPID())
    threadsType = ThreadsType::LINUX_THREADS;
  else threadsType = ThreadsType::POSIX_THREADS;

#else // __linux__
  // Assume POSIX
  threadsType = ThreadsType::POSIX_THREADS;
#endif
}
