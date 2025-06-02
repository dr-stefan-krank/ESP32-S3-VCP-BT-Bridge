/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <functional>  // Add this include
#include <memory>
#include <type_traits>
#include <vector>

#include "usb/cdc_acm_host.h"

namespace esp_usb {
/**
 * @brief Virtual COM Port Service Class
 *
 * Virtual COM Port (VCP) service manages drivers to connected VCP devices -
 * typically USB <-> UART converters. In practice, you rarely care about
 * specifics of the devices; you only want uniform interface for them all. VCP
 * service does just that, after you register drivers for various VCP devices,
 * you can just call VCP::open and the service will load proper driver for
 * device that was just plugged into USB port.
 *
 * Example usage:
 * \code{.cpp}
 * VCP::register_driver<FT23x>();   // Uses FT23x::vids and FT23x::pids
 * VCP::register_driver<CP210x>();  // Uses CP210x::vids and CP210x::pids
 * VCP::register_driver<CH34x>();   // Uses CH34x::vids and CH34x::pids
 * auto vcp = VCP::open(&dev_config);
 * \endcode
 *
 * The example code assumes that you have USB Host Lib already installed.
 */
class VCP {
   public:
    /**
     * @brief Register VCP driver to VCP service
     * @tparam T VCP driver type with vids[] and pids[] arrays
     */
    template <class T>
    static void register_driver() {
        static_assert(T::pids.begin() != nullptr,
                      "Every VCP driver must contain array of supported PIDs");
        static_assert(T::vids.size() > 0,
                      "Every VCP driver must contain array of supported VIDs");

        // Register a driver instance for each VID
        for (uint16_t vid : T::vids) {
            std::vector<uint16_t> pids(T::pids.begin(), T::pids.end());

            // Create device factory function with VID closure
            auto factory = [vid](uint16_t pid,
                                 const cdc_acm_host_device_config_t *dev_config,
                                 uint8_t interface_idx) -> CdcAcmDevice * {
                return new T(vid, pid, dev_config, interface_idx);
            };

            // Register driver for this VID
            drivers.push_back(vcp_driver(factory, vid, pids));
        }
    }

    static CdcAcmDevice *open(uint16_t _vid, uint16_t _pid,
                              const cdc_acm_host_device_config_t *dev_config,
                              uint8_t interface_idx = 0);

    static CdcAcmDevice *open(const cdc_acm_host_device_config_t *dev_config,
                              uint8_t interface_idx = 0);

   private:
    typedef struct vcp_driver {
        // Change from function pointer to std::function
        using factory_func_t = std::function<CdcAcmDevice *(
            uint16_t pid, const cdc_acm_host_device_config_t *dev_config,
            uint8_t interface_idx)>;

        factory_func_t open;
        uint16_t vid;
        std::vector<uint16_t> pids;

        vcp_driver(factory_func_t _open, uint16_t _vid,
                   const std::vector<uint16_t> &_pids)
            : open(_open), vid(_vid), pids(_pids) {}
    } vcp_driver;

    static std::vector<vcp_driver> drivers;

    // Disable default constructors
    VCP() = delete;
    VCP(const VCP &) = delete;
    VCP &operator=(const VCP &) = delete;
};
}  // namespace esp_usb
