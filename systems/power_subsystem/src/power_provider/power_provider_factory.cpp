/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include "power_subsystem/power_provider/power_provider.hpp"
#include "power_subsystem/power_provider/rpm_generator.hpp"
#include "power_subsystem/power_provider/simple_battery.hpp"
#include "power_subsystem/power_provider/simple_generator.hpp"

namespace lotusim::gazebo {

PowerProvider::CreateResult PowerProvider::createFromSdf(
    const std::string& provider_name,
    const std::string& vessel_name,
    const sdf::ElementPtr& sdf,
    rclcpp::Node::SharedPtr node,
    const std::shared_ptr<spdlog::logger>& logger)
{
    if (!sdf->HasElement("type")) {
        if (logger)
            logger->error(
                "PowerProvider::createFromSdf: [{},{}] has <lotusim_power> "
                "but missing required <type> -> skipping",
                provider_name,
                vessel_name);
        return {.provider = nullptr, .type = ProviderType::Unknown};
    }

    std::string providerTypeStr = sdf->Get<std::string>("type", "").first;
    const auto typeOpt = providerTypeFromString(providerTypeStr);
    if (!typeOpt) {
        logger->warn(
            "PowerProvider::createFromSdf : unknown type '{}' on "
            "consumer [{},{}] -> skipping",
            providerTypeStr,
            provider_name,
            vessel_name);
        return {.provider = nullptr, .type = ProviderType::Unknown};
    }

    switch (*typeOpt) {
        case ProviderType::SimpleBattery:
            return {
                .provider = std::make_shared<SimpleBattery>(
                    provider_name,
                    vessel_name,
                    sdf,
                    std::move(node),
                    logger),
                .type = ProviderType::SimpleBattery};

        case ProviderType::SimpleGenerator:
            return {
                .provider = std::make_shared<SimpleGenerator>(
                    provider_name,
                    vessel_name,
                    sdf,
                    std::move(node),
                    logger),
                .type = ProviderType::SimpleGenerator};

        case ProviderType::RPMGenerator:
            return {
                .provider = std::make_shared<RpmGenerator>(
                    provider_name,
                    vessel_name,
                    sdf,
                    std::move(node),
                    logger),
                .type = ProviderType::RPMGenerator};
        default:
            logger->error(
                "PowerProvider::createFromSdf: unknown type '{}' for [{},{}] -> skipping",
                providerTypeStr,
                provider_name,
                vessel_name);
            return {.provider = nullptr, .type = ProviderType::Unknown};
    }
    logger->error(
        "PowerProvider::createFromSdf: unknown type '{}' for [{},{}] -> skipping",
        providerTypeStr,
        provider_name,
        vessel_name);
    return {.provider = nullptr, .type = ProviderType::Unknown};
}

}  // namespace lotusim::gazebo
