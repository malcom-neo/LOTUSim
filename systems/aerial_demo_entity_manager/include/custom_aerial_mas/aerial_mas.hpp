/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#ifndef AERIAL_mas_HPP_
#define AERIAL_mas_HPP_

#include "lotusim_mas/multi_agent_system.hpp"

namespace lotusim::gazebo {

using ClientGoalHandleMASCmd =
    rclcpp_action::ClientGoalHandle<lotusim_msgs::action::MASCmd>;

class AerialMAS : public MultiAgentSystem {
public:
    AerialMAS();
    ~AerialMAS() override;

private:
    void customUserConfiguration(
        const std::shared_ptr<const sdf::Element>& _sdf) override;

    void customUserAddEntity(const lotusim_msgs::msg::MASCmd& msg) final;

    void customUserDeleteEntity(const lotusim_msgs::msg::MASCmd& msg) final;

    void resultCB(const ClientGoalHandleMASCmd::WrappedResult& result);

    void goalResponseCB(const ClientGoalHandleMASCmd::SharedPtr& goal_handle);

private:
    rclcpp_action::Client<lotusim_msgs::action::MASCmd>::SharedPtr
        m_aerial_mas_client;

    std::string m_aerial_namespace;
};
}  // namespace lotusim::gazebo
#endif