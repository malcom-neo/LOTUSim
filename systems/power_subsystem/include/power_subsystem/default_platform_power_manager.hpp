/*
 * Copyright (c) 2026 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
/**
 * @file default_platform_power_manager.hpp
 * @brief Default per-vessel power accounting strategy.
 *
 * Declares DefaultPlatformPowerManager, the built-in implementation of
 * PlatformPowerManagerBase selected by PlatformPowerManagerType::DEFAULT.
 * It implements the abstract handlePowerUpdate() hook that the base class
 * calls on every simulation tick.
 *
 * @section power_model Power model
 *
 * Providers are ranked in vessel-SDF declaration order. At any instant one
 * battery is the "active" bus source (m_active_battery_index); generators are
 * treated as a secondary pool used to charge batteries and, once every battery
 * is depleted, to feed consumers directly.
 *
 * @section tick_flow Per-tick flow (handlePowerUpdate)
 *
 *   1. Sum drawnCurrent() over all active consumers -> total demand (A).
 *   2. Take activeBusVoltage() as the bus reference.
 *   3. distributeLoad(): apply the load to the active battery and, if a
 *      generator has headroom, trickle-charge a battery that is not full;
 *      when all batteries are depleted, let the generator cover consumers
 *      and try to reactivate previously shed loads.
 *   4. If the active battery reports PowerLevel::DEPLETED, handleDepleted()
 *      switches to the next non-depleted battery or falls back to the
 *      generator (shedding consumers it cannot cover).
 *   5. shedLoads() drops low-priority consumers when the active battery is
 *      WARN/CRITICAL and no spare battery or generator can cover them.
 *   6. Push the resulting bus voltage to every consumer via
 *      receiveVoltage() + update().
 *
 * @section priorities Consumer priorities
 *
 * Consumer priority() runs 1 (safety-critical, never shed) to 4 (first to be
 * shed). Load shedding walks groups 4 -> 2; reactivation walks groups 1 -> 4.
 *
 * @see PlatformPowerManagerBase
 */
#pragma once

#include "lotusim_common/common.hpp"
#include "power_subsystem/platform_power_manager_base.hpp"

namespace lotusim::gazebo {

/**
 * @brief Built-in priority-ordered power manager for a single vessel.
 *
 * Instantiated by PlatformPowerManagerBase::create() for
 * PlatformPowerManagerType::DEFAULT. All strategy logic is private and driven
 * through the overridden handlePowerUpdate() hook
 */
class DefaultPlatformPowerManager : public PlatformPowerManagerBase {
public:
    /**
     * @brief Construct the manager and forward everything to the base class.
     *
     * Providers and consumers are discovered from @p sdfptr by the base
     * constructor; this class adds no extra configuration.
     *
     * @param vessel_entity Gazebo model entity for this vessel.
     * @param vessel_name    Vessel name, used for logging and topics.
     * @param node           Shared ROS 2 node.
     * @param sdfptr         Root SDF element of the vessel model.
     */
    DefaultPlatformPowerManager(
        const gz::sim::Entity& vessel_entity,
        const std::string& vessel_name,
        rclcpp::Node::SharedPtr node,
        sdf::ElementPtr sdfptr);

private:
    /**
     * @brief First generator in declaration order that is not depleted.
     * @return The generator, or nullptr if every generator is depleted.
     */
    std::shared_ptr<Generator> firstActiveGenerator();

    /**
     * @brief Per-tick power accounting hook (see @ref tick_flow).
     *
     * Computes total consumer demand, distributes it across sources, handles
     * battery depletion and load shedding, then pushes the final bus voltage
     * to every consumer.
     *
     * @param dt Simulation step in seconds.
     */
    void handlePowerUpdate(float dt) final;

    /**
     * @brief Apply the consumer load to the power sources for one tick.
     *
     * Nominal case: the active battery absorbs @p total_current_a and, if a
     * generator has spare capacity, trickle-charges a battery below full SOC
     * (capped at 10% of generator capacity, see computeChargeCurrentA()).
     * Emergency case (all batteries depleted): the first active generator
     * covers as much consumer demand as it can and reactivateIfPossible() is
     * offered the remaining headroom.
     *
     * @param dt              Simulation step in seconds.
     * @param total_current_a Summed active-consumer demand, in amperes.
     * @param bus_voltage     Bus voltage reference used for W<->A conversion.
     */
    void distributeLoad(float dt, float total_current_a, float bus_voltage);

    /**
     * @brief React to the active battery reaching PowerLevel::DEPLETED.
     *
     * Scans from the current index for the next non-depleted battery and
     * makes it active. If none remains, sets m_all_batteries_depleted and
     * hands consumer demand to the first active generator, shedding
     * priority 4 -> 2 consumers it cannot cover; if there is no generator
     * either, all consumers are cut.
     *
     * @param dt          Simulation step in seconds (currently unused).
     * @param[in,out] bus_voltage Updated to the new source voltage on fallback.
     * @return true if a battery is still supplying the bus, false if the
     *         vessel has fallen back to generator-only (or no) power.
     */
    bool handleDepleted(float dt, float& bus_voltage);

    /**
     * @brief Shed one low-priority consumer when the battery is low.
     *
     * No-op for PowerLevel::NORMAL / DEPLETED, or when a spare battery or an
     * active generator can carry the load. Otherwise deactivates a single
     * consumer per tick: WARN sheds priority 4, CRITICAL sheds priority 3
     * and below, always sparing priority 1.
     *
     * @param level Power level reported by the active battery.
     */
    void shedLoads(PowerLevel level);

    /**
     * @brief Charging current to send from a generator into a battery.
     *
     * Returns 0 if either party is missing/depleted or the battery is full.
     * Otherwise scales the target by the SOC deficit, clamps to 10% of the
     * generator's available power, and converts to amperes at @p safe_voltage.
     *
     * @param gen          Source generator.
     * @param bat          Battery to charge.
     * @param safe_voltage Voltage used for the W->A conversion.
     * @return Charge current in amperes (>= 0).
     */
    float computeChargeCurrentA(
        std::shared_ptr<Generator> gen,
        std::shared_ptr<Battery> bat,
        float safe_voltage) const;

    /**
     * @brief Re-enable previously shed consumers that now fit the budget.
     *
     * Walks consumers from priority 1 to 4 and activates the first inactive
     * one whose nominalPowerW() fits within @p available_w, then returns
     * (one reactivation per call).
     *
     * @param available_w Spare power available on the bus, in watts.
     * @param bus_voltage Voltage used for the W->A conversion.
     */
    void reactivateIfPossible(float available_w, float bus_voltage);
};

}  // namespace lotusim::gazebo
