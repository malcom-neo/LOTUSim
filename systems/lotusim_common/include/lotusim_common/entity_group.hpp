/*
 * Copyright (c) 2025 Naval Group
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 */
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace lotusim::common {

/**
 * @brief EntityGraph class to manage groups of linked entities.
 *
 */
class EntityGraph {
public:
    void addPair(uint64_t entity1, uint64_t entity2);

    bool areLinked(uint64_t entity1, uint64_t entity2);

    std::vector<uint64_t> getGroup(uint64_t entity);

    std::vector<std::vector<uint64_t>> getAllSets();

    std::size_t getSetCount();

    void clearGraph();

private:
    uint64_t find(uint64_t entity);

    std::unordered_map<uint64_t, uint64_t> m_parent;
};

}  // namespace lotusim::common