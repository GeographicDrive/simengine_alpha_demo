// simengine/core/ecs.hpp — Core Engine subsystem.
//
// Sparse-set Entity Component System.
//
// Design notes:
//  - Entities are generational handles (index + generation) so a stale
//    handle to a destroyed-and-recycled entity is detected, not silently
//    aliased onto unrelated data — a correctness requirement once
//    Networking/Replay start referencing entities across frames.
//  - Component storage is a sparse-set: a dense, contiguous array of
//    component data (cache-friendly iteration for systems), plus a
//    sparse index array mapping entity index -> dense slot (O(1)
//    add/remove/lookup, no tombstones, no fragmentation over time).
//  - Callers are expected to reserve() storages up front per the
//    "no allocation in the hot loop" invariant; add() beyond reserved
//    capacity still works correctly but will reallocate.

#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <typeindex>
#include <memory>
#include <functional>
#include <limits>

namespace simengine::core {

struct Entity {
    std::uint32_t index = std::numeric_limits<std::uint32_t>::max();
    std::uint32_t generation = 0;

    bool operator==(const Entity& o) const noexcept { return index == o.index && generation == o.generation; }
    bool operator!=(const Entity& o) const noexcept { return !(*this == o); }
    bool valid() const noexcept { return index != std::numeric_limits<std::uint32_t>::max(); }
};

inline constexpr Entity kNullEntity{};

class EntityManager {
public:
    explicit EntityManager(std::size_t reserveCount = 4096) {
        generations_.reserve(reserveCount);
        freeIndices_.reserve(reserveCount);
    }

    Entity create() {
        if (!freeIndices_.empty()) {
            const std::uint32_t idx = freeIndices_.back();
            freeIndices_.pop_back();
            return Entity{idx, generations_[idx]};
        }
        const std::uint32_t idx = static_cast<std::uint32_t>(generations_.size());
        generations_.push_back(0);
        return Entity{idx, 0};
    }

    void destroy(Entity e) {
        if (!isAlive(e)) return;
        ++generations_[e.index];
        freeIndices_.push_back(e.index);
    }

    bool isAlive(Entity e) const noexcept {
        return e.valid()
            && e.index < generations_.size()
            && generations_[e.index] == e.generation;
    }

    std::size_t aliveCount() const noexcept {
        return generations_.size() - freeIndices_.size();
    }

private:
    std::vector<std::uint32_t> generations_;
    std::vector<std::uint32_t> freeIndices_;
};

// --- Sparse-set component storage for a single component type T ---
template <typename T>
class ComponentStorage {
public:
    explicit ComponentStorage(std::size_t reserveCount = 4096) {
        dense_.reserve(reserveCount);
        denseEntities_.reserve(reserveCount);
    }

    T& add(Entity e, T value = T{}) {
        if (has(e)) {
            dense_[sparse_[e.index]] = std::move(value);
            return dense_[sparse_[e.index]];
        }
        if (e.index >= sparse_.size()) sparse_.resize(e.index + 1, kInvalid);
        sparse_[e.index] = static_cast<std::uint32_t>(dense_.size());
        dense_.push_back(std::move(value));
        denseEntities_.push_back(e);
        return dense_.back();
    }

    void remove(Entity e) {
        if (!has(e)) return;
        const std::uint32_t denseIdx = sparse_[e.index];
        const std::uint32_t lastIdx = static_cast<std::uint32_t>(dense_.size() - 1);
        const Entity lastEntity = denseEntities_[lastIdx];

        // Swap-and-pop keeps the dense array contiguous in O(1).
        dense_[denseIdx] = std::move(dense_[lastIdx]);
        denseEntities_[denseIdx] = lastEntity;
        sparse_[lastEntity.index] = denseIdx;

        dense_.pop_back();
        denseEntities_.pop_back();
        sparse_[e.index] = kInvalid;
    }

    bool has(Entity e) const noexcept {
        return e.index < sparse_.size() && sparse_[e.index] != kInvalid;
    }

    T* get(Entity e) noexcept {
        if (!has(e)) return nullptr;
        return &dense_[sparse_[e.index]];
    }
    const T* get(Entity e) const noexcept {
        if (!has(e)) return nullptr;
        return &dense_[sparse_[e.index]];
    }

    // Direct access to the dense arrays for cache-friendly batch iteration
    // in systems (e.g. "for i in range(size()): integrate(dense[i])").
    std::vector<T>& dense() noexcept { return dense_; }
    const std::vector<T>& dense() const noexcept { return dense_; }
    const std::vector<Entity>& entities() const noexcept { return denseEntities_; }
    std::size_t size() const noexcept { return dense_.size(); }
    void reserve(std::size_t n) { dense_.reserve(n); denseEntities_.reserve(n); }

private:
    static constexpr std::uint32_t kInvalid = std::numeric_limits<std::uint32_t>::max();

    std::vector<std::uint32_t> sparse_;      // entity.index -> dense slot
    std::vector<T> dense_;                   // tightly packed component data
    std::vector<Entity> denseEntities_;      // dense slot -> owning entity
};

// --- World: owns the EntityManager and type-erased component storages ---
class World {
public:
    Entity createEntity() { return entities_.create(); }

    void destroyEntity(Entity e) {
        if (!entities_.isAlive(e)) return;
        for (auto& [type, eraser] : erasers_) eraser(e);
        entities_.destroy(e);
    }

    bool isAlive(Entity e) const noexcept { return entities_.isAlive(e); }
    std::size_t aliveCount() const noexcept { return entities_.aliveCount(); }

    template <typename T>
    ComponentStorage<T>& storage() {
        const auto key = std::type_index(typeid(T));
        auto it = holders_.find(key);
        if (it == holders_.end()) {
            auto holder = std::make_unique<Holder<T>>();
            auto* raw = &holder->value;
            erasers_[key] = [raw](Entity e) { raw->remove(e); };
            it = holders_.emplace(key, std::move(holder)).first;
        }
        return static_cast<Holder<T>*>(it->second.get())->value;
    }

    template <typename T>
    T& addComponent(Entity e, T value = T{}) { return storage<T>().add(e, std::move(value)); }

    template <typename T>
    T* getComponent(Entity e) { return storage<T>().get(e); }

    template <typename T>
    void removeComponent(Entity e) { storage<T>().remove(e); }

private:
    struct HolderBase { virtual ~HolderBase() = default; };
    template <typename T> struct Holder : HolderBase { ComponentStorage<T> value; };

    EntityManager entities_;
    std::unordered_map<std::type_index, std::unique_ptr<HolderBase>> holders_;
    std::unordered_map<std::type_index, std::function<void(Entity)>> erasers_;
};

} // namespace simengine::core
