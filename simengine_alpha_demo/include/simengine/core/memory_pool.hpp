// simengine/core/memory_pool.hpp — Core Engine subsystem.
//
// Fixed-block-size pool allocator. All blocks are the same size, storage
// is a single contiguous buffer allocated once at construction, and
// alloc()/free() are O(1) freelist operations with zero calls into the
// system allocator afterward. This is the allocator every hot-loop
// subsystem (ECS component storage, physics contact pairs, etc.) is
// required to use instead of new/delete once the sim is running.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace simengine::core {

class MemoryPool {
public:
    MemoryPool(std::size_t blockSize, std::size_t blockCount)
        : blockSize_(std::max(blockSize, sizeof(std::uint32_t)))
        , blockCount_(blockCount)
    {
        buffer_ = std::make_unique<std::byte[]>(blockSize_ * blockCount_);
        freeList_.reserve(blockCount_);
        // Freelist initialized back-to-front so index 0 is handed out first.
        for (std::size_t i = blockCount_; i-- > 0;) {
            freeList_.push_back(i);
        }
    }

    MemoryPool(const MemoryPool&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;

    // Returns nullptr on exhaustion rather than throwing or growing —
    // pool exhaustion at runtime is a capacity-planning bug, and the
    // caller (not this allocator) is in the right position to decide
    // whether that's fatal.
    void* alloc() noexcept {
        if (freeList_.empty()) return nullptr;
        const std::size_t idx = freeList_.back();
        freeList_.pop_back();
        ++inUse_;
        return buffer_.get() + idx * blockSize_;
    }

    void free(void* ptr) noexcept {
        if (!ptr) return;
        const auto* base = buffer_.get();
        const auto offset = static_cast<std::byte*>(ptr) - base;
        const auto idx = static_cast<std::size_t>(offset) / blockSize_;
        freeList_.push_back(idx);
        --inUse_;
    }

    std::size_t blockSize() const noexcept { return blockSize_; }
    std::size_t capacity() const noexcept { return blockCount_; }
    std::size_t inUse() const noexcept { return inUse_; }
    std::size_t available() const noexcept { return blockCount_ - inUse_; }

private:
    std::size_t blockSize_;
    std::size_t blockCount_;
    std::size_t inUse_ = 0;
    std::unique_ptr<std::byte[]> buffer_;
    std::vector<std::size_t> freeList_;
};

// Typed convenience wrapper: placement-constructs/destructs T on top of a
// MemoryPool sized for T, so callers get pool-backed allocation with
// normal constructor/destructor semantics.
template <typename T>
class TypedPool {
public:
    explicit TypedPool(std::size_t count) : pool_(sizeof(T), count) {}

    template <typename... Args>
    T* create(Args&&... args) {
        void* mem = pool_.alloc();
        if (!mem) return nullptr;
        return new (mem) T(std::forward<Args>(args)...);
    }

    void destroy(T* obj) {
        if (!obj) return;
        obj->~T();
        pool_.free(obj);
    }

    std::size_t inUse() const noexcept { return pool_.inUse(); }
    std::size_t capacity() const noexcept { return pool_.capacity(); }

private:
    MemoryPool pool_;
};

} // namespace simengine::core
