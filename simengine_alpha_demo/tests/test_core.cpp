#include "simengine/core/memory_pool.hpp"
#include "simengine/core/ecs.hpp"
#include "simengine/core/scheduler.hpp"
#include "simengine/core/job_system.hpp"
#include "simengine/math/vector3.hpp"

#include <cstdio>
#include <cmath>
#include <atomic>

using namespace simengine::core;
using namespace simengine::math;

static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { ++g_failures; std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__); } \
} while (0)

static void test_memory_pool() {
    std::printf("test_memory_pool\n");
    MemoryPool pool(sizeof(double), 4);
    CHECK(pool.capacity() == 4);
    void* a = pool.alloc();
    void* b = pool.alloc();
    CHECK(a != nullptr && b != nullptr && a != b);
    CHECK(pool.inUse() == 2);
    pool.free(a);
    CHECK(pool.inUse() == 1);
    void* c = pool.alloc();
    void* d = pool.alloc();
    CHECK(pool.inUse() == 3); // b, c, d occupy 3 of 4 slots (a was freed)
    void* e = pool.alloc(); // takes the 4th and final slot
    CHECK(e != nullptr);
    void* f = pool.alloc(); // pool now genuinely full -> must fail cleanly
    CHECK(f == nullptr);
    pool.free(b); pool.free(c); pool.free(d); pool.free(e);
    CHECK(pool.inUse() == 0);

    struct Vec3 { double x, y, z; Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {} };
    TypedPool<Vec3> typed(8);
    Vec3* v = typed.create(1.0, 2.0, 3.0);
    CHECK(v != nullptr && v->x == 1.0 && v->y == 2.0 && v->z == 3.0);
    typed.destroy(v);
    CHECK(typed.inUse() == 0);
}

static void test_ecs_generational_handles() {
    std::printf("test_ecs_generational_handles\n");
    EntityManager mgr;
    Entity e1 = mgr.create();
    CHECK(mgr.isAlive(e1));
    mgr.destroy(e1);
    CHECK(!mgr.isAlive(e1)); // stale handle must be detected

    Entity e2 = mgr.create(); // recycles e1's index
    CHECK(e2.index == e1.index);
    CHECK(e2.generation != e1.generation); // but with a bumped generation
    CHECK(mgr.isAlive(e2));
    CHECK(!mgr.isAlive(e1)); // old handle to the recycled slot stays dead
}

static void test_ecs_sparse_set() {
    std::printf("test_ecs_sparse_set\n");
    struct Position { double x = 0, y = 0, z = 0; };
    struct Velocity { double x = 0, y = 0, z = 0; };

    World world;
    std::vector<Entity> entities;
    for (int i = 0; i < 100; ++i) {
        Entity e = world.createEntity();
        world.addComponent<Position>(e, Position{double(i), 0, 0});
        if (i % 2 == 0) world.addComponent<Velocity>(e, Velocity{1, 0, 0});
        entities.push_back(e);
    }

    CHECK(world.storage<Position>().size() == 100);
    CHECK(world.storage<Velocity>().size() == 50);

    // Remove a middle entity and confirm swap-and-pop kept storage dense
    // and correctly re-linked (the classic sparse-set bug to get wrong).
    world.destroyEntity(entities[10]);
    CHECK(world.storage<Position>().size() == 99);
    CHECK(!world.isAlive(entities[10]));
    CHECK(world.getComponent<Position>(entities[99])->x == 99.0); // untouched entity unaffected

    // Every remaining Position must still resolve to the correct entity's data.
    bool allCorrect = true;
    for (int i = 0; i < 100; ++i) {
        if (i == 10) continue;
        Position* p = world.getComponent<Position>(entities[i]);
        if (!p || p->x != double(i)) { allCorrect = false; break; }
    }
    CHECK(allCorrect);
}

static void test_scheduler_determinism_and_cap() {
    std::printf("test_scheduler_determinism_and_cap\n");
    FixedTimestepScheduler sched(100.0, 4); // 100Hz, cap 4 steps/frame
    int stepsRun = 0;
    sched.addSystem([&](double dt) {
        ++stepsRun;
        CHECK(std::abs(dt - 0.01) < 1e-12); // fixed dt must be exact every call
    });

    // Exactly 0.10s of frame time at 100Hz (0.01s/step) should run exactly 10 steps.
    double t = 0.0;
    for (int i = 0; i < 10; ++i) t += sched.advance(0.01);
    CHECK(sched.stepCount() == 10);

    // A huge frame delta (simulated stall) must be capped, not spiral.
    FixedTimestepScheduler sched2(100.0, 4);
    int steps2 = 0;
    sched2.addSystem([&](double) { ++steps2; });
    sched2.advance(10.0); // 10 seconds of backlog at 100Hz would be 1000 steps
    CHECK(steps2 <= 4); // capped, and must not hang or crash
}

static void test_job_system_parallel_for() {
    std::printf("test_job_system_parallel_for\n");
    JobSystem jobs(4);
    const std::size_t N = 10000;
    std::vector<double> data(N, 1.0);

    jobs.parallelFor(N, 100, [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) data[i] *= 2.0;
    });

    bool allDoubled = true;
    for (double v : data) if (v != 2.0) { allDoubled = false; break; }
    CHECK(allDoubled);

    // Concurrent submit + waitAll correctness under contention.
    std::atomic<int> counter{0};
    for (int i = 0; i < 1000; ++i) jobs.submit([&counter] { counter.fetch_add(1, std::memory_order_relaxed); });
    jobs.waitAll();
    CHECK(counter.load() == 1000);
}

static void test_integrated_simulation() {
    std::printf("test_integrated_simulation (ECS + scheduler + job system)\n");
    struct Position { Vector3d p; };
    struct Velocity { Vector3d v; };

    World world;
    for (int i = 0; i < 500; ++i) {
        Entity e = world.createEntity();
        world.addComponent<Position>(e, Position{Vector3d(0, 0, 0)});
        world.addComponent<Velocity>(e, Velocity{Vector3d(1, 0, 0)}); // 1 m/s along X
    }

    JobSystem jobs(4);
    FixedTimestepScheduler sched(60.0, 8);

    sched.addSystem([&](double dt) {
        auto& positions = world.storage<Position>().dense();
        auto& velocities = world.storage<Velocity>().dense();
        jobs.parallelFor(positions.size(), 32, [&](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                positions[i].p += velocities[i].v * dt;
            }
        });
    });

    // Simulate exactly 1.0 second of sim time.
    double acc = 0.0;
    for (int frame = 0; frame < 60; ++frame) acc += sched.advance(1.0 / 60.0);
    CHECK(sched.stepCount() == 60);

    // After 1 second at 1 m/s, every entity should be at x ~= 1.0.
    auto& positions = world.storage<Position>().dense();
    bool allCorrect = true;
    for (auto& pos : positions) {
        if (std::abs(pos.p.x - 1.0) > 1e-9) { allCorrect = false; break; }
    }
    CHECK(allCorrect);
}

int main() {
    test_memory_pool();
    test_ecs_generational_handles();
    test_ecs_sparse_set();
    test_scheduler_determinism_and_cap();
    test_job_system_parallel_for();
    test_integrated_simulation();

    std::printf("\n%d/%d checks passed\n", g_checks - g_failures, g_checks);
    if (g_failures > 0) { std::printf("RESULT: FAIL (%d failures)\n", g_failures); return 1; }
    std::printf("RESULT: PASS\n");
    return 0;
}
