// AI module tests: utility curves and scores, brain switching, sight cone and line of sight, hearing, memory, nav route following.
#include "ai/sim/Perception.h"
#include "ai/sim/Route.h"
#include "ai/sim/Utility.h"
#include "ai/sim/Debug.h"

#include "mealkit/core/Hash.h"

#include <cstdio>

static int g_failed = 0;
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            std::printf("%s(%d): FAIL %s\n", __FILE__, __LINE__, #x);                                                  \
            ++g_failed;                                                                                                \
        }                                                                                                              \
    } while (0)

using namespace ai;
using mealkit::Vec3;

namespace {

void testUtility() {
    CHECK(normalize(50, 0, 100) == 500 && normalize(-5, 0, 100) == 0 && normalize(500, 0, 100) == 1000 && normalize(5, 9, 9) == 0);
    CHECK(response(Curve::Linear, 300) == 300 && response(Curve::Quadratic, 500) == 250 && response(Curve::Inverse, 300) == 700);
    CHECK(response(Curve::InverseQuadratic, 500) == 250 && response(Curve::Step, 499, 500) == 0 && response(Curve::Step, 500, 500) == 1000);
    CHECK(response(Curve::Linear, 5000) == 1000 && response(Curve::Linear, -5) == 0);

    // Product: any zero rules the option out; compensation keeps many considerations comparable
    CHECK(Score().mul(500).mul(0).done(2) == 0);
    CHECK(Score().mul(500).done(1) == 500);
    const i32 two = Score().mul(500).mul(500).done(2);
    CHECK(two > 250 && two < 500); // 250 before compensation
    CHECK(Score().mul(1000).mul(1000).done(2) == 1000);

    // The first offered option wins a tie
    Chooser c(0);
    c.offer(3, 400);
    c.offer(5, 400);
    c.offer(7, 100);
    CHECK(c.best == 3 && c.bestScore == 400 && c.currentScore == 0);
    CHECK(Chooser(9).best == Chooser::kNone);
}

void testBrain() {
    Tuning t;
    t.thinkTicks = 10;
    t.minStayTicks = 30;
    t.switchMargin = 100;
    Brain b;
    enter(b, 1, 0);
    CHECK(b.state == 1 && b.previous == 0 && stateTicks(b, 12) == 12);

    auto decide = [&](u32 tick, i32 sCur, i32 sOther) {
        Chooser c(b.state);
        c.offer(1, b.state == 1 ? sCur : sOther);
        c.offer(2, b.state == 2 ? sCur : sOther);
        return think(b, c, tick, t);
    };
    CHECK(!decide(0, 300, 900));  // too early (this also spends the think timer)
    CHECK(!thinkReady(b, 9) && thinkReady(b, 10));
    CHECK(!decide(10, 300, 900)); // 10 ticks in the state < minStay 30
    CHECK(!decide(20, 300, 900));
    CHECK(decide(30, 300, 900) && b.state == 2 && b.previous == 1 && b.enteredTick == 30);
    CHECK(!decide(40, 900, 1000)); // minStay not over again (10 ticks)
    CHECK(!decide(50, 900, 1000));
    CHECK(decide(60, 900, 1000) && b.state == 1); // 1000 >= 900 + margin 100
}

void testBrainMargin() {
    Tuning t;
    t.thinkTicks = 1;
    t.minStayTicks = 0;
    t.switchMargin = 100;
    Brain b;
    enter(b, 1, 0);
    Chooser c(1);
    c.offer(1, 500);
    c.offer(2, 599);
    CHECK(!think(b, c, 5, t) && b.state == 1);
    Chooser d(1);
    d.offer(1, 500);
    d.offer(2, 600);
    CHECK(think(b, d, 6, t) && b.state == 2);
}

void testCone() {
    Eye e;
    e.position = {0.0f, 1.0f, 0.0f};
    e.forward = {0.0f, 0.0f, 1.0f};
    e.range = 10.0f;
    e.cosHalfFov = 0.5f; // 60° half angle
    CHECK(inCone(e, {0.0f, 1.0f, 5.0f}));          // straight ahead
    CHECK(inCone(e, {4.0f, 1.0f, 5.0f}));          // 38.7° off
    CHECK(inCone(e, {6.0f, 1.0f, 5.0f}));          // 50.2° off
    CHECK(!inCone(e, {8.0f, 1.0f, 2.0f}));         // 76° off
    CHECK(!inCone(e, {0.0f, 1.0f, -5.0f}));        // behind
    CHECK(!inCone(e, {0.0f, 1.0f, 11.0f}));        // out of range
    CHECK(inCone(e, {0.0f, 1.0f, 0.0f}));          // at the eye
    e.cosHalfFov = -1.0f;
    CHECK(inCone(e, {0.0f, 1.0f, -5.0f}));         // all around
    e.cosHalfFov = -0.5f;                           // 120° half angle
    CHECK(inCone(e, {5.0f, 1.0f, -1.0f}) && !inCone(e, {0.0f, 1.0f, -5.0f}));
}

void testSightAndHearing() {
    mealkit::physics::PhysicsWorld world;
    CHECK(world.init(64, 1));
    mealkit::physics::BodyDesc d;
    d.motion = mealkit::physics::Motion::Static;
    d.position = {0.0f, 1.0f, 5.0f};
    world.createBox({3.0f, 2.0f, 0.25f}, d); // a wall across z = 5, x -3..3
    world.optimize();

    Tuning t;
    t.forgetTicks = 100;
    t.noiseTicks = 30;
    Eye e;
    e.position = {0.0f, 1.0f, 0.0f};
    e.forward = {0.0f, 0.0f, 1.0f};
    e.range = 30.0f;
    e.cosHalfFov = 0.0f;

    CHECK(lineClear(world, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 3.0f}, mealkit::physics::kInvalidBody));
    CHECK(!lineClear(world, {0.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 8.0f}, mealkit::physics::kInvalidBody));
    CHECK(lineClear(world, {8.0f, 1.0f, 0.0f}, {8.0f, 1.0f, 8.0f}, mealkit::physics::kInvalidBody)); // past the wall's end

    Candidate cs[3];
    cs[0] = {1, {0.0f, 1.0f, 8.0f}};   // behind the wall
    cs[1] = {2, {10.0f, 1.0f, 12.0f}}; // clear
    cs[2] = {3, {-10.0f, 1.0f, 12.0f}}; // clear, same distance as 2: the first (2) wins
    Awareness a;
    CHECK(see(a, e, world, 0, mealkit::physics::kInvalidBody, cs, 3, 100, t));
    CHECK(a.visible && a.target == 2 && a.knownTick == 100 && a.lastKnown.x == 10.0f);

    // The AI's own candidate id is skipped
    Awareness own;
    CHECK(!see(own, e, world, 2, mealkit::physics::kInvalidBody, &cs[1], 1, 100, t) && !hasTarget(own));

    // Out of sight: the memory stays for forgetTicks
    cs[1].position = {0.0f, 1.0f, -9.0f};
    cs[2].position = {0.0f, 1.0f, -9.0f};
    CHECK(!see(a, e, world, 0, mealkit::physics::kInvalidBody, cs, 3, 150, t));
    CHECK(!a.visible && a.target == 2 && a.lastKnown.x == 10.0f && hasTarget(a));
    CHECK(!see(a, e, world, 0, mealkit::physics::kInvalidBody, cs, 3, 200, t) && !hasTarget(a)); // 100 ticks later

    // Hearing: a ring of noises, the nearest one in reach that is recent and not the listener's own
    NoiseList list;
    emit(list, {10.0f, 0.0f, 0.0f}, 8.0f, 4, 100);
    emit(list, {3.0f, 0.0f, 0.0f}, 5.0f, 5, 100);
    Noise n;
    CHECK(hear(list, {0.0f, 0.0f, 0.0f}, 1.0f, 9, 110, t, n) && n.source == 5);   // nearest within reach
    CHECK(hear(list, {0.0f, 0.0f, 0.0f}, 1.0f, 5, 110, t, n) == false);            // own noise skipped, the far one is 10 > 8
    CHECK(hear(list, {0.0f, 0.0f, 0.0f}, 1.5f, 5, 110, t, n) && n.source == 4);    // better ears reach 12
    CHECK(!hear(list, {0.0f, 0.0f, 0.0f}, 1.0f, 9, 131, t, n));                    // too old
    for (u32 i = 0; i < kMaxNoises + 2; ++i) emit(list, {0.0f, 0.0f, 100.0f}, 1.0f, 6, 200); // overwrites the old ones
    CHECK(!hear(list, {0.0f, 0.0f, 0.0f}, 1.0f, 9, 205, t, n));

    Awareness b;
    NoiseList nl;
    emit(nl, {1.0f, 0.0f, 2.0f}, 3.0f, 7, 50);
    Noise first;
    CHECK(hear(nl, {1.0f, 0.0f, 2.0f}, 1.0f, 0, 50, t, first));
    notice(b, first, 60);
    CHECK(b.target == 7 && b.knownTick == 60 && b.lastKnown.z == 2.0f);
    b.visible = true;
    emit(nl, {9.0f, 0.0f, 9.0f}, 3.0f, 8, 61);
    Noise second;
    CHECK(hear(nl, {9.0f, 0.0f, 9.0f}, 1.0f, 0, 61, t, second));
    notice(b, second, 61);
    CHECK(b.target == 7); // sight is not overwritten by a noise
}

void testRoute() {
    mealkit::physics::PhysicsWorld world;
    CHECK(world.init(64, 1));
    mealkit::physics::BodyDesc d;
    d.motion = mealkit::physics::Motion::Static;
    d.position = {0.0f, -0.5f, 0.0f};
    world.createBox({10.0f, 0.5f, 10.0f}, d);          // floor, top at y = 0
    d.position = {0.0f, 1.0f, 0.0f};
    world.createBox({0.25f, 1.0f, 6.0f}, d);           // a wall along z from -6 to 6 at x = 0
    world.optimize();
    mealkit::sim::NavBakeSettings s;
    const mealkit::sim::NavGrid grid = mealkit::sim::bakeNavGrid(world, {-10.0f, -1.0f, -10.0f}, {10.0f, 3.0f, 10.0f}, s);
    CHECK(!grid.empty());

    mealkit::sim::NavSearch search;
    Route r;
    FollowParams p;
    p.speed = 0.7f;
    Vec3 feet{-5.0f, 0.0f, 0.0f};
    setTarget(r, {5.0f, 0.0f, 0.0f});
    u32 budget = 0;
    Vec3 w = follow(r, grid, search, feet, budget, p);
    CHECK(r.repath && !hasPath(r));                   // no budget: no search yet, heads straight for the target
    CHECK(w.x > 0.69f && w.x < 0.71f);
    budget = 1;
    w = follow(r, grid, search, feet, budget, p);
    CHECK(budget == 0 && !r.repath && hasPath(r));
    CHECK(r.count >= 2 && w.z != 0.0f); // goes around the wall (its end is at z = 6), not through it

    // Walk it: each tick move along the returned direction, reach the target within a bounded number of steps
    u32 steps = 0;
    mealkit::Vec3 pos = feet;
    for (; steps < 2000; ++steps) {
        u32 b = 1;
        const Vec3 v = follow(r, grid, search, pos, b, p);
        if (mealkit::lengthSq(v) == 0.0f) break;
        pos = pos + v * 0.1f;
    }
    CHECK(steps < 2000);
    const f32 dx = pos.x - 5.0f, dz = pos.z;
    CHECK(dx * dx + dz * dz < 4.0f);

    // Same input, same output (hash of the stored route)
    Route r2;
    setTarget(r2, {5.0f, 0.0f, 0.0f});
    u32 b2 = 1;
    follow(r2, grid, search, feet, b2, p);
    Route r3;
    setTarget(r3, {5.0f, 0.0f, 0.0f});
    u32 b3 = 1;
    follow(r3, grid, search, feet, b3, p);
    CHECK(r2.count == r3.count && r2.points[0].x == r3.points[0].x && r2.points[0].z == r3.points[0].z);
}

} // namespace

int main() {
    testUtility();
    testBrain();
    testBrainMargin();
    testCone();
    testSightAndHearing();
    testRoute();
    if (g_failed == 0) std::printf("ai_tests: all passed\n");
    return g_failed == 0 ? 0 : 1;
}
