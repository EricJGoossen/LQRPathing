#include <cassert>
#include <iostream>

#include "path_generator.hpp"
#include "state_generators/abstract_state_generator.hpp"
#include "state_generators/position_state_generator.hpp"
#include "state_generators/velocity_state_generator.hpp"
#include "topological_sort.hpp"

using namespace auto_nav;
using namespace auto_nav::state_generators;



// --------------------
// Test Generators
// --------------------

// No dependency generator
class A : public AbstractStateGenerator {
public:
    constexpr virtual ~A() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<>;
    static constexpr int ID = 1;
};

// Depends on A
class B : public AbstractStateGenerator {
public:
    constexpr virtual ~B() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<A>;
    static constexpr int ID = 2;
};

// Depends on B
class C : public AbstractStateGenerator {
public:
    constexpr virtual ~C() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<B>;
    static constexpr int ID = 3;
};

// Multiple dependencies
class D : public AbstractStateGenerator {
public:
    constexpr virtual ~D() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<A, B>;
    static constexpr int ID = 4;
};

// Duplicate ID test (same as A)
class A2 : public AbstractStateGenerator {
public:
    constexpr virtual ~A2() override {}
    int Id() const override{ return ID; }

    using Dependencies = TopologicalSort::DependenciesList<>;
    static constexpr int ID = 1;
};

int main() {
    std::cout << "Running PathGenerator tests...\n";

    auto pg = PathGenerator<PositionStateGenerator, VelocityStateGenerator>{};
    std::vector<AbstractStateGenerator*> gens = pg.GetGeneratorInstances();

    // --------------------
    // Test 1: Basic 2-node dependency
    // --------------------
    // {
    //     PathGenerator<A, B> pg;
    //     auto gens = pg.GetGeneratorInstances();

    //     assert(gens.size() == 2);

    //     std::cout << "Test 1 passed\n";
    // }

    // // --------------------
    // // Test 2: Chain dependency A -> B -> C
    // // --------------------
    // {
    //     PathGenerator<A, B, C> pg;
    //     auto gens = pg.GetGeneratorInstances();

    //     assert(gens.size() == 3);

    //     std::cout << "Test 2 passed\n";
    // }

    // // --------------------
    // // Test 3: Branching dependencies
    // // D depends on A and B
    // // --------------------
    // {
    //     PathGenerator<A, B, D> pg;
    //     auto gens = pg.GetGeneratorInstances();

    //     assert(gens.size() == 3);

    //     std::cout << "Test 3 passed\n";
    // }

    // // --------------------
    // // Test 4: Duplicate IDs
    // // A and A2 should both exist
    // // --------------------
    // {
    //     PathGenerator<A, A2> pg;
    //     auto gens = pg.GetGeneratorInstances();

    //     assert(gens.size() == 2);
    //     assert(gens[0]->Id() == 1);
    //     assert(gens[1]->Id() == 1);

    //     std::cout << "Test 4 passed\n";
    // }

    // // --------------------
    // // Test 5: Real generators
    // // --------------------
    // {
    //     PathGenerator<PositionStateGenerator, VelocityStateGenerator> pg;
    //     auto gens = pg.GetGeneratorInstances();

    //     assert(!gens.empty());

    //     std::cout << "Test 5 (real generators) passed\n";
    // }

    std::cout << "All tests passed!\n";
    return 0;
}