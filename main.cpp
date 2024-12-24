#include "_Simulator.hpp"

#define TYPES FLOAT, DOUBLE, FIXED(32,5), FAST_FIXED(64,12)

#define SIZES S(4, 5), \
        S(32, 69),     \
        S(100, 100), \
        S(8, 5)

#ifndef TYPES
#error "TYPES is not defined"
#endif

#ifdef SIZES

using Simulator_ = Simulator<ListOfTypes<TYPES>, SIZES>;

#else

using Simulator_ = Simulator<ListOfTypes<TYPES>>;

#endif

int main() {
    const Simulator_ simulator = Simulator_::from_params(SimulationParams{
        .p_type_name             = "FLOAT",
        .velocity_type_name      = "DOUBLE",
        .velocity_flow_type_name = "DOUBLE",
    });

    simulator.start_with_field(FieldContent{
        .field =
            std::vector<std::vector<char>>{
                {'#', '#', '#', '#', '#'},
                {'#', '#', '.', '.', '#'},
                {'#', ' ', ' ', ' ', '#'},
                {'#', '.', ' ', ' ', '#'},
                {'#', '.', ' ', ' ', '#'},
                {'#', '.', ' ', ' ', '#'},
                {'#', ' ', ' ', ' ', '#'},
                {'#', '#', '#', '#', '#'},
            },
    });
}
