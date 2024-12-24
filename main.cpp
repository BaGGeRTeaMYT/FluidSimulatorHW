#include <Simulator.hpp>

#define TYPES FLOAT, DOUBLE, FIXED(32, 8), FAST_FIXED(64,12)

#define SIZES S(8, 6)

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
        .velocity_type_name      = "FAST_FIXED(64, 12)",
        .velocity_flow_type_name = "FIXED(32, 8)",
    });

    // this one is (8, 6) so it should be dynamic
    simulator.start_with_field(FieldContent{
        .field =
            std::vector<std::vector<char>>{
                {'#', '#', '#', '#', '#', '#'},
                {'#', '#', '.', ' ', '.', '#'},
                {'#', ' ', ' ', '.', ' ', '#'},
                {'#', '.', ' ', '.', ' ', '#'},
                {'#', ' ', '.', ' ', '#', '#'},
                {'#', '.', ' ', ' ', ' ', '#'},
                {'#', ' ', ' ', ' ', ' ', '#'},
                {'#', '#', '#', '#', '#', '#'},
            },
    });

    // this one is (8, 5) so it should be static
    // simulator.start_with_field(FieldContent{
    //     .field =
    //         std::vector<std::vector<char>>{
    //             {'#', '#', '#', '#', '#'},
    //             {'#', '#', ' ', '.', '#'},
    //             {'#', ' ', '.', ' ', '#'},
    //             {'#', '.', ' ', ' ', '#'},
    //             {'#', '.', ' ', ' ', '#'},
    //             {'#', '.', ' ', ' ', '#'},
    //             {'#', ' ', ' ', ' ', '#'},
    //             {'#', '#', '#', '#', '#'},
    //         },
    // });
}
