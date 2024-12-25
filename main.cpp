#include <Simulator.hpp>

#define TYPES FLOAT, DOUBLE, FIXED(32, 8), FAST_FIXED(64,12)

#define SIZES S(8, 5)

#ifndef TYPES
#error "TYPES is not defined"
#endif

#ifdef SIZES

using Simulator_ = Simulator<ListOfTypes<TYPES>, SIZES>;

#else

using Simulator_ = Simulator<ListOfTypes<TYPES>>;

#endif

std::string remove_quotes(std::string&& str) {
    if (str[0] == '\"') {
        str.erase(0, 1);
    }
    if (str.back() == '\"') {
        str.pop_back();
    }
    return std::move(str);
}

int main(int argc, char** argv) {

    std::string_view p_type="FLOAT", v_type="FIXED(32, 8)", v_flow_type="FAST_FIXED(64, 12)",
                     save_tick = "-1", input_file = "../input.txt";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        std::cout << "Got argument " << arg << std::endl;
        if (arg.find("--p-type=") == 0) p_type = remove_quotes(arg.substr(9));
        else if (arg.find("--v-type=") == 0) v_type = remove_quotes(arg.substr(9));
        else if (arg.find("--v-flow-type=") == 0) v_flow_type = remove_quotes(arg.substr(14));
        else if (arg.find("--save_tick=") == 0) save_tick = remove_quotes(arg.substr(12)); // useless for now
        else if (arg.find("--input_file=") == 0) input_file = remove_quotes(arg.substr(13));
    }

    if (p_type.size() == 0) {
        throw std::runtime_error("--p-type not specified");
    }
    if (v_type.size() == 0) {
        throw std::runtime_error("--v-type not specified");
    }
    if (v_flow_type.size() == 0) {
        throw std::runtime_error("--v-flow-type not specified");
    }

    const Simulator_ simulator = Simulator_::from_params(SimulationParams{
        .p_type_name             = p_type,
        .velocity_type_name      = v_type,
        .velocity_flow_type_name = v_flow_type,
    });

    // this one is (8, 6) so it should be dynamic
    simulator.start_with_field(FieldContent{
        .rho{},
        .g{},
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
    simulator.start_with_field(FieldContent{
        .rho{},
        .g{},
        .field =
            std::vector<std::vector<char>>{
                {'#', '#', '#', '#', '#'},
                {'#', '#', ' ', '.', '#'},
                {'#', ' ', '.', ' ', '#'},
                {'#', '.', ' ', ' ', '#'},
                {'#', '.', ' ', ' ', '#'},
                {'#', '.', ' ', ' ', '#'},
                {'#', ' ', ' ', ' ', '#'},
                {'#', '#', '#', '#', '#'},
            },
    });

    simulator.start_with_file(std::string(input_file));
}
