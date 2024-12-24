#ifndef SimulatorHEADER
#define SimulatorHEADER

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

#include <Fixed.hpp>

#ifdef FLOAT
#error "FLOAT should not be defined in code"
#endif

#ifdef DOUBLE
#error "DOUBLE should not be defined in code"
#endif

#ifdef FIXED
#error "FIXED should not be defined in code"
#endif

#ifdef FAST_FIXED
#error "FAST_FIXED should not be defined in code"
#endif

#ifdef S
#error "S should not be defined in code"
#endif

#define FLOAT float
#define DOUBLE double
#define FIXED(N, K) Fixed<N, K>
#define FAST_FIXED(N, K) FastFixed<N, K>

struct FieldSize {
    constexpr FieldSize(const size_t n, const size_t m): rows(n), columns(m) {}
    size_t rows{};
    size_t columns{};
};

#define S(N, M) FieldSize(N, M)

struct FieldContent {
    std::vector<std::vector<char>> field;
};

template<class... T>
struct ListOfTypes;

template<size_t Pos, class First, class... Other>
struct RecursiveTypeGetter {
    static_assert(Pos <= sizeof...(Other));
    using type = typename RecursiveTypeGetter<Pos - 1, Other...>::type;
};

template<class First, class... Other>
struct RecursiveTypeGetter<0, First, Other...> {
    using type = First;
};

template<size_t Pos, class LoT>
struct ListUnpacking;

template<size_t Pos, class... Other>
struct ListUnpacking<Pos, ListOfTypes<Other...>> {
    using type = typename RecursiveTypeGetter<Pos, Other...>::type;
};

template<size_t Pos, class... Pack>
using GetTypeFromPackPos = typename RecursiveTypeGetter<Pos, Pack...>::type;

template<size_t Pos, class LoT>
using GetTypeFromListPos = typename ListUnpacking<Pos, LoT>::type;

inline constexpr size_t DynamicExtent = std::numeric_limits<size_t>::max();

template<class PElementType, class VelocityElementType,
         class VelocityFlowElementType,
         size_t Rows = DynamicExtent,
         size_t Columns = DynamicExtent>

class Simulation {

    static_assert(Rows > 0);
    static_assert(Columns > 0);
    static_assert((Rows == DynamicExtent && Columns == DynamicExtent)
                  ^ (Rows != DynamicExtent && Columns != DynamicExtent));

    static constexpr bool UseStaticSize = (Rows != DynamicExtent);
    static constexpr std::array<std::pair<int, int>, 4> deltas{{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}};

    template<class TypeInMatrix>
    using StaticFieldMatrix = typename std::array<std::array<TypeInMatrix, Columns>, Rows>;

    template<class TypeInMatrix>
    using DynamicFieldMatrix = typename std::vector<std::vector<TypeInMatrix>>;

    template<class Type>
    using FieldType = typename std::conditional_t<
                                UseStaticSize,
                                StaticFieldMatrix<Type>,
                                DynamicFieldMatrix<Type>>;

    template<class TypeInMatrix>
    using StaticMatrix = std::array<std::array<TypeInMatrix, Columns>, Rows>;

    template<class TypeInMatrix>
    using DynamicMatrix = std::vector<std::vector<TypeInMatrix>>;

    template<class Type>
    using MatrixType = typename std::conditional_t<
                                UseStaticSize,
                                StaticMatrix<Type>,
                                DynamicMatrix<Type>>;

    template<class Type>
    struct VectorField {
        using Storage = MatrixType<std::array<Type, deltas.size()>>;
        using value_type = Storage::value_type;

        Storage vec;

        VectorField()
            requires(UseStaticSize): vec{} {}

        VectorField(size_t size, const value_type& row)
            requires(!UseStaticSize): vec{size, row} {}

        Type& get(int x, int y, int dx, int dy) {
            size_t i = static_cast<size_t>(std::ranges::find(deltas, std::make_pair(dx, dy)) -
                                                deltas.begin());
            assert(i < deltas.size());
            return vec[x][y][i];
        }

        Type& add(int x, int y, int dx, int dy, const Type& dv) {
            return get(x, y, dx, dy) += dv;
        }
    };
    public:
    using FieldCell = char;
    using Field = FieldType<FieldCell>;
    using PMatrix = MatrixType<PElementType>;
    using VelocityVector = VectorField<VelocityElementType>;
    using VelocityFlowVector = VectorField<VelocityFlowElementType>;
    using SimpleMatrix = MatrixType<int>;
    using size_type = size_t;

    explicit constexpr Simulation(const FieldContent& content) noexcept 
    requires(UseStaticSize) {
        assert(content.field.size() == Rows);
        assert(content.field.front().size() == Columns);

        for (int i = 0; i < content.field.size(); i++) {
            for (int j = 0; j < content.field[i].size(); j++) {
                field[i][j] = content.field[i][j];
            }
        }
    }

    explicit constexpr Simulation(const FieldContent& content)
    requires(!UseStaticSize):
    Simulation(content.field, content.field.size(),
               content.field.front().size()) {}

    [[nodiscard]] inline constexpr size_type get_n() const noexcept {
        return field.size();
    }

    [[nodiscard]] inline constexpr size_type get_m() const noexcept {
        return field.front().size();
    }

    void start() {
        size_type n = get_n();
        size_type m = get_m();
        assert(n > 0);
        assert(m > 0);
        std::cout << "Starting simulation with field size: (" << n << ", " << m << ")\n";
        if constexpr (UseStaticSize) {
            std::cout << "Using static size" << std::endl;
        } else {
            std::cout << "Using dynamic size" << std::endl;
        }
        std::cout << "Field:\n";
        for (auto& i : field) {
            for (auto& j : i) {
                std::cout << j << ' ';
            }
            std::cout << '\n';
        }
        std::cout << "p:\n";
        for (auto& i : p) {
            for (int cnt = 0 ; auto& j : i) {
                std::cout << cnt << ' ';
            }
            std::cout << '\n';
        }
        assert(field.size() == n);
        assert(p.size() == n);
        assert(p_old.size() == n);
        assert(velocity.vec.size() == n);
        assert(velocity_flow.vec.size() == n);
        assert(last_use.size() == n);

        assert(field.front().size() == m);
        assert(p.front().size() == m);
        assert(p_old.front().size() == m);
        assert(velocity.vec.front().size() == m);
        assert(velocity_flow.vec.front().size() == m);
        assert(last_use.front().size() == m);

        assert(field.back().size() == m);
        assert(p.back().size() == m);
        assert(p_old.back().size() == m);
        assert(velocity.vec.back().size() == m);
        assert(velocity_flow.vec.back().size() == m);
        assert(last_use.back().size() == m);
    }

    private:
    explicit constexpr Simulation(const Field& field, size_t rows, size_t columns)
    requires(!UseStaticSize):
        field{field},
        p{rows, typename PMatrix::value_type(columns)},
        p_old{rows, typename PMatrix::value_type(columns)},
        velocity{rows, typename VelocityVector::value_type(columns)},
        velocity_flow{rows, typename VelocityFlowVector::value_type(columns)},
        last_use{rows, typename SimpleMatrix::value_type(columns)} {}

    Field field{};
    std::array<PElementType, 256> rho{};
    PMatrix p{};
    PMatrix p_old{};
    VelocityVector velocity{};
    VelocityFlowVector velocity_flow{};
    SimpleMatrix last_use{};
    int UT = 0;
};

template<class PElementType,
         class VelocityElementType,
         class VelocityFlowElementType,
         size_t Rows = DynamicExtent,
         size_t Columns = DynamicExtent>
void StartSimulation(const FieldContent& content) {
    using SimulationType = Simulation<PElementType, VelocityElementType,
                                      VelocityFlowElementType, Rows, Columns>;
    SimulationType{content}.start();
}

template <class PElementType,
          class VelocityElementType,
          class VelocityFlowElementType,
          FieldSize StaticSize,
          FieldSize... StaticSizes>
void RecursiveSimulationStart(const FieldContent& content) {
    static_assert(StaticSize.rows > 0);
    static_assert(StaticSize.columns > 0);

    if (StaticSize.rows == content.field.size() && StaticSize.columns == content.field.front().size()) {
        StartSimulation<PElementType, VelocityElementType, VelocityFlowElementType,
                         StaticSize.rows, StaticSize.columns>(content);
    } else if constexpr (sizeof...(StaticSizes) == 0) {
        StartSimulation<PElementType, VelocityElementType, VelocityFlowElementType>(content);
    } else {
        RecursiveSimulationStart<PElementType, VelocityElementType,
                                              VelocityFlowElementType, StaticSizes...>(content);
    }
}

template<class PElementType,
         class VelocityElementType,
         class VelocityFlowElementType,
         FieldSize... StaticSizes>
void SimulationSizeSelect(const FieldContent& content) {
    if constexpr (sizeof...(StaticSizes) > 0) {
        RecursiveSimulationStart<PElementType,
                                 VelocityElementType,
                                 VelocityFlowElementType,
                                 StaticSizes...>(content);
    } else {
        StartSimulation<PElementType, VelocityElementType, VelocityFlowElementType>(content);
    }
}

#define STRINGIFY_EXACT_NO_EVAL(expr) #expr

inline constexpr std::string_view FloatTypeName   = STRINGIFY_EXACT_NO_EVAL(FLOAT);
inline constexpr std::string_view DoubleTypeName  = STRINGIFY_EXACT_NO_EVAL(DOUBLE);
inline constexpr std::string_view FastFixedPrefix = "FAST_FIXED(";
inline constexpr std::string_view FastFixedSuffix = ")";
inline constexpr std::string_view FixedPrefix     = "FIXED(";
inline constexpr std::string_view FixedSuffix     = ")";

#undef STRINGIFY_EXACT_NO_EVAL

template<class AllowedTypesList, class SelectedTypesList, FieldSize... StaticSizes>
class TypesSelector;

template<class... AllowedTypes, class... SelectedTypes, FieldSize... StaticSizes>
class TypesSelector<ListOfTypes<AllowedTypes...>,
                    ListOfTypes<SelectedTypes...>,
                    StaticSizes...> {
    using FieldVec = std::vector<std::vector<char>>;

    static constexpr bool CanUseDefaultFloatType = std::disjunction_v<std::is_same<AllowedTypes, FLOAT>...>;

    static constexpr bool CanUseDefaultDoubleType = std::disjunction_v<std::is_same<AllowedTypes, DOUBLE>...>;

    public:
    template<class... Args>
        requires (std::conjunction_v<std::is_same<std::string_view, Args>...>)
    static void RecursivelySelectType(const FieldContent& content,
                           std::string_view type_name,
                           Args... type_names) {
        if (CanUseDefaultFloatType && type_name == FloatTypeName) {
            ProcessOtherTypes<FLOAT, Args...>(content, type_names...);
            return;
        }
        if (CanUseDefaultDoubleType && type_name == DoubleTypeName) {
            ProcessOtherTypes<DOUBLE, Args...>(content, type_names...);
            return;
        }

        size_t n{};
        size_t k{};
        
        if (CheckForFixed(type_name, FastFixedPrefix, FastFixedSuffix, n, k)) {
            if (SelectFixedSizeForSimulation<true, Args...>(n, k, content, type_names...)) {
                return;
            }
        }

        if (CheckForFixed(type_name, FixedPrefix, FixedSuffix, n, k)) {
            if (SelectFixedSizeForSimulation<false, Args...>(n, k, content, type_names...)) {
                return;
            }
        }

        throw std::invalid_argument("Unknown type: " + std::string{type_name});
    }

private:

    template<class Type, class... Args>
    static void ProcessOtherTypes(const FieldContent& content, const Args&... type_names) {
        if constexpr (sizeof...(type_names) > 0) {
            using AllowedTypesList = ListOfTypes<AllowedTypes...>;
            using SelectedTypesList = ListOfTypes<SelectedTypes..., Type>;

            TypesSelector<AllowedTypesList, SelectedTypesList, StaticSizes...>::RecursivelySelectType(content, type_names...);
        } else {
            SimulationSizeSelect<SelectedTypes..., Type, StaticSizes...>(content);
        }
    }

    static bool CheckForFixed(std::string_view type_name, std::string_view prefix,
                              std::string_view suffix, size_t &n, size_t &k) {
        if (!type_name.starts_with(prefix)) {
            return false;
        }
        type_name.remove_prefix(prefix.size());

        if (!type_name.ends_with(suffix)) {
            return false;
        }
        type_name.remove_suffix(suffix.size());

        const std::size_t sep_char_pos = type_name.find(',');
        if (sep_char_pos >= type_name.size()) {
            return false;
        }

        auto strip_sv = [](std::string_view s) noexcept {
            while (!s.empty() && std::isspace(s.front())) {
                s.remove_prefix(1);
            }
            while (!s.empty() && std::isspace(s.back())) {
                s.remove_suffix(1);
            }
            return s;
        };


        const std::string_view n_str = strip_sv(type_name.substr(0, sep_char_pos));
        const std::string_view k_str = strip_sv(type_name.substr(sep_char_pos + 1));

        try {
            n = std::stoull(std::string(n_str));
        } catch (...) {
            return false;
        }

        try {
            k = std::stoull(std::string(k_str));
        } catch (...) {
            return false;
        }
        
        return n > 0 && k > 0;
    }

    template<bool Fast, class... Args>
    static bool SelectFixedSizeForSimulation(size_t n, size_t k,
                                             const FieldContent& content,
                                             Args... type_names) {
        return RecursiveFixedSimulationStart<0, Fast, Args...>(n, k, content, type_names...);
    }

    template<size_t Pos, bool Fast, class... Args>
    static bool RecursiveFixedSimulationStart(size_t n, size_t k,
                                              const FieldContent& content,
                                              Args... type_names) {
        using FloatType = GetTypeFromPackPos<Pos, AllowedTypes...>;
        
        if constexpr (requires {
            {FloatType::N_value == size_t {}} -> std::same_as<bool>;
            {FloatType::K_value == size_t {}} -> std::same_as<bool>;
            {FloatType::is_fast == bool {}} -> std::same_as<bool>;
        }) {
            if constexpr (FloatType::is_fast == Fast) {
                if (FloatType::N_value == n && FloatType::K_value == k) {
                    ProcessOtherTypes<FloatType, Args...>(content, type_names...);
                    return true;
                }
            }
        }

        if constexpr (Pos + 1 < sizeof...(AllowedTypes)) {
            return RecursiveFixedSimulationStart<Pos + 1, Fast, Args...>(n, k, content, type_names...);
        }

        return false;
    }
};
    
struct SimulationParams {
    std::string_view p_type_name{};
    std::string_view velocity_type_name{};
    std::string_view velocity_flow_type_name{};
};

template<class LoT, FieldSize... StaticSizes>
class Simulator;

template<class... Types, FieldSize... StaticSizes>
class Simulator<ListOfTypes<Types...>, StaticSizes...> {
    public:

    static Simulator from_params(SimulationParams params) {
        return Simulator{std::move(params)};
    }

    Simulator(SimulationParams params): params{std::move(params)} {}

    void start_with_field(const FieldContent& content) const {
        assert(!content.field.empty());
        assert(!content.field.front().empty());
        TypesSelector<ListOfTypes<Types...>, ListOfTypes<>, StaticSizes...>::
        RecursivelySelectType(content, params.p_type_name, params.velocity_type_name, params.velocity_flow_type_name);
    }

    private:
    SimulationParams params; 
};

#endif // SimulatorHEADER