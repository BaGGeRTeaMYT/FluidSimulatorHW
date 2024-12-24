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
#include <random>

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
#define FAST_FIXED(N, K) Fixed<N, K, true>

#define AmountOfTicks 500

struct FieldSize {
    constexpr FieldSize(size_t n, size_t m): rows(n), columns(m) {}
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

    std::tuple<VelocityElementType, bool, std::pair<int, int>> propagate_flow(int x, int y, VelocityElementType lim) {
        last_use[x][y] = UT - 1;
        VelocityElementType ret = 0;
        for (auto [dx, dy] : deltas) {
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] != '#' && last_use[nx][ny] < UT) {
                auto cap = velocity.get(x, y, dx, dy);
                auto flow = velocity_flow.get(x, y, dx, dy);
                if (flow == cap) {
                    continue;
                }
                // assert(v >= velocity_flow.get(x, y, dx, dy));
                auto vp = std::min(lim, VelocityElementType(cap) - VelocityElementType(flow));
                if (last_use[nx][ny] == UT - 1) {
                    velocity_flow.add(x, y, dx, dy, VelocityFlowElementType(vp));
                    last_use[x][y] = UT;
                    // cerr << x << " " << y << " -> " << nx << " " << ny << " " << vp << " / " << lim << "\n";
                    return {vp, 1, {nx, ny}};
                }
                auto [t, prop, end] = propagate_flow(nx, ny, vp);
                ret += t;
                if (prop) {
                    velocity_flow.add(x, y, dx, dy, VelocityFlowElementType(t));
                    last_use[x][y] = UT;
                    // cerr << x << " " << y << " -> " << nx << " " << ny << " " << t << " / " << lim << "\n";
                    return {t, prop && end != std::pair<int, int>{x, y}, end};
                }
            }
        }
        last_use[x][y] = UT;
        return {ret, 0, {0, 0}};
    }

    VelocityElementType random01() {
        return VelocityElementType(static_cast<double>(rnd() & ((1 << 16) - 1)) / (1 << 16));
    }

    void propagate_stop(int x, int y, bool force = false) {
        if (!force) {
            bool stop = true;
            for (auto [dx, dy] : deltas) {
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 && velocity.get(x, y, dx, dy) > 0) {
                    stop = false;
                    break;
                }
            }
            if (!stop) {
                return;
            }
        }
        last_use[x][y] = UT;
        for (auto [dx, dy] : deltas) {
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] == '#' || last_use[nx][ny] == UT || velocity.get(x, y, dx, dy) > 0) {
                continue;
            }
            propagate_stop(nx, ny);
        }
    }

    VelocityElementType move_prob(int x, int y) {
        VelocityElementType sum = 0;
        for (size_t i = 0; i < deltas.size(); ++i) {
            auto [dx, dy] = deltas[i];
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] == '#' || last_use[nx][ny] == UT) {
                continue;
            }
            auto v = velocity.get(x, y, dx, dy);
            if (v < 0) {
                continue;
            }
            sum += v;
        }
        return sum;
    }

    struct ParticleParams {
        char type;
        PElementType cur_p;
        std::array<VelocityElementType, deltas.size()> v;

        void swap_with(Simulation* simulation, int x, int y) {
            std::swap(simulation->field[x][y], type);
            std::swap(simulation->p[x][y], cur_p);
            std::swap(simulation->velocity.vec[x][y], v);
        }
    };

    bool propagate_move(int x, int y, bool is_first) {
        last_use[x][y] = UT - is_first;
        bool ret = false;
        int nx = -1, ny = -1;
        do {
            std::array<VelocityElementType, deltas.size()> tres;
            VelocityElementType sum = 0;
            for (size_t i = 0; i < deltas.size(); ++i) {
                auto [dx, dy] = deltas[i];
                int nx = x + dx, ny = y + dy;
                if (field[nx][ny] == '#' || last_use[nx][ny] == UT) {
                    tres[i] = sum;
                    continue;
                }
                auto v = velocity.get(x, y, dx, dy);
                if (v < 0) {
                    tres[i] = sum;
                    continue;
                }
                sum += v;
                tres[i] = sum;
            }

            if (sum == 0) {
                break;
            }

            VelocityElementType p = random01() * sum;
            size_t d = std::ranges::upper_bound(tres, p) - tres.begin();

            auto [dx, dy] = deltas[d];
            nx = x + dx;
            ny = y + dy;
            assert(velocity.get(x, y, dx, dy) > 0 && field[nx][ny] != '#' && last_use[nx][ny] < UT);

            ret = (last_use[nx][ny] == UT - 1 || propagate_move(nx, ny, false));
        } while (!ret);

        last_use[x][y] = UT;
        for (size_t i = 0; i < deltas.size(); ++i) {
            auto [dx, dy] = deltas[i];
            int nx = x + dx, ny = y + dy;
            if (field[nx][ny] != '#' && last_use[nx][ny] < UT - 1 && velocity.get(x, y, dx, dy) < 0) {
                propagate_stop(nx, ny);
            }
        }
        if (ret) {
            if (!is_first) {
                ParticleParams pp{};
                pp.swap_with(this, x, y);
                pp.swap_with(this, nx, ny);
                pp.swap_with(this, x, y);
            }
        }
        return ret;
    }

    void run_simulation() {
        rho[' '] = 0.01;
        rho['.'] = 1000;
        PElementType g = 0.1;

        for (size_t x = 0; x < get_n(); ++x) {
            for (size_t y = 0; y < get_m(); ++y) {
                if (field[x][y] == '#')
                    continue;
                for (auto [dx, dy] : deltas) {
                    dirs[x][y] += (field[x + dx][y + dy] != '#');
                }
            }
        }

        for (size_t i = 0; i < AmountOfTicks; ++i) {
        
            PElementType total_delta_p = 0;
            // Apply external forces
            for (size_t x = 0; x < get_n(); ++x) {
                for (size_t y = 0; y < get_m(); ++y) {
                    if (field[x][y] == '#')
                        continue;
                    if (field[x + 1][y] != '#')
                        velocity.add(x, y, 1, 0, VelocityElementType(g));
                }
            }

            // Apply forces from p
            old_p = p;
            for (size_t x = 0; x < get_n(); ++x) {
                for (size_t y = 0; y < get_m(); ++y) {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas) {
                        int nx = x + dx, ny = y + dy;
                        if (field[nx][ny] != '#' && old_p[nx][ny] < old_p[x][y]) {
                            auto delta_p = old_p[x][y] - old_p[nx][ny];
                            auto force = delta_p;
                            auto &contr = velocity.get(nx, ny, -dx, -dy);
                            if (PElementType(contr) * rho[static_cast<int>(field[nx][ny])] >= force) {
                                contr -= VelocityElementType(force / rho[(int) field[nx][ny]]);
                                continue;
                            }
                            force -= PElementType(contr) * rho[(int) field[nx][ny]];
                            contr = 0;
                            velocity.add(x, y, dx, dy, VelocityElementType(force / rho[(int) field[x][y]]));
                            p[x][y] -= force / dirs[x][y];
                            total_delta_p -= force / dirs[x][y];
                        }
                    }
                }
            }

            // Make flow from velocities
            velocity_flow.vec = {};
            bool prop = false;
            do {
                UT += 2;
                prop = 0;
                for (size_t x = 0; x < get_n(); ++x) {
                    for (size_t y = 0; y < get_m(); ++y) {
                        if (field[x][y] != '#' && last_use[x][y] != UT) {
                            auto [t, local_prop, _] = propagate_flow(x, y, 1);
                            if (t > 0) {
                                prop = 1;
                            }
                        }
                    }
                }
            } while (prop);

            // Recalculate p with kinetic energy
            for (size_t x = 0; x < get_n(); ++x) {
                for (size_t y = 0; y < get_m(); ++y) {
                    if (field[x][y] == '#')
                        continue;
                    for (auto [dx, dy] : deltas) {
                        auto old_v = velocity.get(x, y, dx, dy);
                        auto new_v = velocity_flow.get(x, y, dx, dy);
                        if (old_v > VelocityElementType(0)) {
                            assert(new_v <= old_v);
                            velocity.get(x, y, dx, dy) = VelocityElementType(new_v);
                            auto force = PElementType((PElementType(old_v) - PElementType(new_v)) * rho[static_cast<int>(field[x][y])]);
                            if (field[x][y] == '.')
                                force *= 0.8;
                            if (field[x + dx][y + dy] == '#') {
                                p[x][y] += force / dirs[x][y];
                                total_delta_p += force / dirs[x][y];
                            } else {
                                p[x + dx][y + dy] += force / dirs[x + dx][y + dy];
                                total_delta_p += force / dirs[x + dx][y + dy];
                            }
                        }
                    }
                }
            }

            UT += 2;
            prop = false;
            for (size_t x = 0; x < get_n(); ++x) {
                for (size_t y = 0; y < get_m(); ++y) {
                    if (field[x][y] != '#' && last_use[x][y] != UT) {
                        if (random01() < move_prob(x, y)) {
                            prop = true;
                            propagate_move(x, y, true);
                        } else {
                            propagate_stop(x, y, true);
                        }
                    }
                }
            }

            if (prop) {
                std::cout << "Tick " << i << ":\n";
                for (size_t x = 0; x < get_n(); ++x) {
                    for (size_t y = 0; y < get_m(); ++y) {
                        std::cout << field[x][y] << " ";
                    }
                    std::cout << std::endl;
                }
            }
        }
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
        assert(field.size() == n);
        assert(p.size() == n);
        assert(old_p.size() == n);
        assert(velocity.vec.size() == n);
        assert(velocity_flow.vec.size() == n);
        assert(last_use.size() == n);
        assert(dirs.size() == n);

        assert(field.front().size() == m);
        assert(p.front().size() == m);
        assert(old_p.front().size() == m);
        assert(velocity.vec.front().size() == m);
        assert(velocity_flow.vec.front().size() == m);
        assert(last_use.front().size() == m);
        assert(dirs.front().size() == m);

        assert(field.back().size() == m);
        assert(p.back().size() == m);
        assert(old_p.back().size() == m);
        assert(velocity.vec.back().size() == m);
        assert(velocity_flow.vec.back().size() == m);
        assert(last_use.back().size() == m);
        assert(dirs.back().size() == m);

        run_simulation();

    }

    private:
    explicit constexpr Simulation(const Field& field, size_t rows, size_t columns)
    requires(!UseStaticSize):
        field{field},
        p{rows, typename PMatrix::value_type(columns)},
        old_p{rows, typename PMatrix::value_type(columns)},
        velocity{rows, typename VelocityVector::value_type(columns)},
        velocity_flow{rows, typename VelocityFlowVector::value_type(columns)},
        last_use{rows, typename SimpleMatrix::value_type(columns)},
        dirs{rows, typename SimpleMatrix::value_type(columns)} {}

    Field field{};
    std::array<PElementType, 256> rho{};
    PMatrix p{};
    PMatrix old_p{};
    VelocityVector velocity{};
    VelocityFlowVector velocity_flow{};
    SimpleMatrix last_use{};
    SimpleMatrix dirs{};
    std::mt19937 rnd{1337};
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