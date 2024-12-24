#ifndef FixedHEADER
#define FixedHEADER

#include <cstddef>
#include <cstdint>
#include <iostream>

template <size_t N, size_t K, bool Fast = false>
struct Fixed {
    public:
    static constexpr size_t N_value = N;
    static constexpr size_t K_value = K;
    static constexpr bool is_fast   = Fast;

    private:

    using value_type = std::conditional_t<
        is_fast,
        std::conditional_t<
            N_value == 8,
            std::uint_fast8_t,
            std::conditional_t<
                N_value == 16,
                std::uint_fast16_t,
                std::conditional_t<
                    N_value == 32,
                    std::uint_fast32_t,
                    std::conditional_t<
                        N_value == 64,
                        std::uint_fast64_t,
                        bool
                    >
                >
            >
        >,
        std::conditional_t<
            N_value == 8,
            std::uint8_t,
            std::conditional_t<
                N_value == 16,
                std::uint16_t,
                std::conditional_t<
                    N_value == 32,
                    std::uint32_t,
                    std::conditional_t<
                        N_value == 64,
                        std::uint64_t,
                        bool
                    >
                >
            >
        >
    >;
    static_assert(!std::same_as<value_type, bool>, "Incorrect value of N in Fixed type.");

    value_type v;
    public:
    constexpr Fixed(int v) : v(static_cast<value_type>(v) << K_value) {}
    constexpr Fixed(float f) : v(static_cast<value_type>(f * (1 << K_value))) {}
    constexpr Fixed(double f) : v(static_cast<value_type>(f * (1 << K_value))) {}
    template<size_t N1, size_t K1, bool Fast1>
    constexpr Fixed(const Fixed<N1, K1, Fast1>& other) : Fixed(double(other)) {}
    constexpr Fixed() : v(0) {}

    static constexpr Fixed from_raw(value_type x) {
        Fixed ret;
        ret.v = x;
        return ret;
    }

    inline value_type get_value() const {
        return v;
    }

    constexpr Fixed operator+(const Fixed& other) const {
        return Fixed::from_raw(v + other.v);
    }

    constexpr Fixed& operator+=(const Fixed& other) {
        v += other.v;
        return *this;
    }

    constexpr Fixed operator-(const Fixed& other) const {
        return Fixed::from_raw(v - other.v);
    }

    constexpr Fixed& operator-=(const Fixed& other) {
        v -= other.v;
        return *this;
    }

    constexpr Fixed operator*(const Fixed& other) const {
        return Fixed::from_raw((static_cast<double>(v) * static_cast<double>(other.v)) / (1 << K_value));
    }

    constexpr Fixed& operator*=(const Fixed& other) {
        v = (static_cast<double>(v) * static_cast<double>(other.v)) / (1 << K_value);
        return *this;
    }

    constexpr Fixed operator/(const Fixed& other) const {
        return Fixed::from_raw((static_cast<double>(v) / static_cast<double>(other.v)) * (1 << K_value));
    }

    constexpr Fixed& operator/=(const Fixed& other) {
        v = (static_cast<double>(v) / static_cast<double>(other.v)) * (1 << K_value);
        return *this;
    }

    explicit operator int() const {
        return static_cast<int>(v >> K_value);
    }

    explicit operator float() const {
        return static_cast<float>(v) / (1 << K_value);
    }

    explicit operator double() const {
        return static_cast<double>(v) / (1 << K_value);
    }

    template <size_t OtherN, size_t OtherK, bool OtherFast>
    bool operator==(const Fixed<OtherN, OtherK, OtherFast>& other) const {
        return static_cast<double>(v) / (1 << K_value) == static_cast<double>(other.get_value()) / (1 << OtherK);
    }

    template <size_t OtherN, size_t OtherK, bool OtherFast>
    bool operator!=(const Fixed<OtherN, OtherK, OtherFast>& other) const {
        return !(*this == other);
    }

    template <size_t OtherN, size_t OtherK, bool OtherFast>
    bool operator<(const Fixed<OtherN, OtherK, OtherFast>& other) const {
        return static_cast<double>(v) / (1 << K_value) < static_cast<double>(other.get_value()) / (1 << OtherK);
    }

    template <size_t OtherN, size_t OtherK, bool OtherFast>
    bool operator<=(const Fixed<OtherN, OtherK, OtherFast>& other) const {
        return static_cast<double>(v) / (1 << K_value) <= static_cast<double>(other.get_value()) / (1 << OtherK);
    }

    template <size_t OtherN, size_t OtherK, bool OtherFast>
    bool operator>(const Fixed<OtherN, OtherK, OtherFast>& other) const {
        return static_cast<double>(v) / (1 << K_value) > static_cast<double>(other.get_value()) / (1 << OtherK);
    }

    template <size_t OtherN, size_t OtherK, bool OtherFast>
    bool operator>=(const Fixed<OtherN, OtherK, OtherFast>& other) const {
        return static_cast<double>(v) / (1 << K_value) >= static_cast<double>(other.get_value()) / (1 << OtherK);
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    bool operator==(T other) const {
        return T(*this) == other;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    bool operator>=(T other) const {
        return T(*this) >= other;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    bool operator<=(T other) const {
        return T(*this) <= other;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    bool operator>(T other) const {
        return T(*this) > other;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    bool operator<(T other) const {
        return T(*this) < other;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    friend bool operator<= (T scalar, const Fixed& fixed) {
        return (scalar <= T(fixed));
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    friend Fixed operator+(T scalar, const Fixed& fixed) {
        return fixed + scalar;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    friend Fixed operator-(T scalar, const Fixed& fixed) {
        return Fixed::from_raw(static_cast<Fixed::value_type>(scalar << Fixed::K_value) - fixed.get_value());
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    friend Fixed operator*(T scalar, const Fixed& fixed) {
        return fixed * scalar;
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    friend Fixed operator/(T scalar, const Fixed& fixed) {
        return Fixed::from_raw(static_cast<Fixed::value_type>(scalar << Fixed::K_value) / fixed.get_value());
    }

    template<typename T>
    requires(std::same_as<T, int> || std::same_as<T, float> || std::same_as<T, double>)
    friend float operator-(T scalar, const Fixed& fixed) {
        return T(fixed) - scalar;
    }
};

template <size_t N, size_t K, bool Fast>
using FastFixed = Fixed<N, K, true>;

#endif // FixedHEADER