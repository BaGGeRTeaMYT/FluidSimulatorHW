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
    constexpr Fixed() : v(0) {}

    static constexpr Fixed from_raw(value_type x) {
        Fixed ret;
        ret.v = x;
        return ret;
    }
    auto operator<=>(const Fixed&) const = default;
    bool operator==(const Fixed&) const  = default;

    inline value_type get_value() const {
        return v;
    }
};


template <size_t N, size_t K>
using FastFixed = Fixed<N, K, true>;

template<size_t N, size_t K>
Fixed<N, K> operator+(Fixed<N, K>& a, Fixed<N, K>& b) {
    return Fixed<N, K>::from_raw(a.get_value() + b.get_value());
}

template<size_t N, size_t K>
Fixed<N, K> operator-(Fixed<N, K>& a, Fixed<N, K>& b) {
    return Fixed<N, K>::from_raw(a.get_value() - b.get_value());
}

template<size_t N, size_t K>
Fixed<N, K> operator*(Fixed<N, K>& a, Fixed<N, K>& b) {
    return Fixed<N, K>::from_raw(a.get_value() * b.get_value());
}

template<size_t N, size_t K>
Fixed<N, K> operator/(Fixed<N, K>& a, Fixed<N, K>& b) {
    return Fixed<N, K>::from_raw(a.get_value() / b.get_value());
}

template<class A, class B>
struct NewSize;

template<size_t N1, size_t K1, size_t N2, size_t K2>
struct NewSize<Fixed<N1, K1>, Fixed<N2, K2>> {
    using value_type = std::conditional_t<
        (N1 > N2) || (N1 == N2 && K1 <= K2),
        Fixed<N1, K1>,
        Fixed<N2, K2>
    >;
};

template<size_t N1, size_t K1, size_t N2, size_t K2>
auto operator+(Fixed<N1, K1>& a, Fixed<N2, K2>& b) {
    using type = typename NewSize<Fixed<N1, K1>, Fixed<N2, K2>>::value_type;
    return type(a) + type(b);
}

template<size_t N1, size_t K1, size_t N2, size_t K2>
auto operator-(Fixed<N1, K1>& a, Fixed<N2, K2>& b) {
    using type = typename NewSize<Fixed<N1, K1>, Fixed<N2, K2>>::value_type;
    return type(a) - type(b);
}

template<size_t N1, size_t K1, size_t N2, size_t K2>
auto operator*(Fixed<N1, K1>& a, Fixed<N2, K2>& b) {
    using type = typename NewSize<Fixed<N1, K1>, Fixed<N2, K2>>::value_type;
    return type(a) * type(b);
}

template<size_t N1, size_t K1, size_t N2, size_t K2>
auto operator/(Fixed<N1, K1>& a, Fixed<N2, K2>& b) {
    using type = typename NewSize<Fixed<N1, K1>, Fixed<N2, K2>>::value_type;
    return type(a) / type(b);
}

#endif // FixedHEADER