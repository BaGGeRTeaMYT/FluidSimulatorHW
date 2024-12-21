#if defined(__GNUC__) || defined(__clang__)

#ifndef FixedHEADER
#define FixedHEADER

#include "CustomConcepts.h"

template<int N, int K>
requires (IsPositive<N> && IsPowerOfTwo<N> &&
          IsPositive<K> && IsPowerOfTwo<K> &&
          IsValidFixedParams<N, K>)
class Fixed {
    public:
    
    using ValueType = typename TypeChecker<N>::Type;

    constexpr Fixed(int value) {
        m_value = static_cast<ValueType>(value << k);
    }
    constexpr Fixed(float value): Fixed(static_cast<int>(value * (1 << k))) {}
    constexpr Fixed(double value): Fixed(static_cast<int>(value * (1 << k))) {}
    constexpr Fixed(): Fixed(static_cast<int>(0));

    static constexpr Fixed from_raw(ValueType x) {
        Fixed ret;
        ret.v = x;
        return ret;
    }

    auto operator<=>(const Fixed&) const = default;
    bool operator==(const Fixed&) const = default;
    
    friend class Simulation;

    static constexpr Fixed inf = Fixed::from_raw(std::numeric_limits<ValueType>::max());
    static constexpr Fixed eps = Fixed::from_raw(Simulation<N, K>::deltas.size());
    private:
    ValueType m_value;
};

Fixed operator+(Fixed a, Fixed b) {
    return Fixed::from_raw(a.v + b.v);
}

Fixed operator-(Fixed a, Fixed b) {
    return Fixed::from_raw(a.v - b.v);
}

Fixed operator*(Fixed a, Fixed b) {
    return Fixed::from_raw(((int64_t) a.v * b.v) >> 16);
}

Fixed operator/(Fixed a, Fixed b) {
    return Fixed::from_raw(((int64_t) a.v << 16) / b.v);
}

Fixed &operator+=(Fixed &a, Fixed b) {
    return a = a + b;
}

Fixed &operator-=(Fixed &a, Fixed b) {
    return a = a - b;
}

Fixed &operator*=(Fixed &a, Fixed b) {
    return a = a * b;
}

Fixed &operator/=(Fixed &a, Fixed b) {
    return a = a / b;
}

Fixed operator-(Fixed x) {
    return Fixed::from_raw(-x.v);
}

Fixed abs(Fixed x) {
    if (x.v < 0) {
        x.v = -x.v;
    }
    return x;
}

ostream &operator<<(ostream &out, Fixed x) {
    return out << x.v / (double) (1 << 16);
}

#endif // FixedHEADER

#else

Here to cause compilation error
// use GCC or Clang to compile this code

#endif