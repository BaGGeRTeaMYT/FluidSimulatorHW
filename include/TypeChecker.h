#ifndef TypeCheckerHEADER
#define TypeCheckerHEADER

#include <type_traits>
#include <cstdint>

template<int N>
struct HasBitType {
    static constexpr bool value = false;
};

template<>
struct HasBitType<8> {
    static constexpr bool value = true;
    using Type = std::uint8_t;
};

template<>
struct HasBitType<16> {
    static constexpr bool value = true;
    using Type = std::uint16_t;
};

template<>
struct HasBitType<32> {
    static constexpr bool value = true;
    using Type = std::uint32_t;
};

template<>
struct HasBitType<64> {
    static constexpr bool value = true;
    using Type = std::uint64_t;
};

template<>
struct HasBitType<128> {
    static constexpr bool value = true;
    using Type = __int128_t;
};

template<>
struct HasBitType<256> {
    static constexpr bool value = true;
    using Type = __int256_t;
};

template<>
struct HasBitType<512> {
    static constexpr bool value = true;
    using Type = __int512_t;
};

#endif // TypeCheckerHEADER