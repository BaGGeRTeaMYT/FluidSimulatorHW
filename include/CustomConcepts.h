#include <concepts>
#include "TypeChecker.h"

template<int N>
concept IsPositive = (N > 0);

template<int N>
concept IsPowerOfTwo = (N & (N - 1) == 0);

template<int N, int K>
concept IsValidFixedParams = (N >= K && HasBitType<N>);