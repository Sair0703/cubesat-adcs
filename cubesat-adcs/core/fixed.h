// fixed.h - Q16.16 fixed-point number with saturating arithmetic.
//
// The Cortex-M3 has no floating-point unit, so the flight controller does all
// of its math in fixed point: 16 integer bits, 16 fractional bits (resolution
// about 0.000015). Every operation saturates instead of wrapping, so an
// overflow clips to the largest value rather than flipping sign.
#pragma once
#include <stdint.h>

struct Fx {
    int32_t raw;

    static constexpr int kFracBits = 16;
    static constexpr int32_t kOne = 1 << kFracBits;

    static constexpr Fx fromRaw(int32_t r) { return Fx{r}; }
    static constexpr Fx fromInt(int32_t v) { return Fx{sat(static_cast<int64_t>(v) * kOne)}; }
    static constexpr Fx fromFloat(float v) {
        return Fx{sat(static_cast<int64_t>(v * kOne + (v >= 0 ? 0.5f : -0.5f)))};
    }
    static constexpr Fx max() { return Fx{INT32_MAX}; }
    static constexpr Fx min() { return Fx{INT32_MIN}; }

    float toFloat() const { return static_cast<float>(raw) / kOne; }

    static constexpr int32_t sat(int64_t v) {
        return v > INT32_MAX ? INT32_MAX : (v < INT32_MIN ? INT32_MIN : static_cast<int32_t>(v));
    }

    constexpr Fx operator+(Fx o) const { return Fx{sat(static_cast<int64_t>(raw) + o.raw)}; }
    constexpr Fx operator-(Fx o) const { return Fx{sat(static_cast<int64_t>(raw) - o.raw)}; }
    constexpr Fx operator-() const { return Fx{sat(-static_cast<int64_t>(raw))}; }
    constexpr Fx operator*(Fx o) const {
        // 64-bit product, rounded, shifted back to Q16.16.
        return Fx{sat((static_cast<int64_t>(raw) * o.raw + (1 << (kFracBits - 1))) >> kFracBits)};
    }
    constexpr Fx operator/(Fx o) const {
        if (o.raw == 0) return raw >= 0 ? max() : min();
        return Fx{sat((static_cast<int64_t>(raw) << kFracBits) / o.raw)};
    }
    Fx& operator+=(Fx o) { return *this = *this + o; }
    Fx& operator-=(Fx o) { return *this = *this - o; }

    constexpr bool operator<(Fx o) const { return raw < o.raw; }
    constexpr bool operator>(Fx o) const { return raw > o.raw; }
    constexpr bool operator<=(Fx o) const { return raw <= o.raw; }
    constexpr bool operator>=(Fx o) const { return raw >= o.raw; }
    constexpr bool operator==(Fx o) const { return raw == o.raw; }
    constexpr bool operator!=(Fx o) const { return raw != o.raw; }
};

constexpr Fx fxAbs(Fx v) { return v.raw < 0 ? -v : v; }
constexpr Fx fxClamp(Fx v, Fx lo, Fx hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Wrap an angle in degrees to [-180, 180).
constexpr Fx wrapDeg(Fx a) {
    const int32_t full = 360 * Fx::kOne, half = 180 * Fx::kOne;
    int32_t r = a.raw % full;
    if (r >= half) r -= full;
    if (r < -half) r += full;
    return Fx{r};
}
