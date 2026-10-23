// libc_min.cpp - The two C library routines the compiler may call implicitly.
// There is no C library on this target, so we provide them ourselves.
// Byte-wise with volatile pointers so the compiler can't turn the loops back
// into calls to memset/memcpy (which would recurse forever).
#include <stddef.h>

extern "C" void* memset(void* dst, int value, size_t n) {
    volatile unsigned char* d = static_cast<volatile unsigned char*>(dst);
    while (n--) *d++ = static_cast<unsigned char>(value);
    return dst;
}

extern "C" void* memcpy(void* dst, const void* src, size_t n) {
    volatile unsigned char* d = static_cast<volatile unsigned char*>(dst);
    const volatile unsigned char* s = static_cast<const volatile unsigned char*>(src);
    while (n--) *d++ = *s++;
    return dst;
}
