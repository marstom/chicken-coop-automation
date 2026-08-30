#pragma once

// Pure logic - no Arduino/FreeRTOS includes, so it is natively testable.

#include <cstring>

namespace logic
{
    /// strncpy that always null-terminates and truncates safely.
    /// (Plain strncpy does NOT terminate when src fills the buffer - that bug
    /// existed in WebMessage::setContent.)
    inline void copyBounded(char *dst, size_t dstSize, const char *src)
    {
        if (dst == nullptr || dstSize == 0)
        {
            return;
        }
        if (src == nullptr)
        {
            dst[0] = '\0';
            return;
        }
        strncpy(dst, src, dstSize - 1);
        dst[dstSize - 1] = '\0';
    }
}
