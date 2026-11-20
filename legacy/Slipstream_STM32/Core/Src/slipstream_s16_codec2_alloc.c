/* Codec 2 __EMBEDDED__ allocator interface (upstream debug_alloc.h).
 * Startup-only S16 benchmark: bridge its allocation hooks to STM32/newlib.
 * Allocation happens during codec2_create(), outside the timed frames.
 * Do not use heap allocation in interrupt or real-time DSP processing.
 */
#include <stddef.h>
#include <stdlib.h>

void *codec2_malloc(size_t size)
{
    return malloc(size);
}

void *codec2_calloc(size_t nmemb, size_t size)
{
    return calloc(nmemb, size);
}

void codec2_free(void *ptr)
{
    free(ptr);
}
