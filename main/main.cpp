#include "tests/s18_stm32_link.hpp"

extern "C" void app_main(void)
{
    // S18 isolated wired-link validation. H19 remains available in tests/.
    s18::run();
}