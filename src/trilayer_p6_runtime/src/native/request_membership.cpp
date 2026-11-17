#include <cstddef>
#include <cstdint>

// Current input and fresh table on every call. No state, cache, or threading.
extern "C" void request_membership_u16(const std::uint16_t* labels,
                                      const std::uint8_t* table,
                                      std::uint8_t* output, std::size_t size) {
    for (std::size_t i = 0; i < size; ++i) output[i] = table[labels[i]];
}
