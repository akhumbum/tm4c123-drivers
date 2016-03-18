#include <stdint.h>

static inline uint32_t readl(uint32_t address)
{
    return *(volatile uint32_t *)address;
}

static inline void writel(uint32_t data, uint32_t address)
{
    *(volatile uint32_t *)address = data;
}

static inline void frwritel(uint32_t data, uint32_t address)
{
    *(volatile uint32_t *)address |= data;
}

static inline uint8_t readb(uint32_t address)
{
    return *(volatile uint8_t *)address;
}

static inline void writeb(uint8_t data, uint32_t address)
{
    *(volatile uint8_t *)address = data;
}

static inline void frwriteb(uint8_t data, uint32_t address)
{
    *(volatile uint8_t *)address |= data;
}
