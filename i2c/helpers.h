#include <stdio.h>
#include <stdint.h>

static unsigned int readl(unsigned int address)
{
    return *(volatile unsigned int *)address;
}

static void writel(unsigned int data, unsigned int address)
{
    *(volatile unsigned int *)address = data;
}

static void frwritel(unsigned int data, unsigned int address)
{
    *(volatile unsigned int *)address |= data;
}

static unsigned char readb(unsigned int address)
{
    return *(volatile unsigned char *)address;
}

static void writeb(unsigned char data, unsigned int address)
{
    *(volatile unsigned char *)address = data;
}

static void frwriteb(unsigned char data, unsigned int address)
{
    *(volatile unsigned char *)address |= data;
}
