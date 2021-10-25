unsigned long ENTER_CRITICAL(void) __attribute__((naked));
unsigned long ENTER_CRITICAL(void)
{
    unsigned long ret;
    asm volatile(
        " mrs %0, PRIMASK  \n"
        " cpsid i          \n"
        " isb              \n"
        " bx  lr           \n"
        : "=r"(ret)
        :
        : );
    return ret;
}

void EXIT_CRITICAL(unsigned long primask) __attribute__((naked));
void EXIT_CRITICAL(unsigned long primask)
{
    asm volatile(
        " msr PRIMASK, %0  \n"
        " isb              \n"
        " bx  lr           \n"
        :
        : "r"(primask)
        : );
}
