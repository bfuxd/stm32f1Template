/**
 * 功能: 获取 PSP 的值
 * 参数: 无
 * 返回: PSP 的值
 */
unsigned long __get_PSP(void) __attribute__( ( naked ) );
unsigned long __get_PSP(void)
{
    unsigned long result=0;

    asm volatile ("MRS %0, psp\n\t"
                   "MOV r0, %0 \n\t"
                   "BX  lr     \n\t"  : "=r" (result) );
    return(result);
}

/**
 * 功能: 设定 PSP 的值
 * 参数: PSP 的值
 * 返回: 无
 */
void __set_PSP(unsigned long topOfProcStack) __attribute__( ( naked ) );
void __set_PSP(unsigned long topOfProcStack)
{
    asm volatile ("MSR psp, %0\n\t"
                "BX  lr     \n\t" : : "r" (topOfProcStack) );
}

/**
 * 功能: 获取 MSP 的值
 * 参数: 无
 * 返回: MSP 的值
 */
unsigned long __get_MSP(void) __attribute__( ( naked ) );
unsigned long __get_MSP(void)
{
    unsigned long result=0;

    asm volatile ("MRS %0, msp\n\t"
                "MOV r0, %0 \n\t"
                "BX  lr     \n\t"  : "=r" (result) );
    return(result);
}

/**
 * 功能: 设定 MSP 的值
 * 参数: MSP 的值
 * 返回: 无
 */
void __set_MSP(unsigned long topOfMainStack) __attribute__( ( naked ) );
void __set_MSP(unsigned long topOfMainStack)
{
    asm volatile ("MSR msp, %0\n\t"
                "BX  lr     \n\t" : : "r" (topOfMainStack) );
}

/**
 * 功能: 获取 BASEPRI 的值
 * 参数: 无
 * 返回: BASEPRI 的值
 */
unsigned long __get_BASEPRI(void)
{
    unsigned long result=0;

    asm volatile ("MRS %0, basepri_max" : "=r" (result) );
    return(result);
}

/**
 * 功能: 设定 BASEPRI 的值
 * 参数: 无
 * 返回: BASEPRI 的值
 */
void __set_BASEPRI(unsigned long value)
{
    asm volatile ("MSR basepri, %0" : : "r" (value) );
}

/**
 * 功能: 获取 PRIMASK 的值
 * 参数: 无
 * 返回: PRIMASK 的值
 */
unsigned long __get_PRIMASK(void)
{
    unsigned long result=0;

    asm volatile ("MRS %0, primask" : "=r" (result) );
    return(result);
}

/**
 * 功能: 设定 PRIMASK 的值
 * 参数: 无
 * 返回: PRIMASK 的值
 */
void __set_PRIMASK(unsigned long priMask)
{
    asm volatile ("MSR primask, %0" : : "r" (priMask) );
}

/**
 * 功能: 获取 FAULTMASK 的值
 * 参数: 无
 * 返回: FAULTMASK 的值
 */
unsigned long __get_FAULTMASK(void)
{
    unsigned long result=0;

    asm volatile ("MRS %0, faultmask" : "=r" (result) );
    return(result);
}

/**
 * 功能: 设定 FAULTMASK 的值
 * 参数: 无
 * 返回: FAULTMASK 的值
 */
void __set_FAULTMASK(unsigned long faultMask)
{
    asm volatile ("MSR faultmask, %0" : : "r" (faultMask) );
}

/**
 * 功能: 获取 CONTROL 的值
 * 参数: 无
 * 返回: CONTROL 的值
 */
unsigned long __get_CONTROL(void)
{
    unsigned long result=0;

    asm volatile ("MRS %0, control" : "=r" (result) );
    return(result);
}

/**
 * 功能: 设定 CONTROL 的值
 * 参数: 无
 * 返回: CONTROL 的值
 */
void __set_CONTROL(unsigned long control)
{
    asm volatile ("MSR control, %0" : : "r" (control) );
}

/**
 * 功能: 32 bit 数值字节逆序
 * 参数: 32 bit 无符号数
 * 返回: 字节逆序的数值
 */
unsigned long __REV(unsigned long value)
{
    unsigned long result=0;

    asm volatile ("rev %0, %1" : "=r" (result) : "r" (value) );
    return(result);
}

/**
 * 功能: 16 bit 数值字节逆序
 * 参数: 16 bit 无符号数
 * 返回: 字节逆序的数值
 */
unsigned long __REV16(unsigned short value)
{
    unsigned long result=0;

    asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
    return(result);
}

/**
 * 功能: 16 bit 数值字节逆序, 然后带符号地扩展到 32 bit
 * 参数: 16 bit 有符号数
 * 返回: 32 bit 有符号数
 */
signed long __REVSH(signed short value)
{
    unsigned long result=0;

    asm volatile ("revsh %0, %1" : "=r" (result) : "r" (value) );
    return(result);
}

/**
 * 功能: 32 bit 数值比特逆序
 * 参数: 32 bit 无符号数
 * 返回: 比特逆序的数值
 */
unsigned long __RBIT(unsigned long value)
{
    unsigned long result=0;

    asm volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
    return(result);
}

/**
 * 功能: 加载指定地址的 8 bit 内存到寄存器, 并标记独占访问
 * 参数: 内存指针
 * 返回: 8 bit 数值
 */
unsigned char __LDREXB(unsigned char *addr)
{
    unsigned char result=0;

    asm volatile ("ldrexb %0, [%1]" : "=r" (result) : "r" (addr) );
    return(result);
}

/**
 * 功能: 加载指定地址的 16 bit 内存到寄存器, 并标记独占访问
 * 参数: 内存指针
 * 返回: 16 bit 数值
 */
unsigned short __LDREXH(unsigned short *addr)
{
    unsigned short result=0;

    asm volatile ("ldrexh %0, [%1]" : "=r" (result) : "r" (addr) );
    return(result);
}

/**
 * 功能: 加载指定地址的 32 bit 内存到寄存器, 并标记独占访问
 * 参数: 内存指针
 * 返回: 32 bit 数值
 */
unsigned long __LDREXW(unsigned long *addr)
{
    unsigned long result=0;

    asm volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (addr) );
    return(result);
}

/**
 * 功能: 向指定地址的 8 bit 独占内存写入数据
 * 参数: [in] value 8 bit 数值
 * 参数: [in] 内存指针
 * 返回: 目标地址未被独占则不写入数值返回 1, 目标地址被独占则写入数值返回 0
 */
unsigned long __STREXB(unsigned char value, unsigned char *addr)
{
    unsigned long result=0;

    asm volatile ("strexb %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
    return(result);
}

/**
 * 功能: 向指定地址的 16 bit 独占内存写入数据
 * 参数: [in] value 16 bit 数值
 * 参数: [in] 内存指针
 * 返回: 目标地址未被独占则不写入数值返回 1, 目标地址被独占则写入数值返回 0
 */
unsigned long __STREXH(unsigned short value, unsigned short *addr)
{
    unsigned long result=0;

    asm volatile ("strexh %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
    return(result);
}

/**
 * 功能: 向指定地址的 32 bit 独占内存写入数据
 * 参数: [in] value 32 bit 数值
 * 参数: [in] 内存指针
 * 返回: 目标地址未被独占则不写入数值返回 1, 目标地址被独占则写入数值返回 0
 */
unsigned long __STREXW(unsigned long value, unsigned long *addr)
{
    unsigned long result=0;

    asm volatile ("strex %0, %2, [%1]" : "=r" (result) : "r" (addr), "r" (value) );
    return(result);
}
