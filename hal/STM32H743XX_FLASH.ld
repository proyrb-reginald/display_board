/* 程序入口点 - Reset_Handler函数地址 */
ENTRY(Reset_Handler)

/* 定义存储器区域 
 * xrw 表示可执行(eXecute)、可读(Read)、可写(Write)
 * ITCMRAM: 64KB 紧耦合内存，位于0x00000000
 * FLASH: 2048KB 程序存储区，位于0x08000000
 * DTCMRAM: 128KB 数据紧耦合内存，位于0x20000000
 * RAM: 512KB 主系统内存，位于0x24000000
 * RAM_D2: 288KB 域2内存，位于0x30000000
 * RAM_D3: 64KB 域3内存，位于0x38000000
 */
MEMORY
{
  ITCMRAM(xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  FLASH(rx)    : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM(xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  RAM(xrw)     : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2(xrw)  : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3(xrw)  : ORIGIN = 0x38000000, LENGTH = 64K
}

/* 内核堆栈的最高地址 */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);  /* RAM末尾作为初始栈顶 */

/* 如果堆和栈无法放入RAM则产生链接错误 */
_Min_Heap_Size = 0x0;     /* 所需堆大小 */
_Min_Stack_Size = 0x400;  /* 所需栈大小 */

/* 定义输出段 */
SECTIONS
{
  /* 启动代码首先进入FLASH */
  .isr_vector : ALIGN(4)  /* 地址对齐到4字节边界 */
  {
    KEEP(*(.isr_vector))  /* 启动代码，KEEP确保不会被优化掉 */
    . = ALIGN(4);         /* 再次对齐 */
  } > FLASH

  /* 程序代码和其他数据进入FLASH */
  .text : ALIGN(4)  /* 地址对齐到4字节边界 */
  {
    *(.text)      /* .text段（代码） */
    *(.text*)     /* .text*段（代码） */
    *(.glue_7)    /* ARM到Thumb代码粘连 */
    *(.glue_7t)   /* Thumb到ARM代码粘连 */
    *(.eh_frame)  /* 异常处理帧信息 */

    KEEP(*(.init))            /* 初始化代码 */
    KEEP(*(.fini))            /* 终止代码 */

    . = ALIGN(4);              /* 对齐 */
    _etext = .;                /* 定义全局符号_etext表示代码结束位置 */
  } > FLASH

  /* 常量数据进入FLASH */
  .rodata : ALIGN(4)  /* 地址对齐到4字节边界 */
  {
    *(.rodata)    /* .rodata段（常量、字符串等） */
    *(.rodata*)   /* .rodata*段（常量、字符串等） */
    . = ALIGN(4); /* 对齐 */
  } > FLASH

  /* ARM异常展开表 */
  .ARM.extab(READONLY) : ALIGN(4) /* "READONLY"关键字仅在GCC11及以上版本支持，在GCC10及更早版本中请移除 */
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  /* ARM异常索引表 */
  .ARM(READONLY) : ALIGN(4) /* "READONLY"关键字仅在GCC11及以上版本支持，在GCC10及更早版本中请移除 */
  {
    __exidx_start = .;  /* 异常索引起始地址 */
    *(.ARM.exidx*)      /* ARM异常索引项 */
    __exidx_end = .;    /* 异常索引结束地址 */
  } > FLASH

  /* 预初始化数组 */
  .preinit_array(READONLY) : ALIGN(4) /* "READONLY"关键字仅在GCC11及以上版本支持，在GCC10及更早版本中请移除 */
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);  /* 预初始化函数数组起始地址 */
    KEEP(*(.preinit_array*))                    /* 保留预初始化函数数组 */
    PROVIDE_HIDDEN(__preinit_array_end = .);    /* 预初始化函数数组结束地址 */
  } > FLASH

  /* 初始化函数数组 */
  .init_array(READONLY) : ALIGN(4)  /* "READONLY"关键字仅在GCC11及以上版本支持，在GCC10及更早版本中请移除 */
  {
    PROVIDE_HIDDEN(__init_array_start = .); /* 初始化函数数组起始地址 */
    KEEP(*(SORT(.init_array.*)))            /* 保留排序后的初始化函数数组 */
    KEEP(*(.init_array*))                   /* 保留初始化函数数组 */
    PROVIDE_HIDDEN(__init_array_end = .);   /* 初始化函数数组结束地址 */
  } > FLASH

  /* 终止函数数组 */
  .fini_array(READONLY) : ALIGN(4)  /* "READONLY"关键字仅在GCC11及以上版本支持，在GCC10及更早版本中请移除 */
  {
    PROVIDE_HIDDEN(__fini_array_start = .); /* 终止函数数组起始地址 */
    KEEP(*(SORT(.fini_array.*)))            /* 保留排序后的终止函数数组 */
    KEEP(*(.fini_array*))                   /* 保留终止函数数组 */
    PROVIDE_HIDDEN(__fini_array_end = .);   /* 终止函数数组结束地址 */
  } > FLASH

  /* 由启动代码用来初始化.data段 */
  _sidata = LOADADDR(.data);

  /* 已初始化的数据段放入RAM，在加载时从FLASH复制 */
  .data : ALIGN(4)  /* 地址对齐到4字节边界 */
  {
    _sdata = .;                /* 创建全局符号_sdata表示数据段开始 */
    *(.data)                   /* .data段 */
    *(.data*)                  /* .data*段 */
    *(.RamFunc)                /* .RamFunc段（在RAM中执行的函数） */
    *(.RamFunc*)               /* .RamFunc*段 */
  } > DTCMRAM AT > FLASH

  /* 初始化TLS（线程局部存储）数据段 */
  .tdata : ALIGN(4) /* 地址对齐到4字节边界 */
  {
    *(.tdata .tdata.* .gnu.linkonce.td.*) /* TLS数据 */
    . = ALIGN(4);
    _edata = .;                           /* 定义全局符号_edata表示数据段结束 */
    PROVIDE(__data_end = .);              /* 提供数据段结束符号 */
    PROVIDE(__tdata_end = .);             /* 提供TLS数据段结束符号 */
  } > DTCMRAM AT > FLASH

  /* 提供TLS相关地址和大小信息 */
  PROVIDE(__tdata_start = ADDR(.tdata));
  PROVIDE(__tdata_size = __tdata_end - __tdata_start);

  PROVIDE(__data_start = ADDR(.data));
  PROVIDE(__data_size = __data_end - __data_start);

  PROVIDE(__tdata_source = LOADADDR(.tdata));
  PROVIDE(__tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata));
  PROVIDE(__tdata_source_size = __tdata_source_end - __tdata_source);

  PROVIDE(__data_source = LOADADDR(.data));
  PROVIDE(__data_source_end = __tdata_source_end);
  PROVIDE(__data_source_size = __data_source_end - __data_source);
  
  /* 未初始化的数据段 */
  .tbss(NOLOAD) : ALIGN(4)  /* 地址对齐到4字节边界 */
  {
    /* 启动代码使用此段来初始化.bss段 */
    _sbss = .;                /* 定义全局符号_sbss表示bss段开始 */
    __bss_start__ = _sbss;    /* bss段开始地址 */
    *(.tbss .tbss.*)          /* TLS未初始化数据 */
    . = ALIGN(4);
    PROVIDE(__tbss_end = .);  /* TLS未初始化数据结束 */
  } > DTCMRAM

  /* 提供TBSS段相关信息 */
  PROVIDE(__tbss_start = ADDR(.tbss));
  PROVIDE(__tbss_size = __tbss_end - __tbss_start);
  PROVIDE(__tbss_offset = ADDR(.tbss) - ADDR(.tdata));

  /* TLS相关参数 */
  PROVIDE(__tls_base = __tdata_start);
  PROVIDE(__tls_end = __tbss_end);
  PROVIDE(__tls_size = __tls_end - __tls_base);
  PROVIDE(__tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)));
  PROVIDE(__tls_size_align =(__tls_size + __tls_align - 1) & ~(__tls_align - 1));
  PROVIDE(__arm32_tls_tcb_offset = MAX(8, __tls_align));
  PROVIDE(__arm64_tls_tcb_offset = MAX(16, __tls_align));

  /* BSS段（未初始化数据） */
  .bss(NOLOAD) : ALIGN(4) /* 地址对齐到4字节边界 */
  {
    *(.bss)                 /* .bss段 */
    *(.bss*)                /* .bss*段 */
    *(COMMON)               /* COMMON符号 */
    . = ALIGN(4);
    _ebss = .;              /* 定义全局符号_ebss表示bss段结束 */
    __bss_end__ = _ebss;    /* bss段结束地址 */
    PROVIDE(__bss_end = .); /* 提供bss段结束符号 */
  } > DTCMRAM
  PROVIDE(__non_tls_bss_start = ADDR(.bss));

  /* 设置BSS段相关信息 */
  PROVIDE(__bss_start = __tbss_start);
  PROVIDE(__bss_size = __bss_end - __bss_start);

  /* 用户堆栈段，用于检查是否有足够的RAM空间 */
  ._user_heap_stack(NOLOAD) : ALIGN(8)  /* 地址对齐到8字节边界 */
  {
    PROVIDE(end = .);         /* 提供程序结束符号 */
    PROVIDE(_end = .);        /* 提供程序结束符号 */
    . = . + _Min_Heap_Size;   /* 分配堆空间 */
    . = . + _Min_Stack_Size;  /* 分配栈空间 */
  } > DTCMRAM

  /* 移除标准库中的信息 */
  /DISCARD/ :
  {
    libc.a:*(*)   /* 丢弃libc库中的所有内容 */
    libm.a:*(*)   /* 丢弃libm库中的所有内容 */
    libgcc.a:*(*) /* 丢弃libgcc库中的所有内容 */
  }
}