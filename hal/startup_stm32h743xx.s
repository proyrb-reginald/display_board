.syntax unified
.cpu cortex-m7
.fpu softvfp
.thumb

/* 定义全局符号 */
.global g_pfnVectors
.global Default_Handler

/* 定义数据段地址标号 */
.word _sidata      /* 初始化数据段在Flash中的起始地址 */
.word _sdata       /* 数据段在RAM中的起始地址 */
.word _edata       /* 数据段在RAM中的结束地址 */
.word _sbss        /* BSS段在RAM中的起始地址 */
.word _ebss        /* BSS段在RAM中的结束地址 */

/**
 * 复位处理函数
 * 系统上电或复位后首先执行此函数
 */
.section .text.Reset_Handler
.weak Reset_Handler                /* 弱定义Reset_Handler，允许用户覆盖 */
.type Reset_Handler, %function     /* 指定Reset_Handler为函数类型 */
Reset_Handler:
  ldr sp, =_estack                 /* 设置栈顶指针 */
  bl ExitRun0Mode                  /* 调用ExitRun0Mode函数 */
  bl SystemInit                    /* 调用SystemInit函数进行系统初始化 */
  
  /* 复制初始化数据段(.data段)从Flash到RAM */
  ldr r0, =_sdata                  /* RAM中数据段起始地址 */
  ldr r1, =_edata                  /* RAM中数据段结束地址 */
  ldr r2, =_sidata                 /* Flash中数据段起始地址 */
  movs r3, #0                      /* 偏移量清零 */
  b LoopCopyDataInit               /* 跳转到复制循环 */

CopyDataInit:
  ldr r4, [r2, r3]                 /* 从Flash加载数据 */
  str r4, [r0, r3]                 /* 存储到RAM */
  adds r3, r3, #4                  /* 偏移量增加4字节 */

LoopCopyDataInit:
  adds r4, r0, r3                  /* 计算当前RAM地址 */
  cmp r4, r1                       /* 比较是否到达数据段末尾 */
  bcc CopyDataInit                 /* 如果未到末尾则继续复制 */
  
  /* 清零BSS段(.bss段) */
  ldr r2, =_sbss                   /* BSS段起始地址 */
  ldr r4, =_ebss                   /* BSS段结束地址 */
  movs r3, #0                      /* 清零用的数据 */
  b LoopFillZerobss                /* 跳转到清零循环 */

FillZerobss:
  str r3, [r2]                     /* 将0写入BSS段 */
  adds r2, r2, #4                  /* 地址增加4字节 */

LoopFillZerobss:
  cmp r2, r4                       /* 比较是否到达BSS段末尾 */
  bcc FillZerobss                  /* 如果未到末尾则继续清零 */
  
  /* 调用标准C库初始化函数 */
  bl __libc_init_array             /* 初始化C库 */
  bl entry                         /* 调用rtthread入口函数 */
  bx lr                            /* 返回 */

.size Reset_Handler, .-Reset_Handler

/**
 * 默认中断处理函数
 * 所有未定义的中断都会陷入此函数形成死循环
 */
.section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop                  /* 死循环 */

.size Default_Handler, .-Default_Handler

/**
 * 中断向量表
 * 包含所有异常和中断的入口地址
 */
.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
g_pfnVectors:
  .word _estack                    /* 栈顶地址 */
  .word Reset_Handler              /* 复位处理函数 */

  /* Cortex-M内核异常处理函数 */
  .word NMI_Handler                /* NMI不可屏蔽中断处理函数 */
  .word HardFault_Handler          /* 硬件故障处理函数 */
  .word MemManage_Handler          /* 内存管理故障处理函数 */
  .word BusFault_Handler           /* 总线故障处理函数 */
  .word UsageFault_Handler         /* 使用 fault 处理函数 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word SVC_Handler                /* SVC系统服务调用处理函数 */
  .word DebugMon_Handler           /* 调试监视器处理函数 */
  .word 0                          /* 保留 */
  .word PendSV_Handler             /* 可挂起的系统滴答处理函数 */
  .word SysTick_Handler            /* 系统滴答定时器处理函数 */

  /* 外部中断处理函数 */
  .word WWDG_IRQHandler            /* 窗口看门狗中断 */
  .word PVD_AVD_IRQHandler         /* PVD/AVD通过EXTI线检测中断 */
  .word TAMP_STAMP_IRQHandler      /* Tamper和TimeStamps通过EXTI线中断 */
  .word RTC_WKUP_IRQHandler        /* RTC通过EXTI线唤醒中断 */
  .word FLASH_IRQHandler           /* Flash中断 */
  .word RCC_IRQHandler             /* RCC时钟控制中断 */
  .word EXTI0_IRQHandler           /* EXTI Line0中断 */
  .word EXTI1_IRQHandler           /* EXTI Line1中断 */
  .word EXTI2_IRQHandler           /* EXTI Line2中断 */
  .word EXTI3_IRQHandler           /* EXTI Line3中断 */
  .word EXTI4_IRQHandler           /* EXTI Line4中断 */
  .word DMA1_Stream0_IRQHandler    /* DMA1 Stream 0中断 */
  .word DMA1_Stream1_IRQHandler    /* DMA1 Stream 1中断 */
  .word DMA1_Stream2_IRQHandler    /* DMA1 Stream 2中断 */
  .word DMA1_Stream3_IRQHandler    /* DMA1 Stream 3中断 */
  .word DMA1_Stream4_IRQHandler    /* DMA1 Stream 4中断 */
  .word DMA1_Stream5_IRQHandler    /* DMA1 Stream 5中断 */
  .word DMA1_Stream6_IRQHandler    /* DMA1 Stream 6中断 */
  .word ADC_IRQHandler             /* ADC1, ADC2和ADC3中断 */
  .word FDCAN1_IT0_IRQHandler      /* FDCAN1中断线0 */
  .word FDCAN2_IT0_IRQHandler      /* FDCAN2中断线0 */
  .word FDCAN1_IT1_IRQHandler      /* FDCAN1中断线1 */
  .word FDCAN2_IT1_IRQHandler      /* FDCAN2中断线1 */
  .word EXTI9_5_IRQHandler         /* 外部Line[9:5]中断 */
  .word TIM1_BRK_IRQHandler        /* TIM1刹车中断 */
  .word TIM1_UP_IRQHandler         /* TIM1更新中断 */
  .word TIM1_TRG_COM_IRQHandler    /* TIM1触发和换相中断 */
  .word TIM1_CC_IRQHandler         /* TIM1捕获比较中断 */
  .word TIM2_IRQHandler            /* TIM2中断 */
  .word TIM3_IRQHandler            /* TIM3中断 */
  .word TIM4_IRQHandler            /* TIM4中断 */
  .word I2C1_EV_IRQHandler         /* I2C1事件中断 */
  .word I2C1_ER_IRQHandler         /* I2C1错误中断 */
  .word I2C2_EV_IRQHandler         /* I2C2事件中断 */
  .word I2C2_ER_IRQHandler         /* I2C2错误中断 */
  .word SPI1_IRQHandler            /* SPI1中断 */
  .word SPI2_IRQHandler            /* SPI2中断 */
  .word USART1_IRQHandler          /* USART1中断 */
  .word USART2_IRQHandler          /* USART2中断 */
  .word USART3_IRQHandler          /* USART3中断 */
  .word EXTI15_10_IRQHandler       /* 外部Line[15:10]中断 */
  .word RTC_Alarm_IRQHandler       /* RTC闹钟(A和B)通过EXTI线中断 */
  .word 0                          /* 保留 */
  .word TIM8_BRK_TIM12_IRQHandler  /* TIM8刹车和TIM12中断 */
  .word TIM8_UP_TIM13_IRQHandler   /* TIM8更新和TIM13中断 */
  .word TIM8_TRG_COM_TIM14_IRQHandler /* TIM8触发和换相以及TIM14中断 */
  .word TIM8_CC_IRQHandler         /* TIM8捕获比较中断 */
  .word DMA1_Stream7_IRQHandler    /* DMA1 Stream7中断 */
  .word FMC_IRQHandler             /* FMC中断 */
  .word SDMMC1_IRQHandler          /* SDMMC1中断 */
  .word TIM5_IRQHandler            /* TIM5中断 */
  .word SPI3_IRQHandler            /* SPI3中断 */
  .word UART4_IRQHandler           /* UART4中断 */
  .word UART5_IRQHandler           /* UART5中断 */
  .word TIM6_DAC_IRQHandler        /* TIM6和DAC1&2下溢错误中断 */
  .word TIM7_IRQHandler            /* TIM7中断 */
  .word DMA2_Stream0_IRQHandler    /* DMA2 Stream 0中断 */
  .word DMA2_Stream1_IRQHandler    /* DMA2 Stream 1中断 */
  .word DMA2_Stream2_IRQHandler    /* DMA2 Stream 2中断 */
  .word DMA2_Stream3_IRQHandler    /* DMA2 Stream 3中断 */
  .word DMA2_Stream4_IRQHandler    /* DMA2 Stream 4中断 */
  .word ETH_IRQHandler             /* 以太网中断 */
  .word ETH_WKUP_IRQHandler        /* 以太网通过EXTI线唤醒中断 */
  .word FDCAN_CAL_IRQHandler       /* FDCAN校准单元中断 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word DMA2_Stream5_IRQHandler    /* DMA2 Stream 5中断 */
  .word DMA2_Stream6_IRQHandler    /* DMA2 Stream 6中断 */
  .word DMA2_Stream7_IRQHandler    /* DMA2 Stream 7中断 */
  .word USART6_IRQHandler          /* USART6中断 */
  .word I2C3_EV_IRQHandler         /* I2C3事件中断 */
  .word I2C3_ER_IRQHandler         /* I2C3错误中断 */
  .word OTG_HS_EP1_OUT_IRQHandler  /* USB OTG HS端点1输出中断 */
  .word OTG_HS_EP1_IN_IRQHandler   /* USB OTG HS端点1输入中断 */
  .word OTG_HS_WKUP_IRQHandler     /* USB OTG HS通过EXTI线唤醒中断 */
  .word OTG_HS_IRQHandler          /* USB OTG HS中断 */
  .word DCMI_IRQHandler            /* DCMI中断 */
  .word 0                          /* 保留 */
  .word RNG_IRQHandler             /* 随机数生成器中断 */
  .word FPU_IRQHandler             /* 浮点运算单元中断 */
  .word UART7_IRQHandler           /* UART7中断 */
  .word UART8_IRQHandler           /* UART8中断 */
  .word SPI4_IRQHandler            /* SPI4中断 */
  .word SPI5_IRQHandler            /* SPI5中断 */
  .word SPI6_IRQHandler            /* SPI6中断 */
  .word SAI1_IRQHandler            /* SAI1中断 */
  .word LTDC_IRQHandler            /* LTDC中断 */
  .word LTDC_ER_IRQHandler         /* LTDC错误中断 */
  .word DMA2D_IRQHandler           /* DMA2D中断 */
  .word SAI2_IRQHandler            /* SAI2中断 */
  .word QUADSPI_IRQHandler         /* QUADSPI中断 */
  .word LPTIM1_IRQHandler          /* LPTIM1中断 */
  .word CEC_IRQHandler             /* HDMI_CEC中断 */
  .word I2C4_EV_IRQHandler         /* I2C4事件中断 */
  .word I2C4_ER_IRQHandler         /* I2C4错误中断 */
  .word SPDIF_RX_IRQHandler        /* SPDIF_RX中断 */
  .word OTG_FS_EP1_OUT_IRQHandler  /* USB OTG FS端点1输出中断 */
  .word OTG_FS_EP1_IN_IRQHandler   /* USB OTG FS端点1输入中断 */
  .word OTG_FS_WKUP_IRQHandler     /* USB OTG FS通过EXTI线唤醒中断 */
  .word OTG_FS_IRQHandler          /* USB OTG FS中断 */
  .word DMAMUX1_OVR_IRQHandler     /* DMAMUX1溢出中断 */
  .word HRTIM1_Master_IRQHandler   /* HRTIM主定时器全局中断 */
  .word HRTIM1_TIMA_IRQHandler     /* HRTIM定时器A全局中断 */
  .word HRTIM1_TIMB_IRQHandler     /* HRTIM定时器B全局中断 */
  .word HRTIM1_TIMC_IRQHandler     /* HRTIM定时器C全局中断 */
  .word HRTIM1_TIMD_IRQHandler     /* HRTIM定时器D全局中断 */
  .word HRTIM1_TIME_IRQHandler     /* HRTIM定时器E全局中断 */
  .word HRTIM1_FLT_IRQHandler      /* HRTIM故障全局中断 */
  .word DFSDM1_FLT0_IRQHandler     /* DFSDM滤波器0中断 */
  .word DFSDM1_FLT1_IRQHandler     /* DFSDM滤波器1中断 */
  .word DFSDM1_FLT2_IRQHandler     /* DFSDM滤波器2中断 */
  .word DFSDM1_FLT3_IRQHandler     /* DFSDM滤波器3中断 */
  .word SAI3_IRQHandler            /* SAI3全局中断 */
  .word SWPMI1_IRQHandler          /* 单线协议主接口1全局中断 */
  .word TIM15_IRQHandler           /* TIM15全局中断 */
  .word TIM16_IRQHandler           /* TIM16全局中断 */
  .word TIM17_IRQHandler           /* TIM17全局中断 */
  .word MDIOS_WKUP_IRQHandler      /* MDIOS唤醒中断 */
  .word MDIOS_IRQHandler           /* MDIOS全局中断 */
  .word JPEG_IRQHandler            /* JPEG全局中断 */
  .word MDMA_IRQHandler            /* MDMA全局中断 */
  .word 0                          /* 保留 */
  .word SDMMC2_IRQHandler          /* SDMMC2全局中断 */
  .word HSEM1_IRQHandler           /* HSEM1全局中断 */
  .word 0                          /* 保留 */
  .word ADC3_IRQHandler            /* ADC3全局中断 */
  .word DMAMUX2_OVR_IRQHandler     /* DMAMUX溢出中断 */
  .word BDMA_Channel0_IRQHandler   /* BDMA通道0全局中断 */
  .word BDMA_Channel1_IRQHandler   /* BDMA通道1全局中断 */
  .word BDMA_Channel2_IRQHandler   /* BDMA通道2全局中断 */
  .word BDMA_Channel3_IRQHandler   /* BDMA通道3全局中断 */
  .word BDMA_Channel4_IRQHandler   /* BDMA通道4全局中断 */
  .word BDMA_Channel5_IRQHandler   /* BDMA通道5全局中断 */
  .word BDMA_Channel6_IRQHandler   /* BDMA通道6全局中断 */
  .word BDMA_Channel7_IRQHandler   /* BDMA通道7全局中断 */
  .word COMP1_IRQHandler           /* COMP1全局中断 */
  .word LPTIM2_IRQHandler          /* LP TIM2全局中断 */
  .word LPTIM3_IRQHandler          /* LP TIM3全局中断 */
  .word LPTIM4_IRQHandler          /* LP TIM4全局中断 */
  .word LPTIM5_IRQHandler          /* LP TIM5全局中断 */
  .word LPUART1_IRQHandler         /* LP UART1中断 */
  .word 0                          /* 保留 */
  .word CRS_IRQHandler             /* 时钟恢复全局中断 */
  .word ECC_IRQHandler             /* ECC诊断全局中断 */
  .word SAI4_IRQHandler            /* SAI4全局中断 */
  .word 0                          /* 保留 */
  .word 0                          /* 保留 */
  .word WAKEUP_PIN_IRQHandler      /* 全部6个唤醒引脚的中断 */

 .size g_pfnVectors, .-g_pfnVectors

/* 为所有中断处理函数提供弱定义，使其默认指向Default_Handler */

.weak NMI_Handler
.thumb_set NMI_Handler,Default_Handler

.weak HardFault_Handler
.thumb_set HardFault_Handler,Default_Handler

.weak MemManage_Handler
.thumb_set MemManage_Handler,Default_Handler

.weak BusFault_Handler
.thumb_set BusFault_Handler,Default_Handler

.weak UsageFault_Handler
.thumb_set UsageFault_Handler,Default_Handler

.weak SVC_Handler
.thumb_set SVC_Handler,Default_Handler

.weak DebugMon_Handler
.thumb_set DebugMon_Handler,Default_Handler

.weak PendSV_Handler
.thumb_set PendSV_Handler,Default_Handler

.weak SysTick_Handler
.thumb_set SysTick_Handler,Default_Handler

.weak WWDG_IRQHandler
.thumb_set WWDG_IRQHandler,Default_Handler

.weak PVD_AVD_IRQHandler
.thumb_set PVD_AVD_IRQHandler,Default_Handler

.weak TAMP_STAMP_IRQHandler
.thumb_set TAMP_STAMP_IRQHandler,Default_Handler

.weak RTC_WKUP_IRQHandler
.thumb_set RTC_WKUP_IRQHandler,Default_Handler

.weak FLASH_IRQHandler
.thumb_set FLASH_IRQHandler,Default_Handler

.weak RCC_IRQHandler
.thumb_set RCC_IRQHandler,Default_Handler

.weak EXTI0_IRQHandler
.thumb_set EXTI0_IRQHandler,Default_Handler

.weak EXTI1_IRQHandler
.thumb_set EXTI1_IRQHandler,Default_Handler

.weak EXTI2_IRQHandler
.thumb_set EXTI2_IRQHandler,Default_Handler

.weak EXTI3_IRQHandler
.thumb_set EXTI3_IRQHandler,Default_Handler

.weak EXTI4_IRQHandler
.thumb_set EXTI4_IRQHandler,Default_Handler

.weak DMA1_Stream0_IRQHandler
.thumb_set DMA1_Stream0_IRQHandler,Default_Handler

.weak DMA1_Stream1_IRQHandler
.thumb_set DMA1_Stream1_IRQHandler,Default_Handler

.weak DMA1_Stream2_IRQHandler
.thumb_set DMA1_Stream2_IRQHandler,Default_Handler

.weak DMA1_Stream3_IRQHandler
.thumb_set DMA1_Stream3_IRQHandler,Default_Handler

.weak DMA1_Stream4_IRQHandler
.thumb_set DMA1_Stream4_IRQHandler,Default_Handler

.weak DMA1_Stream5_IRQHandler
.thumb_set DMA1_Stream5_IRQHandler,Default_Handler

.weak DMA1_Stream6_IRQHandler
.thumb_set DMA1_Stream6_IRQHandler,Default_Handler

.weak ADC_IRQHandler
.thumb_set ADC_IRQHandler,Default_Handler

.weak FDCAN1_IT0_IRQHandler
.thumb_set FDCAN1_IT0_IRQHandler,Default_Handler

.weak FDCAN2_IT0_IRQHandler
.thumb_set FDCAN2_IT0_IRQHandler,Default_Handler

.weak FDCAN1_IT1_IRQHandler
.thumb_set FDCAN1_IT1_IRQHandler,Default_Handler

.weak FDCAN2_IT1_IRQHandler
.thumb_set FDCAN2_IT1_IRQHandler,Default_Handler

.weak EXTI9_5_IRQHandler
.thumb_set EXTI9_5_IRQHandler,Default_Handler

.weak TIM1_BRK_IRQHandler
.thumb_set TIM1_BRK_IRQHandler,Default_Handler

.weak TIM1_UP_IRQHandler
.thumb_set TIM1_UP_IRQHandler,Default_Handler

.weak TIM1_TRG_COM_IRQHandler
.thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

.weak TIM1_CC_IRQHandler
.thumb_set TIM1_CC_IRQHandler,Default_Handler

.weak TIM2_IRQHandler
.thumb_set TIM2_IRQHandler,Default_Handler

.weak TIM3_IRQHandler
.thumb_set TIM3_IRQHandler,Default_Handler

.weak TIM4_IRQHandler
.thumb_set TIM4_IRQHandler,Default_Handler

.weak I2C1_EV_IRQHandler
.thumb_set I2C1_EV_IRQHandler,Default_Handler

.weak I2C1_ER_IRQHandler
.thumb_set I2C1_ER_IRQHandler,Default_Handler

.weak I2C2_EV_IRQHandler
.thumb_set I2C2_EV_IRQHandler,Default_Handler

.weak I2C2_ER_IRQHandler
.thumb_set I2C2_ER_IRQHandler,Default_Handler

.weak SPI1_IRQHandler
.thumb_set SPI1_IRQHandler,Default_Handler

.weak SPI2_IRQHandler
.thumb_set SPI2_IRQHandler,Default_Handler

.weak USART1_IRQHandler
.thumb_set USART1_IRQHandler,Default_Handler

.weak USART2_IRQHandler
.thumb_set USART2_IRQHandler,Default_Handler

.weak USART3_IRQHandler
.thumb_set USART3_IRQHandler,Default_Handler

.weak EXTI15_10_IRQHandler
.thumb_set EXTI15_10_IRQHandler,Default_Handler

.weak RTC_Alarm_IRQHandler
.thumb_set RTC_Alarm_IRQHandler,Default_Handler

.weak TIM8_BRK_TIM12_IRQHandler
.thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler

.weak TIM8_UP_TIM13_IRQHandler
.thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler

.weak TIM8_TRG_COM_TIM14_IRQHandler
.thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler

.weak TIM8_CC_IRQHandler
.thumb_set TIM8_CC_IRQHandler,Default_Handler

.weak DMA1_Stream7_IRQHandler
.thumb_set DMA1_Stream7_IRQHandler,Default_Handler

.weak FMC_IRQHandler
.thumb_set FMC_IRQHandler,Default_Handler

.weak SDMMC1_IRQHandler
.thumb_set SDMMC1_IRQHandler,Default_Handler

.weak TIM5_IRQHandler
.thumb_set TIM5_IRQHandler,Default_Handler

.weak SPI3_IRQHandler
.thumb_set SPI3_IRQHandler,Default_Handler

.weak UART4_IRQHandler
.thumb_set UART4_IRQHandler,Default_Handler

.weak UART5_IRQHandler
.thumb_set UART5_IRQHandler,Default_Handler

.weak TIM6_DAC_IRQHandler
.thumb_set TIM6_DAC_IRQHandler,Default_Handler

.weak TIM7_IRQHandler
.thumb_set TIM7_IRQHandler,Default_Handler

.weak DMA2_Stream0_IRQHandler
.thumb_set DMA2_Stream0_IRQHandler,Default_Handler

.weak DMA2_Stream1_IRQHandler
.thumb_set DMA2_Stream1_IRQHandler,Default_Handler

.weak DMA2_Stream2_IRQHandler
.thumb_set DMA2_Stream2_IRQHandler,Default_Handler

.weak DMA2_Stream3_IRQHandler
.thumb_set DMA2_Stream3_IRQHandler,Default_Handler

.weak DMA2_Stream4_IRQHandler
.thumb_set DMA2_Stream4_IRQHandler,Default_Handler

.weak ETH_IRQHandler
.thumb_set ETH_IRQHandler,Default_Handler

.weak ETH_WKUP_IRQHandler
.thumb_set ETH_WKUP_IRQHandler,Default_Handler

.weak FDCAN_CAL_IRQHandler
.thumb_set FDCAN_CAL_IRQHandler,Default_Handler

.weak DMA2_Stream5_IRQHandler
.thumb_set DMA2_Stream5_IRQHandler,Default_Handler

.weak DMA2_Stream6_IRQHandler
.thumb_set DMA2_Stream6_IRQHandler,Default_Handler

.weak DMA2_Stream7_IRQHandler
.thumb_set DMA2_Stream7_IRQHandler,Default_Handler

.weak USART6_IRQHandler
.thumb_set USART6_IRQHandler,Default_Handler

.weak I2C3_EV_IRQHandler
.thumb_set I2C3_EV_IRQHandler,Default_Handler

.weak I2C3_ER_IRQHandler
.thumb_set I2C3_ER_IRQHandler,Default_Handler

.weak OTG_HS_EP1_OUT_IRQHandler
.thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler

.weak OTG_HS_EP1_IN_IRQHandler
.thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler

.weak OTG_HS_WKUP_IRQHandler
.thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler

.weak OTG_HS_IRQHandler
.thumb_set OTG_HS_IRQHandler,Default_Handler

.weak DCMI_IRQHandler
.thumb_set DCMI_IRQHandler,Default_Handler

.weak RNG_IRQHandler
.thumb_set RNG_IRQHandler,Default_Handler

.weak FPU_IRQHandler
.thumb_set FPU_IRQHandler,Default_Handler

.weak UART7_IRQHandler
.thumb_set UART7_IRQHandler,Default_Handler

.weak UART8_IRQHandler
.thumb_set UART8_IRQHandler,Default_Handler

.weak SPI4_IRQHandler
.thumb_set SPI4_IRQHandler,Default_Handler

.weak SPI5_IRQHandler
.thumb_set SPI5_IRQHandler,Default_Handler

.weak SPI6_IRQHandler
.thumb_set SPI6_IRQHandler,Default_Handler

.weak SAI1_IRQHandler
.thumb_set SAI1_IRQHandler,Default_Handler

.weak LTDC_IRQHandler
.thumb_set LTDC_IRQHandler,Default_Handler

.weak LTDC_ER_IRQHandler
.thumb_set LTDC_ER_IRQHandler,Default_Handler

.weak DMA2D_IRQHandler
.thumb_set DMA2D_IRQHandler,Default_Handler

.weak SAI2_IRQHandler
.thumb_set SAI2_IRQHandler,Default_Handler

.weak QUADSPI_IRQHandler
.thumb_set QUADSPI_IRQHandler,Default_Handler

.weak LPTIM1_IRQHandler
.thumb_set LPTIM1_IRQHandler,Default_Handler

.weak CEC_IRQHandler
.thumb_set CEC_IRQHandler,Default_Handler

.weak I2C4_EV_IRQHandler
.thumb_set I2C4_EV_IRQHandler,Default_Handler

.weak I2C4_ER_IRQHandler
.thumb_set I2C4_ER_IRQHandler,Default_Handler

.weak SPDIF_RX_IRQHandler
.thumb_set SPDIF_RX_IRQHandler,Default_Handler

.weak OTG_FS_EP1_OUT_IRQHandler
.thumb_set OTG_FS_EP1_OUT_IRQHandler,Default_Handler

.weak OTG_FS_EP1_IN_IRQHandler
.thumb_set OTG_FS_EP1_IN_IRQHandler,Default_Handler

.weak OTG_FS_WKUP_IRQHandler
.thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

.weak OTG_FS_IRQHandler
.thumb_set OTG_FS_IRQHandler,Default_Handler

.weak DMAMUX1_OVR_IRQHandler
.thumb_set DMAMUX1_OVR_IRQHandler,Default_Handler

.weak HRTIM1_Master_IRQHandler
.thumb_set HRTIM1_Master_IRQHandler,Default_Handler

.weak HRTIM1_TIMA_IRQHandler
.thumb_set HRTIM1_TIMA_IRQHandler,Default_Handler

.weak HRTIM1_TIMB_IRQHandler
.thumb_set HRTIM1_TIMB_IRQHandler,Default_Handler

.weak HRTIM1_TIMC_IRQHandler
.thumb_set HRTIM1_TIMC_IRQHandler,Default_Handler

.weak HRTIM1_TIMD_IRQHandler
.thumb_set HRTIM1_TIMD_IRQHandler,Default_Handler

.weak HRTIM1_TIME_IRQHandler
.thumb_set HRTIM1_TIME_IRQHandler,Default_Handler

.weak HRTIM1_FLT_IRQHandler
.thumb_set HRTIM1_FLT_IRQHandler,Default_Handler

.weak DFSDM1_FLT0_IRQHandler
.thumb_set DFSDM1_FLT0_IRQHandler,Default_Handler

.weak DFSDM1_FLT1_IRQHandler
.thumb_set DFSDM1_FLT1_IRQHandler,Default_Handler

.weak DFSDM1_FLT2_IRQHandler
.thumb_set DFSDM1_FLT2_IRQHandler,Default_Handler

.weak DFSDM1_FLT3_IRQHandler
.thumb_set DFSDM1_FLT3_IRQHandler,Default_Handler

.weak SAI3_IRQHandler
.thumb_set SAI3_IRQHandler,Default_Handler

.weak SWPMI1_IRQHandler
.thumb_set SWPMI1_IRQHandler,Default_Handler

.weak TIM15_IRQHandler
.thumb_set TIM15_IRQHandler,Default_Handler

.weak TIM16_IRQHandler
.thumb_set TIM16_IRQHandler,Default_Handler

.weak TIM17_IRQHandler
.thumb_set TIM17_IRQHandler,Default_Handler

.weak MDIOS_WKUP_IRQHandler
.thumb_set MDIOS_WKUP_IRQHandler,Default_Handler

.weak MDIOS_IRQHandler
.thumb_set MDIOS_IRQHandler,Default_Handler

.weak JPEG_IRQHandler
.thumb_set JPEG_IRQHandler,Default_Handler

.weak MDMA_IRQHandler
.thumb_set MDMA_IRQHandler,Default_Handler

.weak SDMMC2_IRQHandler
.thumb_set SDMMC2_IRQHandler,Default_Handler

.weak HSEM1_IRQHandler
.thumb_set HSEM1_IRQHandler,Default_Handler

.weak ADC3_IRQHandler
.thumb_set ADC3_IRQHandler,Default_Handler

.weak DMAMUX2_OVR_IRQHandler
.thumb_set DMAMUX2_OVR_IRQHandler,Default_Handler

.weak BDMA_Channel0_IRQHandler
.thumb_set BDMA_Channel0_IRQHandler,Default_Handler

.weak BDMA_Channel1_IRQHandler
.thumb_set BDMA_Channel1_IRQHandler,Default_Handler

.weak BDMA_Channel2_IRQHandler
.thumb_set BDMA_Channel2_IRQHandler,Default_Handler

.weak BDMA_Channel3_IRQHandler
.thumb_set BDMA_Channel3_IRQHandler,Default_Handler

.weak BDMA_Channel4_IRQHandler
.thumb_set BDMA_Channel4_IRQHandler,Default_Handler

.weak BDMA_Channel5_IRQHandler
.thumb_set BDMA_Channel5_IRQHandler,Default_Handler

.weak BDMA_Channel6_IRQHandler
.thumb_set BDMA_Channel6_IRQHandler,Default_Handler

.weak BDMA_Channel7_IRQHandler
.thumb_set BDMA_Channel7_IRQHandler,Default_Handler

.weak COMP1_IRQHandler
.thumb_set COMP1_IRQHandler,Default_Handler

.weak LPTIM2_IRQHandler
.thumb_set LPTIM2_IRQHandler,Default_Handler

.weak LPTIM3_IRQHandler
.thumb_set LPTIM3_IRQHandler,Default_Handler

.weak LPTIM4_IRQHandler
.thumb_set LPTIM4_IRQHandler,Default_Handler

.weak LPTIM5_IRQHandler
.thumb_set LPTIM5_IRQHandler,Default_Handler

.weak LPUART1_IRQHandler
.thumb_set LPUART1_IRQHandler,Default_Handler

.weak CRS_IRQHandler
.thumb_set CRS_IRQHandler,Default_Handler

.weak ECC_IRQHandler
.thumb_set ECC_IRQHandler,Default_Handler

.weak SAI4_IRQHandler
.thumb_set SAI4_IRQHandler,Default_Handler

.weak WAKEUP_PIN_IRQHandler
.thumb_set WAKEUP_PIN_IRQHandler,Default_Handler