.syntax unified
.cpu cortex-m4
.thumb

.global g_pfnVectors
.global Reset_Handler

/* Stack top (adjust if needed) */
_estack = 0x20020000

.section .isr_vector, "a", %progbits
g_pfnVectors:
    .word _estack
    .word Reset_Handler

    /* minimal vectors */
    .rept 14
    .word 0
    .endr

    .word USART2_IRQHandler

/* Reset handler */
.section .text.Reset_Handler
Reset_Handler:
    bl SystemInit
    bl main
    b .

/* Default weak handlers */
.weak USART2_IRQHandler
.thumb_set USART2_IRQHandler, Default_Handler

Default_Handler:
    b .
