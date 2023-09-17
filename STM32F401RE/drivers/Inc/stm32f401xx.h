/*
 * stm32f401xx.h
 *
 *  Created on: Sep 14, 2023
 *      Author: Ravikishore A
 */

#ifndef STM32F401XX_H_
#define STM32F401XX_H_
#include <stdio.h>


#define __vo volatile

/*
 *
 * base Addresses of flash and SRAM
 */

#define FLASH_BASEADDR							0x08000000U     /* base address of FLASH memory */
#define SRAM1_BASEADDR                          0x20000000U     /* base address of SRAM memory */
#define SRAM                                    SRAM1_BASEADDR
#define ROM                                     0x1FFF0000U     /* base address of System memory aka ROM  */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE                             0x40000000U
#define APB1PERIPH_BASE                         PERIPH_BASE
#define APB2PERIPH_BASE                         0x40010000U
#define AHB1PHERIPH_BASE                        0x40020000U
#define AHB2PHERIPH_BASE                        0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                          (AHB1PHERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR                          (AHB1PHERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                          (AHB1PHERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                          (AHB1PHERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR                          (AHB1PHERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR                          (AHB1PHERIPH_BASE + 0x1C00)
#define RCC_BASEADDR                            (AHB1PHERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR                          (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR                          (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR                          (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR                          (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR                          (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR                        (APB1PERIPH_BASE + 0x4400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define SPI1_BASEADDR                           (APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR                        (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR                        (APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR                          (APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR                        (APB2PERIPH_BASE + 0x3800)

/*
 * Peripheral register definition Structures
 */

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t reserved_1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t reserved_2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t reserved_3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t reserved_4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t reserved_5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t reserved_6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t reserved_7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;

/*
 * Peripheral Definitions
 */
#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN() ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN() ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN() ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN() ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN() ( RCC->AHB1ENR |= (1<<4) )
#define GPIOH_PCLK_EN() ( RCC->AHB1ENR |= (1<<7) )

/*
 * Clock Enable Macros for I2Cx Peripherals
 */

#define I2C1_PCLK_EN() ( RCC->APB1ENR) |= (1<<21) )
#define I2C2_PCLK_EN() ( RCC->APB1ENR) |= (1<<22) )
#define I2C3_PCLK_EN() ( RCC->APB1ENR) |= (1<<23) )

/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN() ( RCC->APB2ENR) |= (1<<12) )
#define SPI2_PCLK_EN() ( RCC->APB1ENR) |= (1<<14) )
#define SPI3_PCLK_EN() ( RCC->APB1ENR) |= (1<<15) )

/*
 * Clock Enable Macros for USARTx Peripherals
 */
#define USART1_PCLK_EN() ( RCC->APB2ENR) |= (1<<4) )
#define USART2_PCLK_EN() ( RCC->APB1ENR) |= (1<<14) )
#define USART6_PCLK_EN() ( RCC->APB2ENR) |= (1<<5) )

/*
 * Clock Enable Macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN() ( RCC->APB2ENR) |= (1<<14) )

/*
 * Clock Disable Macros for GPIOx Pheripheral
 */
#define GPIOA_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOH_PCLK_DI() ( RCC->AHB1ENR &= ~(1<<7) )



/*
 * Macros to reset GPIOx Peripherals
 */
#define GPIOA_REG_RESET()           do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()           do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()           do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()           do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()           do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOH_REG_RESET()           do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)

/*
 * Some Generic Macros
 */
#define ENABLE                     1
#define DISABLE                    0
#define SET                        ENABLE
#define RESET                      DISABLE

#include "stm32f401xx_gpio_driver.h"
#endif /* STM32F401XX_H_ */
