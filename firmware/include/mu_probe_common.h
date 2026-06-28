#ifndef MU_PROBE_COMMON_H
#define MU_PROBE_COMMON_H

#include "stm32f0xx.h"

#include <cstdint>

namespace mu {

constexpr std::uint32_t kSystemClockHz = 48000000UL;
constexpr std::uint32_t kTimerTickHz = 1000000UL;

constexpr std::uint8_t kCatchOutAdcIndex = 0U;  // PA0
constexpr std::uint8_t kFlashCsIndex = 3U;      // PA3
constexpr std::uint8_t kSpiSckIndex = 5U;       // PA5
constexpr std::uint8_t kSpiMisoIndex = 6U;      // PA6
constexpr std::uint8_t kSpiMosiIndex = 7U;      // PA7
constexpr std::uint8_t kCanRxIndex = 11U;       // PA11
constexpr std::uint8_t kCanTxIndex = 12U;       // PA12
constexpr std::uint8_t kMs5607CsIndex = 1U;     // PB1

constexpr std::uint16_t kCatchOutAdcPin = 1U << kCatchOutAdcIndex;
constexpr std::uint16_t kFlashCsPin = 1U << kFlashCsIndex;
constexpr std::uint16_t kSpiSckPin = 1U << kSpiSckIndex;
constexpr std::uint16_t kSpiMisoPin = 1U << kSpiMisoIndex;
constexpr std::uint16_t kSpiMosiPin = 1U << kSpiMosiIndex;
constexpr std::uint16_t kCanRxPin = 1U << kCanRxIndex;
constexpr std::uint16_t kCanTxPin = 1U << kCanTxIndex;
constexpr std::uint16_t kMs5607CsPin = 1U << kMs5607CsIndex;

enum class SpiWireMode : std::uint8_t {
    Normal = 0U,   // MCU PA7 -> peripheral SDI, peripheral SDO -> MCU PA6.
    Swapped = 1U,  // MCU PA6 -> peripheral DI, peripheral DO -> MCU PA7.
};

enum class SpiClockMode : std::uint8_t {
    Mode0 = 0U,
    Mode3 = 3U,
};

inline void enable_gpio_clocks()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    (void)RCC->AHBENR;
}

inline void set_pin_high(GPIO_TypeDef* port, std::uint16_t pin)
{
    port->BSRR = pin;
}

inline void set_pin_low(GPIO_TypeDef* port, std::uint16_t pin)
{
    port->BRR = pin;
}

inline void configure_input(GPIO_TypeDef* port, std::uint8_t index)
{
    const std::uint32_t shift = static_cast<std::uint32_t>(index) * 2UL;
    port->MODER &= ~(3UL << shift);
    port->PUPDR &= ~(3UL << shift);
}

inline void configure_output(GPIO_TypeDef* port, std::uint8_t index)
{
    const std::uint32_t shift = static_cast<std::uint32_t>(index) * 2UL;
    port->MODER = (port->MODER & ~(3UL << shift)) | (1UL << shift);
    port->OTYPER &= ~(1UL << index);
    port->PUPDR &= ~(3UL << shift);
    port->OSPEEDR |= (3UL << shift);
}

inline void configure_analog(GPIO_TypeDef* port, std::uint8_t index)
{
    const std::uint32_t shift = static_cast<std::uint32_t>(index) * 2UL;
    port->MODER |= (3UL << shift);
    port->PUPDR &= ~(3UL << shift);
}

inline void configure_alternate(GPIO_TypeDef* port, std::uint8_t index, std::uint8_t af)
{
    const std::uint32_t shift = static_cast<std::uint32_t>(index) * 2UL;
    port->MODER = (port->MODER & ~(3UL << shift)) | (2UL << shift);
    port->OTYPER &= ~(1UL << index);
    port->PUPDR &= ~(3UL << shift);
    port->OSPEEDR |= (3UL << shift);

    const std::uint8_t afr_index = index < 8U ? 0U : 1U;
    const std::uint8_t afr_shift = static_cast<std::uint8_t>((index & 7U) * 4U);
    port->AFR[afr_index] =
        (port->AFR[afr_index] & ~(0xFUL << afr_shift)) |
        (static_cast<std::uint32_t>(af) << afr_shift);
}

inline void clock_init_hsi48()
{
    RCC->CR2 |= RCC_CR2_HSI48ON;
    while ((RCC->CR2 & RCC_CR2_HSI48RDY) == 0U) {}

    FLASH->ACR |= FLASH_ACR_LATENCY;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI48;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI48) {}

    SystemCoreClock = kSystemClockHz;
}

inline void systick_init_1khz()
{
    SysTick_Config(kSystemClockHz / 1000UL);
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

inline void timebase_init()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->PSC = (kSystemClockHz / kTimerTickHz) - 1UL;
    TIM3->ARR = 0xFFFFU;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_CEN;
}

inline void delay_us(std::uint32_t us)
{
    while (us > 0U) {
        const std::uint32_t chunk = us > 60000U ? 60000U : us;
        TIM3->CNT = 0U;
        while (TIM3->CNT < chunk) {}
        us -= chunk;
    }
}

inline void delay_ms(std::uint32_t ms)
{
    while (ms-- > 0U) {
        delay_us(1000U);
    }
}

inline void configure_safe_chip_selects()
{
    enable_gpio_clocks();
    configure_output(GPIOA, kFlashCsIndex);
    configure_output(GPIOB, kMs5607CsIndex);
    set_pin_high(GPIOA, kFlashCsPin);
    set_pin_high(GPIOB, kMs5607CsPin);
}

} // namespace mu

#endif // MU_PROBE_COMMON_H
