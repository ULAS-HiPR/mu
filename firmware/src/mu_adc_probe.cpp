#include <mu_probe_common.h>

#include <cstdint>

namespace {

constexpr std::uint32_t kMagic = 0x4D554144UL; // MUAD
constexpr std::uint32_t kVrefMv = 3300UL;

struct MuAdcProbe {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t system_hz;
    std::uint32_t loop_count;
    std::uint32_t sample_count;
    std::uint32_t latest_raw;
    std::uint32_t latest_mv;
    std::uint32_t min_raw;
    std::uint32_t max_raw;
    std::uint32_t average_raw;
    std::uint32_t average_mv;
    std::uint32_t baseline_low_mv;
    std::uint32_t baseline_high_mv;
    std::uint32_t baseline_ok;
};

void adc_init()
{
    mu::enable_gpio_clocks();
    mu::configure_analog(GPIOA, mu::kCatchOutAdcIndex);

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    if ((ADC1->CR & ADC_CR_ADEN) != 0U) {
        ADC1->CR |= ADC_CR_ADDIS;
        while ((ADC1->CR & ADC_CR_ADEN) != 0U) {}
    }

    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0U) {}

    ADC1->CFGR1 = 0U;
    ADC1->CFGR2 = 0U;
    ADC1->SMPR = ADC_SMPR_SMP;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;

    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}
}

std::uint16_t adc_read()
{
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {}
    return static_cast<std::uint16_t>(ADC1->DR & 0x0FFFU);
}

std::uint32_t raw_to_mv(std::uint32_t raw)
{
    return (raw * kVrefMv) / 4095UL;
}

} // namespace

extern "C" {
volatile MuAdcProbe g_mu_adc_probe = {};
}

int main()
{
    mu::clock_init_hsi48();
    mu::timebase_init();
    mu::configure_safe_chip_selects();
    adc_init();

    g_mu_adc_probe.magic = kMagic;
    g_mu_adc_probe.version = 1U;
    g_mu_adc_probe.system_hz = SystemCoreClock;
    g_mu_adc_probe.min_raw = 0xFFFFFFFFUL;
    g_mu_adc_probe.baseline_low_mv = 2200UL;
    g_mu_adc_probe.baseline_high_mv = 2800UL;

    std::uint32_t sum = 0U;
    while (true) {
        const std::uint32_t raw = adc_read();
        g_mu_adc_probe.latest_raw = raw;
        g_mu_adc_probe.latest_mv = raw_to_mv(raw);
        if (raw < g_mu_adc_probe.min_raw) {
            g_mu_adc_probe.min_raw = raw;
        }
        if (raw > g_mu_adc_probe.max_raw) {
            g_mu_adc_probe.max_raw = raw;
        }

        sum += raw;
        ++g_mu_adc_probe.sample_count;
        if ((g_mu_adc_probe.sample_count & 0x3FUL) == 0U) {
            g_mu_adc_probe.average_raw = sum / 64UL;
            g_mu_adc_probe.average_mv = raw_to_mv(g_mu_adc_probe.average_raw);
            g_mu_adc_probe.baseline_ok =
                g_mu_adc_probe.average_mv >= g_mu_adc_probe.baseline_low_mv &&
                g_mu_adc_probe.average_mv <= g_mu_adc_probe.baseline_high_mv ? 1U : 0U;
            sum = 0U;
        }

        ++g_mu_adc_probe.loop_count;
        mu::delay_us(500U);
    }
}
