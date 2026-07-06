#include <mu_probe_common.h>

#include <cstdint>

namespace {

constexpr std::uint32_t kMagic = 0x4D555055UL; // MUPU
constexpr std::uint32_t kVrefMv = 3300UL;
constexpr std::uint32_t kSampleIntervalUs = 10UL;
constexpr std::uint32_t kPreSamples = 256UL;
constexpr std::uint32_t kPostSamples = 768UL;
constexpr std::uint32_t kTotalSamples = kPreSamples + kPostSamples;
constexpr std::uint32_t kTriggerDeltaRaw = 32UL; // About 26 mV at 3.3 V.
constexpr std::uint32_t kTriggerConsecutiveSamples = 1UL;
constexpr std::uint32_t kWarmupSamples = 2048UL;

enum CaptureState : std::uint32_t {
    Arming = 0U,
    Armed = 1U,
    CapturingPost = 2U,
    Captured = 3U,
};

struct MuPulseCapture {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t system_hz;
    std::uint32_t sample_interval_us;
    std::uint32_t pre_samples;
    std::uint32_t post_samples;
    std::uint32_t total_samples;
    std::uint32_t command;
    std::uint32_t state;
    std::uint32_t ready;
    std::uint32_t event_seq;
    std::uint32_t loop_count;
    std::uint32_t sample_count;
    std::uint32_t latest_raw;
    std::uint32_t latest_mv;
    std::uint32_t baseline_raw;
    std::uint32_t baseline_mv;
    std::uint32_t trigger_delta_raw;
    std::uint32_t trigger_raw;
    std::uint32_t trigger_mv;
    std::uint32_t trigger_baseline_raw;
    std::uint32_t trigger_baseline_mv;
    std::uint32_t trigger_sample_count;
    std::uint32_t min_raw;
    std::uint32_t max_raw;
};

static_assert((kPreSamples & (kPreSamples - 1UL)) == 0UL, "pre-trigger ring must be a power of 2");

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
    ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1; // 28.5 ADC cycles; PA0 is op-amp driven.
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

void wait_until_next_sample(std::uint16_t& next_tick)
{
    next_tick = static_cast<std::uint16_t>(next_tick + kSampleIntervalUs);
    while (static_cast<std::int16_t>(TIM3->CNT - next_tick) < 0) {}
}

void update_baseline(std::int32_t& baseline_acc, std::uint32_t raw, std::uint8_t shift)
{
    const std::int32_t target = static_cast<std::int32_t>(raw << 8U);
    baseline_acc += (target - baseline_acc) >> shift;
}

} // namespace

extern "C" {
volatile MuPulseCapture g_mu_pulse = {};
volatile std::uint16_t g_mu_pulse_samples[kTotalSamples] = {};
}

int main()
{
    mu::clock_init_hsi48();
    mu::timebase_init();
    mu::configure_safe_chip_selects();
    adc_init();

    g_mu_pulse.magic = kMagic;
    g_mu_pulse.version = 1U;
    g_mu_pulse.system_hz = SystemCoreClock;
    g_mu_pulse.sample_interval_us = kSampleIntervalUs;
    g_mu_pulse.pre_samples = kPreSamples;
    g_mu_pulse.post_samples = kPostSamples;
    g_mu_pulse.total_samples = kTotalSamples;
    g_mu_pulse.trigger_delta_raw = kTriggerDeltaRaw;
    g_mu_pulse.min_raw = 0xFFFFFFFFUL;
    g_mu_pulse.state = Arming;

    std::uint16_t ring[kPreSamples] = {};
    std::uint32_t ring_index = 0U;
    std::uint32_t post_index = 0U;
    std::uint32_t warmup = 0U;
    std::uint32_t below_trigger_count = 0U;
    std::int32_t baseline_acc = 0;
    std::uint16_t next_tick = TIM3->CNT;

    while (true) {
        wait_until_next_sample(next_tick);

        const std::uint32_t raw = adc_read();
        ring[ring_index] = static_cast<std::uint16_t>(raw);
        ring_index = (ring_index + 1UL) & (kPreSamples - 1UL);

        g_mu_pulse.latest_raw = raw;
        g_mu_pulse.latest_mv = raw_to_mv(raw);
        ++g_mu_pulse.sample_count;
        ++g_mu_pulse.loop_count;

        if (raw < g_mu_pulse.min_raw) {
            g_mu_pulse.min_raw = raw;
        }
        if (raw > g_mu_pulse.max_raw) {
            g_mu_pulse.max_raw = raw;
        }

        if (baseline_acc == 0U) {
            baseline_acc = static_cast<std::int32_t>(raw << 8U);
        }

        const std::uint32_t baseline = static_cast<std::uint32_t>(baseline_acc >> 8U);
        const bool near_baseline =
            raw + (kTriggerDeltaRaw / 2UL) >= baseline &&
            raw <= baseline + (kTriggerDeltaRaw * 2UL);

        if (warmup < kWarmupSamples) {
            update_baseline(baseline_acc, raw, 4U);
            ++warmup;
            g_mu_pulse.state = Arming;
        } else if (g_mu_pulse.state == Armed && near_baseline) {
            update_baseline(baseline_acc, raw, 8U);
        }

        const std::uint32_t updated_baseline = static_cast<std::uint32_t>(baseline_acc >> 8U);
        g_mu_pulse.baseline_raw = updated_baseline;
        g_mu_pulse.baseline_mv = raw_to_mv(updated_baseline);

        if (g_mu_pulse.command == 1U) {
            g_mu_pulse.command = 0U;
            g_mu_pulse.ready = 0U;
            g_mu_pulse.state = Armed;
            g_mu_pulse.min_raw = raw;
            g_mu_pulse.max_raw = raw;
            warmup = kWarmupSamples;
        }

        if (warmup >= kWarmupSamples && g_mu_pulse.state == Arming) {
            g_mu_pulse.state = Armed;
        }

        if (g_mu_pulse.state == Armed &&
            (raw + kTriggerDeltaRaw < updated_baseline ||
             raw > updated_baseline + kTriggerDeltaRaw)) {
            ++below_trigger_count;
        } else if (g_mu_pulse.state == Armed) {
            below_trigger_count = 0U;
        }

        if (g_mu_pulse.state == Armed && below_trigger_count >= kTriggerConsecutiveSamples) {
            for (std::uint32_t i = 0U; i < kPreSamples; ++i) {
                g_mu_pulse_samples[i] =
                    ring[(ring_index + i) & (kPreSamples - 1UL)];
            }

            post_index = kPreSamples;
            g_mu_pulse.trigger_raw = raw;
            g_mu_pulse.trigger_mv = raw_to_mv(raw);
            g_mu_pulse.trigger_baseline_raw = updated_baseline;
            g_mu_pulse.trigger_baseline_mv = raw_to_mv(updated_baseline);
            g_mu_pulse.trigger_sample_count = g_mu_pulse.sample_count;
            g_mu_pulse.state = CapturingPost;
            below_trigger_count = 0U;
        } else if (g_mu_pulse.state == CapturingPost) {
            if (post_index < kTotalSamples) {
                g_mu_pulse_samples[post_index] = static_cast<std::uint16_t>(raw);
                ++post_index;
            }

            if (post_index >= kTotalSamples) {
                ++g_mu_pulse.event_seq;
                g_mu_pulse.ready = 1U;
                g_mu_pulse.state = Captured;
            }
        }
    }
}
