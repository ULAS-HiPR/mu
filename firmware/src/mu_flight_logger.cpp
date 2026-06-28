#include <mu_probe_common.h>

#include <Baro/MS5607.h>
#include <Flash/FlashLogger.h>
#include <Flash/MX25L128.h>
#include <SPI/SPI_Handler.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace {

constexpr std::uint32_t kStatusMagic = 0x4D55464CUL; // MUFL
constexpr std::uint32_t kRecordMagic = 0x4D555245UL; // MURE
constexpr std::uint32_t kFirmwareVersion = 1UL;
constexpr std::uint32_t kLogStart = 0x00000000UL;
constexpr std::uint32_t kLogLength = 0x00100000UL; // 1 MB: enough for ~2 h, faster preflight erase.
constexpr std::uint32_t kSampleIntervalUs = 10UL;
constexpr std::uint32_t kTriggerDeltaRaw = 250UL;
constexpr std::uint32_t kTriggerConsecutiveSamples = 4UL;
constexpr std::uint32_t kWarmupSamples = 2048UL;
constexpr std::uint32_t kWaveformSamples = 64UL;
constexpr std::uint32_t kWaveformPreSamples = 16UL;
constexpr std::uint32_t kPressurePeriodMs = 1000UL;
constexpr std::uint32_t kAdcDmaSamples = 1024UL;
constexpr std::uint32_t kAdcDmaHalfSamples = kAdcDmaSamples / 2UL;
constexpr std::uint32_t kEventQueueDepth = 4UL;

#ifndef MU_ERASE_FLASH_ON_BOOT
constexpr bool kEraseFlashOnBoot = false;
#else
constexpr bool kEraseFlashOnBoot = true;
#endif

enum class MuLogPayloadType : std::uint16_t {
    Boot = 100,
    Pressure = 101,
    Event = 102,
};

enum class CaptureState : std::uint32_t {
    Arming = 0U,
    Armed = 1U,
    Capturing = 2U,
};

struct __attribute__((packed)) MuBootRecord {
    std::uint32_t magic;
    std::uint32_t firmware_version;
    std::uint32_t system_hz;
    std::uint32_t run_id;
    std::uint32_t flash_jedec;
    std::uint32_t log_start;
    std::uint32_t log_length;
    std::uint32_t sample_interval_us;
    std::uint32_t trigger_delta_raw;
    std::uint32_t trigger_consecutive_samples;
    std::uint32_t waveform_samples;
    std::uint32_t waveform_pre_samples;
    std::uint32_t pressure_period_ms;
    std::uint32_t erase_on_boot;
    std::uint32_t baro_ok;
};

struct __attribute__((packed)) MuPressureRecord {
    std::uint32_t magic;
    std::uint32_t pressure_seq;
    std::uint32_t timestamp_ms;
    std::int32_t pressure_pa;
    std::int32_t temperature_centi_c;
    std::int32_t altitude_mm;
    std::uint32_t ok;
};

struct __attribute__((packed)) MuEventRecord {
    std::uint32_t magic;
    std::uint32_t event_seq;
    std::uint32_t timestamp_ms;
    std::uint32_t sample_count;
    std::uint32_t baseline_raw;
    std::uint32_t trigger_raw;
    std::uint32_t min_raw;
    std::uint32_t max_raw;
    std::uint32_t baseline_mv;
    std::uint32_t min_mv;
    std::uint32_t amplitude_mv;
    std::int32_t pressure_pa;
    std::int32_t temperature_centi_c;
    std::int32_t altitude_mm;
    std::uint32_t pressure_age_ms;
    std::uint32_t pressure_ok;
    std::uint32_t waveform_count;
    std::uint32_t waveform_pre_samples;
    std::uint32_t sample_interval_us;
    std::uint16_t samples[kWaveformSamples];
};

static_assert(kWaveformPreSamples < kWaveformSamples,
              "waveform needs pre-trigger and post-trigger samples");
static_assert(sizeof(MuEventRecord) <= 1024U,
              "FlashLogger default payload cap must hold event records");

struct MuFlightStatus {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t system_hz;
    std::uint32_t uptime_ms;
    std::uint32_t loop_count;
    std::uint32_t sample_count;
    std::uint32_t latest_raw;
    std::uint32_t latest_mv;
    std::uint32_t baseline_raw;
    std::uint32_t baseline_mv;
    std::uint32_t state;
    std::uint32_t flash_ok;
    std::uint32_t flash_jedec;
    std::uint32_t logger_ok;
    std::uint32_t logger_status;
    std::uint32_t run_id;
    std::uint32_t records_written;
    std::uint32_t event_count;
    std::uint32_t pressure_count;
    std::uint32_t boot_logged;
    std::uint32_t event_log_failures;
    std::uint32_t pressure_log_failures;
    std::uint32_t baro_ok;
    std::int32_t pressure_pa;
    std::int32_t temperature_centi_c;
    std::int32_t altitude_mm;
    std::uint32_t last_pressure_ms;
    std::uint32_t last_event_ms;
    std::uint32_t last_event_baseline_mv;
    std::uint32_t last_event_min_mv;
    std::uint32_t last_event_amplitude_mv;
    std::uint32_t dma_half_count;
    std::uint32_t dma_full_count;
    std::uint32_t dma_error_count;
    std::uint32_t adc_overrun_count;
    std::uint32_t event_queue_depth;
    std::uint32_t event_queue_max_depth;
    std::uint32_t event_queue_drops;
};

alignas(4) volatile std::uint16_t g_adc_dma_samples[kAdcDmaSamples] = {};

std::uint16_t g_pretrigger_ring[kWaveformPreSamples] = {};
std::uint16_t g_capture_waveform[kWaveformSamples] = {};
MuEventRecord g_event_queue[kEventQueueDepth] = {};

std::uint32_t g_pretrigger_index = 0U;
std::uint32_t g_capture_index = 0U;
std::uint32_t g_below_trigger_count = 0U;
std::uint32_t g_warmup_samples = 0U;
std::uint32_t g_adc_process_index = 0U;
std::int32_t g_baseline_acc = 0;
CaptureState g_capture_state = CaptureState::Arming;

volatile std::uint32_t g_event_queue_head = 0U;
volatile std::uint32_t g_event_queue_tail = 0U;
volatile std::uint32_t g_event_queue_count = 0U;

volatile std::uint32_t g_ms_ticks = 0U;

std::uint32_t millis()
{
    return g_ms_ticks;
}

std::uint32_t raw_to_mv(std::uint32_t raw)
{
    return (raw * 3300UL) / 4095UL;
}

std::int32_t scaled_float_to_i32(float value, float scale)
{
    return static_cast<std::int32_t>(value * scale);
}

void adc_dma_start()
{
    mu::enable_gpio_clocks();
    mu::configure_analog(GPIOA, mu::kCatchOutAdcIndex);

    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->AHBENR;
    (void)RCC->APB2ENR;
    (void)RCC->APB1ENR;

    TIM2->CR1 = 0U;
    ADC1->IER = 0U;

    if ((ADC1->CR & ADC_CR_ADSTART) != 0U) {
        ADC1->CR |= ADC_CR_ADSTP;
        while ((ADC1->CR & ADC_CR_ADSTP) != 0U) {}
    }

    if ((ADC1->CR & ADC_CR_ADEN) != 0U) {
        ADC1->CR |= ADC_CR_ADDIS;
        while ((ADC1->CR & ADC_CR_ADEN) != 0U) {}
    }

    DMA1_Channel1->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF1 |
                 DMA_IFCR_CTCIF1 |
                 DMA_IFCR_CHTIF1 |
                 DMA_IFCR_CTEIF1;
    DMA1_Channel1->CPAR = reinterpret_cast<std::uint32_t>(&ADC1->DR);
    DMA1_Channel1->CMAR = reinterpret_cast<std::uint32_t>(g_adc_dma_samples);
    DMA1_Channel1->CNDTR = kAdcDmaSamples;
    DMA1_Channel1->CCR = DMA_CCR_MINC |
                         DMA_CCR_CIRC |
                         DMA_CCR_PSIZE_0 |
                         DMA_CCR_MSIZE_0 |
                         DMA_CCR_PL_1 |
                         DMA_CCR_HTIE |
                         DMA_CCR_TCIE |
                         DMA_CCR_TEIE;

    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0U) {}

    ADC1->CFGR1 = ADC_CFGR1_DMAEN |
                  ADC_CFGR1_DMACFG |
                  ADC_CFGR1_EXTSEL_1 |
                  ADC_CFGR1_EXTEN_0;
    ADC1->CFGR2 = 0U;
    ADC1->SMPR = ADC_SMPR_SMP_0 | ADC_SMPR_SMP_1;
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;
    ADC1->IER = ADC_IER_OVRIE;

    ADC1->ISR = ADC_ISR_ADRDY | ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1U);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(ADC1_IRQn, 2U);
    NVIC_EnableIRQ(ADC1_IRQn);

    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {}

    TIM2->PSC = (mu::kSystemClockHz / 1000000UL) - 1UL;
    TIM2->ARR = kSampleIntervalUs - 1UL;
    TIM2->CR2 = (TIM2->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;
    TIM2->EGR = TIM_EGR_UG;
    ADC1->CR |= ADC_CR_ADSTART;
    TIM2->CR1 = TIM_CR1_CEN;
}

void update_baseline(std::int32_t& baseline_acc, std::uint32_t raw, std::uint8_t shift)
{
    const std::int32_t target = static_cast<std::int32_t>(raw << 8U);
    baseline_acc += (target - baseline_acc) >> shift;
}

void spi_idle(mu::SpiClockMode mode)
{
    if (mode == mu::SpiClockMode::Mode3) {
        mu::set_pin_high(GPIOA, mu::kSpiSckPin);
    } else {
        mu::set_pin_low(GPIOA, mu::kSpiSckPin);
    }
}

std::uint16_t tx_pin(mu::SpiWireMode wire)
{
    return wire == mu::SpiWireMode::Normal ? mu::kSpiMosiPin : mu::kSpiMisoPin;
}

std::uint16_t rx_pin(mu::SpiWireMode wire)
{
    return wire == mu::SpiWireMode::Normal ? mu::kSpiMisoPin : mu::kSpiMosiPin;
}

std::uint8_t tx_index(mu::SpiWireMode wire)
{
    return wire == mu::SpiWireMode::Normal ? mu::kSpiMosiIndex : mu::kSpiMisoIndex;
}

std::uint8_t rx_index(mu::SpiWireMode wire)
{
    return wire == mu::SpiWireMode::Normal ? mu::kSpiMisoIndex : mu::kSpiMosiIndex;
}

class MuSpiHandler final : public SPI_Handler {
public:
    MuSpiHandler(GPIO_TypeDef* cs_port,
                 std::uint16_t cs_pin,
                 mu::SpiWireMode wire,
                 mu::SpiClockMode mode)
        : cs_port_(cs_port),
          cs_pin_(cs_pin),
          wire_(wire),
          mode_(mode) {}

    void init()
    {
        mu::enable_gpio_clocks();
        mu::configure_output(GPIOA, mu::kSpiSckIndex);
        mu::configure_output(GPIOA, tx_index(wire_));
        mu::configure_input(GPIOA, rx_index(wire_));
        cs_high();
        spi_idle(mode_);
    }

    bool read(int cs, std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        cs_select(cs);
        const std::uint8_t command = static_cast<std::uint8_t>(reg | 0x80U);
        const bool ok = transmit(&command, 1U) && receive(buf, len);
        cs_deselect(cs);
        return ok;
    }

    bool read_no_cs(std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        const std::uint8_t command = static_cast<std::uint8_t>(reg | 0x80U);
        return transmit(&command, 1U) && receive(buf, len);
    }

    bool write(int cs, std::uint8_t reg, std::uint8_t* buf, std::uint16_t len) override
    {
        cs_select(cs);
        const bool ok = write_no_cs(reg, buf, len);
        cs_deselect(cs);
        return ok;
    }

    bool write_no_cs(std::uint8_t reg, std::uint8_t const* buf, std::uint16_t len) override
    {
        const std::uint8_t command = static_cast<std::uint8_t>(reg & 0x7FU);
        return transmit(&command, 1U) && (len == 0U || transmit(buf, len));
    }

    bool transmit(const std::uint8_t* data, std::size_t len) override
    {
        if (data == nullptr && len > 0U) {
            last_error_ = 1U;
            return false;
        }
        for (std::size_t i = 0U; i < len; ++i) {
            (void)transfer_byte(data[i]);
        }
        last_error_ = 0U;
        return true;
    }

    bool receive(std::uint8_t* buf, std::size_t len) override
    {
        if (buf == nullptr && len > 0U) {
            last_error_ = 2U;
            return false;
        }
        for (std::size_t i = 0U; i < len; ++i) {
            buf[i] = transfer_byte(0xFFU);
        }
        last_error_ = 0U;
        return true;
    }

    bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t len) override
    {
        if ((tx == nullptr || rx == nullptr) && len > 0U) {
            last_error_ = 3U;
            return false;
        }
        for (std::size_t i = 0U; i < len; ++i) {
            rx[i] = transfer_byte(tx[i]);
        }
        last_error_ = 0U;
        return true;
    }

    void cs_low() override
    {
        mu::set_pin_low(cs_port_, cs_pin_);
        mu::delay_us(2U);
    }

    void cs_high() override
    {
        mu::set_pin_high(cs_port_, cs_pin_);
        spi_idle(mode_);
        mu::delay_us(2U);
    }

    void cs_select(int) override
    {
        cs_low();
    }

    void cs_deselect(int) override
    {
        cs_high();
    }

    void delay_ms(int ms) override
    {
        if (ms > 0) {
            mu::delay_ms(static_cast<std::uint32_t>(ms));
        }
    }

    std::uint32_t last_error() const override
    {
        return last_error_;
    }

private:
    std::uint8_t transfer_byte(std::uint8_t out)
    {
        std::uint8_t in = 0U;
        const std::uint16_t tx = tx_pin(wire_);
        const std::uint16_t rx = rx_pin(wire_);

        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((out & 0x80U) != 0U) {
                mu::set_pin_high(GPIOA, tx);
            } else {
                mu::set_pin_low(GPIOA, tx);
            }
            out = static_cast<std::uint8_t>(out << 1U);

            if (mode_ == mu::SpiClockMode::Mode3) {
                mu::set_pin_low(GPIOA, mu::kSpiSckPin);
                mu::delay_us(1U);
                mu::set_pin_high(GPIOA, mu::kSpiSckPin);
                mu::delay_us(1U);
            } else {
                mu::set_pin_high(GPIOA, mu::kSpiSckPin);
                mu::delay_us(1U);
            }

            in = static_cast<std::uint8_t>(in << 1U);
            if ((GPIOA->IDR & rx) != 0U) {
                in |= 1U;
            }

            if (mode_ == mu::SpiClockMode::Mode0) {
                mu::set_pin_low(GPIOA, mu::kSpiSckPin);
                mu::delay_us(1U);
            }
        }

        return in;
    }

    GPIO_TypeDef* cs_port_;
    std::uint16_t cs_pin_;
    mu::SpiWireMode wire_;
    mu::SpiClockMode mode_;
    std::uint32_t last_error_{0U};
};

bool append_typed(FlashLogger& logger,
                  const void* payload,
                  std::size_t payload_size,
                  std::uint32_t timestamp_ms,
                  MuLogPayloadType type)
{
    return logger.append_typed(payload,
                               payload_size,
                               timestamp_ms,
                               static_cast<FlashLogPayloadType>(type),
                               kFirmwareVersion);
}

} // namespace

extern "C" {
volatile MuFlightStatus g_mu_flight = {};

void SysTick_Handler()
{
    ++g_ms_ticks;
}
}

namespace {

void update_queue_depth_status()
{
    const std::uint32_t depth = g_event_queue_count;
    g_mu_flight.event_queue_depth = depth;
    if (depth > g_mu_flight.event_queue_max_depth) {
        g_mu_flight.event_queue_max_depth = depth;
    }
}

void enqueue_event(const MuEventRecord& event)
{
    if (g_event_queue_count >= kEventQueueDepth) {
        ++g_mu_flight.event_queue_drops;
        update_queue_depth_status();
        return;
    }

    std::memcpy(&g_event_queue[g_event_queue_tail], &event, sizeof(event));
    g_event_queue_tail = (g_event_queue_tail + 1UL) % kEventQueueDepth;
    ++g_event_queue_count;
    update_queue_depth_status();
}

void finalize_capture(std::uint32_t now, std::uint32_t baseline)
{
    std::uint32_t min_raw = 0xFFFFFFFFUL;
    std::uint32_t max_raw = 0U;
    for (std::uint32_t i = 0U; i < kWaveformSamples; ++i) {
        if (g_capture_waveform[i] < min_raw) {
            min_raw = g_capture_waveform[i];
        }
        if (g_capture_waveform[i] > max_raw) {
            max_raw = g_capture_waveform[i];
        }
    }

    ++g_mu_flight.event_count;
    g_mu_flight.last_event_ms = now;
    g_mu_flight.last_event_baseline_mv = raw_to_mv(baseline);
    g_mu_flight.last_event_min_mv = raw_to_mv(min_raw);
    g_mu_flight.last_event_amplitude_mv =
        raw_to_mv(baseline > min_raw ? baseline - min_raw : 0U);

    MuEventRecord event{};
    event.magic = kRecordMagic;
    event.event_seq = g_mu_flight.event_count;
    event.timestamp_ms = now;
    event.sample_count = g_mu_flight.sample_count;
    event.baseline_raw = baseline;
    event.trigger_raw = g_capture_waveform[kWaveformPreSamples - 1U];
    event.min_raw = min_raw;
    event.max_raw = max_raw;
    event.baseline_mv = raw_to_mv(baseline);
    event.min_mv = raw_to_mv(min_raw);
    event.amplitude_mv = g_mu_flight.last_event_amplitude_mv;
    event.pressure_pa = g_mu_flight.pressure_pa;
    event.temperature_centi_c = g_mu_flight.temperature_centi_c;
    event.altitude_mm = g_mu_flight.altitude_mm;
    event.pressure_age_ms =
        g_mu_flight.last_pressure_ms == 0U ? 0xFFFFFFFFUL : now - g_mu_flight.last_pressure_ms;
    event.pressure_ok = g_mu_flight.baro_ok;
    event.waveform_count = kWaveformSamples;
    event.waveform_pre_samples = kWaveformPreSamples;
    event.sample_interval_us = kSampleIntervalUs;
    std::memcpy(event.samples, g_capture_waveform, sizeof(event.samples));

    enqueue_event(event);
}

void process_sample(std::uint16_t raw)
{
    const std::uint32_t now = millis();

    g_pretrigger_ring[g_pretrigger_index] = raw;
    g_pretrigger_index = (g_pretrigger_index + 1UL) % kWaveformPreSamples;

    if (g_baseline_acc == 0) {
        g_baseline_acc = static_cast<std::int32_t>(raw << 8U);
    }

    const std::uint32_t baseline = static_cast<std::uint32_t>(g_baseline_acc >> 8U);
    const bool near_baseline =
        raw + (kTriggerDeltaRaw / 2UL) >= baseline &&
        raw <= baseline + (kTriggerDeltaRaw * 2UL);

    if (g_warmup_samples < kWarmupSamples) {
        update_baseline(g_baseline_acc, raw, 4U);
        ++g_warmup_samples;
        g_capture_state = CaptureState::Arming;
    } else if (g_capture_state == CaptureState::Arming) {
        g_capture_state = CaptureState::Armed;
    } else if (g_capture_state == CaptureState::Armed && near_baseline) {
        update_baseline(g_baseline_acc, raw, 8U);
    }

    const std::uint32_t updated_baseline =
        static_cast<std::uint32_t>(g_baseline_acc >> 8U);

    g_mu_flight.uptime_ms = now;
    ++g_mu_flight.sample_count;
    g_mu_flight.latest_raw = raw;
    g_mu_flight.latest_mv = raw_to_mv(raw);
    g_mu_flight.baseline_raw = updated_baseline;
    g_mu_flight.baseline_mv = raw_to_mv(updated_baseline);
    g_mu_flight.state = static_cast<std::uint32_t>(g_capture_state);

    if (g_capture_state == CaptureState::Armed &&
        raw + kTriggerDeltaRaw < updated_baseline) {
        ++g_below_trigger_count;
    } else if (g_capture_state == CaptureState::Armed) {
        g_below_trigger_count = 0U;
    }

    if (g_capture_state == CaptureState::Armed &&
        g_below_trigger_count >= kTriggerConsecutiveSamples) {
        for (std::uint32_t i = 0U; i < kWaveformPreSamples; ++i) {
            g_capture_waveform[i] =
                g_pretrigger_ring[(g_pretrigger_index + i) % kWaveformPreSamples];
        }
        g_capture_index = kWaveformPreSamples;
        g_capture_state = CaptureState::Capturing;
        g_below_trigger_count = 0U;
    } else if (g_capture_state == CaptureState::Capturing) {
        if (g_capture_index < kWaveformSamples) {
            g_capture_waveform[g_capture_index++] = raw;
        }

        if (g_capture_index >= kWaveformSamples) {
            finalize_capture(now, updated_baseline);
            g_capture_state = CaptureState::Armed;
        }
    }
}

std::uint32_t adc_dma_write_index()
{
    return (kAdcDmaSamples - DMA1_Channel1->CNDTR) % kAdcDmaSamples;
}

std::uint32_t process_adc_available(std::uint32_t max_samples)
{
    std::uint32_t processed = 0U;
    while (processed < max_samples) {
        const std::uint32_t write_index = adc_dma_write_index();
        if (g_adc_process_index == write_index) {
            break;
        }

        process_sample(
            static_cast<std::uint16_t>(g_adc_dma_samples[g_adc_process_index] & 0x0FFFU));
        g_adc_process_index = (g_adc_process_index + 1UL) % kAdcDmaSamples;
        ++processed;
    }

    return processed;
}

bool dequeue_event(MuEventRecord& event)
{
    if (g_event_queue_count == 0U) {
        return false;
    }

    std::memcpy(&event, &g_event_queue[g_event_queue_head], sizeof(event));
    g_event_queue_head = (g_event_queue_head + 1UL) % kEventQueueDepth;
    --g_event_queue_count;
    update_queue_depth_status();
    return true;
}

} // namespace

extern "C" void DMA1_Channel1_IRQHandler()
{
    const std::uint32_t flags = DMA1->ISR;

    if ((flags & DMA_ISR_HTIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
        ++g_mu_flight.dma_half_count;
    }

    if ((flags & DMA_ISR_TCIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTCIF1;
        ++g_mu_flight.dma_full_count;
    }

    if ((flags & DMA_ISR_TEIF1) != 0U) {
        DMA1->IFCR = DMA_IFCR_CTEIF1;
        ++g_mu_flight.dma_error_count;
    }
}

extern "C" void ADC1_IRQHandler()
{
    const std::uint32_t flags = ADC1->ISR;
    if ((flags & ADC_ISR_OVR) != 0U) {
        ADC1->ISR = ADC_ISR_OVR;
        ++g_mu_flight.adc_overrun_count;
    }
}

int main()
{
    mu::clock_init_hsi48();
    mu::timebase_init();
    mu::systick_init_1khz();
    mu::configure_safe_chip_selects();

    g_mu_flight.magic = kStatusMagic;
    g_mu_flight.version = kFirmwareVersion;
    g_mu_flight.system_hz = SystemCoreClock;
    g_mu_flight.state = static_cast<std::uint32_t>(CaptureState::Arming);

    MuSpiHandler flash_spi(GPIOA,
                           mu::kFlashCsPin,
                           mu::SpiWireMode::Normal,
                           mu::SpiClockMode::Mode0);
    MuSpiHandler baro_spi(GPIOB,
                          mu::kMs5607CsPin,
                          mu::SpiWireMode::Normal,
                          mu::SpiClockMode::Mode0);
    flash_spi.init();
    baro_spi.init();

    MX25L128 flash(flash_spi);
    const bool flash_ok = flash.init();
    g_mu_flight.flash_ok = flash_ok ? 1U : 0U;
    g_mu_flight.flash_jedec = flash.jedec_id();

    MS5607 baro(baro_spi);
    const bool baro_init_ok = baro.init();
    g_mu_flight.baro_ok = baro_init_ok ? 1U : 0U;

    FlashLogger logger(flash);
    FlashLogConfig config = FlashLogger::default_mx25_config(kLogStart, kLogLength);
    config.max_payload_size = sizeof(MuEventRecord);
    config.verify_writes = false;

    bool logger_ok = false;
    if (flash_ok) {
        const bool begin_ok = logger.begin(config);
        if (kEraseFlashOnBoot) {
            logger_ok = logger.erase_all() && logger.begin(config);
        } else {
            logger_ok = begin_ok;
        }
    }

    g_mu_flight.logger_ok = logger_ok ? 1U : 0U;
    g_mu_flight.logger_status = static_cast<std::uint32_t>(logger.status());
    if (logger_ok) {
        const FlashLogInfo info = logger.info();
        g_mu_flight.run_id = info.run_id;
        g_mu_flight.records_written = info.record_count;
    }

    if (logger_ok) {
        MuBootRecord boot{};
        boot.magic = kRecordMagic;
        boot.firmware_version = kFirmwareVersion;
        boot.system_hz = SystemCoreClock;
        boot.run_id = g_mu_flight.run_id;
        boot.flash_jedec = g_mu_flight.flash_jedec;
        boot.log_start = kLogStart;
        boot.log_length = kLogLength;
        boot.sample_interval_us = kSampleIntervalUs;
        boot.trigger_delta_raw = kTriggerDeltaRaw;
        boot.trigger_consecutive_samples = kTriggerConsecutiveSamples;
        boot.waveform_samples = kWaveformSamples;
        boot.waveform_pre_samples = kWaveformPreSamples;
        boot.pressure_period_ms = kPressurePeriodMs;
        boot.erase_on_boot = kEraseFlashOnBoot ? 1U : 0U;
        boot.baro_ok = baro_init_ok ? 1U : 0U;
        if (append_typed(logger, &boot, sizeof(boot), millis(), MuLogPayloadType::Boot)) {
            g_mu_flight.boot_logged = 1U;
            g_mu_flight.records_written = logger.info().record_count;
        }
    }

    baro_data latest_baro{};
    std::uint32_t last_pressure_ms = 0U;
    if (baro_init_ok && baro.update(&latest_baro)) {
        g_mu_flight.pressure_pa = latest_baro.pressure;
        g_mu_flight.temperature_centi_c =
            scaled_float_to_i32(latest_baro.temperature, 100.0f);
        g_mu_flight.altitude_mm = scaled_float_to_i32(latest_baro.altitude, 1000.0f);
        last_pressure_ms = millis();
        g_mu_flight.last_pressure_ms = last_pressure_ms;
    }

    adc_dma_start();

    MuEventRecord event{};
    while (true) {
        (void)process_adc_available(kAdcDmaSamples);

        const std::uint32_t now = millis();
        g_mu_flight.uptime_ms = now;
        ++g_mu_flight.loop_count;

        if (dequeue_event(event) && logger_ok) {
            if (!append_typed(logger,
                              &event,
                              sizeof(event),
                              event.timestamp_ms,
                              MuLogPayloadType::Event)) {
                ++g_mu_flight.event_log_failures;
                g_mu_flight.logger_status =
                    static_cast<std::uint32_t>(logger.status());
            } else {
                g_mu_flight.records_written = logger.info().record_count;
            }
            (void)process_adc_available(kAdcDmaSamples);
        }

        if (baro_init_ok && (now - last_pressure_ms) >= kPressurePeriodMs) {
            const bool pressure_ok = baro.update(&latest_baro);
            g_mu_flight.baro_ok = pressure_ok ? 1U : 0U;
            last_pressure_ms = now;
            g_mu_flight.last_pressure_ms = now;

            if (pressure_ok) {
                g_mu_flight.pressure_pa = latest_baro.pressure;
                g_mu_flight.temperature_centi_c =
                    scaled_float_to_i32(latest_baro.temperature, 100.0f);
                g_mu_flight.altitude_mm =
                    scaled_float_to_i32(latest_baro.altitude, 1000.0f);
            }

            ++g_mu_flight.pressure_count;
            if (logger_ok) {
                MuPressureRecord pressure{};
                pressure.magic = kRecordMagic;
                pressure.pressure_seq = g_mu_flight.pressure_count;
                pressure.timestamp_ms = now;
                pressure.pressure_pa = g_mu_flight.pressure_pa;
                pressure.temperature_centi_c = g_mu_flight.temperature_centi_c;
                pressure.altitude_mm = g_mu_flight.altitude_mm;
                pressure.ok = pressure_ok ? 1U : 0U;

                if (!append_typed(logger,
                                  &pressure,
                                  sizeof(pressure),
                                  now,
                                  MuLogPayloadType::Pressure)) {
                    ++g_mu_flight.pressure_log_failures;
                    g_mu_flight.logger_status =
                        static_cast<std::uint32_t>(logger.status());
                } else {
                    g_mu_flight.records_written = logger.info().record_count;
                }
            }
            (void)process_adc_available(kAdcDmaSamples);
        }
    }
}
