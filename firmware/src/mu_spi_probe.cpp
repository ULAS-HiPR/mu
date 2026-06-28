#include <mu_probe_common.h>

#include <cstdint>

namespace {

constexpr std::uint32_t kMagic = 0x4D555350UL; // MUSP
constexpr std::uint32_t kFlashSize = 0x1000000UL;
constexpr std::uint32_t kFlashTestAddress = kFlashSize - 0x1000UL;
constexpr std::uint32_t kW25q128Jedec = 0xEF4018UL;
constexpr std::uint32_t kMx25l128Jedec = 0xC22018UL;

struct MuSpiProbe {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t system_hz;
    std::uint32_t loop_count;
    std::uint32_t flash_jedec_normal_mode0;
    std::uint32_t flash_jedec_normal_mode3;
    std::uint32_t flash_jedec_swapped_mode0;
    std::uint32_t flash_jedec_swapped_mode3;
    std::uint32_t flash_selected_wire;
    std::uint32_t flash_selected_mode;
    std::uint32_t flash_status_before;
    std::uint32_t flash_status_after;
    std::uint32_t flash_erase_ok;
    std::uint32_t flash_write_ok;
    std::uint32_t flash_read_ok;
    std::uint32_t flash_verify_ok;
    std::uint8_t flash_expected[16];
    std::uint8_t flash_readback[16];
    std::uint16_t ms5607_prom_mode0[8];
    std::uint16_t ms5607_prom_mode3[8];
    std::uint32_t ms5607_crc_read_mode0;
    std::uint32_t ms5607_crc_calc_mode0;
    std::uint32_t ms5607_ok_mode0;
    std::uint32_t ms5607_crc_read_mode3;
    std::uint32_t ms5607_crc_calc_mode3;
    std::uint32_t ms5607_ok_mode3;
    std::uint16_t ms5607_prom_swapped_mode0[8];
    std::uint16_t ms5607_prom_swapped_mode3[8];
    std::uint32_t ms5607_crc_read_swapped_mode0;
    std::uint32_t ms5607_crc_calc_swapped_mode0;
    std::uint32_t ms5607_ok_swapped_mode0;
    std::uint32_t ms5607_crc_read_swapped_mode3;
    std::uint32_t ms5607_crc_calc_swapped_mode3;
    std::uint32_t ms5607_ok_swapped_mode3;
};

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

void spi_config(mu::SpiWireMode wire, mu::SpiClockMode mode)
{
    mu::configure_output(GPIOA, mu::kSpiSckIndex);
    mu::configure_output(GPIOA, tx_index(wire));
    mu::configure_input(GPIOA, rx_index(wire));
    spi_idle(mode);
}

void cs_all_high()
{
    mu::set_pin_high(GPIOA, mu::kFlashCsPin);
    mu::set_pin_high(GPIOB, mu::kMs5607CsPin);
}

std::uint8_t spi_transfer(mu::SpiWireMode wire, mu::SpiClockMode mode,
                          std::uint8_t out)
{
    std::uint8_t in = 0U;
    const std::uint16_t tx = tx_pin(wire);
    const std::uint16_t rx = rx_pin(wire);

    for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
        if ((out & 0x80U) != 0U) {
            mu::set_pin_high(GPIOA, tx);
        } else {
            mu::set_pin_low(GPIOA, tx);
        }
        out = static_cast<std::uint8_t>(out << 1U);

        if (mode == mu::SpiClockMode::Mode3) {
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

        if (mode == mu::SpiClockMode::Mode0) {
            mu::set_pin_low(GPIOA, mu::kSpiSckPin);
            mu::delay_us(1U);
        }
    }

    return in;
}

void flash_select()
{
    cs_all_high();
    mu::delay_us(10U);
    mu::set_pin_low(GPIOA, mu::kFlashCsPin);
    mu::delay_us(10U);
}

void flash_deselect(mu::SpiClockMode mode)
{
    mu::delay_us(10U);
    mu::set_pin_high(GPIOA, mu::kFlashCsPin);
    spi_idle(mode);
    mu::delay_us(10U);
}

void ms5607_select()
{
    cs_all_high();
    mu::delay_us(10U);
    mu::set_pin_low(GPIOB, mu::kMs5607CsPin);
    mu::delay_us(10U);
}

void ms5607_deselect(mu::SpiClockMode mode)
{
    mu::delay_us(10U);
    mu::set_pin_high(GPIOB, mu::kMs5607CsPin);
    spi_idle(mode);
    mu::delay_us(10U);
}

void flash_command(mu::SpiWireMode wire, mu::SpiClockMode mode, std::uint8_t cmd)
{
    spi_config(wire, mode);
    flash_select();
    (void)spi_transfer(wire, mode, cmd);
    flash_deselect(mode);
}

std::uint8_t flash_read_status(mu::SpiWireMode wire, mu::SpiClockMode mode)
{
    spi_config(wire, mode);
    flash_select();
    (void)spi_transfer(wire, mode, 0x05U);
    const std::uint8_t status = spi_transfer(wire, mode, 0xFFU);
    flash_deselect(mode);
    return status;
}

bool flash_wait_ready(mu::SpiWireMode wire, mu::SpiClockMode mode,
                      std::uint32_t timeout_ms)
{
    while (timeout_ms-- > 0U) {
        if ((flash_read_status(wire, mode) & 0x01U) == 0U) {
            return true;
        }
        mu::delay_ms(1U);
    }
    return false;
}

std::uint32_t flash_read_jedec(mu::SpiWireMode wire, mu::SpiClockMode mode)
{
    flash_command(wire, mode, 0xABU);
    mu::delay_us(50U);

    spi_config(wire, mode);
    flash_select();
    (void)spi_transfer(wire, mode, 0x9FU);
    const std::uint32_t b0 = spi_transfer(wire, mode, 0xFFU);
    const std::uint32_t b1 = spi_transfer(wire, mode, 0xFFU);
    const std::uint32_t b2 = spi_transfer(wire, mode, 0xFFU);
    flash_deselect(mode);
    return (b0 << 16U) | (b1 << 8U) | b2;
}

bool valid_flash_id(std::uint32_t id)
{
    return id == kW25q128Jedec || id == kMx25l128Jedec;
}

void flash_send_address(mu::SpiWireMode wire, mu::SpiClockMode mode,
                        std::uint32_t address)
{
    (void)spi_transfer(wire, mode, static_cast<std::uint8_t>(address >> 16U));
    (void)spi_transfer(wire, mode, static_cast<std::uint8_t>(address >> 8U));
    (void)spi_transfer(wire, mode, static_cast<std::uint8_t>(address));
}

bool flash_write_enable(mu::SpiWireMode wire, mu::SpiClockMode mode)
{
    flash_command(wire, mode, 0x06U);
    const std::uint8_t status = flash_read_status(wire, mode);
    return (status & 0x02U) != 0U;
}

bool flash_sector_erase(mu::SpiWireMode wire, mu::SpiClockMode mode,
                        std::uint32_t address)
{
    if (!flash_write_enable(wire, mode)) {
        return false;
    }
    spi_config(wire, mode);
    flash_select();
    (void)spi_transfer(wire, mode, 0x20U);
    flash_send_address(wire, mode, address);
    flash_deselect(mode);
    return flash_wait_ready(wire, mode, 800U);
}

bool flash_page_program(mu::SpiWireMode wire, mu::SpiClockMode mode,
                        std::uint32_t address, const std::uint8_t* data,
                        std::uint32_t len)
{
    if (!flash_write_enable(wire, mode)) {
        return false;
    }
    spi_config(wire, mode);
    flash_select();
    (void)spi_transfer(wire, mode, 0x02U);
    flash_send_address(wire, mode, address);
    for (std::uint32_t i = 0U; i < len; ++i) {
        (void)spi_transfer(wire, mode, data[i]);
    }
    flash_deselect(mode);
    return flash_wait_ready(wire, mode, 20U);
}

bool flash_read(mu::SpiWireMode wire, mu::SpiClockMode mode, std::uint32_t address,
                std::uint8_t* data, std::uint32_t len)
{
    spi_config(wire, mode);
    flash_select();
    (void)spi_transfer(wire, mode, 0x03U);
    flash_send_address(wire, mode, address);
    for (std::uint32_t i = 0U; i < len; ++i) {
        data[i] = spi_transfer(wire, mode, 0xFFU);
    }
    flash_deselect(mode);
    return true;
}

void ms5607_command(mu::SpiWireMode wire, mu::SpiClockMode mode, std::uint8_t command)
{
    spi_config(wire, mode);
    ms5607_select();
    (void)spi_transfer(wire, mode, command);
    ms5607_deselect(mode);
}

std::uint16_t ms5607_read_prom_word(mu::SpiWireMode wire, mu::SpiClockMode mode,
                                    std::uint8_t index)
{
    spi_config(wire, mode);
    ms5607_select();
    (void)spi_transfer(wire, mode,
                       static_cast<std::uint8_t>(0xA0U + (index * 2U)));
    const std::uint16_t high = spi_transfer(wire, mode, 0xFFU);
    const std::uint16_t low = spi_transfer(wire, mode, 0xFFU);
    ms5607_deselect(mode);
    return static_cast<std::uint16_t>((high << 8U) | low);
}

std::uint8_t ms5607_crc4(const std::uint16_t* prom)
{
    std::uint16_t local[8];
    for (std::uint8_t i = 0U; i < 8U; ++i) {
        local[i] = prom[i];
    }

    local[7] = static_cast<std::uint16_t>(local[7] & 0xFF00U);
    std::uint16_t rem = 0U;
    for (std::uint8_t cnt = 0U; cnt < 16U; ++cnt) {
        if ((cnt & 1U) != 0U) {
            rem ^= static_cast<std::uint16_t>(local[cnt >> 1U] & 0x00FFU);
        } else {
            rem ^= static_cast<std::uint16_t>(local[cnt >> 1U] >> 8U);
        }
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((rem & 0x8000U) != 0U) {
                rem = static_cast<std::uint16_t>((rem << 1U) ^ 0x3000U);
            } else {
                rem = static_cast<std::uint16_t>(rem << 1U);
            }
        }
    }
    return static_cast<std::uint8_t>((rem >> 12U) & 0x0FU);
}

bool prom_has_signal(const std::uint16_t* prom)
{
    std::uint16_t ored = 0U;
    std::uint16_t anded = 0xFFFFU;
    for (std::uint8_t i = 0U; i < 8U; ++i) {
        ored |= prom[i];
        anded &= prom[i];
    }
    return ored != 0U && anded != 0xFFFFU;
}

void read_ms5607(mu::SpiWireMode wire, mu::SpiClockMode mode,
                 volatile std::uint16_t* out,
                 volatile std::uint32_t& crc_read,
                 volatile std::uint32_t& crc_calc,
                 volatile std::uint32_t& ok)
{
    std::uint16_t prom[8]{};
    ms5607_command(wire, mode, 0x1EU);
    mu::delay_ms(5U);

    for (std::uint8_t i = 0U; i < 8U; ++i) {
        prom[i] = ms5607_read_prom_word(wire, mode, i);
        out[i] = prom[i];
    }

    crc_read = prom[7] & 0x0FU;
    crc_calc = ms5607_crc4(prom);
    ok = prom_has_signal(prom) && crc_read == crc_calc ? 1U : 0U;
}

void gpio_init()
{
    mu::configure_safe_chip_selects();
    mu::configure_output(GPIOA, mu::kSpiSckIndex);
    mu::configure_input(GPIOA, mu::kSpiMisoIndex);
    mu::configure_input(GPIOA, mu::kSpiMosiIndex);
    cs_all_high();
}

} // namespace

extern "C" {
volatile MuSpiProbe g_mu_spi_probe = {};
}

int main()
{
    mu::clock_init_hsi48();
    mu::timebase_init();
    gpio_init();

    g_mu_spi_probe.magic = kMagic;
    g_mu_spi_probe.version = 1U;
    g_mu_spi_probe.system_hz = SystemCoreClock;

    g_mu_spi_probe.flash_jedec_normal_mode0 =
        flash_read_jedec(mu::SpiWireMode::Normal, mu::SpiClockMode::Mode0);
    g_mu_spi_probe.flash_jedec_normal_mode3 =
        flash_read_jedec(mu::SpiWireMode::Normal, mu::SpiClockMode::Mode3);
    g_mu_spi_probe.flash_jedec_swapped_mode0 =
        flash_read_jedec(mu::SpiWireMode::Swapped, mu::SpiClockMode::Mode0);
    g_mu_spi_probe.flash_jedec_swapped_mode3 =
        flash_read_jedec(mu::SpiWireMode::Swapped, mu::SpiClockMode::Mode3);

    mu::SpiWireMode flash_wire = mu::SpiWireMode::Normal;
    mu::SpiClockMode flash_mode = mu::SpiClockMode::Mode0;
    bool flash_found = false;
    if (valid_flash_id(g_mu_spi_probe.flash_jedec_normal_mode0)) {
        flash_found = true;
    } else if (valid_flash_id(g_mu_spi_probe.flash_jedec_normal_mode3)) {
        flash_found = true;
        flash_mode = mu::SpiClockMode::Mode3;
    } else if (valid_flash_id(g_mu_spi_probe.flash_jedec_swapped_mode0)) {
        flash_found = true;
        flash_wire = mu::SpiWireMode::Swapped;
    } else if (valid_flash_id(g_mu_spi_probe.flash_jedec_swapped_mode3)) {
        flash_found = true;
        flash_wire = mu::SpiWireMode::Swapped;
        flash_mode = mu::SpiClockMode::Mode3;
    }

    g_mu_spi_probe.flash_selected_wire =
        flash_wire == mu::SpiWireMode::Normal ? 0U : 1U;
    g_mu_spi_probe.flash_selected_mode =
        flash_mode == mu::SpiClockMode::Mode0 ? 0U : 3U;

    const std::uint8_t pattern[16] = {
        0x4DU, 0x55U, 0x2DU, 0x46U, 0x30U, 0x34U, 0x32U, 0x2DU,
        0x50U, 0x52U, 0x4FU, 0x42U, 0x45U, 0x21U, 0x0DU, 0x0AU,
    };
    for (std::uint32_t i = 0U; i < sizeof(pattern); ++i) {
        g_mu_spi_probe.flash_expected[i] = pattern[i];
    }

    if (flash_found) {
        g_mu_spi_probe.flash_status_before = flash_read_status(flash_wire, flash_mode);
        g_mu_spi_probe.flash_erase_ok =
            flash_sector_erase(flash_wire, flash_mode, kFlashTestAddress) ? 1U : 0U;
        if (g_mu_spi_probe.flash_erase_ok != 0U) {
            g_mu_spi_probe.flash_write_ok =
                flash_page_program(flash_wire, flash_mode, kFlashTestAddress,
                                   pattern, sizeof(pattern)) ? 1U : 0U;
        }
        if (g_mu_spi_probe.flash_write_ok != 0U) {
            g_mu_spi_probe.flash_read_ok =
                flash_read(flash_wire, flash_mode, kFlashTestAddress,
                           const_cast<std::uint8_t*>(g_mu_spi_probe.flash_readback),
                           sizeof(pattern)) ? 1U : 0U;
        }
        if (g_mu_spi_probe.flash_read_ok != 0U) {
            g_mu_spi_probe.flash_verify_ok = 1U;
            for (std::uint32_t i = 0U; i < sizeof(pattern); ++i) {
                if (g_mu_spi_probe.flash_readback[i] != pattern[i]) {
                    g_mu_spi_probe.flash_verify_ok = 0U;
                }
            }
        }
        g_mu_spi_probe.flash_status_after = flash_read_status(flash_wire, flash_mode);
    }

    read_ms5607(mu::SpiWireMode::Normal, mu::SpiClockMode::Mode0,
                g_mu_spi_probe.ms5607_prom_mode0,
                g_mu_spi_probe.ms5607_crc_read_mode0,
                g_mu_spi_probe.ms5607_crc_calc_mode0,
                g_mu_spi_probe.ms5607_ok_mode0);
    read_ms5607(mu::SpiWireMode::Normal, mu::SpiClockMode::Mode3,
                g_mu_spi_probe.ms5607_prom_mode3,
                g_mu_spi_probe.ms5607_crc_read_mode3,
                g_mu_spi_probe.ms5607_crc_calc_mode3,
                g_mu_spi_probe.ms5607_ok_mode3);
    read_ms5607(mu::SpiWireMode::Swapped, mu::SpiClockMode::Mode0,
                g_mu_spi_probe.ms5607_prom_swapped_mode0,
                g_mu_spi_probe.ms5607_crc_read_swapped_mode0,
                g_mu_spi_probe.ms5607_crc_calc_swapped_mode0,
                g_mu_spi_probe.ms5607_ok_swapped_mode0);
    read_ms5607(mu::SpiWireMode::Swapped, mu::SpiClockMode::Mode3,
                g_mu_spi_probe.ms5607_prom_swapped_mode3,
                g_mu_spi_probe.ms5607_crc_read_swapped_mode3,
                g_mu_spi_probe.ms5607_crc_calc_swapped_mode3,
                g_mu_spi_probe.ms5607_ok_swapped_mode3);

    while (true) {
        ++g_mu_spi_probe.loop_count;
        __NOP();
    }
}
