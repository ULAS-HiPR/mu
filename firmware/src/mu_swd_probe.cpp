#include <mu_probe_common.h>

#include <cstdint>

namespace {

constexpr std::uint32_t kMagic = 0x4D555331UL; // MUS1

struct MuSwdProbe {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t system_hz;
    std::uint32_t flash_kb;
    std::uint32_t uid[3];
    std::uint32_t gpio_ready;
    std::uint32_t loop_count;
};

} // namespace

extern "C" {
volatile MuSwdProbe g_mu_swd_probe = {};
}

int main()
{
    mu::clock_init_hsi48();
    mu::configure_safe_chip_selects();

    g_mu_swd_probe.magic = kMagic;
    g_mu_swd_probe.version = 1U;
    g_mu_swd_probe.system_hz = SystemCoreClock;
    g_mu_swd_probe.flash_kb =
        *reinterpret_cast<volatile const std::uint16_t*>(FLASHSIZE_BASE);
    g_mu_swd_probe.uid[0] = *reinterpret_cast<volatile const std::uint32_t*>(UID_BASE);
    g_mu_swd_probe.uid[1] = *reinterpret_cast<volatile const std::uint32_t*>(UID_BASE + 4U);
    g_mu_swd_probe.uid[2] = *reinterpret_cast<volatile const std::uint32_t*>(UID_BASE + 8U);
    g_mu_swd_probe.gpio_ready =
        ((GPIOA->ODR & mu::kFlashCsPin) != 0U ? 1U : 0U) |
        ((GPIOB->ODR & mu::kMs5607CsPin) != 0U ? 2U : 0U);

    while (true) {
        ++g_mu_swd_probe.loop_count;
        __NOP();
    }
}
