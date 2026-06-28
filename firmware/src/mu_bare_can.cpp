#include "stm32f0xx_hal.h"

#include <mu_probe_common.h>

#include <cstdint>

namespace {

constexpr std::uint32_t kMagic = 0x4D554341UL; // MUCA
constexpr std::uint32_t kCanBitrate = 500000UL;
constexpr std::uint32_t kTxPeriodMs = 250UL;
constexpr std::uint32_t kMaxPeers = 8UL;

CAN_HandleTypeDef hcan;

struct MuCanProbe {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t system_hz;
    std::uint32_t pclk1_hz;
    std::uint32_t can_bitrate;
    std::uint32_t uid[3];
    std::uint32_t node_id;
    std::uint32_t can_init_status;
    std::uint32_t filter_status;
    std::uint32_t start_status;
    std::uint32_t loop_count;
    std::uint32_t tx_attempts;
    std::uint32_t tx_queued;
    std::uint32_t tx_ok;
    std::uint32_t tx_error;
    std::uint32_t arbitration_lost;
    std::uint32_t rx_total;
    std::uint32_t rx_valid;
    std::uint32_t fifo_overruns;
    std::uint32_t hal_error;
    std::uint32_t esr;
    std::uint32_t tsr;
    std::uint32_t msr;
    std::uint32_t tec;
    std::uint32_t rec;
    std::uint32_t bus_off;
    std::uint32_t last_rx_id;
    std::uint32_t last_rx_dlc;
    std::uint8_t last_rx_data[8];
    std::uint32_t peer_ids[kMaxPeers];
    std::uint32_t peer_counters[kMaxPeers];
    std::uint32_t peer_rx_counts[kMaxPeers];
};

std::uint32_t hash_uid(std::uint32_t a, std::uint32_t b, std::uint32_t c)
{
    std::uint32_t hash = 2166136261UL;
    const std::uint32_t words[] = {a, b, c};
    for (std::uint32_t word : words) {
        for (std::uint32_t shift = 0; shift < 32UL; shift += 8UL) {
            hash ^= (word >> shift) & 0xFFUL;
            hash *= 16777619UL;
        }
    }
    return hash;
}

void configure_can_rx_alternate()
{
    mu::enable_gpio_clocks();
    mu::configure_alternate(GPIOA, mu::kCanRxIndex, GPIO_AF4_CAN);
    const std::uint32_t shift = static_cast<std::uint32_t>(mu::kCanRxIndex) * 2UL;
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << shift)) | (1UL << shift);
}

void drive_can_tx_recessive()
{
    mu::enable_gpio_clocks();
    mu::set_pin_high(GPIOA, mu::kCanTxPin);
    mu::configure_output(GPIOA, mu::kCanTxIndex);
    mu::set_pin_high(GPIOA, mu::kCanTxPin);
}

void configure_can_tx_alternate()
{
    mu::configure_alternate(GPIOA, mu::kCanTxIndex, GPIO_AF4_CAN);
}

HAL_StatusTypeDef configure_can()
{
    __HAL_RCC_CAN1_CLK_ENABLE();
    drive_can_tx_recessive();
    configure_can_rx_alternate();

    hcan.Instance = CAN;
    hcan.Init.Prescaler = 6U;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    return HAL_CAN_Init(&hcan);
}

HAL_StatusTypeDef configure_accept_all_filter()
{
    CAN_FilterTypeDef filter{};
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;
    return HAL_CAN_ConfigFilter(&hcan, &filter);
}

void record_mailbox_result(volatile MuCanProbe& probe, std::uint32_t tsr,
                           std::uint32_t rqcp, std::uint32_t txok,
                           std::uint32_t alst, std::uint32_t terr)
{
    if ((tsr & rqcp) == 0U) {
        return;
    }
    if ((tsr & txok) != 0U) {
        ++probe.tx_ok;
    }
    if ((tsr & alst) != 0U) {
        ++probe.arbitration_lost;
    }
    if ((tsr & terr) != 0U) {
        ++probe.tx_error;
    }
    hcan.Instance->TSR = rqcp;
}

void poll_tx(volatile MuCanProbe& probe)
{
    const std::uint32_t tsr = hcan.Instance->TSR;
    record_mailbox_result(probe, tsr, CAN_TSR_RQCP0, CAN_TSR_TXOK0,
                          CAN_TSR_ALST0, CAN_TSR_TERR0);
    record_mailbox_result(probe, tsr, CAN_TSR_RQCP1, CAN_TSR_TXOK1,
                          CAN_TSR_ALST1, CAN_TSR_TERR1);
    record_mailbox_result(probe, tsr, CAN_TSR_RQCP2, CAN_TSR_TXOK2,
                          CAN_TSR_ALST2, CAN_TSR_TERR2);
}

void record_peer(volatile MuCanProbe& probe, std::uint32_t peer_id,
                 std::uint32_t peer_counter)
{
    for (std::uint32_t i = 0U; i < kMaxPeers; ++i) {
        if (probe.peer_ids[i] == peer_id) {
            probe.peer_counters[i] = peer_counter;
            ++probe.peer_rx_counts[i];
            return;
        }
    }
    for (std::uint32_t i = 0U; i < kMaxPeers; ++i) {
        if (probe.peer_ids[i] == 0U) {
            probe.peer_ids[i] = peer_id;
            probe.peer_counters[i] = peer_counter;
            probe.peer_rx_counts[i] = 1U;
            return;
        }
    }
}

void poll_rx(volatile MuCanProbe& probe)
{
    if (__HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_FOV0) != RESET) {
        ++probe.fifo_overruns;
        __HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV0);
    }

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) != 0U) {
        CAN_RxHeaderTypeDef header{};
        std::uint8_t data[8]{};
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }

        ++probe.rx_total;
        probe.last_rx_id = header.IDE == CAN_ID_STD ? header.StdId : header.ExtId;
        probe.last_rx_dlc = header.DLC;
        for (std::uint32_t i = 0U; i < 8U; ++i) {
            probe.last_rx_data[i] = data[i];
        }

        if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
            header.DLC == 8U && data[0] == 0xCAU && data[1] == 0x4EU) {
            const std::uint32_t peer_id = static_cast<std::uint32_t>(data[2]) |
                                          (static_cast<std::uint32_t>(data[3]) << 8U);
            if (peer_id != probe.node_id) {
                const std::uint32_t peer_counter =
                    static_cast<std::uint32_t>(data[4]) |
                    (static_cast<std::uint32_t>(data[5]) << 8U) |
                    (static_cast<std::uint32_t>(data[6]) << 16U) |
                    (static_cast<std::uint32_t>(data[7]) << 24U);
                ++probe.rx_valid;
                record_peer(probe, peer_id, peer_counter);
            }
        }
    }
}

void snapshot_can_state(volatile MuCanProbe& probe)
{
    probe.hal_error = HAL_CAN_GetError(&hcan);
    probe.esr = hcan.Instance->ESR;
    probe.tsr = hcan.Instance->TSR;
    probe.msr = hcan.Instance->MSR;
    probe.tec = (probe.esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
    probe.rec = (probe.esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
    probe.bus_off = (probe.esr & CAN_ESR_BOFF) != 0U;
}

void send_heartbeat(volatile MuCanProbe& probe, std::uint32_t counter)
{
    ++probe.tx_attempts;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
        return;
    }

    CAN_TxHeaderTypeDef header{};
    header.StdId = probe.node_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;

    std::uint8_t data[8] = {
        0xCAU,
        0x4EU,
        static_cast<std::uint8_t>(probe.node_id),
        static_cast<std::uint8_t>(probe.node_id >> 8U),
        static_cast<std::uint8_t>(counter),
        static_cast<std::uint8_t>(counter >> 8U),
        static_cast<std::uint8_t>(counter >> 16U),
        static_cast<std::uint8_t>(counter >> 24U),
    };
    std::uint32_t mailbox = 0U;
    if (HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox) == HAL_OK) {
        ++probe.tx_queued;
    } else {
        ++probe.tx_error;
    }
}

} // namespace

extern "C" {
volatile MuCanProbe g_mu_can_probe = {};
}

extern "C" void SysTick_Handler(void)
{
    HAL_IncTick();
}

int main()
{
    HAL_Init();
    mu::clock_init_hsi48();
    mu::systick_init_1khz();
    mu::configure_safe_chip_selects();

    g_mu_can_probe.magic = kMagic;
    g_mu_can_probe.version = 1U;
    g_mu_can_probe.system_hz = HAL_RCC_GetSysClockFreq();
    g_mu_can_probe.pclk1_hz = HAL_RCC_GetPCLK1Freq();
    g_mu_can_probe.can_bitrate = kCanBitrate;
    g_mu_can_probe.uid[0] = *reinterpret_cast<volatile const std::uint32_t*>(UID_BASE);
    g_mu_can_probe.uid[1] = *reinterpret_cast<volatile const std::uint32_t*>(UID_BASE + 4U);
    g_mu_can_probe.uid[2] = *reinterpret_cast<volatile const std::uint32_t*>(UID_BASE + 8U);
    const std::uint32_t uid_hash = hash_uid(g_mu_can_probe.uid[0],
                                            g_mu_can_probe.uid[1],
                                            g_mu_can_probe.uid[2]);
    g_mu_can_probe.node_id = 0x500UL | (uid_hash & 0x3FFUL);

    g_mu_can_probe.can_init_status = configure_can();
    if (g_mu_can_probe.can_init_status == HAL_OK) {
        g_mu_can_probe.filter_status = configure_accept_all_filter();
    }
    if (g_mu_can_probe.filter_status == HAL_OK) {
        g_mu_can_probe.start_status = HAL_CAN_Start(&hcan);
        if (g_mu_can_probe.start_status == HAL_OK) {
            configure_can_tx_alternate();
        }
    }

    std::uint32_t counter = 0U;
    std::uint32_t next_tx = HAL_GetTick() + 250U;
    while (true) {
        ++g_mu_can_probe.loop_count;
        poll_tx(g_mu_can_probe);
        poll_rx(g_mu_can_probe);
        snapshot_can_state(g_mu_can_probe);

        const std::uint32_t now = HAL_GetTick();
        if (g_mu_can_probe.start_status == HAL_OK &&
            static_cast<std::int32_t>(now - next_tx) >= 0) {
            send_heartbeat(g_mu_can_probe, counter++);
            next_tx += kTxPeriodMs;
        }
    }
}
