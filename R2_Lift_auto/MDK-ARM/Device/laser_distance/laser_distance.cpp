#include "laser_distance.h"

LaserDistance laser;

LaserDistance::LaserDistance()
{
    clearData();
}

void LaserDistance::clearData(void)
{
    data.distance_m  = 0.0f;
    data.distance_mm = 0U;
    data.error_code  = 0U;
    data.valid       = 0U;
    data.error       = 0U;
}

void LaserDistance::laser_parse_dma_data(uint8_t *rx_data, uint16_t len)
{
    uint16_t i, j;

    if (rx_data == nullptr || len == 0U) {
        clearData();
        return;
    }

    clearData();

    for (i = 0U; i + 3U < len; i++) {
        if ((rx_data[i] == 'D' && rx_data[i + 1U] == '=') ||
            (rx_data[i] == 'E' && rx_data[i + 1U] == '=')) {
            for (j = i + 2U; j + 1U < len; j++) {
                if (rx_data[j] == '\r' && rx_data[j + 1U] == '\n') {
                    if (rx_data[i] == 'D') {
                        uint32_t int_part  = 0U;
                        uint32_t frac_part = 0U;
                        uint8_t frac_cnt   = 0U;
                        uint8_t has_dot    = 0U;
                        uint16_t k         = i + 2U;
                        uint8_t ok         = 1U;

                        while (k < j) {
                            if (rx_data[k] >= '0' && rx_data[k] <= '9') {
                                if (has_dot == 0U) {
                                    int_part = int_part * 10U + (uint32_t)(rx_data[k] - '0');
                                } else if (frac_cnt < 3U) {
                                    frac_part = frac_part * 10U + (uint32_t)(rx_data[k] - '0');
                                    frac_cnt++;
                                } else {
                                    ok = 0U;
                                    break;
                                }
                            } else if (rx_data[k] == '.') {
                                if (has_dot != 0U) {
                                    ok = 0U;
                                    break;
                                }
                                has_dot = 1U;
                            } else if (rx_data[k] == 'm') {
                                break;
                            } else {
                                ok = 0U;
                                break;
                            }

                            k++;
                        }

                        if (ok != 0U && k < j && rx_data[k] == 'm') {
                            while (frac_cnt < 3U) {
                                frac_part *= 10U;
                                frac_cnt++;
                            }

                            data.distance_mm = int_part * 1000U + frac_part;
                            data.distance_m  = (float)data.distance_mm / 1000.0f;
                            data.valid       = 1U;
                            data.error       = 0U;
                            data.error_code  = 0U;
                            return;
                        }
                    } else if (rx_data[i] == 'E') {
                        uint16_t err = 0U;
                        uint16_t k   = i + 2U;
                        uint8_t ok   = 1U;

                        while (k < j) {
                            if (rx_data[k] >= '0' && rx_data[k] <= '9') {
                                err = (uint16_t)(err * 10U + (uint16_t)(rx_data[k] - '0'));
                            } else {
                                ok = 0U;
                                break;
                            }
                            k++;
                        }

                        if (ok != 0U) {
                            data.error_code = err;
                            data.error      = 1U;
                            data.valid      = 0U;
                            return;
                        }
                    }

                    i = (uint16_t)(j + 1U);
                    break;
                }
            }
        }
    }

    clearData();
}
