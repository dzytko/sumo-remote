#include <stddef.h>
#include <stdint.h>
#include "ch32v003fun.h"
#include <errno.h>
#include <stdbool.h>

typedef enum Address {
    ADDRESS_START = 0x07,
    ADDRESS_PROGRAM = 0x0B,
} Address_t;

typedef enum Action {
    ACTION_START,
    ACTION_STOP,
    ACTION_PROGRAM,
} Action_t;


static uint8_t toggle_bit = 1;


static void calc_presc_reload(
    uint32_t *target_frequency,
    uint32_t *target_resolution,
    uint16_t *prescaler,
    uint16_t *reload_value
) {
    const uint32_t cpu_freq = 48 * 1000 * 1000;
    *reload_value = cpu_freq / *target_frequency;
    *prescaler = *target_resolution / (1 << 16);

    *target_frequency = cpu_freq / ((*prescaler + 1) * *reload_value);
    *target_resolution = *reload_value;
}

/**
 * @brief Encode input data using Manchester encoding.
 *
 * Each byte in the array is treated as a bit (!=0 high, ==0 low), and
 * accordingly each byte in output array represents single bit.
 *
 * @param[in]  input      Pointer to input data array.
 * @param[in]  input_len  Number of bytes in input array.
 * @param[out] output     Pointer to output array (must be at least input_len*2 bytes).
 * @param[in,out] output_len Pointer to output length. Must be at least input_len*2 on input;
 *                           set to number of bytes written on success.
 * @return 0 on success, negative error code on failure:
 * - -ENOMEM if output buffer is too small
 */
static error_t encode_manchester(
    const uint8_t input[],
    const size_t input_len,
    uint8_t output[],
    size_t *output_len
) {
    if (output_len == NULL || *output_len < input_len * 2) {
        return -ENOMEM;
    }

    for (size_t i = 0; i < input_len; i++) {
        if (input[i] != 0) {
            output[2 * i] = 0;
            output[2 * i + 1] = 1;
        }
        else {
            output[2 * i] = 1;
            output[2 * i + 1] = 0;
        }
    }
    *output_len = input_len * 2;

    return 0;
}

static uint8_t read_dohyo_id(void) {
    return GPIOC->INDR & 0x1f;
}

static error_t build_command(const Action_t action, uint8_t out_command[14]) {
    if (action != ACTION_START && action != ACTION_STOP && action != ACTION_PROGRAM) {
        return -EINVAL;
    }

    out_command[0] = 1;
    out_command[1] = 1;
    out_command[2] = toggle_bit;
    // toggle_bit = !toggle_bit;

    Address_t address = 0;
    switch (action) {
        case ACTION_START:
        case ACTION_STOP:
            address = ADDRESS_START;
            break;
        case ACTION_PROGRAM:
            address = ADDRESS_PROGRAM;
            break;
    }

    for (int i = 0; i < 5; ++i) {
        out_command[3 + i] = (address & 1 << 4) != 0;
        address <<= 1;
    }
    uint8_t dohyo_id = read_dohyo_id();
    for (int i = 0; i < 5; ++i) {
        out_command[8 + i] = (dohyo_id & 1 << 4) != 0;
        dohyo_id <<= 1;
    }

    switch (action) {
        case ACTION_START:
            out_command[13] = 1;
            break;
        case ACTION_STOP:
        case ACTION_PROGRAM:
        default:
            out_command[13] = 0;
            break;
    }

    return 0;
}

static void delay_us(uint16_t us) {
    for (uint16_t i = 0; i < us; i++) {
        for (uint32_t j = 0; j < 1; j++) {
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
            asm volatile("nop");
        }
    }
}

static void delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

static void send_command(uint8_t command[], size_t command_len, Action_t action) {
    uint8_t bit_stream[50];
    size_t encoded_len = sizeof(bit_stream);
    encode_manchester(command, command_len, bit_stream, &encoded_len);

    switch (action) {
        case ACTION_START:
        case ACTION_STOP:
            TIM2->CCER |= TIM_CC1E;
            TIM2->CCER &= ~TIM_CC2E;
            break;
        case ACTION_PROGRAM:
        default:
            TIM2->CCER |= TIM_CC2E;
            TIM2->CCER &= ~TIM_CC1E;
            break;
    }

    for (uint8_t i = 0; i < (uint8_t) encoded_len; i++) {
        if (bit_stream[i]) {
            TIM2->CTLR1 |= TIM_CEN;
        }
        else {
            TIM2->CTLR1 &= ~TIM_CEN;
        }
        delay_us(889);
    }
    TIM2->CCER &= ~TIM_CC1E;
    TIM2->CCER &= ~TIM_CC2E;
    TIM2->CTLR1 &= ~TIM_CEN;
    delay_us(5000);
}

static void init_timer(void) {
    RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

    funGpioInitD();
    funPinMode(PD3, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF); // program
    funPinMode(PD4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF); // start

    // Reset TIM2 to init all regs
    RCC->APB1PRSTR |= RCC_APB1Periph_TIM2;
    RCC->APB1PRSTR &= ~RCC_APB1Periph_TIM2;

    // CTLR1: default is up, events generated, edge align
    TIM2->CTLR1 = TIM_ARPE;

    // CTLR2: set output idle states (MOE off) via OIS1 and OIS1N bits
    TIM2->CTLR2 = 0;

    // SMCFGR: default clk input is 48MHz CK_INT
    uint32_t frequency = 38 * 1000;
    uint32_t resolution = (1 << 16) - 1;
    uint16_t prescaler = 0;
    uint16_t reload_value = 0;

    calc_presc_reload(&frequency, &resolution, &prescaler, &reload_value);
    TIM2->PSC = prescaler;
    TIM2->ATRLR = reload_value - 1;

    TIM2->SWEVGR |= TIM_UG;

    // CH1 Mode is output, PWM1 (CC1S = 00, OC1M = 110)
    TIM2->CHCTLR1 |= TIM_OC1M_2 | TIM_OC1M_1 | TIM_OC1PE;
    TIM2->CHCTLR1 |= TIM_OC2M_2 | TIM_OC2M_1 | TIM_OC2PE;

    TIM2->CH1CVR = reload_value / 2;
    TIM2->CH2CVR = reload_value / 2;

    TIM2->BDTR |= TIM_MOE;
}

static void init_gpio(void) {
    funGpioInitC();
    funPinMode(PC0, GPIO_CNF_IN_PUPD);
    funPinMode(PC1, GPIO_CNF_IN_PUPD);
    funPinMode(PC2, GPIO_CNF_IN_PUPD);
    funPinMode(PC3, GPIO_CNF_IN_PUPD);
    funPinMode(PC4, GPIO_CNF_IN_PUPD);
    funPinMode(PC5, GPIO_CNF_IN_PUPD);
    funPinMode(PC6, GPIO_CNF_IN_PUPD);
    funPinMode(PC7, GPIO_CNF_IN_PUPD);
    GPIOC->OUTDR |= 0xff;
}

static void start_button_press_handler(void) {
    const Action_t action = ACTION_START;
    uint8_t command[14];
    build_command(action, command);
    send_command(command, sizeof(command), action);
}

static void stop_button_press_handler(void) {
    const Action_t action = ACTION_STOP;
    uint8_t command[14];
    build_command(action, command);
    send_command(command, sizeof(command), action);
}

static void program_button_press_handler(void) {
    const Action_t action = ACTION_PROGRAM;
    uint8_t command[14];
    build_command(action, command);
    send_command(command, sizeof(command), action);
}

int main(void) {
    SystemInit();
    funGpioInitD();
    funPinMode(PD6, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    funDigitalWrite(PD6, 1);


    // funGpioInitD();
    // funPinMode(PD4, GPIO_Speed_10MHz | GPIO_CNF_OUT_PP);
    //
    // while (1) {
    //     funDigitalWrite(PD4, 1);
    //     delay_us(889);
    //     funDigitalWrite(PD4, 0);
    //     delay_us(889);
    // }

    init_gpio();
    init_timer();

    // ReSharper disable once CppDFAEndlessLoop
    while (1) {
        bool command_handled = false;
        while (!funDigitalRead(PC5)) {
            start_button_press_handler();
            command_handled = true;
        }
        if (command_handled) {
            toggle_bit = !toggle_bit;
        }
        command_handled = false;

        while (!funDigitalRead(PC6)) {
            stop_button_press_handler();
            command_handled = true;
        }
        if (command_handled) {
            toggle_bit = !toggle_bit;
        }
        command_handled = false;

        while (!funDigitalRead(PC7)) {
            program_button_press_handler();
            command_handled = true;
        }
        if (command_handled) {
            toggle_bit = !toggle_bit;
        }
        command_handled = false;
    }
}
