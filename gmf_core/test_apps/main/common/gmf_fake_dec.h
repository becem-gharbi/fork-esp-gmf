/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO., LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mock_dec.h"
#include "esp_gmf_audio_element.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/**
 * @brief  Fake Decoder configurations
 */
typedef struct {
    int               in_buf_size;  /*!< Size of output ringbuffer */
    int               out_buf_size;
    esp_gmf_event_cb  cb;
    const char       *name;
    bool              is_pass;
    bool              is_shared;
    uint8_t           in_addr_align;   /*!< IN port buffer address alignment */
    uint8_t           in_size_align;   /*!< IN port buffer size alignment */
    uint8_t           out_addr_align;  /*!< OUT port buffer address alignment */
    uint8_t           out_size_align;  /*!< OUT port buffer size alignment */
} fake_dec_cfg_t;

#define FAKE_DEC_BUFFER_SIZE  (5 * 1024)

#define DEFAULT_FAKE_DEC_CONFIG()  {         \
    .in_buf_size    = FAKE_DEC_BUFFER_SIZE,  \
    .out_buf_size   = FAKE_DEC_BUFFER_SIZE,  \
    .cb             = NULL,                  \
    .name           = NULL,                  \
    .is_pass        = false,                 \
    .is_shared      = true,                  \
    .in_addr_align  = 0,                     \
    .in_size_align  = 0,                     \
    .out_addr_align = 0,                     \
    .out_size_align = 0,                     \
}

esp_err_t fake_dec_init(fake_dec_cfg_t *config, esp_gmf_obj_handle_t *handle);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
