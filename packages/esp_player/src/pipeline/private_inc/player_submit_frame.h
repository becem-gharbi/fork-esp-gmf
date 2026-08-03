/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_gmf_data_queue.h"
#include "esp_gmf_payload.h"

#include "esp_player_advance.h"
#include "player_stream.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#define ESP_PLAYER_FILL_QUEUE_NUM  (64)

typedef struct player_frame_node player_frame_node_t;

uint32_t player_frame_queue_size(const esp_player_stream_t *stream, bool is_audio, uint32_t queue_num);
esp_player_err_t player_frame_queue_create(uint32_t size, esp_gmf_data_queue_t **out_queue);
void player_frame_queue_destroy(esp_player_stream_t *stream, esp_gmf_data_queue_t **queue,
                                player_frame_node_t **read_node);
void player_frame_queue_reset(esp_player_stream_t *stream, esp_gmf_data_queue_t *queue,
                              player_frame_node_t **read_node);
void player_frame_queue_drain(esp_player_stream_t *stream, esp_gmf_data_queue_t *queue,
                              player_frame_node_t **read_node);
esp_gmf_err_io_t player_frame_queue_push_ref(esp_gmf_data_queue_t *queue, const esp_gmf_payload_t *load,
                                             esp_player_dec_frame_mode_t mode, uint32_t timeout_ms);
esp_gmf_err_io_t player_frame_queue_acquire(esp_gmf_data_queue_t *queue, player_frame_node_t **read_node,
                                            esp_gmf_payload_t *load, uint32_t timeout_ms);
esp_gmf_err_io_t player_frame_queue_release(esp_player_stream_t *stream, esp_gmf_data_queue_t *queue,
                                            player_frame_node_t **read_node);
esp_gmf_err_io_t player_frame_queue_send_wakeup(esp_gmf_data_queue_t *queue, bool is_done, uint32_t timeout_ms);
uint32_t player_frame_queue_count(esp_gmf_data_queue_t *queue);

esp_player_err_t player_submit_frame_fill(esp_player_stream_t *stream, const esp_player_frame_t *frame, uint32_t timeout_ms);
esp_player_err_t player_submit_frame_block(esp_player_stream_t *stream, esp_player_frame_t *frame);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
