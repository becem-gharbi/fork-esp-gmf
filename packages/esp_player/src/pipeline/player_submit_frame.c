/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO., LTD
 * SPDX-License-Identifier: LicenseRef-Espressif-Modified-MIT
 *
 * See LICENSE file for details.
 */

#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_gmf_err.h"
#include "esp_gmf_oal_mem.h"

#include "player_submit_frame.h"
#include "player_events.h"

static const char *TAG = "ESP_PLAYER_SUBMIT";

/* esp_gmf_data_queue stores a 4-byte record length before every acquired block. */
#define PLAYER_FRAME_QUEUE_RECORD_HEAD_SIZE  (sizeof(uint32_t))

/* The queue writes that length header with a 32-bit store, and the record start only
 * moves by the released size, so every record must stay a multiple of 4 bytes. */
#define PLAYER_FRAME_RECORD_ALIGN  (4)

/* Must stay >= the alignment player_frame_node_t requires; 8 covers the uint64_t pts
 * inside its esp_gmf_payload_t. Raise it if the struct gains a stricter member. */
#define PLAYER_FRAME_NODE_ALIGN  (8)

/* The queue hands back `base + wp + head`, where base is malloc-aligned and both wp and
 * head are multiples of PLAYER_FRAME_RECORD_ALIGN, so that address is always 4-byte
 * aligned and the node can never need more than this much padding. */
#define PLAYER_FRAME_NODE_PAD_MAX  (PLAYER_FRAME_NODE_ALIGN - PLAYER_FRAME_RECORD_ALIGN)

/* Upper bound of the non-payload bytes a record consumes: the node plus worst-case padding. */
#define PLAYER_FRAME_RECORD_OVERHEAD  \
    (sizeof(player_frame_node_t) + PLAYER_FRAME_NODE_PAD_MAX + (PLAYER_FRAME_RECORD_ALIGN - 1))

/* FILL payload address align (feeds decoder directly). Pick by esp_player_frame_t::track_type. */
#define PLAYER_FILL_AUDIO_BUF_ALIGN  (16)
#define PLAYER_FILL_VIDEO_BUF_ALIGN  (64)

/* Default data buffer budget for FILL mode copied payloads (bytes).
 * This is independent of extractor_pool_size which belongs to EXTRACTOR mode. */
#define DEFAULT_FILL_QUEUE_DATA_SIZE  (32 * 1024)

struct player_frame_node {
    esp_gmf_payload_t            load;
    esp_player_dec_frame_mode_t  mode;
    uint32_t                     data_offset;
};

static inline uint32_t player_fill_buf_align(esp_player_track_type_t track_type, uint8_t av_mask)
{
    if (track_type == ESP_PLAYER_TRACK_TYPE_VIDEO
        || (track_type == ESP_PLAYER_TRACK_TYPE_NONE && av_mask == ESP_PLAYER_MASK_VIDEO)) {
        return PLAYER_FILL_VIDEO_BUF_ALIGN;
    }
    return PLAYER_FILL_AUDIO_BUF_ALIGN;
}

static inline uint32_t player_frame_record_size(uint32_t data_size, uint32_t align_pad)
{
    size_t raw = sizeof(player_frame_node_t) + PLAYER_FRAME_NODE_PAD_MAX + align_pad + data_size;
    return (uint32_t)ESP_GMF_OAL_ALIGN_UP(raw, (size_t)PLAYER_FRAME_RECORD_ALIGN);
}

static inline player_frame_node_t *player_frame_node_from_raw(uint8_t *raw)
{
    return (player_frame_node_t *)ESP_GMF_OAL_ALIGN_UP((uintptr_t)raw, (uintptr_t)PLAYER_FRAME_NODE_ALIGN);
}

static uint32_t player_block_done_event(esp_player_stream_t *stream, esp_gmf_data_queue_t *queue)
{
    if (stream->audio_side != NULL && queue == stream->audio_side->frame_queue) {
        return _CTRL_DECODER_AUDIO_FRAME_DONE;
    }
    if (stream->video_side != NULL && queue == stream->video_side->frame_queue) {
        return _CTRL_DECODER_VIDEO_FRAME_DONE;
    }
    return 0;
}

static esp_gmf_data_queue_t *player_submit_queue(esp_player_stream_t *stream,
                                                 esp_player_track_type_t track_type)
{
    uint8_t want = 0;
    if (track_type == ESP_PLAYER_TRACK_TYPE_NONE) {
        if (stream->av_mask == ESP_PLAYER_MASK_AUDIO) {
            want = ESP_PLAYER_MASK_AUDIO;
        } else if (stream->av_mask == ESP_PLAYER_MASK_VIDEO) {
            want = ESP_PLAYER_MASK_VIDEO;
        } else {
            /* MASK_AV requires an explicit track_type on every frame. */
            return NULL;
        }
    } else if (track_type == ESP_PLAYER_TRACK_TYPE_AUDIO) {
        want = ESP_PLAYER_MASK_AUDIO;
    } else if (track_type == ESP_PLAYER_TRACK_TYPE_VIDEO) {
        want = ESP_PLAYER_MASK_VIDEO;
    } else {
        return NULL;
    }
    if ((stream->av_mask & want) == 0) {
        return NULL;
    }
    if (want == ESP_PLAYER_MASK_AUDIO) {
        return stream->audio_side ? stream->audio_side->frame_queue : NULL;
    }
    return stream->video_side ? stream->video_side->frame_queue : NULL;
}

/* FILL: deep-copy payload into the queue with decoder address alignment. */
static esp_gmf_err_io_t player_frame_queue_push_copy(esp_gmf_data_queue_t *queue,
                                                     const esp_gmf_payload_t *load,
                                                     uint32_t buf_align,
                                                     uint32_t timeout_ms)
{
    if (queue == NULL || load == NULL || load->buf == NULL || load->valid_size == 0) {
        return ESP_GMF_IO_FAIL;
    }
    uint32_t data_size = load->valid_size;
    uint32_t align_pad = (buf_align > 1U) ? (buf_align - 1U) : 0U;
    if (data_size > (uint32_t)(INT_MAX - PLAYER_FRAME_RECORD_OVERHEAD - align_pad)) {
        return ESP_GMF_IO_FAIL;
    }

    uint32_t record_size = player_frame_record_size(data_size, align_pad);
    uint8_t *raw = NULL;
    int ret = esp_gmf_data_queue_acquire_write(queue, (void **)&raw, (int)record_size, timeout_ms);
    if (ret != 0 || raw == NULL) {
        return ret == ESP_GMF_IO_ABORT ? ESP_GMF_IO_ABORT : ESP_GMF_IO_TIMEOUT;
    }
    player_frame_node_t *node = player_frame_node_from_raw(raw);
    memset(node, 0, sizeof(*node));
    node->load = *load;
    node->mode = ESP_PLAYER_DEC_FRAME_MODE_FILL;
    uint8_t *buf = (uint8_t *)node + sizeof(*node);
    if (buf_align > 1U) {
        buf = (uint8_t *)ESP_GMF_OAL_ALIGN_UP((uintptr_t)buf, (uintptr_t)buf_align);
    }
    node->data_offset = (uint32_t)(buf - (uint8_t *)node);
    node->load.buf = buf;
    node->load.buf_length = data_size;
    memcpy(node->load.buf, load->buf, data_size);
    if (esp_gmf_data_queue_release_write(queue, (int)record_size) != 0) {
        ESP_LOGE(TAG, "Failed to release frame queue write block");
        return ESP_GMF_IO_FAIL;
    }
    return ESP_GMF_IO_OK;
}

/* EXTRACTOR / BLOCK / wakeup: keep the payload outside the queue, store the node only. */
esp_gmf_err_io_t player_frame_queue_push_ref(esp_gmf_data_queue_t *queue,
                                             const esp_gmf_payload_t *load,
                                             esp_player_dec_frame_mode_t mode,
                                             uint32_t timeout_ms)
{
    if (queue == NULL || load == NULL) {
        return ESP_GMF_IO_FAIL;
    }
    uint32_t record_size = player_frame_record_size(0, 0);
    uint8_t *raw = NULL;
    /* esp_gmf_data_queue timeouts are milliseconds (see esp_gmf_data_queue.h). */
    int ret = esp_gmf_data_queue_acquire_write(queue, (void **)&raw, (int)record_size, timeout_ms);
    if (ret != 0 || raw == NULL) {
        return ret == ESP_GMF_IO_ABORT ? ESP_GMF_IO_ABORT : ESP_GMF_IO_TIMEOUT;
    }
    player_frame_node_t *node = player_frame_node_from_raw(raw);
    memset(node, 0, sizeof(*node));
    node->load = *load;
    node->mode = mode;
    if (esp_gmf_data_queue_release_write(queue, (int)record_size) != 0) {
        ESP_LOGE(TAG, "Failed to release frame queue write block");
        return ESP_GMF_IO_FAIL;
    }
    return ESP_GMF_IO_OK;
}

uint32_t player_frame_queue_size(const esp_player_stream_t *stream, bool is_audio, uint32_t queue_num)
{
    uint32_t slot = PLAYER_FRAME_RECORD_OVERHEAD + PLAYER_FRAME_QUEUE_RECORD_HEAD_SIZE;
    if (stream->dec_frame_mode == ESP_PLAYER_DEC_FRAME_MODE_FILL) {
        uint32_t align = is_audio ? PLAYER_FILL_AUDIO_BUF_ALIGN : PLAYER_FILL_VIDEO_BUF_ALIGN;
        slot += (align - 1U);
    }
    uint64_t node_overhead = (uint64_t)queue_num * slot;
    if (stream->dec_frame_mode == ESP_PLAYER_DEC_FRAME_MODE_FILL) {
        node_overhead += DEFAULT_FILL_QUEUE_DATA_SIZE;
    }
    return node_overhead > INT_MAX ? 0 : (uint32_t)node_overhead;
}

esp_player_err_t player_frame_queue_create(uint32_t size, esp_gmf_data_queue_t **out_queue)
{
    if (out_queue == NULL || size == 0 || size > INT_MAX) {
        return ESP_PLAYER_ERR_INVALID_ARG;
    }
    *out_queue = esp_gmf_data_queue_create((int)size);
    if (*out_queue == NULL) {
        return ESP_PLAYER_ERR_NO_MEM;
    }
    return ESP_PLAYER_ERR_OK;
}

esp_gmf_err_io_t player_frame_queue_send_wakeup(esp_gmf_data_queue_t *queue, bool is_done,
                                                uint32_t timeout_ms)
{
    esp_gmf_payload_t load = {
        .is_done = is_done,
    };
    return player_frame_queue_push_ref(queue, &load, ESP_PLAYER_DEC_FRAME_MODE_UNKNOWN, timeout_ms);
}

esp_gmf_err_io_t player_frame_queue_acquire(esp_gmf_data_queue_t *queue,
                                            player_frame_node_t **read_node,
                                            esp_gmf_payload_t *load,
                                            uint32_t timeout_ms)
{
    if (queue == NULL || read_node == NULL || load == NULL || *read_node != NULL) {
        return ESP_GMF_IO_FAIL;
    }
    int record_size = 0;
    uint8_t *raw = NULL;
    int ret = esp_gmf_data_queue_acquire_read(queue, (void **)&raw, &record_size, timeout_ms);
    if (ret != 0) {
        return ret == ESP_GMF_IO_ABORT ? ESP_GMF_IO_ABORT : ESP_GMF_IO_TIMEOUT;
    }
    if (raw == NULL || record_size <= 0) {
        if (raw != NULL) {
            esp_gmf_data_queue_release_read(queue);
        }
        return ESP_GMF_IO_FAIL;
    }
    /* The writer aligned the node inside the record; recompute the same padding here. */
    player_frame_node_t *node = player_frame_node_from_raw(raw);
    uint32_t pad = (uint32_t)((uint8_t *)node - raw);
    if ((uint32_t)record_size < pad + sizeof(*node)) {
        esp_gmf_data_queue_release_read(queue);
        return ESP_GMF_IO_FAIL;
    }

    *load = node->load;
    if (node->mode == ESP_PLAYER_DEC_FRAME_MODE_FILL) {
        if (node->data_offset < sizeof(*node)
            || node->load.valid_size > (uint32_t)record_size
            || pad + node->data_offset > (uint32_t)record_size - (uint32_t)node->load.valid_size) {
            esp_gmf_data_queue_release_read(queue);
            return ESP_GMF_IO_FAIL;
        }
        load->buf = (uint8_t *)node + node->data_offset;
    }
    *read_node = node;
    return ESP_GMF_IO_OK;
}

esp_gmf_err_io_t player_frame_queue_release(esp_player_stream_t *stream,
                                            esp_gmf_data_queue_t *queue,
                                            player_frame_node_t **read_node)
{
    if (stream == NULL || queue == NULL || read_node == NULL || *read_node == NULL) {
        return ESP_GMF_IO_FAIL;
    }

    player_frame_node_t *node = *read_node;
    esp_gmf_err_io_t ret = ESP_GMF_IO_OK;
    if (node->mode == ESP_PLAYER_DEC_FRAME_MODE_EXTRACTOR) {
        ret = player_release_extractor_payload(stream, &node->load);
    } else if (node->mode == ESP_PLAYER_DEC_FRAME_MODE_BLOCK) {
        uint32_t done_bit = player_block_done_event(stream, queue);
        if (done_bit != 0) {
            player_set_events(stream, done_bit);
        }
    }
    *read_node = NULL;
    if (esp_gmf_data_queue_release_read(queue) != 0) {
        return ESP_GMF_IO_FAIL;
    }
    return ret;
}

void player_frame_queue_drain(esp_player_stream_t *stream, esp_gmf_data_queue_t *queue,
                              player_frame_node_t **read_node)
{
    if (stream == NULL || queue == NULL || read_node == NULL) {
        return;
    }
    if (*read_node != NULL) {
        (void)player_frame_queue_release(stream, queue, read_node);
    }
    while (true) {
        esp_gmf_payload_t load = {0};
        if (player_frame_queue_acquire(queue, read_node, &load, ESP_GMF_DATA_QUEUE_NO_WAIT) != ESP_GMF_IO_OK) {
            break;
        }
        if (player_frame_queue_release(stream, queue, read_node) != ESP_GMF_IO_OK) {
            break;
        }
    }
}

void player_frame_queue_reset(esp_player_stream_t *stream, esp_gmf_data_queue_t *queue,
                              player_frame_node_t **read_node)
{
    if (queue == NULL) {
        return;
    }
    player_frame_queue_drain(stream, queue, read_node);
    if (read_node == NULL || *read_node == NULL) {
        esp_gmf_data_queue_reset(queue);
    }
}

void player_frame_queue_destroy(esp_player_stream_t *stream, esp_gmf_data_queue_t **queue,
                                player_frame_node_t **read_node)
{
    if (queue == NULL || *queue == NULL) {
        return;
    }
    if (read_node && *read_node) {
        player_frame_queue_release(stream, *queue, read_node);
    }
    player_frame_queue_drain(stream, *queue, read_node);
    esp_gmf_data_queue_destroy(*queue);
    *queue = NULL;
}

uint32_t player_frame_queue_count(esp_gmf_data_queue_t *queue)
{
    int count = 0;
    int size = 0;
    if (queue == NULL || esp_gmf_data_queue_query(queue, &count, &size) != 0 || count < 0) {
        return 0;
    }
    return (uint32_t)count;
}

esp_player_err_t player_submit_frame_fill(esp_player_stream_t *stream,
                                          const esp_player_frame_t *frame,
                                          uint32_t timeout_ms)
{
    esp_gmf_payload_t load = {
        .buf = frame->data,
        .buf_length = frame->data_len,
        .valid_size = frame->data_len,
        .is_done = frame->eos,
        .pts = frame->pts,
        .needs_free = false,
        .meta_flag = frame->is_bad ? ESP_GMF_META_FLAG_AUD_RECOVERY_PLC : 0,
    };
    esp_gmf_data_queue_t *queue = player_submit_queue(stream, frame->track_type);
    if (queue == NULL) {
        ESP_LOGE(TAG, "Invalid submit track_type=%d for av_mask=0x%x", (int)frame->track_type, stream->av_mask);
        return ESP_PLAYER_ERR_INVALID_ARG;
    }
    uint32_t align = player_fill_buf_align(frame->track_type, stream->av_mask);
    esp_gmf_err_io_t ret = player_frame_queue_push_copy(queue, &load, align, timeout_ms);
    if (ret != ESP_GMF_IO_OK) {
        return ESP_PLAYER_ERR_TIMEOUT;
    }
    return ESP_PLAYER_ERR_OK;
}

esp_player_err_t player_submit_frame_block(esp_player_stream_t *stream, esp_player_frame_t *frame)
{
    esp_gmf_data_queue_t *queue = player_submit_queue(stream, frame->track_type);
    if (queue == NULL) {
        ESP_LOGE(TAG, "Invalid submit track_type=%d for av_mask=0x%x", (int)frame->track_type, stream->av_mask);
        return ESP_PLAYER_ERR_INVALID_ARG;
    }
    uint32_t done_bit = player_block_done_event(stream, queue);
    if (done_bit == 0) {
        ESP_LOGE(TAG, "No BLOCK done event for submit queue");
        return ESP_PLAYER_ERR_FAIL;
    }
    esp_gmf_payload_t load = {
        .buf = frame->data,
        .valid_size = frame->data_len,
        .pts = frame->pts,
        .is_done = frame->eos,
        .needs_free = false,
        .meta_flag = frame->is_bad ? ESP_GMF_META_FLAG_AUD_RECOVERY_PLC : 0,
    };
    /* Per-stream done bit: audio and video threads may block concurrently on AV. */
    player_clear_events(stream, done_bit);
    if (player_frame_queue_push_ref(queue, &load, ESP_PLAYER_DEC_FRAME_MODE_BLOCK,
                                    ESP_GMF_DATA_QUEUE_WAIT_FOREVER) != ESP_GMF_IO_OK) {
        return ESP_PLAYER_ERR_FAIL;
    }
    return player_wait_events(stream, done_bit, portMAX_DELAY);
}
