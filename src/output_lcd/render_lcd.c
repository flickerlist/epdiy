#include <stdint.h>
#include <string.h>

#include "../output_common/render_method.h"

#ifdef RENDER_METHOD_LCD

#include <esp_log.h>
#include <rom/cache.h>

#include "../epd_internals.h"
#include "../output_common/line_queue.h"
#include "../output_common/lut.h"
#include "../output_common/render_context.h"
#include "epd_board.h"
#include "epdiy.h"
#include "lcd_driver.h"
#include "render_lcd.h"

// declare vector optimized line mask application.
void epd_apply_line_mask_VE(uint8_t* line, const uint8_t* mask, int mask_len);
static bool retrieve_line_isr(RenderContext_t* ctx, uint8_t* buf);

static inline uint8_t IRAM_ATTR line_ready_load(RenderContext_t* ctx, int line) {
    return __atomic_load_n(&ctx->line_ready[line], __ATOMIC_ACQUIRE);
}

static inline uint8_t IRAM_ATTR line_thread_load(RenderContext_t* ctx, int line) {
    return __atomic_load_n(&ctx->line_threads[line], __ATOMIC_RELAXED);
}

static inline void IRAM_ATTR publish_line(RenderContext_t* ctx, int line, uint8_t thread_id) {
    __atomic_store_n(&ctx->line_threads[line], thread_id, __ATOMIC_RELAXED);
    __atomic_store_n(&ctx->line_ready[line], 1, __ATOMIC_RELEASE);
}

static inline bool IRAM_ATTR try_start_frame_once(RenderContext_t* ctx) {
    bool expected = false;
    if (!atomic_compare_exchange_strong(&ctx->frame_started, &expected, true)) {
        return false;
    }

    epd_lcd_line_source_cb((line_cb_func_t)&retrieve_line_isr, ctx);
    epd_lcd_start_frame();
    return true;
}

static inline bool IRAM_ATTR ready_prefix_committed(RenderContext_t* ctx, int line_count) {
    int limit = line_count < ctx->lines_total ? line_count : ctx->lines_total;
    for (int i = 0; i < limit; i++) {
        if (!line_ready_load(ctx, i)) {
            return false;
        }
    }
    return true;
}

static inline bool IRAM_ATTR frame_started_load(RenderContext_t* ctx) {
    return atomic_load_explicit(&ctx->frame_started, memory_order_acquire);
}

static inline int IRAM_ATTR lq_effective_capacity(LineQueue_t* queue) {
    return queue->size - 1;
}

static inline int IRAM_ATTR lq_used(LineQueue_t* queue) {
    int current = atomic_load_explicit(&queue->current, memory_order_acquire);
    int last = atomic_load_explicit(&queue->last, memory_order_acquire);
    int used = current - last;
    if (used < 0) {
        used += queue->size;
    }
    return used;
}

static inline void IRAM_ATTR maybe_start_frame(
    RenderContext_t* ctx,
    LineQueue_t* lq,
    int startup_ready_target,
    int minimum_ready_lines,
    int prestart_queue_limit
) {
    if (frame_started_load(ctx)) {
        return;
    }

    if (ready_prefix_committed(ctx, startup_ready_target)) {
        try_start_frame_once(ctx);
        return;
    }

    if (lq_used(lq) >= prestart_queue_limit && ready_prefix_committed(ctx, minimum_ready_lines)) {
        try_start_frame_once(ctx);
    }
}

__attribute__((optimize("O3"))) static bool IRAM_ATTR
retrieve_line_isr(RenderContext_t* ctx, uint8_t* buf) {
    if (ctx->lines_consumed >= ctx->lines_total) {
        return false;
    }
    if (!line_ready_load(ctx, ctx->lines_consumed)) {
        ctx->error |= EPD_DRAW_EMPTY_LINE_QUEUE;
        memset(buf, 0x00, ctx->display_width / 4);
        return false;
    }

    int thread = line_thread_load(ctx, ctx->lines_consumed);
    if (thread >= NUM_RENDER_THREADS) {
        ctx->error |= EPD_DRAW_EMPTY_LINE_QUEUE;
        memset(buf, 0x00, ctx->display_width / 4);
        return false;
    }

    LineQueue_t* lq = &ctx->line_queues[thread];

    BaseType_t awoken = pdFALSE;

    if (lq_read(lq, buf) != 0) {
        ctx->error |= EPD_DRAW_EMPTY_LINE_QUEUE;
        memset(buf, 0x00, ctx->display_width / 4);
    }

    if (ctx->lines_consumed >= ctx->display_height) {
        memset(buf, 0x00, ctx->display_width / 4);
    }
    ctx->lines_consumed += 1;
    return awoken;
}

/// start the next frame in the current update cycle
static void IRAM_ATTR handle_lcd_frame_done(RenderContext_t* ctx) {
    epd_lcd_frame_done_cb(NULL, NULL);
    epd_lcd_line_source_cb(NULL, NULL);

    BaseType_t task_awoken = pdFALSE;
    xSemaphoreGiveFromISR(ctx->frame_done, &task_awoken);

    portYIELD_FROM_ISR();
}

void lcd_do_update(RenderContext_t* ctx) {
    epd_set_mode(1);

    for (uint8_t k = 0; k < ctx->cycle_frames; k++) {
        epd_lcd_frame_done_cb((frame_done_func_t)handle_lcd_frame_done, ctx);
        prepare_context_for_next_frame(ctx);

        // start both feeder tasks
        xTaskNotifyGive(ctx->feed_tasks[!xPortGetCoreID()]);
        xTaskNotifyGive(ctx->feed_tasks[xPortGetCoreID()]);

        // transmission is started in renderer threads, now wait util it's done
        xSemaphoreTake(ctx->frame_done, portMAX_DELAY);

        for (int i = 0; i < NUM_RENDER_THREADS; i++) {
            xSemaphoreTake(ctx->feed_done_smphr[i], portMAX_DELAY);
        }

        ctx->current_frame++;

        // make the watchdog happy.
        vTaskDelay(0);
    }

    epd_lcd_line_source_cb(NULL, NULL);
    epd_lcd_frame_done_cb(NULL, NULL);

    epd_set_mode(0);
}

__attribute__((optimize("O3"))) static bool IRAM_ATTR
push_pixels_isr(RenderContext_t* ctx, uint8_t* buf) {
    // Output no-op outside of drawn area
    if (ctx->lines_consumed < ctx->area.y) {
        memset(buf, 0, ctx->display_width / 4);
    } else if (ctx->lines_consumed >= ctx->area.y + ctx->area.height) {
        memset(buf, 0, ctx->display_width / 4);
    } else {
        memcpy(buf, ctx->static_line_buffer, ctx->display_width / 4);
    }
    ctx->lines_consumed += 1;
    return pdFALSE;
}

/**
 * Populate the line mask for use in epd_push_pixels.
 */
static void push_pixels_populate_line(RenderContext_t* ctx, int color) {
    // Select fill pattern by draw color
    int fill_byte = 0;
    switch (color) {
        case 0:
            fill_byte = DARK_BYTE;
            break;
        case 1:
            fill_byte = CLEAR_BYTE;
            break;
        default:
            fill_byte = 0x00;
    }

    // Compute a line mask based on the drawn area
    uint8_t* dirtyness = malloc(ctx->display_width / 2);
    assert(dirtyness != NULL);

    memset(dirtyness, 0, ctx->display_width / 2);

    for (int i = 0; i < ctx->display_width; i++) {
        if ((i >= ctx->area.x) && (i < ctx->area.x + ctx->area.width)) {
            dirtyness[i / 2] |= i % 2 ? 0xF0 : 0x0F;
        }
    }
    epd_populate_line_mask(ctx->line_mask, dirtyness, ctx->display_width / 4);

    // mask the line pattern with the populated mask
    memset(ctx->static_line_buffer, fill_byte, ctx->display_width / 4);
    epd_apply_line_mask(ctx->static_line_buffer, ctx->line_mask, ctx->display_width / 4);

    free(dirtyness);
}

void epd_push_pixels_lcd(RenderContext_t* ctx, short time, int color) {
    ctx->current_frame = 0;
    ctx->lines_total = ctx->display_height;
    ctx->lines_consumed = 0;
    ctx->static_line_buffer = malloc(ctx->display_width / 4);
    assert(ctx->static_line_buffer != NULL);

    push_pixels_populate_line(ctx, color);
    epd_lcd_frame_done_cb((frame_done_func_t)handle_lcd_frame_done, ctx);
    epd_lcd_line_source_cb((line_cb_func_t)&push_pixels_isr, ctx);

    epd_set_mode(1);
    epd_lcd_start_frame();
    xSemaphoreTake(ctx->frame_done, portMAX_DELAY);
    epd_set_mode(0);

    free(ctx->static_line_buffer);
    ctx->static_line_buffer = NULL;
}

#define int_min(a, b) (((a) < (b)) ? (a) : (b))
__attribute__((optimize("O3"))) void IRAM_ATTR
lcd_calculate_frame(RenderContext_t* ctx, int thread_id) {
    assert(ctx->lut_lookup_func != NULL);
    uint8_t* input_line = ctx->feed_line_buffers[thread_id];

    LineQueue_t* lq = &ctx->line_queues[thread_id];
    int l = 0;

    // if there is an error, start the frame but don't feed data.
    if (ctx->error) {
        try_start_frame_once(ctx);
        ESP_LOGW("epd_lcd", "draw frame draw initiated, but an error flag is set: %X", ctx->error);
        return;
    }

    // line must be able to hold 2-pixel-per-byte or 1-pixel-per-byte data
    memset(input_line, 0x00, ctx->display_width);

    EpdRect area = ctx->area;
    int min_y, max_y, bytes_per_line, _ppB;
    const uint8_t* ptr_start;
    get_buffer_params(ctx, &bytes_per_line, &ptr_start, &min_y, &max_y, &_ppB);

    assert(area.width == ctx->display_width && area.x == 0 && !ctx->error);

    // Keep one slot per queue free before startup so producers cannot self-deadlock
    // while we are still building a larger ready prefix.
    int prestart_queue_limit = lq_effective_capacity(lq) - 1;
    int minimum_ready_lines = int_min(NUM_RENDER_THREADS * 4, ctx->lines_total);
    // Deeper queues are used to improve steady-state headroom, but we keep the
    // startup prefetch target capped so larger queues do not delay frame start.
    int startup_ready_target = int_min(prestart_queue_limit * NUM_RENDER_THREADS, 60);
    startup_ready_target = int_min(startup_ready_target, ctx->lines_total);

    while (true) {
        if (!frame_started_load(ctx)) {
            maybe_start_frame(ctx, lq, startup_ready_target, minimum_ready_lines, prestart_queue_limit);
            if (!frame_started_load(ctx) && lq_used(lq) >= prestart_queue_limit) {
                taskYIELD();
                continue;
            }
        }

        l = atomic_fetch_add(&ctx->lines_prepared, 1);
        if (l >= ctx->lines_total) {
            break;
        }

        if (l < min_y || l >= max_y
            || (ctx->drawn_lines != NULL && !ctx->drawn_lines[l - area.y])) {
            uint8_t* buf = NULL;
            while (buf == NULL) {
                // break in case of errors
                if (ctx->error & EPD_DRAW_EMPTY_LINE_QUEUE) {
                    printf("on err 1: %d %d\n", ctx->lines_prepared, ctx->lines_consumed);
                    lq_reset(lq);
                    return;
                };

                if (!frame_started_load(ctx)) {
                    maybe_start_frame(
                        ctx, lq, startup_ready_target, minimum_ready_lines, prestart_queue_limit
                    );
                    taskYIELD();
                }
                buf = lq_current(lq);
            }
            memset(buf, 0x00, lq->element_size);
            lq_commit(lq);
            publish_line(ctx, l, thread_id);
            maybe_start_frame(ctx, lq, startup_ready_target, minimum_ready_lines, prestart_queue_limit);
            continue;
        }

        uint32_t* lp = (uint32_t*)input_line;
        const uint8_t* ptr = ptr_start + bytes_per_line * (l - min_y);

        Cache_Start_DCache_Preload((uint32_t)ptr, ctx->display_width, 0);

        lp = (uint32_t*)ptr;

        uint8_t* buf = NULL;
        while (buf == NULL) {
            // break in case of errors
            if (ctx->error & EPD_DRAW_EMPTY_LINE_QUEUE) {
                lq_reset(lq);
                printf("on err 2: %d %d\n", ctx->lines_prepared, ctx->lines_consumed);
                return;
            };

            if (!frame_started_load(ctx)) {
                maybe_start_frame(ctx, lq, startup_ready_target, minimum_ready_lines, prestart_queue_limit);
                taskYIELD();
            }
            buf = lq_current(lq);
        }

        ctx->lut_lookup_func(lp, buf, ctx->conversion_lut, ctx->display_width);

        // apply the line mask
        epd_apply_line_mask_VE(buf, ctx->line_mask, ctx->display_width / 4);

        lq_commit(lq);
        publish_line(ctx, l, thread_id);
        maybe_start_frame(ctx, lq, startup_ready_target, minimum_ready_lines, prestart_queue_limit);
    }
}

#endif
