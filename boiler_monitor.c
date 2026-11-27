#include <furi.h>
#include <furi_hal_subghz.h>
#include <furi_hal_region.h>
#include <gui/gui.h>
#include <gui/view_port.h>
#include <input/input.h>
#include <dialogs/dialogs.h>
#include <storage/storage.h>
#include <flipper_format/flipper_format.h>
#include <flipper_format/flipper_format_i.h>
#include <lib/toolbox/stream/stream.h>
#include <lib/subghz/transmitter.h>
#include <lib/subghz/subghz_protocol_registry.h>
#include <lib/subghz/devices/devices.h>
#include <lib/subghz/devices/cc1101_int/cc1101_int_interconnect.h>
#include <lib/subghz/protocols/raw.h>
#include <furi_hal_power.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>

#include "dht11.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define SUBGHZ_DIR     "/ext/subghz"
#define CONFIG_DIR     "/ext/apps_data/boiler_monitor"
#define CONFIG_PATH    CONFIG_DIR "/config.bin"
#define CONFIG_MAGIC   0x424D5031U
#define CONFIG_VERSION 3U
#define DHT_READ_MS    2000U
#define TX_RETRY_MS    2000U
#define MIN_TARGET_C   0.0f
#define MAX_TARGET_C   30.0f
#define SUB_PATH_SIZE  256U

static const uint32_t tx_intervals_sec[] = {10, 15, 30, 60, 300, 900, 1800, 3600};
#define TX_INTERVAL_COUNT (sizeof(tx_intervals_sec) / sizeof(tx_intervals_sec[0]))

typedef enum {
    SendTargetAbove,
    SendTargetBelow,
} SendTarget;

typedef enum {
    SendModeOnce,
    SendModeInterval,
} SendMode;

typedef enum {
    TemperatureUnitCelsius,
    TemperatureUnitFahrenheit,
} TemperatureUnit;

typedef struct {
    uint32_t magic;
    uint32_t version;
    float above_temp;
    float below_temp;
    uint8_t send_target;
    uint8_t send_mode;
    uint8_t temperature_unit;
    uint8_t reserved;
    uint32_t tx_interval_sec;
    char above_sub_path[SUB_PATH_SIZE];
    char below_sub_path[SUB_PATH_SIZE];
} PresetConfig;

typedef enum {
    MenuSendTarget,
    MenuTemperatureUnit,
    MenuSubFile,
    MenuSendMode,
    MenuInterval,
    MenuItemCount,
} PresetMenuItem;

typedef struct {
    char path[256];
    char status[24];
    volatile bool complete;
    bool success;
} TxWorkerContext;

typedef struct {
    FuriMutex* mutex;
    bool running;
    bool armed;
    bool has_reading;
    bool sensor_error;
    bool menu_open;
    bool condition_active;
    bool once_sent;
    float temperature_c;
    float humidity_percent;
    PresetConfig config;
    uint8_t menu_selection;
    uint32_t tx_count;
    uint32_t next_read_tick;
    uint32_t next_tx_tick;
    const char* tx_status;
    FuriThread* tx_thread;
    TxWorkerContext* tx_worker;
    bool tx_in_progress;
} BoilerMonitorApp;

static void extract_filename(const char* full_path, char* out_name, size_t out_size) {
    const char* slash = strrchr(full_path, '/');
    const char* base = slash ? (slash + 1) : full_path;
    snprintf(out_name, out_size, "%s", base);
}

static void draw_scrolling_filename(
    Canvas* canvas,
    int32_t x,
    int32_t y,
    uint16_t max_width,
    const char* filename) {
    size_t filename_length = strlen(filename);
    if(canvas_string_width(canvas, filename) <= max_width) {
        canvas_draw_str(canvas, x, y, filename);
        return;
    }

    const size_t gap_length = 3;
    size_t cycle_length = filename_length + gap_length;
    uint32_t step_ticks = furi_ms_to_ticks(400);
    if(step_ticks == 0) step_ticks = 1;
    size_t offset = (furi_get_tick() / step_ticks) % cycle_length;
    char window[SUB_PATH_SIZE];
    size_t window_length = 0;

    while(window_length + 1 < sizeof(window)) {
        size_t position = (offset + window_length) % cycle_length;
        window[window_length] = position < filename_length ? filename[position] : ' ';
        window[window_length + 1] = '\0';
        if(canvas_string_width(canvas, window) > max_width) {
            window[window_length] = '\0';
            break;
        }
        window_length++;
    }

    canvas_draw_str(canvas, x, y, window);
}

static void config_set_defaults(PresetConfig* config) {
    memset(config, 0, sizeof(*config));
    config->magic = CONFIG_MAGIC;
    config->version = CONFIG_VERSION;
    config->above_temp = 20.0f;
    config->below_temp = 19.5f;
    config->send_target = SendTargetAbove;
    config->send_mode = SendModeInterval;
    config->temperature_unit = TemperatureUnitCelsius;
    config->tx_interval_sec = 15;
}

static bool config_is_valid(const PresetConfig* config) {
    bool interval_valid = false;
    for(size_t i = 0; i < TX_INTERVAL_COUNT; i++) {
        interval_valid |= config->tx_interval_sec == tx_intervals_sec[i];
    }
    return config->magic == CONFIG_MAGIC && config->version == CONFIG_VERSION &&
           config->above_temp >= MIN_TARGET_C && config->above_temp <= MAX_TARGET_C &&
           config->below_temp >= MIN_TARGET_C && config->below_temp <= MAX_TARGET_C &&
           config->send_target <= SendTargetBelow && config->send_mode <= SendModeInterval &&
           config->temperature_unit <= TemperatureUnitFahrenheit && interval_valid &&
           memchr(config->above_sub_path, '\0', sizeof(config->above_sub_path)) &&
           memchr(config->below_sub_path, '\0', sizeof(config->below_sub_path));
}

static float config_get_threshold(const PresetConfig* config) {
    return config->send_target == SendTargetAbove ? config->above_temp : config->below_temp;
}

static void update_condition(BoilerMonitorApp* app, uint32_t now) {
    bool above_target = app->config.send_target == SendTargetAbove;
    float threshold = config_get_threshold(&app->config);
    bool enters_condition = above_target ? app->temperature_c >= threshold :
                                           app->temperature_c <= threshold;
    bool exits_condition = above_target ? app->temperature_c < threshold :
                                          app->temperature_c > threshold;
    if(!app->condition_active && enters_condition) {
        app->condition_active = true;
        app->once_sent = false;
        app->next_tx_tick = now;
    } else if(app->condition_active && exits_condition) {
        app->condition_active = false;
        app->once_sent = false;
        if(app->armed) app->tx_status = "Condition cleared";
    }
}

static float temperature_to_display(const PresetConfig* config, float temperature_c) {
    return config->temperature_unit == TemperatureUnitFahrenheit ?
               temperature_c * 9.0f / 5.0f + 32.0f :
               temperature_c;
}

static void format_temperature(
    const PresetConfig* config,
    float temperature_c,
    char* output,
    size_t output_size) {
    if(config->temperature_unit == TemperatureUnitFahrenheit) {
        snprintf(
            output, output_size, "%.0f", (double)temperature_to_display(config, temperature_c));
    } else {
        snprintf(output, output_size, "%.1f", (double)temperature_c);
    }
}

static char* config_get_sub_path(PresetConfig* config) {
    return config->send_target == SendTargetAbove ? config->above_sub_path :
                                                    config->below_sub_path;
}

static const char* config_get_sub_path_const(const PresetConfig* config) {
    return config->send_target == SendTargetAbove ? config->above_sub_path :
                                                    config->below_sub_path;
}

static void config_load(PresetConfig* config) {
    config_set_defaults(config);

    Storage* storage = furi_record_open(RECORD_STORAGE);
    File* file = storage_file_alloc(storage);
    PresetConfig loaded;
    bool opened = storage_file_open(file, CONFIG_PATH, FSAM_READ, FSOM_OPEN_EXISTING);
    bool read_ok = opened && storage_file_read(file, &loaded, sizeof(loaded)) == sizeof(loaded);
    if(read_ok && loaded.magic == CONFIG_MAGIC && loaded.version == 2U) {
        loaded.version = CONFIG_VERSION;
        loaded.temperature_unit = TemperatureUnitCelsius;
        loaded.reserved = 0;
    }
    bool valid = read_ok && config_is_valid(&loaded);
    storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);

    if(valid) {
        *config = loaded;
    }
}

static bool config_save(const PresetConfig* config) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    storage_common_mkdir(storage, CONFIG_DIR);
    File* file = storage_file_alloc(storage);
    bool opened = storage_file_open(file, CONFIG_PATH, FSAM_WRITE, FSOM_CREATE_ALWAYS);
    size_t written = opened ? storage_file_write(file, config, sizeof(*config)) : 0;
    bool closed = storage_file_close(file);
    storage_file_free(file);
    furi_record_close(RECORD_STORAGE);
    return opened && written == sizeof(*config) && closed;
}

static bool select_sub_file(BoilerMonitorApp* app) {
    DialogsApp* dialogs = furi_record_open(RECORD_DIALOGS);
    DialogsFileBrowserOptions browser_options;
    dialog_file_browser_set_basic_options(&browser_options, ".sub", NULL);
    browser_options.base_path = SUBGHZ_DIR;
    browser_options.hide_ext = false;

    const char* current_path = config_get_sub_path_const(&app->config);
    FuriString* selected = furi_string_alloc_set(current_path[0] ? current_path : SUBGHZ_DIR);

    bool ok = dialog_file_browser_show(dialogs, selected, selected, &browser_options);
    if(ok && strlen(furi_string_get_cstr(selected)) < SUB_PATH_SIZE) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        snprintf(
            config_get_sub_path(&app->config), SUB_PATH_SIZE, "%s", furi_string_get_cstr(selected));
        app->once_sent = false;
        app->next_tx_tick = furi_get_tick();
        app->tx_status = "File ready";
        furi_mutex_release(app->mutex);
    } else if(ok) {
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        app->tx_status = "Path too long";
        furi_mutex_release(app->mutex);
        ok = false;
    }

    furi_string_free(selected);
    furi_record_close(RECORD_DIALOGS);
    return ok;
}

static FuriHalSubGhzPreset parse_preset_name(const char* name) {
    if(strcmp(name, "FuriHalSubGhzPresetCustom") == 0) {
        return FuriHalSubGhzPresetCustom;
    }
    if(strcmp(name, "FuriHalSubGhzPresetOok270Async") == 0) {
        return FuriHalSubGhzPresetOok270Async;
    } else if(strcmp(name, "FuriHalSubGhzPreset2FSKDev238Async") == 0) {
        return FuriHalSubGhzPreset2FSKDev238Async;
    } else if(strcmp(name, "FuriHalSubGhzPreset2FSKDev476Async") == 0) {
        return FuriHalSubGhzPreset2FSKDev476Async;
    }
    return FuriHalSubGhzPresetOok650Async;
}

static const char* raw_parse_status(SubGhzProtocolStatus status) {
    switch(status) {
    case SubGhzProtocolStatusErrorParserHeader:
        return "RAW bad header";
    case SubGhzProtocolStatusErrorParserFrequency:
        return "RAW missing freq";
    case SubGhzProtocolStatusErrorParserPreset:
        return "RAW missing preset";
    case SubGhzProtocolStatusErrorParserCustomPreset:
        return "RAW bad custom preset";
    case SubGhzProtocolStatusErrorParserProtocolName:
        return "RAW missing protocol";
    case SubGhzProtocolStatusErrorParserOthers:
        return "RAW bad data";
    case SubGhzProtocolStatusErrorEncoderGetUpload:
        return "RAW encode err";
    default:
        return "RAW deserializer err";
    }
}

static bool transmit_sub_file(const char* path, const char** out_status) {
    Storage* storage = furi_record_open(RECORD_STORAGE);
    FlipperFormat* ff = flipper_format_file_alloc(storage);
    FlipperFormat* raw_data = flipper_format_string_alloc();
    FuriString* str_val = furi_string_alloc();
    FuriString* protocol_name = furi_string_alloc();
    NotificationApp* notifications = NULL;
    bool success = false;

    do {
        if(!flipper_format_file_open_existing(ff, path)) {
            *out_status = "Open failed";
            break;
        }

        uint32_t version = 0;
        if(!flipper_format_read_header(ff, str_val, &version) ||
           (furi_string_cmp_str(str_val, "Flipper SubGhz RAW File") != 0 &&
            furi_string_cmp_str(str_val, "Flipper SubGhz Key File") != 0)) {
            *out_status = "Not SubGHz .sub";
            break;
        }

        uint32_t frequency = 868350000U;
        if(!flipper_format_read_uint32(ff, "Frequency", &frequency, 1)) {
            *out_status = "Bad freq";
            break;
        }

        if(!flipper_format_read_string(ff, "Preset", str_val)) {
            *out_status = "Bad preset";
            break;
        }
        FuriHalSubGhzPreset preset = parse_preset_name(furi_string_get_cstr(str_val));

        if(!flipper_format_read_string(ff, "Protocol", protocol_name)) {
            *out_status = "Bad protocol";
            break;
        }

        if(!furi_hal_region_is_frequency_allowed(frequency)) {
            *out_status = "TX region lock";
            break;
        }

        subghz_devices_init();
        const SubGhzDevice* device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);
        if(!device) {
            subghz_devices_deinit();
            *out_status = "No CC1101";
            break;
        }

        subghz_devices_begin(device);
        subghz_devices_reset(device);
        subghz_devices_load_preset(device, preset, NULL);
        subghz_devices_set_frequency(device, frequency);

        notifications = furi_record_open(RECORD_NOTIFICATION);
        furi_hal_power_suppress_charge_enter();
        notification_message(notifications, &sequence_blink_stop);
        notification_message(notifications, &sequence_blink_start_cyan);

        SubGhzEnvironment* env = subghz_environment_alloc();
        subghz_environment_set_protocol_registry(env, (void*)&subghz_protocol_registry);
        SubGhzTransmitter* tx =
            subghz_transmitter_alloc_init(env, furi_string_get_cstr(protocol_name));

        if(furi_string_cmp_str(protocol_name, "RAW") == 0) {
            flipper_format_file_close(ff);
            subghz_protocol_raw_gen_fff_data(raw_data, path, subghz_devices_get_name(device));
        } else {
            flipper_format_rewind(ff);
            stream_clean(flipper_format_get_raw_stream(raw_data));
            stream_copy_full(
                flipper_format_get_raw_stream(ff), flipper_format_get_raw_stream(raw_data));
        }

        SubGhzProtocolStatus deserialize_status =
            tx ? subghz_transmitter_deserialize(tx, raw_data) : SubGhzProtocolStatusError;

        if(deserialize_status == SubGhzProtocolStatusOk) {
            if(subghz_devices_start_async_tx(device, subghz_transmitter_yield, tx)) {
                uint32_t tx_wait_ticks = furi_ms_to_ticks(10000);
                uint32_t tx_started_tick = furi_get_tick();
                while(!subghz_devices_is_async_complete_tx(device) &&
                      (int32_t)(furi_get_tick() - tx_started_tick) < (int32_t)tx_wait_ticks) {
                    furi_delay_ms(5);
                }
                bool tx_completed = subghz_devices_is_async_complete_tx(device);
                subghz_devices_stop_async_tx(device);
                subghz_devices_idle(device);
                subghz_transmitter_stop(tx);
                success = tx_completed;
                *out_status = tx_completed ? "TX OK" : "TX timeout";
            } else {
                *out_status = "TX start err";
            }
        } else {
            *out_status = raw_parse_status(deserialize_status);
        }

        subghz_devices_idle(device);
        if(tx) subghz_transmitter_free(tx);
        subghz_environment_free(env);
        if(notifications) {
            notification_message(notifications, &sequence_blink_stop);
            furi_record_close(RECORD_NOTIFICATION);
            notifications = NULL;
            furi_hal_power_suppress_charge_exit();
        }
        subghz_devices_sleep(device);
        subghz_devices_end(device);
        subghz_devices_deinit();
    } while(false);

    flipper_format_free(raw_data);
    furi_string_free(str_val);
    furi_string_free(protocol_name);
    flipper_format_free(ff);
    furi_record_close(RECORD_STORAGE);
    return success;
}

static int32_t tx_worker(void* context) {
    TxWorkerContext* worker = context;
    const char* status = "TX err";
    worker->success = transmit_sub_file(worker->path, &status);
    snprintf(worker->status, sizeof(worker->status), "%s", status);
    worker->complete = true;
    return worker->success ? 0 : -1;
}

static void draw_callback(Canvas* canvas, void* context) {
    BoilerMonitorApp* app = context;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    canvas_clear(canvas);

    if(app->menu_open) {
        static const char* labels[MenuItemCount] = {
            "Trigger",
            "Units",
            "File",
            "Send mode",
            "Interval",
        };
        char values[MenuItemCount][20];
        char filename[SUB_PATH_SIZE];

        snprintf(
            values[MenuSendTarget],
            sizeof(values[MenuSendTarget]),
            "%s",
            app->config.send_target == SendTargetAbove ? "Above" : "Below");
        snprintf(
            values[MenuTemperatureUnit],
            sizeof(values[MenuTemperatureUnit]),
            "%s",
            app->config.temperature_unit == TemperatureUnitCelsius ? "Celsius" : "Fahrenheit");
        const char* selected_path = config_get_sub_path_const(&app->config);
        if(selected_path[0]) {
            extract_filename(selected_path, filename, sizeof(filename));
        } else {
            snprintf(filename, sizeof(filename), "<choose file>");
        }
        snprintf(
            values[MenuSendMode],
            sizeof(values[MenuSendMode]),
            "%s",
            app->config.send_mode == SendModeOnce ? "Once" : "Interval");
        uint32_t interval = app->config.tx_interval_sec;
        if(app->config.send_mode == SendModeOnce) {
            snprintf(values[MenuInterval], sizeof(values[MenuInterval]), "Not used");
        } else if(interval < 60) {
            snprintf(
                values[MenuInterval],
                sizeof(values[MenuInterval]),
                "%lus",
                (unsigned long)interval);
        } else if(interval >= 3600) {
            snprintf(
                values[MenuInterval],
                sizeof(values[MenuInterval]),
                "%luh",
                (unsigned long)(interval / 3600));
        } else {
            snprintf(
                values[MenuInterval],
                sizeof(values[MenuInterval]),
                "%lum",
                (unsigned long)(interval / 60));
        }

        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str(canvas, 2, 9, "TX SETTINGS");
        canvas_draw_str_aligned(canvas, 126, 9, AlignRight, AlignBottom, "BACK: SAVE");
        canvas_draw_line(canvas, 0, 12, 128, 12);

        uint8_t first_item = app->menu_selection > 3 ? app->menu_selection - 3 : 0;
        for(uint8_t row = 0; row < 4 && first_item + row < MenuItemCount; row++) {
            uint8_t item = first_item + row;
            uint8_t y = 23 + row * 10;
            if(item == app->menu_selection) {
                canvas_set_color(canvas, ColorBlack);
                canvas_draw_box(canvas, 1, y - 8, 126, 9);
                canvas_invert_color(canvas);
            }
            if(item == MenuSubFile) {
                canvas_draw_str(canvas, 11, y, labels[item]);
                draw_scrolling_filename(canvas, 43, y, 80, filename);
            } else {
                canvas_draw_str(canvas, 11, y, labels[item]);
                canvas_draw_str_aligned(canvas, 124, y, AlignRight, AlignBottom, values[item]);
            }
            if(item == app->menu_selection) {
                canvas_invert_color(canvas);
                canvas_set_color(canvas, ColorBlack);
            }
        }
        if(app->menu_selection == MenuSubFile) {
            canvas_draw_str(canvas, 2, 63, "UP/DN move  OK pick  L clear");
        } else if(app->menu_selection == MenuInterval && app->config.send_mode == SendModeOnce) {
            canvas_draw_str(canvas, 2, 63, "Interval off in Once mode");
        } else {
            canvas_draw_str(canvas, 2, 63, "UP/DN move   OK change");
        }

        furi_mutex_release(app->mutex);
        return;
    }

    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 9, "BOILER");
    canvas_draw_str_aligned(
        canvas,
        126,
        9,
        AlignRight,
        AlignBottom,
        app->armed ? "ACTIVE | OK: PAUSE" : "PAUSED | OK: START");
    canvas_draw_line(canvas, 0, 12, 128, 12);

    char now_label[12];
    char set_label[16];
    snprintf(
        now_label,
        sizeof(now_label),
        "NOW (%c)",
        app->config.temperature_unit == TemperatureUnitCelsius ? 'C' : 'F');
    snprintf(
        set_label,
        sizeof(set_label),
        "SET %s (%c)",
        app->config.send_target == SendTargetAbove ? ">" : "<",
        app->config.temperature_unit == TemperatureUnitCelsius ? 'C' : 'F');
    canvas_draw_str_aligned(canvas, 32, 21, AlignCenter, AlignBottom, now_label);
    canvas_draw_str_aligned(canvas, 96, 21, AlignCenter, AlignBottom, set_label);

    char big_now[12];
    char big_set[12];
    if(app->has_reading) {
        format_temperature(&app->config, app->temperature_c, big_now, sizeof(big_now));
    } else {
        snprintf(
            big_now,
            sizeof(big_now),
            app->config.temperature_unit == TemperatureUnitCelsius ? "--.-" : "---");
    }
    format_temperature(&app->config, config_get_threshold(&app->config), big_set, sizeof(big_set));

    canvas_set_font(canvas, FontBigNumbers);
    canvas_draw_str_aligned(canvas, 32, 38, AlignCenter, AlignBottom, big_now);
    canvas_draw_str_aligned(canvas, 96, 38, AlignCenter, AlignBottom, big_set);

    canvas_draw_line(canvas, 64, 13, 64, 42);
    canvas_draw_line(canvas, 0, 42, 128, 42);

    canvas_set_font(canvas, FontSecondary);
    char selected_name[SUB_PATH_SIZE];
    const char* selected_path = config_get_sub_path_const(&app->config);
    if(selected_path[0]) {
        extract_filename(selected_path, selected_name, sizeof(selected_name));
    } else {
        snprintf(selected_name, sizeof(selected_name), "<choose in menu>");
    }
    canvas_draw_str(canvas, 2, 52, "SUB >:");
    draw_scrolling_filename(canvas, 31, 52, 94, selected_name);

    char status_line[24];
    if(app->sensor_error) {
        snprintf(status_line, sizeof(status_line), "DHT11 ERROR");
    } else {
        snprintf(status_line, sizeof(status_line), "%s", app->tx_status);
    }

    char tx_info[20];
    if(app->sensor_error) {
        snprintf(tx_info, sizeof(tx_info), "WAIT:SENSOR");
    } else if(
        app->armed && app->condition_active &&
        (app->config.send_mode == SendModeInterval || !app->once_sent)) {
        uint32_t now = furi_get_tick();
        uint32_t remaining = (int32_t)(app->next_tx_tick - now) > 0 ?
                                 ((app->next_tx_tick - now) / furi_kernel_get_tick_frequency()) :
                                 0;
        snprintf(tx_info, sizeof(tx_info), "NEXT:%lus", (unsigned long)remaining);
    } else {
        snprintf(tx_info, sizeof(tx_info), "TX:%lu", (unsigned long)app->tx_count);
    }

    uint16_t status_width_limit = 122 - canvas_string_width(canvas, tx_info) - 4;
    size_t status_length = strlen(status_line);
    while(status_length && canvas_string_width(canvas, status_line) > status_width_limit) {
        status_line[--status_length] = '\0';
    }
    canvas_draw_str(canvas, 2, 62, status_line);
    canvas_draw_str_aligned(canvas, 126, 62, AlignRight, AlignBottom, tx_info);

    furi_mutex_release(app->mutex);
}

static void input_callback(InputEvent* event, void* context) {
    FuriMessageQueue* queue = context;
    if(event->type == InputTypeShort || event->type == InputTypeRepeat ||
       event->type == InputTypeLong) {
        furi_message_queue_put(queue, event, 0);
    }
}

static void adjust_config_value(PresetConfig* config, PresetMenuItem item, int8_t delta) {
    switch(item) {
    case MenuSendTarget:
        config->send_target = config->send_target == SendTargetAbove ? SendTargetBelow :
                                                                       SendTargetAbove;
        break;
    case MenuTemperatureUnit:
        config->temperature_unit = config->temperature_unit == TemperatureUnitCelsius ?
                                       TemperatureUnitFahrenheit :
                                       TemperatureUnitCelsius;
        break;
    case MenuSendMode:
        config->send_mode = config->send_mode == SendModeOnce ? SendModeInterval : SendModeOnce;
        break;
    case MenuInterval: {
        if(config->send_mode == SendModeOnce) break;
        size_t index = 0;
        while(index + 1 < TX_INTERVAL_COUNT &&
              tx_intervals_sec[index] != config->tx_interval_sec) {
            index++;
        }
        index = (index + (delta > 0 ? 1 : TX_INTERVAL_COUNT - 1)) % TX_INTERVAL_COUNT;
        config->tx_interval_sec = tx_intervals_sec[index];
        break;
    }
    case MenuSubFile:
    case MenuItemCount:
        break;
    }
}

static void adjust_active_threshold(PresetConfig* config, int8_t delta) {
    float step_c = config->temperature_unit == TemperatureUnitFahrenheit ? (5.0f / 9.0f) : 0.5f;
    if(config->send_target == SendTargetAbove) {
        float value = config->above_temp + (delta > 0 ? step_c : -step_c);
        if(value <= MAX_TARGET_C && value >= MIN_TARGET_C) {
            config->above_temp = value;
        }
    } else {
        float value = config->below_temp + (delta > 0 ? step_c : -step_c);
        if(value >= MIN_TARGET_C && value <= MAX_TARGET_C) {
            config->below_temp = value;
        }
    }
}

int32_t boiler_monitor_app(void* context) {
    UNUSED(context);

    BoilerMonitorApp app = {
        .mutex = furi_mutex_alloc(FuriMutexTypeNormal),
        .running = true,
        .armed = false,
        .has_reading = false,
        .sensor_error = false,
        .menu_open = false,
        .condition_active = false,
        .once_sent = false,
        .temperature_c = 0.0f,
        .humidity_percent = 0.0f,
        .tx_count = 0,
        .next_read_tick = furi_get_tick(),
        .next_tx_tick = furi_get_tick(),
        .tx_status = "No file",
        .tx_thread = NULL,
        .tx_worker = NULL,
        .tx_in_progress = false,
    };
    config_load(&app.config);
    app.tx_status = config_get_sub_path_const(&app.config)[0] ? "OK: ARM" : "Pick file in menu";

    FuriMessageQueue* queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    ViewPort* view_port = view_port_alloc();

    dht11_release();
    view_port_draw_callback_set(view_port, draw_callback, &app);
    view_port_input_callback_set(view_port, input_callback, queue);

    Gui* gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(gui, view_port, GuiLayerFullscreen);

    InputEvent event;
    while(app.running) {
        bool open_browser = false;

        if(app.tx_worker && app.tx_worker->complete) {
            furi_mutex_acquire(app.mutex, FuriWaitForever);
            if(app.tx_worker->success) app.tx_count++;
            app.tx_status = app.tx_worker->status;
            app.tx_in_progress = false;
            furi_mutex_release(app.mutex);

            furi_thread_join(app.tx_thread);
            furi_thread_free(app.tx_thread);
            free(app.tx_worker);
            app.tx_thread = NULL;
            app.tx_worker = NULL;
        }

        if(furi_message_queue_get(queue, &event, 100) == FuriStatusOk) {
            furi_mutex_acquire(app.mutex, FuriWaitForever);
            if(!app.menu_open) {
                if(event.key == InputKeyRight && event.type == InputTypeLong) {
                    app.menu_open = true;
                    app.menu_selection = MenuSendTarget;
                } else if(
                    event.key == InputKeyUp &&
                    (event.type == InputTypeShort || event.type == InputTypeRepeat)) {
                    adjust_active_threshold(&app.config, 1);
                    app.condition_active = false;
                    app.once_sent = false;
                } else if(
                    event.key == InputKeyDown &&
                    (event.type == InputTypeShort || event.type == InputTypeRepeat)) {
                    adjust_active_threshold(&app.config, -1);
                    app.condition_active = false;
                    app.once_sent = false;
                } else if(event.key == InputKeyOk && event.type == InputTypeShort) {
                    if(!config_get_sub_path_const(&app.config)[0]) {
                        app.tx_status = "Pick file in menu";
                    } else {
                        app.armed = !app.armed;
                        app.next_read_tick = furi_get_tick();
                        app.tx_status = app.armed ? "Monitoring" : "Paused";
                    }
                } else if(event.key == InputKeyBack && event.type == InputTypeShort) {
                    app.running = false;
                }
            } else if(event.key == InputKeyBack && event.type == InputTypeShort) {
                app.menu_open = false;
                furi_mutex_release(app.mutex);
                if(!config_save(&app.config)) {
                    furi_mutex_acquire(app.mutex, FuriWaitForever);
                    app.tx_status = "Config save err";
                    furi_mutex_release(app.mutex);
                }
                view_port_update(view_port);
                continue;
            } else if(
                (event.key == InputKeyUp || event.key == InputKeyDown) &&
                (event.type == InputTypeShort || event.type == InputTypeRepeat)) {
                int8_t delta = event.key == InputKeyUp ? -1 : 1;
                app.menu_selection =
                    (uint8_t)((app.menu_selection + delta + MenuItemCount) % MenuItemCount);
            } else if(
                (event.key == InputKeyLeft || event.key == InputKeyRight) &&
                event.type == InputTypeShort) {
                PresetMenuItem item = (PresetMenuItem)app.menu_selection;
                if(item == MenuSubFile) {
                    if(event.key == InputKeyLeft && event.type == InputTypeShort) {
                        config_get_sub_path(&app.config)[0] = '\0';
                        app.tx_status = "File cleared";
                    } else if(event.key == InputKeyRight && event.type == InputTypeShort) {
                        open_browser = true;
                    }
                } else {
                    uint8_t previous_target = app.config.send_target;
                    uint8_t previous_mode = app.config.send_mode;
                    adjust_config_value(&app.config, item, event.key == InputKeyRight ? 1 : -1);
                    if(previous_target != app.config.send_target) {
                        app.condition_active = false;
                        app.once_sent = false;
                        if(app.has_reading && !app.sensor_error) {
                            update_condition(&app, furi_get_tick());
                        }
                    }
                    if(previous_mode != app.config.send_mode) {
                        app.once_sent = false;
                        app.next_tx_tick = furi_get_tick();
                        if(app.config.send_mode == SendModeInterval) {
                            app.next_tx_tick +=
                                furi_ms_to_ticks(app.config.tx_interval_sec * 1000U);
                        }
                    }
                }
            } else if(event.key == InputKeyOk && event.type == InputTypeShort) {
                PresetMenuItem item = (PresetMenuItem)app.menu_selection;
                if(item == MenuSubFile) {
                    open_browser = true;
                } else {
                    uint8_t previous_target = app.config.send_target;
                    uint8_t previous_mode = app.config.send_mode;
                    adjust_config_value(&app.config, item, 1);
                    if(previous_target != app.config.send_target) {
                        app.condition_active = false;
                        app.once_sent = false;
                        if(app.has_reading && !app.sensor_error) {
                            update_condition(&app, furi_get_tick());
                        }
                    }
                    if(previous_mode != app.config.send_mode) {
                        app.once_sent = false;
                        app.next_tx_tick = furi_get_tick();
                        if(app.config.send_mode == SendModeInterval) {
                            app.next_tx_tick +=
                                furi_ms_to_ticks(app.config.tx_interval_sec * 1000U);
                        }
                    }
                }
            }
            furi_mutex_release(app.mutex);
        }

        if(open_browser) {
            select_sub_file(&app);
            view_port_update(view_port);
            continue;
        }

        furi_mutex_acquire(app.mutex, FuriWaitForever);
        uint32_t now = furi_get_tick();
        bool read_due = (int32_t)(now - app.next_read_tick) >= 0;
        /* Keep DHT11 pulse timing isolated from the active radio worker. */
        bool should_read = read_due && !app.tx_in_progress;
        if(should_read) {
            app.next_read_tick = now + furi_ms_to_ticks(DHT_READ_MS);
        }
        furi_mutex_release(app.mutex);

        if(should_read) {
            Dht11Reading reading;
            bool read_ok = dht11_read(&reading);

            furi_mutex_acquire(app.mutex, FuriWaitForever);
            app.sensor_error = !read_ok;
            if(read_ok) {
                app.temperature_c = reading.temperature_c;
                app.humidity_percent = reading.humidity_percent;
                app.has_reading = true;
                update_condition(&app, furi_get_tick());
            }
            furi_mutex_release(app.mutex);
        }

        char path_copy[SUB_PATH_SIZE] = {0};
        bool do_tx = false;
        furi_mutex_acquire(app.mutex, FuriWaitForever);
        now = furi_get_tick();
        bool due = (int32_t)(now - app.next_tx_tick) >= 0 &&
                   (app.config.send_mode == SendModeInterval || !app.once_sent);
        if(app.armed && app.has_reading && !app.sensor_error && app.condition_active && due &&
           !app.tx_in_progress) {
            const char* selected_path = config_get_sub_path_const(&app.config);
            if(selected_path[0]) {
                if(app.config.send_mode == SendModeInterval) {
                    app.next_tx_tick = now + furi_ms_to_ticks(app.config.tx_interval_sec * 1000U);
                }
                snprintf(path_copy, sizeof(path_copy), "%s", selected_path);
                do_tx = true;
            } else {
                app.tx_status = "Pick file in menu";
                if(app.config.send_mode == SendModeOnce) {
                    app.once_sent = true;
                } else {
                    app.next_tx_tick = now + furi_ms_to_ticks(app.config.tx_interval_sec * 1000U);
                }
            }
        }
        furi_mutex_release(app.mutex);

        if(do_tx) {
            TxWorkerContext* worker = malloc(sizeof(TxWorkerContext));
            if(worker) {
                memset(worker, 0, sizeof(TxWorkerContext));
                snprintf(worker->path, sizeof(worker->path), "%s", path_copy);
                FuriThread* thread = furi_thread_alloc_ex("BoilerTx", 4096, tx_worker, worker);
                if(thread) {
                    furi_mutex_acquire(app.mutex, FuriWaitForever);
                    app.tx_worker = worker;
                    app.tx_thread = thread;
                    app.tx_in_progress = true;
                    if(app.config.send_mode == SendModeOnce) app.once_sent = true;
                    app.tx_status = "Transmitting";
                    furi_mutex_release(app.mutex);
                    furi_thread_start(thread);
                } else {
                    free(worker);
                    furi_mutex_acquire(app.mutex, FuriWaitForever);
                    app.tx_status = "TX alloc err";
                    if(app.config.send_mode == SendModeOnce) {
                        app.next_tx_tick = furi_get_tick() + furi_ms_to_ticks(TX_RETRY_MS);
                    }
                    furi_mutex_release(app.mutex);
                }
            } else {
                furi_mutex_acquire(app.mutex, FuriWaitForever);
                app.tx_status = "TX alloc err";
                if(app.config.send_mode == SendModeOnce) {
                    app.next_tx_tick = furi_get_tick() + furi_ms_to_ticks(TX_RETRY_MS);
                }
                furi_mutex_release(app.mutex);
            }
        }

        view_port_update(view_port);
    }

    if(!config_save(&app.config)) {
        FURI_LOG_E("BoilerMonitor", "Failed to save settings");
    }
    if(app.tx_thread) {
        furi_thread_join(app.tx_thread);
        furi_thread_free(app.tx_thread);
        free(app.tx_worker);
        app.tx_thread = NULL;
        app.tx_worker = NULL;
    }
    dht11_release();
    gui_remove_view_port(gui, view_port);
    view_port_free(view_port);
    furi_record_close(RECORD_GUI);
    furi_message_queue_free(queue);
    furi_mutex_free(app.mutex);

    return 0;
}
