// JLP WASM entry point. Mirrors the firmware's layout-apply pipeline
// (parse JSON, build under a staging parent, swap atomically) but
// runs in the browser instead of on the P4. The widget LVGL code path
// is the firmware's widget_factory.cpp compiled byte-for-byte.
//
// JS bridge:
//   jlp_init(w, h)                — set up LVGL + SDL window (canvas).
//   jlp_resize(w, h)              — follow a device of another size.
//   jlp_apply_layout(json)        — render the layout, return "" on
//                                   success or a UTF-8 error string.
//   jlp_set_value_float(p, v)     — push a simulated SK float.
//   jlp_set_value_int  (p, v)     — push a simulated SK int.
//   jlp_set_value_bool (p, b)     — push a simulated SK bool.
//
// jlp_apply_layout is the designer's preview: the first screen only,
// no tab strip, no input. A browser display (signalk-espos-hmi) runs
// the layout instead, through the firmware's own LayoutPlayer:
//   jlp_run_layout(json)          — every screen, tab strip, swipes.
//   jlp_select_screen(id)         — show a screen as a tab tap would.
//   jlp_pointer(x, y, pressed)    — touch in, in display pixels.
//   jlp_widget_catalog()          — the kinds and fields this build has.
// Writes leave through Module.jlpPut / Module.jlpAck (wasm_stubs.cpp).
// LayoutManager itself stays out: persistence, the status strip and the
// device overlays belong to the panel.

#include <emscripten.h>
#include <emscripten/html5.h>
#include <emscripten/bind.h>

#include <deque>
#include <set>
#include <string>

#include "lvgl.h"
#include "src/drivers/sdl/lv_sdl_window.h"

// SDL2 is statically linked through emscripten's port. We only need
// SDL_EventState here to tell SDL "don't watch for keyboard / mouse"
// — the designer drives LVGL from JS, not from page input. Without
// this, SDL's keydown handler captures every key on the page (the
// canvas + document target) and the inspector inputs / F12 / Ctrl-R
// stop working because preventDefault eats every keystroke.
#include <SDL2/SDL_events.h>

#include <ArduinoJson.h>

#include "subject_registry.h"
#include "zone_registry.h"
#include "notifications_registry.h"
#include "widgets/widget_factory.h"
#include "widgets/widget_catalog.h"
#include "layout/layout_player.h"

static lv_display_t* disp = nullptr;
static lv_obj_t*     current_root = nullptr;

// Touch for a browser display. The page feeds it through jlp_pointer;
// the designer never does, so its preview stays inert.
// Events queue up so a tap shorter than LVGL's read period still lands.
struct PointerEvent {
  lv_point_t at;
  bool down;
};
static lv_indev_t* pointer = nullptr;
static std::deque<PointerEvent> pointer_events;
static PointerEvent pointer_last{{0, 0}, false};

static void read_pointer(lv_indev_t*, lv_indev_data_t* data) {
  if (!pointer_events.empty()) {
    pointer_last = pointer_events.front();
    pointer_events.pop_front();
  }
  data->point = pointer_last.at;
  data->state = pointer_last.down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  data->continue_reading = !pointer_events.empty();
}

// Height of the panel's status strip (firmware status_overlay.cpp,
// kStripHeight). The panel puts every page below it, so a widget's y is
// measured from the strip's lower edge; the designer draws the strip
// itself over this canvas.
static constexpr int kStatusStripHeight = 28;

// LVGL time follows the clock, not the frame rate: a 120 Hz screen or a
// throttled tab would otherwise run the swipe, peek and toggle snap-back
// timers at the wrong speed.
static EM_BOOL frame(double t, void*) {
  static double last = t;
  const double elapsed = t - last;
  last = t;
  if (elapsed > 0) lv_tick_inc(static_cast<uint32_t>(elapsed + 0.5));
  lv_timer_handler();
  return EM_TRUE;
}

extern "C" {

EMSCRIPTEN_KEEPALIVE
int jlp_init(int w, int h) {
  if (disp) return 0;  // idempotent
  lv_init();
  disp = lv_sdl_window_create(w, h);
  if (!disp) return -1;
  // Stop SDL from intercepting keyboard + mouse on the page. Each
  // SDL_EventState(..., SDL_DISABLE) call removes the corresponding
  // event class from SDL's processing AND tells emscripten's SDL2
  // port to stop installing its document-level listeners for it.
  // Without these, the embedded LVGL canvas grabs every keystroke
  // (F12, Ctrl-Shift-R, typing in the designer's inspector inputs)
  // because SDL's default keydown handler calls preventDefault.
  SDL_EventState(SDL_KEYDOWN,     SDL_DISABLE);
  SDL_EventState(SDL_KEYUP,       SDL_DISABLE);
  SDL_EventState(SDL_TEXTINPUT,   SDL_DISABLE);
  SDL_EventState(SDL_TEXTEDITING, SDL_DISABLE);
  SDL_EventState(SDL_MOUSEMOTION, SDL_DISABLE);
  SDL_EventState(SDL_MOUSEBUTTONDOWN, SDL_DISABLE);
  SDL_EventState(SDL_MOUSEBUTTONUP,   SDL_DISABLE);
  SDL_EventState(SDL_MOUSEWHEEL,      SDL_DISABLE);
  // Before any LayoutPlayer: it watches presses on the first input device.
  pointer = lv_indev_create();
  lv_indev_set_type(pointer, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(pointer, read_pointer);
  lv_obj_t* scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_hex(jlp::theme::palette().screen),
                            LV_PART_MAIN);
  emscripten_request_animation_frame_loop(frame, nullptr);
  return 0;
}

// The last layout applied, so a theme switch can rebuild it.
static std::string last_layout;

// The running layout of a browser display, once jlp_run_layout was called.
static jlp::LayoutPlayer* player = nullptr;
static lv_obj_t* run_parent = nullptr;
static std::string run_layout;
const char* jlp_run_layout(const char* json);

// jlp_init runs once per page, usually before the designer knows which
// device it talks to, and SDL stays bound to that first window. A panel
// of another size (a 320x240 M5Stack Tough after the 1024x600 default)
// then left the canvas resized by the page but LVGL still drawing the
// old resolution: an empty preview. The SDL driver resizes its buffers,
// texture and window on LV_EVENT_RESOLUTION_CHANGED. A layout already
// applied is rebuilt, as on a theme switch: its pages were sized from the
// old resolution.
const char* jlp_apply_layout(const char* json);

EMSCRIPTEN_KEEPALIVE
int jlp_resize(int w, int h) {
  if (!disp) return -1;
  if (w <= 0 || h <= 0) return -2;
  if (lv_display_get_horizontal_resolution(disp) == w &&
      lv_display_get_vertical_resolution(disp) == h) {
    return 0;
  }
  const int32_t old_w = lv_display_get_horizontal_resolution(disp);
  const int32_t old_h = lv_display_get_vertical_resolution(disp);
  lv_display_set_resolution(disp, w, h);
  if (player != nullptr && !run_layout.empty()) {
    const std::string json = run_layout;
    const std::string screen = player->active_screen();
    if (*jlp_run_layout(json.c_str()) != '\0') {
      lv_display_set_resolution(disp, old_w, old_h);
      return -3;
    }
    // Back to the tab shown; screens without an id cannot be told apart
    // from outside the player and start on the first.
    if (!screen.empty()) player->select_screen(screen);
  } else if (!last_layout.empty()) {
    // A copy: the apply stores the JSON it was given into last_layout.
    const std::string json = last_layout;
    if (*jlp_apply_layout(json.c_str()) != '\0') {
      // The previous layout stays up; so does its size, and a retry of
      // the same size rebuilds again instead of returning early.
      lv_display_set_resolution(disp, old_w, old_h);
      return -3;
    }
  }
  lv_obj_invalidate(lv_screen_active());
  return 0;
}

EMSCRIPTEN_KEEPALIVE
const char* jlp_apply_layout(const char* json) {
  static std::string err_storage;
  err_storage.clear();
  if (!disp) { err_storage = "jlp_init not called"; return err_storage.c_str(); }

  JsonDocument doc;
  DeserializationError de = deserializeJson(doc, json);
  if (de) {
    err_storage = std::string("parse: ") + de.c_str();
    return err_storage.c_str();
  }

  lv_obj_t* screen = lv_screen_active();
  if (player != nullptr) {
    err_storage = "a layout is running";
    return err_storage.c_str();
  }
  lv_obj_t* staging = lv_obj_create(screen);
  // status_overlay defaults to true on the panel (layout_manager.cpp).
  const bool strip = doc["status_overlay"] | true;
  const int top = strip ? kStatusStripHeight : 0;
  lv_obj_set_size(staging, lv_pct(100), lv_display_get_vertical_resolution(disp) - top);
  lv_obj_set_pos(staging, 0, top);
  lv_obj_set_style_bg_opa(staging, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(staging, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(staging, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(staging, 0, LV_PART_MAIN);
  lv_obj_clear_flag(staging, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(staging, LV_OBJ_FLAG_HIDDEN);

  // Pull the first screen's widgets only — the designer's WASM canvas
  // is single-screen by definition (the tab strip lives outside the
  // canvas surface). Switching tabs in the designer re-calls
  // jlp_apply_layout with the active screen's spec wrapped in the
  // same top-level shape.
  JsonArrayConst screens = doc["screens"];
  if (screens.isNull() || screens.size() == 0) {
    lv_obj_delete(staging);
    err_storage = "no screens";
    return err_storage.c_str();
  }
  // The layout's `theme`, handled as layout_manager.cpp does on the
  // device: apply_theme() sets the mode and the default fg/accent before
  // any widget is built, then the root takes the palette's screen color,
  // or `theme.bg` except at night. A failed build puts the previous
  // theme back, since the previous layout stays on screen and its
  // widgets read the globals at update time.
  JsonObjectConst theme = doc["theme"];
  const jlp::ThemeColors prev_theme = jlp::current_theme();
  jlp::apply_theme(theme);
  const jlp::theme::Palette& pal = jlp::theme::palette();
  uint32_t screen_bg = pal.screen;
  uint32_t theme_bg = 0;
  if (pal.mode != jlp::theme::Mode::Night && !theme.isNull() &&
      jlp::parse_hex_color(theme["bg"] | (const char*)nullptr, &theme_bg)) {
    screen_bg = theme_bg;
  }
  lv_obj_set_style_bg_color(staging, lv_color_hex(screen_bg), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(staging, LV_OPA_COVER, LV_PART_MAIN);

  JsonArrayConst widgets = screens[0]["widgets"];
  std::set<std::string> live_paths;
  jlp::BuildCtx ctx{ staging, jlp::registry(), live_paths };
  if (!widgets.isNull()) {
    for (JsonObjectConst spec : widgets) {
      std::string werr;
      lv_obj_t* w = jlp::build_widget(ctx, spec, &werr);
      if (!w) {
        jlp::restore_theme(prev_theme);
        lv_obj_delete(staging);
        err_storage = std::string("widget: ") + werr;
        return err_storage.c_str();
      }
    }
  }

  // Atomic swap: unhide, then delete the previous root.
  lv_obj_clear_flag(staging, LV_OBJ_FLAG_HIDDEN);
  if (current_root) lv_obj_delete(current_root);
  current_root = staging;
  lv_obj_set_style_bg_color(screen, lv_color_hex(pal.screen), LV_PART_MAIN);
  if (json != last_layout.c_str()) last_layout = json;

  return "";  // success
}

// The panel's theme for the preview, as the panel's own setting would
// pick it: "auto", "day", "dark" or "night". Rebuilds the last layout.
// Returns "" on success, or why not.
EMSCRIPTEN_KEEPALIVE
const char* jlp_set_theme(const char* mode) {
  jlp::theme::Mode m;
  if (!jlp::theme::parse_mode(mode, &m)) return "unknown theme";
  if (m == jlp::theme::g_panel_choice) return "";
  const jlp::theme::Mode prev = jlp::theme::g_panel_choice;
  jlp::theme::g_panel_choice = m;
  if (player != nullptr) {
    static std::string err_storage;
    jlp::LayoutPlayer::Result r = player->rebuild_for_theme();
    if (!r.ok) {
      jlp::theme::g_panel_choice = prev;
      jlp::theme::sync_lvgl_theme();
      err_storage = r.err;
      return err_storage.c_str();
    }
    lv_obj_set_style_bg_color(lv_screen_active(),
                              lv_color_hex(jlp::theme::palette().screen),
                              LV_PART_MAIN);
    return "";
  }
  if (last_layout.empty()) return "";
  // A copy: the apply stores the JSON it was given into last_layout.
  const std::string json = last_layout;
  const char* err = jlp_apply_layout(json.c_str());
  // The previous layout stays up on a failed build, so keep its theme.
  if (*err) {
    jlp::theme::g_panel_choice = prev;
    jlp::theme::sync_lvgl_theme();
  }
  return err;
}

// Runs `json` as the panel does: every screen with its tab strip and page
// swipes, built and swapped by the firmware's LayoutPlayer. Returns "" on
// success or why not; a failed layout leaves the previous one running.
// The designer's single-screen preview is gone from then on.
EMSCRIPTEN_KEEPALIVE
const char* jlp_run_layout(const char* json) {
  static std::string err_storage;
  if (!disp) return "jlp_init not called";
  lv_obj_t* screen = lv_screen_active();
  const bool first = player == nullptr;
  if (first) {
    run_parent = lv_obj_create(screen);
    lv_obj_set_style_bg_opa(run_parent, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(run_parent, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(run_parent, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(run_parent, 0, LV_PART_MAIN);
    lv_obj_set_scrollable(run_parent, false);
    player = new jlp::LayoutPlayer();
    // The pages start below the panel's status strip, as on the device,
    // so a layout's y means the same here; the page draws no strip.
    player->init(run_parent, [](JsonObjectConst doc, bool) {
      const bool strip = doc["status_overlay"] | true;
      const int top = strip ? kStatusStripHeight : 0;
      lv_obj_set_pos(run_parent, 0, top);
      lv_obj_set_size(run_parent, lv_pct(100),
                      lv_display_get_vertical_resolution(disp) - top);
    });
  }
  jlp::LayoutPlayer::Result r = player->apply(json);
  if (!r.ok) {
    // A first layout that fails leaves the preview as it was.
    if (first) {
      delete player;
      player = nullptr;
      lv_obj_delete(run_parent);
      run_parent = nullptr;
    }
    err_storage = r.err;
    return err_storage.c_str();
  }
  if (first && current_root) {
    lv_obj_delete(current_root);
    current_root = nullptr;
    last_layout.clear();
  }
  if (json != run_layout.c_str()) run_layout = json;
  lv_obj_set_style_bg_color(screen, lv_color_hex(jlp::theme::palette().screen),
                            LV_PART_MAIN);
  return "";
}

// Shows the screen with this `screens[].id`; 0 when it is unknown or the
// layout has one screen.
EMSCRIPTEN_KEEPALIVE
int jlp_select_screen(const char* id) {
  return player != nullptr && id != nullptr && player->select_screen(id) ? 1 : 0;
}

// Touch at (x, y) in display pixels. LVGL reads it on the next frame.
EMSCRIPTEN_KEEPALIVE
void jlp_pointer(int x, int y, int pressed) {
  // A drag only needs its latest position; a press or release is kept.
  const bool down = pressed != 0;
  if (!pointer_events.empty() && pointer_events.back().down && down) {
    pointer_events.back().at = {x, y};
    return;
  }
  if (pointer_events.size() >= 64) pointer_events.pop_front();
  pointer_events.push_back({{x, y}, down});
}

// The firmware's /hello widget catalog, so a browser display offers the
// designer exactly the kinds this build renders.
EMSCRIPTEN_KEEPALIVE
const char* jlp_widget_catalog() { return jlp::kWidgetCatalogJson; }

EMSCRIPTEN_KEEPALIVE
void jlp_set_value_float(const char* path, float v) {
  lv_subject_t* s = jlp::registry().lookup(path);
  if (!s) return;
  lv_subject_set_float(s, v);
}

EMSCRIPTEN_KEEPALIVE
void jlp_set_value_int(const char* path, int v) {
  lv_subject_t* s = jlp::registry().lookup(path);
  if (!s) return;
  lv_subject_set_int(s, v);
}

EMSCRIPTEN_KEEPALIVE
void jlp_set_value_bool(const char* path, int v) {
  lv_subject_t* s = jlp::registry().lookup(path);
  if (!s) return;
  lv_subject_set_int(s, v ? 1 : 0);
}

// Feed a SignalK meta object for a path. Designer fetches meta via
// the SK REST tree (zones, description, displayUnits, units) and
// pushes the relevant shape here so wasm widgets can do zone-colored
// renders + description-over-value labels, matching the device.
//
// Expected JSON shape (subset of SK meta):
//   { "description": "...", "zones": [{"lower":N,"upper":N,"state":"alarm"}, ...] }
EMSCRIPTEN_KEEPALIVE
void jlp_set_path_meta(const char* path, const char* meta_json) {
  JsonDocument doc;
  if (deserializeJson(doc, meta_json)) return;
  jlp::zones().apply_meta(path, doc.as<JsonObjectConst>());
}

// Replace the entire notifications snapshot from a JSON array of
// `{path, state, message}` rows. Removes any previously-known
// notification whose path isn't in the new array (so cleared
// notifications disappear from list widgets and the alert overlay
// dismisses). Fires the registry's on_change observers so listeners
// (list widget rows, alert overlay) re-render.
//
// `path` is the suffix after `notifications.` — same convention the
// firmware NotificationsRegistry uses, and what designer's
// fetchNotifications() returns.
EMSCRIPTEN_KEEPALIVE
void jlp_set_notifications(const char* snapshot_json) {
  JsonDocument doc;
  if (deserializeJson(doc, snapshot_json)) return;
  JsonArrayConst arr = doc.as<JsonArrayConst>();
  if (arr.isNull()) return;

  // Build the new key set so we can clear removals first.
  std::set<std::string> incoming;
  for (JsonObjectConst row : arr) {
    const char* p = row["path"] | (const char*)nullptr;
    if (p) incoming.insert(p);
  }

  // Clear any prior path not in the new snapshot. snapshot(true)
  // returns the registry's current set including cleared rows.
  auto current = jlp::notifications().snapshot(true);
  for (const auto& n : current) {
    if (!incoming.count(n.path)) {
      JsonDocument null_doc;  // a default-constructed variant reads as null
      jlp::notifications().apply(n.path, null_doc.as<JsonVariantConst>());
    }
  }

  // Apply each row. The registry treats this as upsert.
  for (JsonObjectConst row : arr) {
    const char* p = row["path"] | (const char*)nullptr;
    if (!p) continue;
    // Build a fresh value object the registry's apply() expects
    // (it reads `state` and `message`).
    JsonDocument val_doc;
    JsonObject val = val_doc.to<JsonObject>();
    val["state"]   = row["state"]   | "normal";
    val["message"] = row["message"] | "";
    jlp::notifications().apply(p, val_doc.as<JsonVariantConst>());
  }
}

}  // extern "C"
