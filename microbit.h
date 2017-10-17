#ifndef MICROBIT_H
#define MICROBIT_H

/* micro:bit IO definitions */
#define PIN_BUTTON_A 17
#define PIN_BUTTON_B 26
#define PIN_BUTTON_R 19

#define PIN_LED_ROW_1 13
#define PIN_LED_ROW_2 14
#define PIN_LED_ROW_3 15
#define PIN_LED_ROWS_MASK ((1 << PIN_LED_ROW_1) | (1 << PIN_LED_ROW_2) | (1 << PIN_LED_ROW_3))

#define PIN_LED_COL_1 4
#define PIN_LED_COL_2 5
#define PIN_LED_COL_3 6
#define PIN_LED_COL_4 7
#define PIN_LED_COL_5 8
#define PIN_LED_COL_6 9
#define PIN_LED_COL_7 10
#define PIN_LED_COL_8 11
#define PIN_LED_COL_9 12
#define PIN_LED_COLS_MASK ((1 << PIN_LED_COL_1) | (1 << PIN_LED_COL_2) | (1 << PIN_LED_COL_3) | (1 << PIN_LED_COL_4) | (1 << PIN_LED_COL_5) | (1 << PIN_LED_COL_6) | (1 << PIN_LED_COL_7) | (1 << PIN_LED_COL_8) | (1 << PIN_LED_COL_9))

#define PIN_DEBUG_TX 24
#define PIN_DEBUG_RX 25

#define PIN_EDGE_P0 3
#define PIN_EDGE_P1 2
#define PIN_EDGE_P2 1
#define PIN_EDGE_P3 4
#define PIN_EDGE_P4 5
#define PIN_EDGE_P5 17
#define PIN_EDGE_P6 12
#define PIN_EDGE_P7 11
#define PIN_EDGE_P8 18
#define PIN_EDGE_P9 10
#define PIN_EDGE_P10 6
#define PIN_EDGE_P11 26
#define PIN_EDGE_P12 20
#define PIN_EDGE_P13 23
#define PIN_EDGE_SCK PIN_EDGE_P13
#define PIN_EDGE_14 22
#define PIN_EDGE_MISO PIN_EDGE_P14
#define PIN_EDGE_15 21
#define PIN_EDGE_MOSI PIN_EDGE_15
#define PIN_EDGE_P16 16
#define PIN_EDGE_19 0
#define PIN_EDGE_SCL PIN_EDGE_19
#define PIN_EDGE_20 30
#define PIN_EDGE_SDA PIN_EDGE_20

#define PIN_INT_ACCEL_1 28
#define PIN_INT_ACCEL_2 27

#define PIN_INT_MAG_1 29

/* Main module build configuration - split this to a config file? */
#define MB_DEBUG_ENABLED
#define MB_BUTTONS_ENABLED
#define MB_DISPLAY_ENABLED
#define MB_TEMPERATURE_ENABLED
#define MB_ACCELEROMETER_ENABLED
#define MB_MAGNETOMETER_ENABLED

/* Bitmap of modules for runtime initialisation */
#define MBF_INIT_DEBUG         (1 << 0)
#define MBF_INIT_BUTTONS       (1 << 1)
#define MBF_INIT_DISPLAY       (1 << 2)
#define MBF_INIT_TEMPERATURE   (1 << 3)
#define MBF_INIT_ACCELEROMETER (1 << 4)
#define MBF_INIT_MAGNETOMETER  (1 << 5)

/* Logical display dimensions */
#define MB_DISPLAY_X_MAX 5
#define MB_DISPLAY_Y_MAX 5

/* Events */
enum
{
  MBE_NONE = 0,
  MBE_BUTTON_A_DOWN,
  MBE_BUTTON_A_UP,
  MBE_BUTTON_A_HELD,
  MBE_BUTTON_B_DOWN,
  MBE_BUTTON_B_UP,
  MBE_BUTTON_B_HELD,
  MBE_BUTTON_AB_DOWN,
  MBE_BUTTON_AB_UP,
  MBE_BUTTON_AB_HELD,
  MBE_TEMPERATURE,
  MBE_ACCELEROMETER,
  MBE_MAGNETOMETER,
  MBE_COUNT
} MBEvent;

#define MBE_MASK_EVENT(v) (v & 0x000000ff)
#define MBE_MASK_X(v)     ((int8_t)((v >> 24) & 0xff))
#define MBE_MASK_Y(v)     ((int8_t)((v >> 16) & 0xff))
#define MBE_MASK_Z(v)     ((int8_t)((v >>  8) & 0xff))
#define MBE_MASK_U8(v)    ((uint8_t)(v >> 24))
#define MBE_MASK_U16(v)   ((uint16_t)(v >> 16))
#define MBE_MASK_U24(v)   ((uint32_t)(v >>  8))
#define MBE_MASK_S8(v)    ((int8_t)(v >> 24))
#define MBE_MASK_S16(v)   ((int16_t)(v >> 16))
#define MBE_MASK_S24(v)   ((int32_t)(v >>  8))

/* Macros for constructing microbit events from individual components */
#define MB_EVENT_QUEUE_XYZ(e, x, y, z) mb_event_queue(e | (x << 24) | (y << 16) | (z << 8))
#define MB_EVENT_QUEUE_8(e, v)         mb_event_queue(e | (((uint32_t)v) << 24))
#define MB_EVENT_QUEUE_16(e, v)        mb_event_queue(e | (((uint32_t)v) << 16))
#define MB_EVENT_QUEUE_24(e, v)        mb_event_queue(e | (((uint32_t)v) << 8))

typedef void (*mb_event_handler_t)(uint32_t event);

extern int mb_event_queue(uint32_t event);

#ifdef MB_DEBUG_ENABLED
extern void mb_debug_initialise(void);
extern void mb_debug_printf(const char *format, ...);
extern void mb_debug_shutdown(void);
#endif

#ifdef MB_BUTTONS_ENABLED
extern bool mb_button_a_down(void);
extern bool mb_button_b_down(void);
extern void mb_button_repeat(uint16_t first_ms, uint16_t other_ms);
#endif

#ifdef MB_DISPLAY_ENABLED
typedef uint8_t Display[MB_DISPLAY_X_MAX][MB_DISPLAY_Y_MAX];

extern void mb_display_pixel(uint8_t x, uint8_t y, uint8_t brightness);
extern void mb_display_fill(uint8_t value);
extern void mb_display_copy(Display copy);
extern void mb_display_draw(const Display copy);
extern void mb_display_clear(void);
extern void mb_display_busy(uint8_t x, uint8_t y);
extern void mb_display_percent(uint8_t x, uint8_t y, uint8_t percentage);
#endif

#ifdef MB_TEMPERATURE_ENABLED
extern int8_t mb_temperature_degrees(void);
extern void mb_temperature_periodic(uint16_t ms);
#endif

#ifdef MB_ACCELEROMETER_ENABLED
extern bool mb_accelerometer_fast(int8_t *x, int8_t *y, int8_t *z);
extern bool mb_accelerometer_full(int16_t *x, int16_t *y, int16_t *z);
extern void mb_accelerometer_periodic(uint16_t ms);
#endif

#ifdef MB_MAGNETOMETER_ENABLED
extern bool mb_magnetometer_fast(int8_t *x, int8_t *y, int8_t *z);
extern bool mb_magnetometer_full(int16_t *x, int16_t *y, int16_t *z);
extern void mb_magnetometer_periodic(uint16_t ms);
#endif

extern int mb_initialise(uint32_t flags, mb_event_handler_t handler);
extern int mb_shutdown(void);

extern const char * const mb_event_name(uint32_t event);

#endif /* MICROBIT_H */
