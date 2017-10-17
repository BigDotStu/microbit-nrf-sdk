#include <stdarg.h>

#include "boards.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_dis.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "ble_hci.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "ble_radio_notification.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_temp.h"
#include "microbit.h"

/* Defines & Constants*/

/* Run the RTC at 32768Hz with no prescalar */
#define APP_TIMER_PRESCALAR 0
#define MB_SHARED_TIMER_QUEUE 8
#define MB_SHARED_TIMER_TICKS 16 /* 32768 / 16 = 2048Hz */

/* Size of each microbit event */
#define MB_EVENT_Q_SIZE sizeof(uint32_t)
/* Maximum number of queued microbit events */
#define MB_EVENT_Q_MAX 16

#ifdef MB_DISPLAY_ENABLED

/* LED matrix dimensions */
#define MB_DISPLAY_ROW_MAX 3
#define MB_DISPLAY_COL_MAX 9

typedef struct
{
  uint8_t x;
  uint8_t y;
} PixelRowColMap;

static uint8_t rowpin[MB_DISPLAY_ROW_MAX] = { PIN_LED_ROW_1, PIN_LED_ROW_2, PIN_LED_ROW_3 };
static uint32_t colpin[MB_DISPLAY_COL_MAX] = { 1 << PIN_LED_COL_1, 1 << PIN_LED_COL_2, 1 << PIN_LED_COL_3,
                                               1 << PIN_LED_COL_4, 1 << PIN_LED_COL_5, 1 << PIN_LED_COL_6,
                                               1 << PIN_LED_COL_7, 1 << PIN_LED_COL_8, 1 << PIN_LED_COL_9 };
static const PixelRowColMap rowcolmap[MB_DISPLAY_ROW_MAX][MB_DISPLAY_COL_MAX] =
{
  {
    { 0, 0 }, { 2, 0 }, { 4, 0 }, { 4, 3 }, { 3, 3 }, { 2, 3 }, { 1, 3 }, { 0, 3 }, { 1, 2 },
  },
  {
    { 4, 2 }, { 0, 2 }, { 2, 2 }, { 1, 0 }, { 3, 0 }, { 3, 4 }, { 1, 4 }, { 0xff, 0xff }, { 0xff, 0xff },
  },
  {
    { 2, 4 }, { 4, 4 }, { 0, 4 }, { 0, 1 }, { 1, 1 }, { 2, 1 }, { 3, 1 }, { 4, 1 }, { 3, 2 },
  }
};

static uint8_t busy_col = MB_DISPLAY_X_MAX;
static uint8_t busy_row = MB_DISPLAY_Y_MAX;
static uint8_t busy_count = 0;
static uint8_t busy_index = 0;
static int8_t  busy_delta = 1;

#endif /* MB_DISPLAY_ENABLED */

/* Globals */

/* Overall state - which modules are in use */
static uint32_t mb_initialised;

/* Is the softdevice enabled (checked at initialisation) */
static uint8_t softdevice_enabled;

/* Shared timer used for buttons/display/etc */
APP_TIMER_DEF(shared_timer_id);

#ifdef MB_BUTTONS_ENABLED
/* Button states */
uint8_t button_a_current = 0;
uint8_t button_a_pending = 0;
uint8_t button_b_current = 0;
uint8_t button_b_pending = 0;
/* Auto-repeat timings */
uint16_t button_repeat_count = 0;
uint16_t button_repeat_first = APP_TIMER_TICKS(500, MB_SHARED_TIMER_TICKS);
uint16_t button_repeat_other = APP_TIMER_TICKS(250, MB_SHARED_TIMER_TICKS);
#endif

#ifdef MB_DISPLAY_ENABLED
/* Current state of the 5 x 5 logical LED display */
static uint8_t pixels[MB_DISPLAY_X_MAX][MB_DISPLAY_Y_MAX];
static uint8_t update_row = 0;
static uint8_t update_count = 0;
#endif

/* Application handler for microbit events */
static mb_event_handler_t mb_event_handler = NULL;

/* Event queue - uses the Nordic scheduler, so all internal */

static int mb_event_initialise(mb_event_handler_t handler)
{
  mb_event_handler = handler;

  APP_SCHED_INIT(MB_EVENT_Q_SIZE, MB_EVENT_Q_MAX);

  return 0;
}

static void mb_event_unqueue(void *pVoid, uint16_t len)
{
  if (mb_event_handler != NULL)
    mb_event_handler(*((uint32_t *)pVoid));
}

int mb_event_queue(uint32_t event)
{
  app_sched_event_put(&event, sizeof(event), mb_event_unqueue);

  return 0;
}

static void mb_event_shutdown(void)
{
  mb_event_handler = NULL;
}

/* Shared timer (Buttons & Display) */

static void shared_timer_handler(void *pVoid)
{
  uint32_t pins = 0;
  uint8_t nested;

  sd_nvic_critical_region_enter(&nested);

#ifdef MB_BUTTONS_ENABLED
  /* Buttons */

  if (mb_initialised & MBF_INIT_BUTTONS)
  {
    if (button_a_pending && (button_a_pending != button_a_current))
    {
      button_a_current = button_a_pending;
      button_a_pending = 0;
      if (button_b_current == MBE_BUTTON_B_DOWN)
        mb_event_queue(button_a_current == MBE_BUTTON_A_DOWN ? MBE_BUTTON_AB_DOWN : MBE_BUTTON_AB_UP);
      else
        mb_event_queue(button_a_current);
      if (button_a_current == MBE_BUTTON_A_DOWN)
        button_repeat_count = button_repeat_first;
      else if (button_b_current == MBE_BUTTON_B_UP)
        button_repeat_count = 0;
    }
    if (button_b_pending && (button_b_pending != button_b_current))
    {
      button_b_current = button_b_pending;
      button_b_pending = 0;
      if (button_a_current == MBE_BUTTON_A_DOWN)
        mb_event_queue(button_b_current == MBE_BUTTON_B_DOWN ? MBE_BUTTON_AB_DOWN : MBE_BUTTON_AB_UP);
      else
        mb_event_queue(button_b_current);
      if (button_b_current == MBE_BUTTON_B_DOWN)
        button_repeat_count = button_repeat_first;
      else if (button_a_current == MBE_BUTTON_A_UP)
        button_repeat_count = 0;
    }
    if (button_repeat_count > 0)
    {
      if (--button_repeat_count == 0)
      {
        if (button_a_current == MBE_BUTTON_A_DOWN)
        {
          if (button_b_current == MBE_BUTTON_B_DOWN)
            mb_event_queue(MBE_BUTTON_AB_HELD);
          else
            mb_event_queue(MBE_BUTTON_A_HELD);
        }
        else if (button_b_current == MBE_BUTTON_B_DOWN)
        {
          mb_event_queue(MBE_BUTTON_B_HELD);
        }
        button_repeat_count = button_repeat_other;
      }
    }
  }
#endif /* MB_BUTTONS_ENABLED */

#ifdef MB_DISPLAY_ENABLED
  /* Display */

  if (mb_initialised & MBF_INIT_DISPLAY)
  {
    if ((busy_col < MB_DISPLAY_X_MAX) && (busy_count++ == 0))
    {
      for (uint8_t i = 0; i < MB_DISPLAY_Y_MAX; i++)
        mb_display_pixel(busy_col, i, (busy_index == i) ? 255 : 0);

      if (busy_delta < 0)
      {
        if (busy_index < 4)
          mb_display_pixel(busy_col, busy_index + 1, 95);
        if (busy_index < 3)
          mb_display_pixel(busy_col, busy_index + 2, 7);
        if (busy_index == 0)
        {
          busy_delta = 1;
        }
      }
      else
      {
        if (busy_index > 0)
          mb_display_pixel(busy_col, busy_index - 1, 95);
        if (busy_index > 1)
          mb_display_pixel(busy_col, busy_index - 2, 7);
        if (busy_index == 4)
        {
          busy_delta = -1;
        }
      }
      busy_index += busy_delta;
    }
    if ((busy_row < MB_DISPLAY_Y_MAX) && (busy_count++ == 0))
    {
      for (uint8_t i = 0; i < MB_DISPLAY_Y_MAX; i++)
        mb_display_pixel(i, busy_row, (busy_index == i) ? 255 : 0);

      if (busy_delta < 0)
      {
        if (busy_index < 4)
          mb_display_pixel(busy_index + 1, busy_row, 95);
        if (busy_index < 3)
          mb_display_pixel(busy_index + 2, busy_row, 7);
        if (busy_index == 0)
        {
          busy_delta = 1;
        }
      }
      else
      {
        if (busy_index > 0)
          mb_display_pixel(busy_index - 1, busy_row, 95);
        if (busy_index > 1)
          mb_display_pixel(busy_index - 2, busy_row, 7);
        if (busy_index == 4)
        {
          busy_delta = -1;
        }
      }
      busy_index += busy_delta;
    }
    if (update_count++ == 0)
    {
      nrf_gpio_pins_clear(PIN_LED_ROWS_MASK);
      if (++update_row == MB_DISPLAY_ROW_MAX)
        update_row = 0;
      /* All columns off */
      for (uint8_t i = 0; i < MB_DISPLAY_COL_MAX; i++)
      {
        if (rowcolmap[update_row][i].x != 0xff && rowcolmap[update_row][i].y != 0xff)
          if (pixels[rowcolmap[update_row][i].x][rowcolmap[update_row][i].y] > 0)
            pins |= colpin[i];
      }
      nrf_gpio_pins_set(pins ^ PIN_LED_COLS_MASK);
      nrf_gpio_pins_clear(pins);
      nrf_gpio_pin_set(rowpin[update_row]);
    }
    else
    {
      for (uint8_t i = 0; i < MB_DISPLAY_COL_MAX; i++)
      {
        if (rowcolmap[update_row][i].x != 0xff && rowcolmap[update_row][i].y != 0xff)
          if (pixels[rowcolmap[update_row][i].x][rowcolmap[update_row][i].y] < update_count)
            pins |= colpin[i];
      }
      /* Turn off columns with pixels now off */
      nrf_gpio_pins_set(pins);
    }
    update_count += 15;
  }
#endif /* MB_DISPLAY_ENABLED */

  sd_nvic_critical_region_exit(nested);
}

/* Debug UART connected to interface chip */

void mb_debug_initialise(void)
{
#ifdef MB_DEBUG_ENABLED
  if (!(mb_initialised & MBF_INIT_DEBUG))
  {
    /** @snippet [Configure UART RX and TX pin] */
    nrf_gpio_cfg_output(PIN_DEBUG_TX);
    nrf_gpio_cfg_input(PIN_DEBUG_RX, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD = PIN_DEBUG_TX;
    NRF_UART0->PSELRXD = PIN_DEBUG_RX;
    /** @snippet [Configure UART RX and TX pin] */

    NRF_UART0->BAUDRATE      = (UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos);
    NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
    NRF_UART0->EVENTS_RXDRDY = 0;

    mb_initialised |= MBF_INIT_DEBUG;
  }
#endif
}

void mb_debug_printf(const char *format, ...)
{
#ifdef MB_DEBUG_ENABLED
  if (mb_initialised & MBF_INIT_DEBUG)
  {
    char debug[128];
    va_list ap;
    int len;

    va_start(ap, format);
    len = vsnprintf(debug, sizeof(debug), format, ap);
    va_end(ap);

    for (int i = 0; i < len; i++)
    {
      NRF_UART0->TXD = (uint8_t)debug[i];
      while (NRF_UART0->EVENTS_TXDRDY != 1);
      NRF_UART0->EVENTS_TXDRDY = 0;
    }
  }
#endif
}

void mb_debug_shutdown(void)
{
#ifdef MB_DEBUG_ENABLED
  if (mb_initialised & MBF_INIT_DEBUG)
  {
    NRF_UART0->TASKS_STOPTX = 1;
    NRF_UART0->TASKS_STOPRX = 1;
    NRF_UART0->ENABLE       = (UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos);
  }
#endif
}

/* Buttons */

#ifdef MB_BUTTONS_ENABLED

static void button_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  /* Just set the pending events here - shared timer will handle debounce and queue actual events */
  if (pin == PIN_BUTTON_A)
  {
    button_a_pending = nrf_drv_gpiote_in_is_set(PIN_BUTTON_A) ? MBE_BUTTON_A_UP : MBE_BUTTON_A_DOWN;
  }
  if (pin == PIN_BUTTON_B)
  {
    button_b_pending = nrf_drv_gpiote_in_is_set(PIN_BUTTON_B) ? MBE_BUTTON_B_UP : MBE_BUTTON_B_DOWN;
  }
}

static int mb_button_initialise(void)
{
  uint32_t ec;

  /* Buttons A & B use GPIOTE toggle events to generate 'interrupts' */
  if (!nrf_drv_gpiote_is_init())
  {
    ec = nrf_drv_gpiote_init();
    if (ec != 0)
      mb_debug_printf("Error %u in nrf_drv_gpiote_init\n", ec);
  }

  nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  config.pull = NRF_GPIO_PIN_NOPULL;

  if ((ec = nrf_drv_gpiote_in_init(PIN_BUTTON_A, &config, button_event_handler)) != 0)
  {
    mb_debug_printf("Error %u in nrf_drv_gpiote_in_init\n", ec);
    return -1;
  }
  if ((ec = nrf_drv_gpiote_in_init(PIN_BUTTON_B, &config, button_event_handler)) != 0)
  {
    mb_debug_printf("Error %u in nrf_drv_gpiote_in_init\n", ec);
    return -1;
  }

  nrf_drv_gpiote_in_event_enable(PIN_BUTTON_A, true);
  nrf_drv_gpiote_in_event_enable(PIN_BUTTON_B, true);

  return 0;
}

static void mb_button_shutdown(void)
{
  nrf_drv_gpiote_in_event_disable(PIN_BUTTON_A);
  nrf_drv_gpiote_in_event_disable(PIN_BUTTON_B);

  nrf_drv_gpiote_in_uninit(PIN_BUTTON_A);
  nrf_drv_gpiote_in_uninit(PIN_BUTTON_B);

  nrf_drv_gpiote_uninit();
}

bool mb_button_a_down(void)
{
  return button_a_current == MBE_BUTTON_A_DOWN;
}

bool mb_button_b_down(void)
{
  return button_b_current == MBE_BUTTON_B_DOWN;
}

void mb_button_repeat(uint16_t first_ms, uint16_t other_ms)
{
  button_repeat_first = APP_TIMER_TICKS(first_ms, MB_SHARED_TIMER_TICKS);
  button_repeat_other = APP_TIMER_TICKS(other_ms, MB_SHARED_TIMER_TICKS);
}

#endif

/* Display */

#ifdef MB_DISPLAY_ENABLED

static int mb_display_initialise(void)
{
  /* LED Matrix */
  nrf_gpio_cfg_output(PIN_LED_ROW_1);
  nrf_gpio_cfg_output(PIN_LED_ROW_2);
  nrf_gpio_cfg_output(PIN_LED_ROW_3);

  nrf_gpio_cfg_output(PIN_LED_COL_1);
  nrf_gpio_cfg_output(PIN_LED_COL_2);
  nrf_gpio_cfg_output(PIN_LED_COL_3);
  nrf_gpio_cfg_output(PIN_LED_COL_4);
  nrf_gpio_cfg_output(PIN_LED_COL_5);
  nrf_gpio_cfg_output(PIN_LED_COL_6);
  nrf_gpio_cfg_output(PIN_LED_COL_7);
  nrf_gpio_cfg_output(PIN_LED_COL_8);
  nrf_gpio_cfg_output(PIN_LED_COL_9);

  mb_display_clear();

  return 0;
}

void mb_display_pixel(uint8_t x, uint8_t y, uint8_t brightness)
{
  if ((x < MB_DISPLAY_X_MAX) && (y < MB_DISPLAY_Y_MAX))
    pixels[x][y] = brightness;
}

void mb_display_fill(uint8_t brightness)
{
  for (uint8_t x = 0; x < MB_DISPLAY_X_MAX; x++)
  {
    for (uint8_t y = 0; y < MB_DISPLAY_Y_MAX; y++)
    {
      pixels[x][y] = brightness;
    }
  }
}

void mb_display_clear(void)
{
  mb_display_fill(0);
}

void mb_display_copy(Display copy)
{
  for (uint8_t x = 0; x < MB_DISPLAY_X_MAX; x++)
  {
    for (uint8_t y = 0; y < MB_DISPLAY_Y_MAX; y++)
    {
      copy[x][y] = pixels[x][y];
    }
  }
}

void mb_display_draw(const Display copy)
{
  for (uint8_t x = 0; x < MB_DISPLAY_X_MAX; x++)
  {
    for (uint8_t y = 0; y < MB_DISPLAY_Y_MAX; y++)
    {
      pixels[x][y] = copy[x][y];
    }
  }
}

void mb_display_busy(uint8_t x, uint8_t y)
{
  busy_col = x;
  busy_row = y;
  busy_index = 0;
  busy_delta = 1;
  busy_count = 0;
}

void mb_display_percent(uint8_t x, uint8_t y, uint8_t percentage)
{
  static const uint8_t brightness[4] = { 0, 7, 31, 95 };
  uint8_t highest = percentage / 20, delta = percentage % 20;

  if (x < MB_DISPLAY_X_MAX)
  {
    for (uint8_t i = MB_DISPLAY_Y_MAX; i-- > highest; )
      mb_display_pixel(x, 4 - i, 0);
    if (highest < 5)
      mb_display_pixel(x, 4 - highest, brightness[delta / 4]);
    while (highest-- > 0)
      mb_display_pixel(x, 4 - highest, 255);
  }
  if (y < MB_DISPLAY_Y_MAX)
  {
    for (uint8_t i = MB_DISPLAY_Y_MAX; i-- > highest; )
      mb_display_pixel(i, y, 0);
    if (highest < 5)
      mb_display_pixel(highest, y, brightness[delta / 4]);
    while (highest-- > 0)
      mb_display_pixel(highest, y, 255);
  }
}

#endif /* MB_DISPLAY_ENABLED */

/* Temperature */

#ifdef MB_TEMPERATURE_ENABLED

APP_TIMER_DEF(temperature_timer_id);

static void temperature_timer_handler(void *pVoid)
{
  int8_t t = mb_temperature_degrees();

  MB_EVENT_QUEUE_8(MBE_TEMPERATURE, t);
}

static int mb_temperature_initialise(void)
{
  uint32_t ec = app_timer_create(&temperature_timer_id, APP_TIMER_MODE_REPEATED, temperature_timer_handler);

  return ec;
}

int8_t mb_temperature_degrees(void)
{
  int8_t degrees = 0;

  if (softdevice_enabled)
  {
    int32_t t;

    sd_temp_get(&t);
    degrees = t / 4.0;
  }
  else
  {
    NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

    while (NRF_TEMP->EVENTS_DATARDY == 0)
    {
      // Do nothing.
    }
    NRF_TEMP->EVENTS_DATARDY = 0;

    degrees = (nrf_temp_read() / 4);

    NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */
  }
  return degrees;
}

void mb_temperature_periodic(uint16_t ms)
{
  app_timer_start(temperature_timer_id, APP_TIMER_TICKS(ms, APP_TIMER_PRESCALAR), NULL);
}

static void mb_temperature_shutdown(void)
{
  app_timer_stop(temperature_timer_id);
}

#endif /* MB_TEMPERATURE_ENABLED */

/* Accelerometer */

#ifdef MB_ACCELEROMETER_ENABLED

APP_TIMER_DEF(accelerometer_timer_id);

static void accelerometer_timer_handler(void *pVoid)
{
  int8_t x, y, z;

  if (mb_accelerometer_fast(&x, &y, &z))
    MB_EVENT_QUEUE_XYZ(MBE_ACCELEROMETER, x, y, z);
}

static int mb_accelerometer_initialise(void)
{
  uint32_t ec = app_timer_create(&accelerometer_timer_id, APP_TIMER_MODE_REPEATED, accelerometer_timer_handler);

  return ec;
}

bool mb_accelerometer_fast(int8_t *x, int8_t *y, int8_t *z)
{
#if 1
  (*x) = (*y) = (*z) = 0;
  return true;
#else
  return false;
#endif
}

bool mb_accelerometer_full(int16_t *x, int16_t *y, int16_t *z)
{
  return false;
}

void mb_accelerometer_periodic(uint16_t ms)
{
  app_timer_start(accelerometer_timer_id, APP_TIMER_TICKS(ms, APP_TIMER_PRESCALAR), NULL);
}

static void mb_accelerometer_shutdown(void)
{
  app_timer_stop(accelerometer_timer_id);
}

#endif

/* Magnetometer */

#ifdef MB_MAGNETOMETER_ENABLED

APP_TIMER_DEF(magnetometer_timer_id);

static void magnetometer_timer_handler(void *pVoid)
{
  int8_t x, y, z;

  if (mb_magnetometer_fast(&x, &y, &z))
    MB_EVENT_QUEUE_XYZ(MBE_MAGNETOMETER, x, y, z);
}

static int mb_magnetometer_initialise(void)
{
  uint32_t ec = app_timer_create(&magnetometer_timer_id, APP_TIMER_MODE_REPEATED, magnetometer_timer_handler);

  return ec;
}

bool mb_magnetometer_fast(int8_t *x, int8_t *y, int8_t *z)
{
#if 1
  (*x) = (*y) = (*z) = 0;
  return true;
#else
  return false;
#endif
}

bool mb_magnetometer_full(int16_t *x, int16_t *y, int16_t *z)
{
  return false;
}

void mb_magnetometer_periodic(uint16_t ms)
{
  app_timer_start(magnetometer_timer_id, APP_TIMER_TICKS(ms, APP_TIMER_PRESCALAR), NULL);
}

static void mb_magnetometer_shutdown(void)
{
  app_timer_stop(magnetometer_timer_id);
}

#endif

/* General initialisation - buttons and display */

int mb_initialise(uint32_t flags, mb_event_handler_t handler)
{
  APP_TIMER_INIT(APP_TIMER_PRESCALAR, MB_SHARED_TIMER_QUEUE, NULL);

  mb_event_initialise(handler);

#ifdef MB_DEBUG_ENABLED
  if (flags & MBF_INIT_DEBUG)
    mb_debug_initialise();
  mb_debug_printf("\nmicro:bit debug enabled\n");
#endif

#ifdef MB_BUTTONS_ENABLED
  if (flags & MBF_INIT_BUTTONS)
  {
    if (!mb_button_initialise())
      mb_debug_printf("buttons initialised\n");
  }
#else
  if (flags & MBF_INIT_BUTTONS)
      mb_debug_printf("buttons not available\n");
#endif

#ifdef MB_DISPLAY_ENABLED
  if (flags & MBF_INIT_DISPLAY)
  {
    if (!mb_display_initialise())
      mb_debug_printf("display initialised\n");
  }
#else
  if (flags & MBF_INIT_DISPLAY)
      mb_debug_printf("display not available\n");
#endif

#ifdef MB_TEMPERATURE_ENABLED
  if (flags & MBF_INIT_TEMPERATURE)
  {
    int ec;

    if ((ec = mb_temperature_initialise()) == 0)
      mb_debug_printf("temperature initialised\n");
    else
      mb_debug_printf("temperature error %d\n", ec);
  }
#else
  if (flags & MBF_INIT_TEMPERATURE)
      mb_debug_printf("temperature not available\n");
#endif

#ifdef MB_ACCELEROMETER_ENABLED
  if (flags & MBF_INIT_ACCELEROMETER)
  {
    if (!mb_accelerometer_initialise())
      mb_debug_printf("accelerometer initialised\n");
  }
#else
  if (flags & MBF_INIT_ACCELEROMETER)
      mb_debug_printf("accelerometer not available\n");
#endif

#ifdef MB_MAGNETOMETER_ENABLED
  if (flags & MBF_INIT_MAGNETOMETER)
  {
    if (!mb_magnetometer_initialise())
      mb_debug_printf("magnetometer initialised\n");
  }
#else
  if (flags & MBF_INIT_MAGNETOMETER)
      mb_debug_printf("magnetometer not available\n");
#endif

  mb_initialised = flags;

  uint32_t ec = app_timer_create(&shared_timer_id, APP_TIMER_MODE_REPEATED, shared_timer_handler);

  if (ec != NRF_SUCCESS)
    return -1;

  ec = app_timer_start(shared_timer_id, MB_SHARED_TIMER_TICKS, NULL);

  if (ec != NRF_SUCCESS)
    return -1;

  sd_softdevice_is_enabled (&softdevice_enabled);

  return 0;
}

int microbit_shutdown(void)
{
  app_timer_stop(shared_timer_id);

#ifdef MB_MAGNETOMETER_ENABLED
  if (mb_initialised & MBF_INIT_MAGNETOMETER)
    mb_magnetometer_shutdown();
#endif

#ifdef MB_ACCELEROMETER_ENABLED
  if (mb_initialised & MBF_INIT_ACCELEROMETER)
    mb_accelerometer_shutdown();
#endif

#ifdef MB_TEMPERATURE_ENABLED
  if (mb_initialised & MBF_INIT_TEMPERATURE)
    mb_temperature_shutdown();
#endif

#ifdef MB_BUTTONS_ENABLED
  if (mb_initialised & MBF_INIT_BUTTONS)
    mb_button_shutdown();
#endif

#ifdef MB_DEBUG_ENABLED
  if (mb_initialised & MBF_INIT_DEBUG)
    mb_debug_shutdown();
#endif

  mb_event_shutdown();

  mb_initialised = 0;

  return 0;
}

static const char * const event_names[MBE_COUNT] =
{
  "None",
  "A  Down",
  "A  Up",
  "A  Held",
  "B  Down",
  "B  Up",
  "B  Held",
  "AB Down",
  "AB Up",
  "AB Held",
  "Temperature",
  "Accelerometer",
  "Magnetometer"
};

const char * const mb_event_name(uint32_t value)
{
  uint8_t event = MBE_MASK_EVENT(value);

  return (event < MBE_COUNT) ? event_names[event] : "Unknown";
}
