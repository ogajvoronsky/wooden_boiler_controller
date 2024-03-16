#include "mgos.h"
#include "mgos_gpio.h"
#include "mgos_spi.h"
#include "mgos_mqtt.h"
#include "ds18b20.h"

//#include "mgos_rpc_service_ota.h"
//#include "mg_rpc.h"

// #define TEST

/* for gpio. low level = on */
#define ON false
#define OFF true
/* dumper positions */
#define OPEN 100
#define CLOSED 0

/* states */
#define WB_state_stop 0
#define WB_state_warming_up 1
#define WB_state_burning_up 2
#define WB_state_run 3
#define WB_state_burning_down 4
#define WB_state_overheat 5
// errors
#define WB_state_burning_up_timeout -1
#define WB_state_warming_up_timeout -2
#define WB_state_sensor_error -3
/* pins */
#define WB_ds_sensors_pin 5
#define WB_pump_pin 2
#define WB_air_on_relay_pin 16
#define WB_air_off_relay_pin 4
#define WB_burner_pin 0

/* constants (default values) */
#define WB_temp_resolution 12    // 12-bit resolution
#define WB_chimney_work_temp 66 // when reached - switch to run
#define WB_overheat_temp 110
#define WB_pump_temp 70  // т-ра включення ЦН
#define WB_warming_up_timeout 1200000 // 20m макс. час роботи режиму розігріву перед розпалом
#define WB_warming_up_setpoint 30     // температура до якої розігрівати котел перед розпалом
#define WB_burning_up_timeout 2400000 // 40m макс. час роботи фену розпалу

/* settings see mos.yml*/
static int WB_upper_temp = 107;         // верхня межа т-ри
static int WB_lower_temp = 100;         // нижня межа т-ри
static int WB_dumper_open_time = 5000; // час за який заслонка відкривається на 100%

/* vars */
static char json_status[1600];
static int wb_state;
bool starting;
static int dumper_state;
static bool burner_state;
static bool pump_state;
static float chimney_temp;
static float feed_temp;
static float return_temp;
static mgos_timer_id air_dumper_timerid;
static mgos_timer_id burning_up_timerid;
static mgos_timer_id warm_up_timerid;
struct mgos_spi *spi;
struct mgos_spi_txn txn = {
    .cs = 0, /* Use CS0 line as configured by cs0_gpio */
    .mode = 0,
    .freq = 0 };

static char sensors_json[1000];

void temperatures_cb(struct ds18b20_result *results)
{

  void sensors_walk(char *buf, struct ds18b20_result *r)
  {
    char res[400] = "\0";
    if (r->next != NULL)
    {
      sensors_walk(res, r->next);
    }
    sprintf(buf, "{\"mac\":\"%s\",\"temp\":%.2f},%s", r->mac, r->temp, res);
  }
  sensors_walk(sensors_json, results);

  // Loop over results and grab temperatures
  while (results != NULL)
  {
    // results->rom - uint8_t - Sensor ROM
    // results->mac - char* - MAC address string
    // results->temp - float - Temperature in celsius
    // LOG(LL_INFO, ("ROM: %s, Temp: %f\n", results->mac, results->temp));

    // pick measurements
    if (strcmp(results->mac, mgos_sys_config_get_feed_sensor_mac()) == 0)
    {
      feed_temp = results->temp;
    };
    if (strcmp(results->mac, mgos_sys_config_get_ret_sensor_mac()) == 0)
    {
      return_temp = results->temp;
    };

    results = results->next;
  }
};

void air_stop()
{ // Stop moving air dumper
  if (air_dumper_timerid != 0)
  {
    mgos_clear_timer(air_dumper_timerid);
    air_dumper_timerid = 0;
  };
  mgos_gpio_write(WB_air_on_relay_pin, OFF);
  mgos_gpio_write(WB_air_off_relay_pin, OFF);
}
void air_open(uint16_t t)
{ // open air dumper for t ms
  mgos_gpio_write(WB_air_on_relay_pin, ON);
  air_dumper_timerid = mgos_set_timer(t, MGOS_TIMER_REPEAT, air_stop, NULL);
};
void air_close(uint16_t t)
{ // close air for t ms
  mgos_gpio_write(WB_air_off_relay_pin, ON);
  air_dumper_timerid = mgos_set_timer(t, MGOS_TIMER_REPEAT, air_stop, NULL);
}

void burner(bool state)
{ // turn on burner
  mgos_gpio_write(WB_burner_pin, state);
  burner_state = state;
}

void pump(bool state)
{
  mgos_gpio_write(WB_pump_pin, state);
  pump_state = state;
}

void dumper(int8_t state)
{ // state 0-100%
  int new_position;
  new_position = (int)(WB_dumper_open_time * state) / 100;
  if (air_dumper_timerid != 0 || dumper_state == new_position)
  {
    return;
  }

  if (new_position > dumper_state)
  {
    air_open(new_position - dumper_state);
  };

  if (new_position < dumper_state)
  {
    air_close(dumper_state - new_position);
  };

  dumper_state = new_position;
};

void burning_up_timeout_cb()
{
  // Таймаут розпалу
  burner(OFF);
  wb_state = WB_state_burning_up_timeout;
  mgos_clear_timer(burning_up_timerid);
  burning_up_timerid = 0;
};

void warm_up_timeout_cb()
{
  // таймаут передпускового підігріву
  wb_state = WB_state_warming_up_timeout;
  mgos_clear_timer(warm_up_timerid);
  warm_up_timerid = 0;
};

void mqtt_handler_cb(struct mg_connection *c, const char *topic, int topic_len,
                     const char *msg, int msg_len, void *userdata)
{
  int new_mode;
  LOG(LL_INFO, ("Got message on topic %.*s", topic_len, topic));
  LOG(LL_INFO, ("Message: %.*s", msg_len, (char *)msg));
  // команда - номер режиму в який переходим
  new_mode = (int)strtof(msg, NULL);
  switch (new_mode)
  {
  case 0:
  { // stop
    LOG(LL_INFO, ("%s", "STOP received"));
    wb_state = WB_state_stop;
    dumper(CLOSED);
    break;
  }
  case 1:
  { // warmup
    LOG(LL_INFO, ("%s", "WARMUP received"));
    if (wb_state == WB_state_stop)
    {
      wb_state = WB_state_warming_up;
    };
    break;
  }
  case 2:
  { // burningup
    LOG(LL_INFO, ("%s", "BURNUP received"));
    if (wb_state == WB_state_stop)
    {
      wb_state = WB_state_burning_up;
    };
    break;
  }
  default:
    break;
  };
}

float read_chimney_temp()
{
  // Read chimney thermocouple
  uint8_t rx_data[2] = {0, 0};
  uint16_t rv;
  txn.hd.tx_len = 0;
  txn.hd.dummy_len = 0;
  txn.hd.rx_len = 2;
  txn.hd.rx_data = rx_data;
  if (!mgos_spi_run_txn(spi, false /* full_duplex */, &txn))
  {
    LOG(LL_ERROR, ("SPI transaction failed"));
    return -2;
  };
  rv = rx_data[0];
  rv <<= 8;
  rv |= rx_data[1];
  if (rv & 0x4)
  {
    // no thermocouple attached!
    return -100;
  };
  rv >>= 3;
  return rv * 0.25;
};

void reset_timers()
{
  if (burning_up_timerid != 0) {
      mgos_clear_timer(burning_up_timerid);
        burning_up_timerid = 0;
    }
  if (warm_up_timerid != 0)
      {
        mgos_clear_timer(warm_up_timerid);
        warm_up_timerid = 0;
      };
}

void initialize()
{
  mgos_gpio_setup_output(WB_air_off_relay_pin, OFF);
  mgos_gpio_setup_output(WB_air_on_relay_pin, OFF);
  mgos_gpio_setup_output(WB_pump_pin, OFF);
  mgos_gpio_setup_output(WB_burner_pin, OFF);

  starting = false;
  wb_state = 0;
  dumper_state = CLOSED;
  burner_state = OFF;
  pump_state = OFF;
  chimney_temp = -100;
  feed_temp = -100;
  return_temp = -100;
  air_dumper_timerid = 0;
  burning_up_timerid = 0;

  air_close(10000);

  // init spi
  spi = mgos_spi_get_global();
  if (spi == NULL)
  {
    LOG(LL_ERROR, ("SPI is not configured, make sure spi.enable is true"));
    return;
  };
};

void wb_tick()
{
  // main loop (every 5 sec)

  // ===========    query sensors
  #ifndef TEST
    ds18b20_read_all(WB_ds_sensors_pin, WB_temp_resolution, temperatures_cb);
    chimney_temp = read_chimney_temp();
  #endif
  // ================================================

  sprintf(json_status, "{\"state\":%i,\"dumper\":\"%i\",\"pump\":\"%s\",\"feed\":%.2f,\"return\":%.2f,\"chimney\":%.0f,\"sensors\":[%s]}",
          wb_state, dumper_state, pump_state ? "OFF" : "ON", feed_temp, return_temp, chimney_temp, sensors_json);

  // DEBUG
  #ifdef TEST
    LOG(LL_INFO, ("status: %s", json_status));
  #endif
  

  // publish status json to mqtt topic
  #ifdef TEST
   mgos_mqtt_pub("rio/wboiler2/status", json_status, strlen(json_status), 1, false);
  #endif

  #ifndef TEST
    mgos_mqtt_pub(mgos_sys_config_get_status_topic(), json_status, strlen(json_status), 1, false);
  #endif

  switch (wb_state)
  {
  case WB_state_stop:
  {
    reset_timers();
    burner(OFF);
    pump(OFF);
    dumper(CLOSED);
    //if ( feed_temp < 35 || feed_temp >= WB_lower_temp ) { empty = false; }
    if (chimney_temp >= WB_chimney_work_temp || feed_temp >= WB_lower_temp )
    {
      starting = true;
      wb_state = WB_state_run;
      break;
    };

    break;
  } /* stop */

  case WB_state_warming_up:
  {
    if (feed_temp < WB_warming_up_setpoint)
    { // т-ра менша уставки нагріву - заганяєм теплу воду в котел
      pump(ON);
      dumper(CLOSED);
      if (warm_up_timerid == 0)
      {
        warm_up_timerid = mgos_set_timer(WB_warming_up_timeout, MGOS_TIMER_REPEAT, warm_up_timeout_cb, NULL);
      }
    }
    else
    { // т-ра >= уставки - скидаєм таймер і переходим до розпалу
      if (warm_up_timerid != 0)
      {
        mgos_clear_timer(warm_up_timerid);
        warm_up_timerid = 0;
      };
      wb_state = WB_state_burning_up;
    };
    break;
  } /* warming_up */

  case WB_state_burning_up:
  { // Режим розпалу котла
    
    if (chimney_temp >= WB_chimney_work_temp)
    { // т-ра комина в нормі - котел горить
      starting = true;
      if (burning_up_timerid != 0)
      {
        mgos_clear_timer(burning_up_timerid);
        burning_up_timerid = 0;
      };
      wb_state = WB_state_run;
      burner(OFF);
    }
    else
    { // димохід ще холодний - продовжуєм розпалювати
      dumper(OPEN);
      pump(ON);
      burner(ON);
      if (burning_up_timerid == 0) 
      { // сетапим таймер таймауту якщо нема
        burning_up_timerid = mgos_set_timer(WB_burning_up_timeout, MGOS_TIMER_REPEAT, burning_up_timeout_cb, NULL);
      };
    }
    break; 
  } /* burnimg_up */

  case WB_state_run:
  {
    dumper(OPEN);
    // DEBUG
    #ifdef TEST
      LOG(LL_INFO, ("starting: %d", starting));
    #endif
  
    // якщо комин холодніший за робочу т-ру і не в розпалі - переходим в затухання

    if ( (chimney_temp <= WB_chimney_work_temp) && !starting )
    {
      wb_state = WB_state_burning_down;
    };
    // якщо вийшли на режим - обнуляєм прапор старту
    if ( feed_temp > WB_pump_temp && chimney_temp > WB_chimney_work_temp ) { starting = false; }

    // Переходим в стоп якщо холодний
    if (feed_temp < 38 && chimney_temp < 38) { 
      starting=false;
      wb_state = WB_state_stop; 
    }

    //
    if (feed_temp >= WB_overheat_temp)
    {
      wb_state = WB_state_overheat;
    };

    // заслонка логіка роботи
    if (feed_temp >= WB_upper_temp)
    {
      dumper(CLOSED);
    };
    if (feed_temp <= WB_lower_temp)
    {
      dumper(OPEN);
    };
    // насос
    if ( (feed_temp - return_temp) > 10 || feed_temp >= WB_pump_temp ) 
      { pump(ON); } else { pump(OFF); };

    break;
  } /* run */

  case WB_state_burning_down:
  { 
    pump(OFF);
    starting=false;
    dumper(50); // прикриваєм заслонку (чи потрібно отримувати цей пераметр з зовні?)

    if ((feed_temp >= WB_pump_temp && chimney_temp >= WB_pump_temp + 10) || feed_temp >= WB_lower_temp)
    { // напевно підкинули дрова переходим в роботу
      starting = true;
      wb_state = WB_state_run;
      break;
    };
    
    // Переходим в стоп якщо холодний
    if ( chimney_temp < 40) { 
      starting=false;
      wb_state = WB_state_stop; 
      break;
    }

    if (feed_temp >= WB_overheat_temp)
    {
      wb_state = WB_state_overheat;
      dumper(CLOSED);
      break;
    };

    break;
  } /* burning_down */

  case WB_state_overheat:
  {
    dumper(CLOSED);
    pump(ON);
    if (feed_temp < WB_upper_temp)
    {
      wb_state = WB_state_run;
    };
    break;
  } /* overheat */

  case WB_state_sensor_error:
  {
    dumper(CLOSED);
    burner(OFF);
    pump(OFF);
    wb_state = WB_state_stop; /* пробуєм вернутися в режим */
  }                           /* sensor_error */

  case WB_state_burning_up_timeout:
  {
    burner(OFF);
    pump(OFF);
    wb_state = WB_state_stop; /* пробуєм вернутися в режим */
    break;
  } /* помилка розпалу */

  case WB_state_warming_up_timeout:
  {
    wb_state = WB_state_burning_up; /* пробуєм розпалювати */
    break;
  } /* помилка не вдалося повністю нагріти котел перед розпалом */

  default:
    break;
  } /*switch*/

} /* кінець робочого циклу */

static void timer_cb(void *arg)
{

  wb_tick(); // MAIN CYCLE

  // LOG(LL_INFO,
  //     ("state: %.2lf, RAM: %lu, %lu free",
  //      mgos_uptime(), (unsigned long)mgos_get_heap_size(),
  //      (unsigned long)mgos_get_free_heap_size()));

  (void)arg;
};

// debug - get sensors measurement from mqtt
#ifdef TEST
  static void mqtt_chimney_temp_cb(struct mg_connection *c, const char *topic, int topic_len,
                    const char *msg, int msg_len, void *userdata) {
  chimney_temp = strtof(msg, NULL);
  };
  static void mqtt_feed_temp_cb(struct mg_connection *c, const char *topic, int topic_len,
                    const char *msg, int msg_len, void *userdata) {
  feed_temp = strtof(msg, NULL);
  };
  static void mqtt_ret_temp_cb(struct mg_connection *c, const char *topic, int topic_len,
                    const char *msg, int msg_len, void *userdata) {
  return_temp = strtof(msg, NULL);
  };
#endif
// debug

enum mgos_app_init_result mgos_app_init(void)
{

  initialize();

  // debug - get sensors measurement from mqtt
  #ifdef TEST
   mgos_mqtt_sub("rio/wboiler2/wb_chimney_temp_s", mqtt_chimney_temp_cb, NULL);
   mgos_mqtt_sub("rio/wboiler2/wb_feed_temp_s", mqtt_feed_temp_cb, NULL);
   mgos_mqtt_sub("rio/wboiler2/wb_ret_temp_s",mqtt_ret_temp_cb, NULL);
  #endif
  // debug

  // subscribe and read params
  mgos_mqtt_sub(mgos_sys_config_get_command_topic(), mqtt_handler_cb, NULL);
  WB_upper_temp = mgos_sys_config_get_app_uptemp();
  WB_lower_temp = mgos_sys_config_get_app_lowtemp();
  WB_dumper_open_time = mgos_sys_config_get_app_dumpertime();

  // work cycle timer every 5sec
  mgos_set_timer(5000 /* ms */, MGOS_TIMER_REPEAT, timer_cb, NULL);
  return MGOS_APP_INIT_SUCCESS;
};
