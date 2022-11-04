#include "mgos.h"
#include "mgos_gpio.h"
#include "mgos_spi.h"
#include "mgos_mqtt.h"
#include "ds18b20.h"

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
#define WB_temp_resolution 12   // 12-bit resolution
#define WB_chimney_stop_temp 40 // when eq or lower - switch to stop
#define WB_chimney_work_temp 70 // when reached - switch to run
#define WB_overheat_temp 110
#define WB_pump_on_temp 60
#define WB_dumper_idle 5               // air dumper 5% open on idle
#define WB_heating_up_timeout 1200000  // 20m макс. час за який котел має зреагувати на відкриту заслонку
#define WB_warming_up_timeout 1200000  // 20m макс. час роботи режиму розігріву перед розпалом
#define WB_warming_up_setpoint 40      // температура до якої розігрівати котел перед розпалом
#define WB_burning_up_timeout  2400000 // 40m макс. час роботи фену розпалу
#define WB_dumper_close_delay  600000   // 10m час через який закриється заслонка після переходу в стоп


/* settings see mos.yml*/
static int WB_upper_temp = 95;   // верхня межа т-ри
static int WB_lower_temp = 80;   // нижня межа т-ри
static int WB_chimney_high_temp = 170; // верхня межа димогазів
static int WB_dumper_choke = 50; // % - положення заслонки в режимі "придушення"
static int WB_dumper_open_time = 5000; // час за який заслонка відкривається на 100% 


/* vars */

  static int wb_state;
  static int dumper_state;
  static int _dumper_open_position;
  static bool burner_state;
  static bool pump_state;
  static float chimney_temp;  
  static float feed_temp;
  static float return_temp;
  static mgos_timer_id air_dumper_timerid;
  static mgos_timer_id burning_up_timerid;
  static mgos_timer_id dumper_close_timerid;
  static mgos_timer_id warm_up_timerid;
  static float _start_temp; 
  struct mgos_spi *spi;  
  struct mgos_spi_txn txn = {
      .cs = 0, /* Use CS0 line as configured by cs0_gpio */
      .mode = 0,
      .freq = 0
  };


void temperatures_cb(struct ds18b20_result *results) {
    // Loop over each result
    while ( results != NULL ) {
        // results->rom - uint8_t - Sensor ROM
        // results->mac - char* - MAC address string
        // results->temp - float - Temperature in celsius
        LOG(LL_INFO, ("ROM: %s, Temp: %f\n", results->mac, results->temp));

        // pick measurement
        if (strcmp(results->mac, mgos_sys_config_get_feed_sensor_mac()) == 0) {
          feed_temp = results->temp;
        }; 
        if (strcmp(results->mac, mgos_sys_config_get_ret_sensor_mac()) == 0) {
          return_temp = results->temp;
        };
        results = results->next;
    }
    
};

void air_stop() { // Stop moving air dumper
  if (air_dumper_timerid != 0) { 
    mgos_clear_timer(air_dumper_timerid);
    air_dumper_timerid = 0;
  };
  mgos_gpio_write(WB_air_on_relay_pin, OFF);
  mgos_gpio_write(WB_air_off_relay_pin, OFF);
}
void air_open(uint16_t t) { // open air dumper for t ms 
  mgos_gpio_write(WB_air_on_relay_pin, ON);
  air_dumper_timerid = mgos_set_timer(t,  MGOS_TIMER_REPEAT, air_stop, NULL);
};
void air_close(uint16_t t) { // close air for t ms
  mgos_gpio_write(WB_air_off_relay_pin, ON);
  air_dumper_timerid = mgos_set_timer(t, MGOS_TIMER_REPEAT, air_stop, NULL);
}

void burner(bool state) {
  mgos_gpio_write(WB_burner_pin, state);
  burner_state = state;
}

void pump(bool state) {
  mgos_gpio_write(WB_pump_pin, state);
  pump_state = state;
}

void dumper(int8_t state) { // state 0-100%
  int new_position;
  new_position = (int) (WB_dumper_open_time * state)/ 100;
  if ( air_dumper_timerid != 0 || dumper_state == new_position ) { return; }
  if ( new_position > dumper_state ) { 
    air_open(new_position - dumper_state); 
  };
  if ( new_position == CLOSED )  { 
    air_close(dumper_state + 3000); // +3sec for shure 
    dumper_state = new_position;
    return;
  }; 
  if ( new_position < dumper_state ) {
    air_close(dumper_state - new_position);
  };
  dumper_state = new_position;
};

void dumper_close_cb() { 
  mgos_clear_timer(dumper_close_timerid);
  dumper_close_timerid=0;
  if (wb_state == WB_state_stop) { dumper(CLOSED); }; 
}

void burning_up_timeout_cb() {
  // Таймаут розпалу
  burner(OFF);
  wb_state = WB_state_burning_up_timeout;
  mgos_clear_timer(burning_up_timerid);
  burning_up_timerid = 0;
};

void warm_up_timeout_cb() {
  // таймаут передпускового підігріву
  wb_state = WB_state_warming_up_timeout;
  mgos_clear_timer(warm_up_timerid);
  warm_up_timerid = 0;
};

void mqtt_handler_cb(struct mg_connection *c, const char *topic, int topic_len,
                    const char *msg, int msg_len, void *userdata) {
  int new_mode;
  LOG(LL_INFO, ("Got message on topic %.*s", topic_len, topic));
  LOG(LL_INFO, ("Message: %.*s", msg_len, (char *) msg));
  // команда - номер режиму в який переходим
  new_mode = (int) strtof(msg, NULL);
  switch( new_mode ) {
    case 0: {// stop 
      LOG(LL_INFO, ("%s", "STOP received"));
      wb_state = WB_state_stop;
      break;
    }
    case 1: {// warmup
      LOG(LL_INFO, ("%s", "WARMUP received"));
      if (wb_state == WB_state_stop ) { wb_state = WB_state_warming_up; };
      break; 
    }
    case 2: {// burningup
      LOG(LL_INFO, ("%s", "BURNUP received"));
      if (wb_state == WB_state_stop ) { wb_state = WB_state_burning_up; };
      break;
    }
    default: break;
  };

}


float read_chimney_temp() {
  // Read chimney thermocouple
  uint8_t rx_data[2] = {0, 0};
  uint16_t rv;
  txn.hd.tx_len = 0;
  txn.hd.dummy_len = 0;
  txn.hd.rx_len = 2;
  txn.hd.rx_data = rx_data;
  if (!mgos_spi_run_txn(spi, false /* full_duplex */, &txn)) {
    LOG(LL_ERROR, ("SPI transaction failed"));
    return -2;
  };
  rv = rx_data[0];
  rv <<= 8;
  rv |= rx_data[1];
  if (rv & 0x4) {
    // no thermocouple attached!
    return -100; 
  };
  rv >>= 3;
  return rv * 0.25;
};


void initialize() {
  mgos_gpio_setup_output(WB_air_off_relay_pin, OFF);
  mgos_gpio_setup_output(WB_air_on_relay_pin, OFF);
  mgos_gpio_setup_output(WB_pump_pin, OFF);
  mgos_gpio_setup_output(WB_burner_pin, OFF);

  wb_state = 0;
  dumper_state = CLOSED;
  burner_state = OFF;
  pump_state = OFF;
  chimney_temp = -100;
  feed_temp  = -100;
  return_temp = -100;
  air_dumper_timerid = 0;
  burning_up_timerid = 0;
  dumper_close_timerid = 0;
  _start_temp = 0; 
  _dumper_open_position = 100;
  
  air_close(10000);

  // init spi
  spi = mgos_spi_get_global();
  if (spi == NULL) {
    LOG(LL_ERROR, ("SPI is not configured, make sure spi.enable is true"));
    return;
  };
};

void wb_tick() {
  char json_status[150];

   // main loop (every 5 sec)
  
  // ===========    query sensors
  ds18b20_read_all(WB_ds_sensors_pin, WB_temp_resolution, temperatures_cb);
  chimney_temp = read_chimney_temp();
  // ================================================

  sprintf(json_status, "{\"state\":%i,\"dumper\":\"%i\",\"pump\":\"%s\",\"feed\":%.2f,\"return\":%.2f,\"chimney\":%.0f}",
   wb_state, dumper_state, pump_state ? "OFF" : "ON", feed_temp, return_temp, chimney_temp);
  
  // DEBUG
  // LOG(LL_INFO, ("status: %s", json_status));
  // LOG(LL_INFO, ("up: %i", WB_upper_temp));
  // LOG(LL_INFO, ("down: %i", WB_lower_temp));
  // LOG(LL_INFO, ("choke: %i", WB_dumper_choke));

  // publish status json to mqtt topic
  mgos_mqtt_pub(mgos_sys_config_get_status_topic(), json_status, strlen(json_status), 1, false);

  switch (wb_state)
  {
  case WB_state_stop: {
    pump(OFF);
    if ( chimney_temp >= WB_chimney_work_temp ) { 
      _start_temp = -1 ; // заходим в роботу з цим значенням щоб уникнути хибних переходів в затухання
      wb_state = WB_state_run;
      if (dumper_close_timerid != 0) {
         mgos_clear_timer(dumper_close_timerid);
         dumper_close_timerid = 0;
      }; 
      break;
    };
    if ( chimney_temp <= WB_chimney_stop_temp ) {
      if ( dumper_state > CLOSED && dumper_close_timerid == 0 ) { 
          dumper_close_timerid = mgos_set_timer(WB_dumper_close_delay, MGOS_TIMER_REPEAT, dumper_close_cb, NULL);
      };
    };
    break;   
  } /* stop */

  case WB_state_warming_up: {
    if (feed_temp < WB_warming_up_setpoint) {
        pump(ON);
        dumper(CLOSED);
        if ( warm_up_timerid == 0 ) {
          warm_up_timerid = mgos_set_timer(WB_warming_up_timeout, MGOS_TIMER_REPEAT, warm_up_timeout_cb, NULL);
        }
    } else { 
      if ( warm_up_timerid != 0 ) { 
        mgos_clear_timer(warm_up_timerid); 
        warm_up_timerid = 0;
      };
      wb_state = WB_state_burning_up; };
   break;   
  }  /* warming_up */

  case WB_state_burning_up: {
    if ( chimney_temp >= WB_chimney_work_temp ) {
      if (burning_up_timerid != 0 ) { 
        mgos_clear_timer(burning_up_timerid); 
        burning_up_timerid = 0; 
      };
      wb_state = WB_state_run;
      burner(OFF);
    } else {
      /* димохід ще холодний */
      dumper(100);
      pump(OFF);
      burner(ON);
      if ( burning_up_timerid == 0 ) { burning_up_timerid = mgos_set_timer(WB_burning_up_timeout, MGOS_TIMER_REPEAT, burning_up_timeout_cb, NULL); };
    }
   break;   
  } /* burnimg_up */

  case WB_state_run: {
// Коли відкриваєм заслонку - берем температуру комина
// Якщо протягом відкритої заслонки т. комина впаде < початкової - затухання
//
    // комин
    // притискаєм коли вихлоп надто високий і котел прогрітий
    if (chimney_temp >= WB_chimney_high_temp && feed_temp > WB_pump_on_temp ) { _dumper_open_position = WB_dumper_choke; };
    if (chimney_temp < WB_chimney_high_temp ) { _dumper_open_position = 100; };


    if (chimney_temp < _start_temp || chimney_temp <= WB_chimney_stop_temp ) {
      wb_state = WB_state_burning_down; 
      // break; 
    };

    if ( feed_temp >= WB_overheat_temp ) {
      wb_state = WB_state_overheat;
      break;
    };

    if ( feed_temp >= WB_upper_temp && dumper_state > WB_dumper_idle ) {
      _start_temp = 0;
      dumper(WB_dumper_idle);
    };
    if ( feed_temp < WB_upper_temp-((WB_upper_temp-WB_lower_temp)/2) ) { 
      if (_start_temp == 0) { _start_temp = chimney_temp; };
      dumper(_dumper_open_position); 
    };
    if (feed_temp >= WB_upper_temp-((WB_upper_temp-WB_lower_temp)/2) && 
        feed_temp < WB_upper_temp ) { 
          _start_temp = 0;
          dumper(WB_dumper_choke);
    };
    
    // насос 
    if ( feed_temp >= WB_pump_on_temp ) { pump(ON); }
    if ( feed_temp < WB_pump_on_temp ) { pump(OFF); }

   break;   
  } /* run */

  case WB_state_burning_down: {
    // комин
    dumper(WB_dumper_choke);

    if (chimney_temp >= WB_chimney_work_temp) {
      wb_state = WB_state_run; 
      break;
    };
    if ( chimney_temp <= WB_chimney_stop_temp ) { 
      wb_state = WB_state_stop;
      break;
    };

    // насос 
    if (feed_temp < WB_upper_temp) { pump(OFF); };
    if ( feed_temp > WB_upper_temp ) { pump(ON); };

    if ( feed_temp >= WB_overheat_temp ) { 
      wb_state = WB_state_overheat;
      break;
    };

    break;   
  } /* burning_down */

  case WB_state_overheat: {
    dumper(CLOSED);
    pump(ON);
    if ( feed_temp < WB_overheat_temp ) { wb_state = WB_state_run; };
   break;   
  } /* overheat */

  case  WB_state_sensor_error: {
    dumper(CLOSED);
    burner(OFF);
    pump(OFF);
    wb_state = WB_state_stop; /* пробуєм вернутися в режим */
  } /* sensor_error */

  case  WB_state_burning_up_timeout: {
    burner(OFF);
    pump(OFF);
    wb_state = WB_state_stop; /* пробуєм вернутися в режим */
    break;
  } /* помилка розпалу */ 

  case WB_state_warming_up_timeout: {
    wb_state = WB_state_burning_up; /* пробуєм розпалювати */
    break;
  } /* помилка не вдалося нагріти котел перед розпалом */ 

  default:
    break;
  } /*switch*/

} /* tick function */



static void timer_cb(void *arg) {
  
  wb_tick();
    
  LOG(LL_INFO,
      ("state: %.2lf, RAM: %lu, %lu free",
       mgos_uptime(), (unsigned long) mgos_get_heap_size(),
       (unsigned long) mgos_get_free_heap_size()));
  

  (void) arg;
};

// debug - get sensors from openhab items
// static void mqtt_chimney_temp_cb(struct mg_connection *c, const char *topic, int topic_len,
//                    const char *msg, int msg_len, void *userdata) {
//  chimney_temp = strtof(msg, NULL);
// };
// static void mqtt_feed_temp_cb(struct mg_connection *c, const char *topic, int topic_len,
//                    const char *msg, int msg_len, void *userdata) {
//  feed_temp = strtof(msg, NULL);
// };
// debug


enum mgos_app_init_result mgos_app_init(void) {

 initialize();
  
// debug - get sensors measurement from mqtt
//  mgos_mqtt_sub("rio/wboiler/wb_chimney_temp_s", mqtt_chimney_temp_cb, NULL);
//  mgos_mqtt_sub("rio/wboiler/wb_feed_temp_s", mqtt_feed_temp_cb, NULL); 
// debug

// subscribe and read params
mgos_mqtt_sub(mgos_sys_config_get_command_topic(), mqtt_handler_cb, NULL); 
WB_upper_temp = mgos_sys_config_get_app_uptemp();
WB_lower_temp = mgos_sys_config_get_app_lowtemp();
WB_dumper_choke = mgos_sys_config_get_app_choke();
WB_chimney_high_temp = mgos_sys_config_get_app_chimneymax();
WB_dumper_open_time = mgos_sys_config_get_app_dumpertime();

// work cycle timer (5sec)
mgos_set_timer(5000 /* ms */, MGOS_TIMER_REPEAT, timer_cb, NULL);
  return MGOS_APP_INIT_SUCCESS;
};
