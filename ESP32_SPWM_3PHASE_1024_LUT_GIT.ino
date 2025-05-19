/*------------------------------------------------------------------------*/ 
// 3-phase PWM sine / 2-phase SPWM
// (c) 2025 Pavol Filek 
// GNU 3.0
/*------------------------------------------------------------------------*/  
#include <driver/gpio.h>
#include "driver/mcpwm_prelude.h"
#include <driver/mcpwm_timer.h>
#include <esp_err.h>
#include <esp_log.h>

#define GPIO_0_OUT 2   // H
#define GPIO_0L_OUT 17  // L

#define GPIO_1_OUT 4    // H
#define GPIO_1L_OUT 16  // L

#define GPIO_2_OUT 18   // H 
#define GPIO_2L_OUT 23

#define UPDATE_PERIOD_US    20000

const static uint32_t sine1024[1024]  = {
502,
505,508,511,514,517,520,523,526,529,532,535,538,541,544,547,550,
553,556,559,562,565,568,571,574,577,580,583,586,589,592,595,598,
601,604,607,610,612,615,618,621,624,627,630,633,636,639,642,645,
648,650,653,656,659,662,665,668,670,673,676,679,682,685,687,690,
693,696,699,701,704,707,710,712,715,718,720,723,726,729,731,734,
737,739,742,744,747,750,752,755,758,760,763,765,768,770,773,775,
778,780,783,785,788,790,793,795,798,800,802,805,807,810,812,814,
817,819,821,824,826,828,830,833,835,837,839,841,844,846,848,850,
852,854,856,859,861,863,865,867,869,871,873,875,877,879,881,883,
885,886,888,890,892,894,896,897,899,901,903,905,906,908,910,911,
913,915,916,918,920,921,923,924,926,927,929,930,932,933,935,936,
938,939,940,942,943,945,946,947,948,950,951,952,953,955,956,957,
958,959,960,961,963,964,965,966,967,968,969,970,971,971,972,973,
974,975,976,977,977,978,979,980,980,981,982,983,983,984,984,985,
986,986,987,987,988,988,989,989,990,990,990,991,991,991,992,992,
992,993,993,993,993,994,994,994,994,994,994,994,994,994,994,995,
994,994,994,994,994,994,994,994,994,994,993,993,993,993,992,992,
992,991,991,991,990,990,990,989,989,988,988,987,987,986,986,985,
984,984,983,983,982,981,980,980,979,978,977,977,976,975,974,973,
972,971,971,970,969,968,967,966,965,964,963,961,960,959,958,957,
956,955,953,952,951,950,948,947,946,945,943,942,940,939,938,936,
935,933,932,930,929,927,926,924,923,921,920,918,916,915,913,911,
910,908,906,905,903,901,899,897,896,894,892,890,888,886,885,883,
881,879,877,875,873,871,869,867,865,863,861,859,856,854,852,850,
848,846,844,841,839,837,835,833,830,828,826,824,821,819,817,814,
812,810,807,805,802,800,798,795,793,790,788,785,783,780,778,775,
773,770,768,765,763,760,758,755,752,750,747,744,742,739,737,734,
731,729,726,723,720,718,715,712,710,707,704,701,699,696,693,690,
687,685,682,679,676,673,670,668,665,662,659,656,653,650,648,645,
642,639,636,633,630,627,624,621,618,615,612,610,607,604,601,598,
595,592,589,586,583,580,577,574,571,568,565,562,559,556,553,550,
547,544,541,538,535,532,529,526,523,520,517,514,511,508,505,502,
498,495,492,489,486,483,480,477,474,471,468,465,462,459,456,453,
450,447,444,441,438,435,432,429,426,423,420,417,414,411,408,405,
402,399,396,393,391,388,385,382,379,376,373,370,367,364,361,358,
355,353,350,347,344,341,338,335,333,330,327,324,321,318,316,313,
310,307,304,302,299,296,293,291,288,285,283,280,277,274,272,269,
266,264,261,259,256,253,251,248,245,243,240,238,235,233,230,228,
225,223,220,218,215,213,210,208,205,203,201,198,196,193,191,189,
186,184,182,179,177,175,173,170,168,166,164,162,159,157,155,153,
151,149,147,144,142,140,138,136,134,132,130,128,126,124,122,120,
118,117,115,113,111,109,107,106,104,102,100,98,97,95,93,92,
90,88,87,85,83,82,80,79,77,76,74,73,71,70,68,67,
65,64,63,61,60,58,57,56,55,53,52,51,50,48,47,46,
45,44,43,42,40,39,38,37,36,35,34,33,32,32,31,30,
29,28,27,26,26,25,24,23,23,22,21,20,20,19,19,18,
17,17,16,16,15,15,14,14,13,13,13,12,12,12,11,11,
11,10,10,10,10,9,9,9,9,9,9,9,9,9,9,9,
9,9,9,9,9,9,9,9,9,9,10,10,10,10,11,11,
11,12,12,12,13,13,13,14,14,15,15,16,16,17,17,18,
19,19,20,20,21,22,23,23,24,25,26,26,27,28,29,30,
31,32,32,33,34,35,36,37,38,39,40,42,43,44,45,46,
47,48,50,51,52,53,55,56,57,58,60,61,63,64,65,67,
68,70,71,73,74,76,77,79,80,82,83,85,87,88,90,92,
93,95,97,98,100,102,104,106,107,109,111,113,115,117,118,120,
122,124,126,128,130,132,134,136,138,140,142,144,147,149,151,153,
155,157,159,162,164,166,168,170,173,175,177,179,182,184,186,189,
191,193,196,198,201,203,205,208,210,213,215,218,220,223,225,228,
230,233,235,238,240,243,245,248,251,253,256,259,261,264,266,269,
272,274,277,280,283,285,288,291,293,296,299,302,304,307,310,313,
316,318,321,324,327,330,333,335,338,341,344,347,350,353,355,358,
361,364,367,370,373,376,379,382,385,388,391,393,396,399,402,405,
408,411,414,417,420,423,426,429,432,435,438,441,444,447,450,453,
456,459,462,465,468,471,474,477,480,483,486,489,492,495,498
};


static   mcpwm_oper_handle_t operator_t0_ret;  
static   mcpwm_oper_handle_t operator_t1_ret;
static   mcpwm_oper_handle_t operator_t2_ret;

static   mcpwm_cmpr_handle_t comparator_t0_ret;
static   mcpwm_cmpr_handle_t comparator_t1_ret;
static   mcpwm_cmpr_handle_t comparator_t2_ret;
static   mcpwm_cmpr_handle_t comparator_t0L_ret;
static   mcpwm_cmpr_handle_t comparator_t1L_ret;
static   mcpwm_cmpr_handle_t comparator_t2L_ret;

static   mcpwm_gen_handle_t generator_t0_ret;
static   mcpwm_gen_handle_t generator_t1_ret;
static   mcpwm_gen_handle_t generator_t2_ret;
static   mcpwm_gen_handle_t  generator_t0L_ret;
static   mcpwm_gen_handle_t  generator_t1L_ret;
static   mcpwm_gen_handle_t  generator_t2L_ret; 

uint16_t ipwm = 0; 

DRAM_ATTR static    uint16_t i = 0;
DRAM_ATTR static    uint16_t j = 0;
DRAM_ATTR static    uint16_t k = 0;
DRAM_ATTR  float freq=1;
DRAM_ATTR const float refclk=76.5931372;     //  60000000 Hz/1020/768

DRAM_ATTR unsigned long sigma;   // phase accumulator
DRAM_ATTR unsigned long delta;  // phase increment
DRAM_ATTR uint16_t phase0, phase1, phase2 ;
DRAM_ATTR boolean forward = 0;

static bool IRAM_ATTR inverter_isr (mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_data)
{
  
  sigma=sigma+delta; // soft DDS, phase accu with 32 bits
  phase0=sigma >> 22;
  if ( forward )  
  {                    
    phase1 = phase0 + 340;
    phase2 = phase0 + 680;
  }
  else
  {                    
    phase1 = phase0 + 680;
    phase2 = phase0 + 340;
  }
    // --------- 2-phase
    // mcpwm_comparator_set_compare_value(comparator_t0_ret, sine1024[i & 0x03FF] );
    //mcpwm_comparator_set_compare_value(comparator_t1_ret, sine1024[(i+511 ) & 0x03FF] );
    //mcpwm_comparator_set_compare_value(comparator_t2_ret, sine1024[(i+511 ) & 0x03FF] );
      // --------- 3-phase
   mcpwm_comparator_set_compare_value(comparator_t0_ret, sine1024[phase0 & 0x03FF] );
   mcpwm_comparator_set_compare_value(comparator_t1_ret, sine1024[phase1 & 0x03FF] );
   mcpwm_comparator_set_compare_value(comparator_t2_ret, sine1024[phase2 & 0x03FF] );
   return false;
}

  
void setup() {
  
  Serial.begin(115200);
  delta = (1LL<<22)*freq/refclk ;
  mcpwm_timer_config_t new_timer_0;
  new_timer_0.group_id = 0;
  new_timer_0.intr_priority = 1;
  new_timer_0.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  new_timer_0.resolution_hz = 60000000;
  new_timer_0.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  new_timer_0.period_ticks = 1020;
  mcpwm_timer_event_callbacks_t cbs_0;
  mcpwm_timer_event_data_t edata;
  esp_err_t timer_OK;
  mcpwm_timer_handle_t mcpwm_timer_t0_ret;
  timer_OK = mcpwm_new_timer ( &new_timer_0, &mcpwm_timer_t0_ret );
  if ( timer_OK == ESP_OK ) 
  { Serial.println("OK"); }   else     { Serial.println("NOT OK");   }

  mcpwm_timer_config_t new_timer_1;
  new_timer_1.group_id = 0;
  new_timer_1.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  new_timer_1.intr_priority = 1;
  new_timer_1.resolution_hz = 60000000;
  new_timer_1.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  new_timer_1.period_ticks = 1020;
  mcpwm_timer_handle_t mcpwm_timer_t1_ret;
  timer_OK = mcpwm_new_timer ( &new_timer_1, &mcpwm_timer_t1_ret );
  if ( timer_OK == ESP_OK ) 
  { Serial.println("OK"); }   else     { Serial.println("NOT OK");   }

  mcpwm_timer_config_t new_timer_2;
  new_timer_2.group_id = 0;
  new_timer_2.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  new_timer_2.intr_priority = 1;
  new_timer_2.resolution_hz = 60000000;
  new_timer_2.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  new_timer_2.period_ticks = 1020;
  mcpwm_timer_handle_t mcpwm_timer_t2_ret;
  timer_OK = mcpwm_new_timer ( &new_timer_2, &mcpwm_timer_t2_ret );
  if ( timer_OK == ESP_OK ) 
  { Serial.println("OK"); }   else     { Serial.println("NOT OK");   }

   ESP_LOGI(TAG, "Create operators"); 
   mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator should be in the same group of the above timers    
  };
   ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_t0_ret));
   ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_t1_ret));
   ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator_t2_ret));
   
   ESP_LOGI(TAG, "Connect timers and operators with each other");
   ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_t0_ret, mcpwm_timer_t0_ret));
   ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_t1_ret, mcpwm_timer_t1_ret));
   ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_t2_ret, mcpwm_timer_t2_ret)); 

   ESP_LOGI(TAG, "Create comparators");
   mcpwm_comparator_config_t compare_config;
   compare_config.intr_priority = 1,
   compare_config.flags.update_cmp_on_tez = true;  

   ESP_ERROR_CHECK(mcpwm_new_comparator(operator_t0_ret, &compare_config, &comparator_t0_ret));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_t0_ret, 800));
   ESP_ERROR_CHECK(mcpwm_new_comparator(operator_t1_ret, &compare_config, &comparator_t1_ret));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_t1_ret, 800));
   ESP_ERROR_CHECK(mcpwm_new_comparator(operator_t2_ret, &compare_config, &comparator_t2_ret));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_t2_ret, 800));
  
   ESP_ERROR_CHECK(mcpwm_new_comparator(operator_t0_ret, &compare_config, &comparator_t0L_ret));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_t0_ret, 800));
   ESP_ERROR_CHECK(mcpwm_new_comparator(operator_t1_ret, &compare_config, &comparator_t1L_ret));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_t1_ret, 800));
   ESP_ERROR_CHECK(mcpwm_new_comparator(operator_t2_ret, &compare_config, &comparator_t2L_ret));
   ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_t2_ret, 800));

   ESP_LOGI(TAG, "Create generators"); 
   mcpwm_generator_config_t gen_config = {};
   gen_config.gen_gpio_num = GPIO_0_OUT;
   ESP_ERROR_CHECK(mcpwm_new_generator(operator_t0_ret, &gen_config, &generator_t0_ret));
   gen_config.gen_gpio_num = GPIO_1_OUT;
   ESP_ERROR_CHECK(mcpwm_new_generator(operator_t1_ret, &gen_config, &generator_t1_ret));
   gen_config.gen_gpio_num = GPIO_2_OUT;
   ESP_ERROR_CHECK(mcpwm_new_generator(operator_t2_ret, &gen_config, &generator_t2_ret));
   
   gen_config.gen_gpio_num = GPIO_0L_OUT;
   ESP_ERROR_CHECK(mcpwm_new_generator(operator_t0_ret, &gen_config, &generator_t0L_ret));
   gen_config.gen_gpio_num = GPIO_1L_OUT;
   ESP_ERROR_CHECK(mcpwm_new_generator(operator_t1_ret, &gen_config, &generator_t1L_ret));
   gen_config.gen_gpio_num = GPIO_2L_OUT;
   ESP_ERROR_CHECK(mcpwm_new_generator(operator_t2_ret, &gen_config, &generator_t2L_ret));
   
   ESP_LOGI(TAG, "Set generator actions on timer and compare event");
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_t0_ret,
       MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
   
   mcpwm_generator_set_action_on_compare_event(generator_t0_ret,
       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_t0_ret, MCPWM_GEN_ACTION_LOW));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_t0L_ret,
       MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
   mcpwm_generator_set_action_on_compare_event(generator_t0L_ret,
       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_t0_ret, MCPWM_GEN_ACTION_LOW));
       
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_t1_ret,
       MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))); 
      
   mcpwm_generator_set_action_on_compare_event(generator_t1_ret,
       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_t1_ret, MCPWM_GEN_ACTION_LOW));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_t1L_ret,
       MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))); 
   mcpwm_generator_set_action_on_compare_event(generator_t1L_ret,
       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_t1_ret, MCPWM_GEN_ACTION_LOW));
 
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_t2_ret,
       MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
       
   mcpwm_generator_set_action_on_compare_event(generator_t2_ret,
       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_t2_ret, MCPWM_GEN_ACTION_LOW));
   ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_t2L_ret,
       MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))); 
   mcpwm_generator_set_action_on_compare_event(generator_t2L_ret,
       MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_t2_ret, MCPWM_GEN_ACTION_LOW));

    mcpwm_dead_time_config_t dt_config;         // dead time config for positive pwm output
    dt_config.posedge_delay_ticks = 22;
    dt_config.negedge_delay_ticks = 0;
    dt_config.flags.invert_output=false;
 
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(generator_t0_ret, generator_t0_ret, &dt_config));
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(generator_t1_ret, generator_t1_ret, &dt_config));
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(generator_t2_ret, generator_t2_ret, &dt_config));

    dt_config.posedge_delay_ticks = 0;
    dt_config.negedge_delay_ticks = 22;
    dt_config.flags.invert_output=true;
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(generator_t0_ret, generator_t0L_ret, &dt_config));
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(generator_t1_ret, generator_t1L_ret, &dt_config));   
    ESP_ERROR_CHECK( mcpwm_generator_set_dead_time(generator_t2_ret, generator_t2L_ret, &dt_config));

    mcpwm_timer_event_callbacks_t cbs = {
       .on_empty = inverter_isr}; // empty callback
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(mcpwm_timer_t0_ret, &cbs, NULL));
     
   ESP_LOGI(TAG, "Start timers one by one, so they are not synced");
   ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer_t0_ret));
   ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer_t0_ret, MCPWM_TIMER_START_NO_STOP));
   vTaskDelay(pdMS_TO_TICKS(10));
   ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer_t1_ret));
   ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer_t1_ret, MCPWM_TIMER_START_NO_STOP));
   vTaskDelay(pdMS_TO_TICKS(10));
   ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer_t2_ret));
   ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer_t2_ret, MCPWM_TIMER_START_NO_STOP));
  
   vTaskDelay(pdMS_TO_TICKS(100));

   ESP_LOGI(TAG, "Force the output level to low, timer still running");
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator_t0_ret, 0, true));
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator_t1_ret, 0, true));
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator_t2_ret, 0, true)); 
 
   ESP_LOGI(TAG, "Create TEZ sync source from timer0");
   mcpwm_sync_handle_t timer_sync_source = NULL;
   mcpwm_timer_sync_src_config_t timer_sync_config = {
        .timer_event = MCPWM_TIMER_EVENT_EMPTY, // generate sync event on timer empty
   };
   ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(mcpwm_timer_t0_ret, &timer_sync_config, &timer_sync_source));
   ESP_LOGI(TAG, "Set other timers sync to the first timer");
   mcpwm_timer_sync_phase_config_t sync_phase_config = {
       .sync_src = timer_sync_source,
       .count_value = 0,
       .direction = MCPWM_TIMER_DIRECTION_UP,      
   };
   ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(mcpwm_timer_t1_ret, &sync_phase_config));
   mcpwm_timer_sync_phase_config_t sync_phase_config_1 = {
       .sync_src = timer_sync_source,
       .count_value = 0,
       .direction = MCPWM_TIMER_DIRECTION_UP,      
   };
   ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(mcpwm_timer_t2_ret, &sync_phase_config_1));
   vTaskDelay(pdMS_TO_TICKS(10));

   ESP_LOGI(TAG, "Now the output PWMs should in sync");
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator_t0_ret, -1, true));
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator_t1_ret, -1, true));
   ESP_ERROR_CHECK(mcpwm_generator_set_force_level(generator_t2_ret, -1, true));  
    vTaskDelay(pdMS_TO_TICKS(100));
       
}

void changeFreq(float _freq){
 
  freq = _freq;
  delta=(1LL<<22)*freq/refclk;  // update phase increment
 
} 

/*------------------------------------------------------------------------*/
void loop() {

  changeFreq(20);
  vTaskDelay(pdMS_TO_TICKS(2000)); 
  changeFreq(22);
  vTaskDelay(pdMS_TO_TICKS(2000));
  changeFreq(24);
  vTaskDelay(pdMS_TO_TICKS(2000)); 
/* changeFreq(26);
  vTaskDelay(pdMS_TO_TICKS(1000)); 
 changeFreq(28);
  vTaskDelay(pdMS_TO_TICKS(1000));
  */
  changeFreq(30);
  vTaskDelay(pdMS_TO_TICKS(2000)); 
  changeFreq(32);
  vTaskDelay(pdMS_TO_TICKS(2000)); 
   changeFreq(34);
  vTaskDelay(pdMS_TO_TICKS(2000)); 
  changeFreq(36);
  vTaskDelay(pdMS_TO_TICKS(2000)); 
   changeFreq(38);
  vTaskDelay(pdMS_TO_TICKS(2000));
}
