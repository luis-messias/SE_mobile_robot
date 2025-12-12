#include <EngineDriver.h>
#include <esp_log.h>

EngineDriver::EngineDriver(std::pair<int, int> gpios, ledc_timer_t timer,
                           std::pair<ledc_channel_t, ledc_channel_t> channels,
                           float deadZone)
    : m_channels(channels), m_deadZone(deadZone) {
  xSemaphore = xSemaphoreCreateMutex();
  if (xSemaphore == NULL) {
    ESP_LOGE("ENGINEDRIVER", "Failed to create mutex semaphore!");
  }

  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_15_BIT,
      .timer_num = timer,
      .freq_hz = 1000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false, // keep timer configured
  };

  ledc_channel_config_t motorConfA = {
      .gpio_num = gpios.first,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel = m_channels.first,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = timer,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = ledc_sleep_mode_t::LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags = {0},
  };

  ledc_channel_config_t motorConfB = {
      .gpio_num = gpios.second,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel = m_channels.second,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = timer,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = ledc_sleep_mode_t::LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags = {0},
  };

  ledc_timer_config(&timer_conf);
  ledc_channel_config(&motorConfA);
  ledc_channel_config(&motorConfB);
}

void EngineDriver::setOutput(float percentage){
  percentage += (percentage > 0) ? m_deadZone : (percentage < 0) ? -m_deadZone : 0;
  if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
        if(percentage > 1.0){
      percentage = 1;
    }
        if(percentage < -1.0){
      percentage = -1;
    }

        if(percentage == 0.0){
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 0);
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 0);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
      xSemaphoreGive(xSemaphore);
      return;
    }
    if(percentage > 0){
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first,
                    (int)(percentage * (1<<LEDC_TIMER_15_BIT)));
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second, 0);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
      xSemaphoreGive(xSemaphore);
      return;
    }
    if(percentage < 0){
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.first, 0);
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, m_channels.second,
                    (int)(-percentage * (1<<LEDC_TIMER_15_BIT)));
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.first);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, m_channels.second);
      xSemaphoreGive(xSemaphore);
      return;
    }
  }
}