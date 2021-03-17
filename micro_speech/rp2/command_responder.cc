#include "command_responder.h"

#include <stdio.h>
#include "pico/stdlib.h"

#define LED_PIN 25

// The default implementation writes out the name of the recognized command
// to the error console. Real applications will want to take some custom
// action instead, and should implement their own versions of this function.
void RespondToCommand(tflite::ErrorReporter* error_reporter,
                      int32_t current_time, const char* found_command,
                      uint8_t score, bool is_new_command) {

  // led settings
  static bool is_initialized = false;
  //const uint LED_PIN = 25;
  // if not initialized, setup
  if(!is_initialized) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    is_initialized = true;
  }

  if (is_new_command) {
    TF_LITE_REPORT_ERROR(error_reporter, "Heard %s (%d) @%dms", found_command,
                         score, current_time);

    if (found_command == "yes"){
      //turn led on
      gpio_put(LED_PIN, 1);
    }
    else if (found_command == "no") {
      //turn led off
      gpio_put(LED_PIN, 0);
    }
  }
}
