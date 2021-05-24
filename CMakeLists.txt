cmake_minimum_required(VERSION 3.12)

# initialize pico_sdk from GIT
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(micro_speech C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

# initialize the Pico SDK
pico_sdk_init()

add_executable(pico_micro_speech_analog "")

set(PICO_TFLMICRO_MICRO_SPEECH_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-tflmicro/examples/micro_speech)

target_include_directories(pico_micro_speech_analog
  PRIVATE
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/.
)

set_target_properties(
  pico_micro_speech_analog
  PROPERTIES
  COMPILE_FLAGS -fno-rtti
  COMPILE_FLAGS -fno-exceptions
  COMPILE_FLAGS -fno-threadsafe-statics
  COMPILE_FLAGS -nostdlib
)

target_sources(pico_micro_speech_analog
  PRIVATE
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft_util.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_lut.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window_util.c
  ${CMAKE_CURRENT_LIST_DIR}/src/analog_audio_provider.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/command_responder.cc
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/feature_provider.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/main_functions.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_features_generator.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_model_settings.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/model.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/recognize_commands.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/kiss_fft.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftr.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/bits.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_lut.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/audio_provider.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/command_responder.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/feature_provider.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/main_functions.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_features_generator.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_features_generator.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_model_settings.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/model.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/recognize_commands.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/COPYING
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/_kiss_fft_guts.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/kiss_fft.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftr.h
)

target_link_libraries(
  pico_micro_speech_analog
  pico-tflmicro
  pico_analog_microphone
  hardware_adc
  hardware_dma
  hardware_pwm
)

pico_add_extra_outputs(pico_micro_speech_analog)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_micro_speech_analog 1)
pico_enable_stdio_uart(pico_micro_speech_analog 0)


add_executable(pico_micro_speech_pdm "")

set(PICO_TFLMICRO_MICRO_SPEECH_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-tflmicro/examples/micro_speech)

target_include_directories(pico_micro_speech_pdm
  PRIVATE
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/.
)

set_target_properties(
  pico_micro_speech_pdm
  PROPERTIES
  COMPILE_FLAGS -fno-rtti
  COMPILE_FLAGS -fno-exceptions
  COMPILE_FLAGS -fno-threadsafe-statics
  COMPILE_FLAGS -nostdlib
)

target_sources(pico_micro_speech_pdm
  PRIVATE
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft_util.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_lut.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control_util.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window_util.c
  ${CMAKE_CURRENT_LIST_DIR}/src/pdm_audio_provider.cc
  ${CMAKE_CURRENT_LIST_DIR}/src/command_responder.cc
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/feature_provider.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/main_functions.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_features_generator.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_model_settings.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/model.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/recognize_commands.cpp
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/kiss_fft.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftr.c
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/bits.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/fft_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/filterbank_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/frontend_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_lut.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/log_scale_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/noise_reduction_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/pcan_gain_control_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/experimental/microfrontend/lib/window_util.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/audio_provider.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/command_responder.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/feature_provider.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/main_functions.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_features_generator.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_features_generator.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/micro_model_settings.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/micro_features/model.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/recognize_commands.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/COPYING
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/_kiss_fft_guts.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/kiss_fft.h
  ${PICO_TFLMICRO_MICRO_SPEECH_PATH}/tensorflow/lite/micro/tools/make/downloads/kissfft/tools/kiss_fftr.h
)

target_link_libraries(
  pico_micro_speech_pdm
  pico-tflmicro
  pico_pdm_microphone
  hardware_adc
  hardware_dma
  hardware_pwm
  hardware_pio
)

pico_add_extra_outputs(pico_micro_speech_pdm)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_micro_speech_pdm 1)
pico_enable_stdio_uart(pico_micro_speech_pdm 0)

add_subdirectory("lib/pico-tflmicro" EXCLUDE_FROM_ALL)
add_subdirectory("lib/pico-microphone")
