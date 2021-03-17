#include "audio_provider.h"
#include "micro_features/micro_model_settings.h"
#include "no_1000ms_sample_data.h"
#include "yes_1000ms_sample_data.h"
#include <stdio.h>
#include "pico/stdlib.h"


namespace {
int16_t g_dummy_audio_data[kMaxAudioSampleSize];
int32_t g_latest_audio_timestamp = 0;
}  // namespace

TfLiteStatus GetAudioSamples(tflite::ErrorReporter* error_reporter,
                             int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  const int yes_start = (0 * kAudioSampleFrequency) / 1000;
  const int yes_end = (1000 * kAudioSampleFrequency) / 1000;
  const int no_start = (4000 * kAudioSampleFrequency) / 1000;
  const int no_end = (5000 * kAudioSampleFrequency) / 1000;
  const int wraparound = (8000 * kAudioSampleFrequency) / 1000;
  const int start_sample = (start_ms * kAudioSampleFrequency) / 1000;
  for (int i = 0; i < kMaxAudioSampleSize; ++i) {
    const int sample_index = (start_sample + i) % wraparound;
    int16_t sample;
    if ((sample_index >= yes_start) && (sample_index < yes_end)) {
      sample = g_yes_1000ms_sample_data[sample_index - yes_start];
    } else if ((sample_index >= no_start) && (sample_index < no_end)) {
      sample = g_no_1000ms_sample_data[sample_index - no_start];
    } else {
      sample = 0;
    }
    g_dummy_audio_data[i] = sample;
  }
  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_dummy_audio_data;
  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() {
  g_latest_audio_timestamp += 100;
  return g_latest_audio_timestamp;
}
