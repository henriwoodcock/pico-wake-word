#include "audio_provider.h"
#include "micro_features/micro_model_settings.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include <limits>

// set this to determine sample rate
// 0     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
// -> 3000 = 16,000 Hz
#define CAPTURE_FREQUENCY 16000
// Channel 0 is GPIO26
#define ADC_PIN 26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25

// BE CAREFUL: anything over about 9000 here will cause things
// to silently break. The code will compile and upload, but due
// to memory issues nothing will work properly
#define NSAMP 1024
#define BUFFER_SIZE 16000

namespace {
// dma settings
dma_channel_config cfg;
uint dma_chan;

uint16_t g_audio_sample_buffer[2][NSAMP * 4];
// tflite micro settings
bool g_is_audio_initialized = false;
volatile int capture_index = 0;
// An internal buffer able to fit 16x our sample size
constexpr int kAudioCaptureBufferSize = NSAMP * 16;
int16_t g_audio_capture_buffer[kAudioCaptureBufferSize];
// A buffer that holds our output
int16_t g_audio_output_buffer[kMaxAudioSampleSize];
// Mark as volatile so we can check in a while loop to see if
// any samples have arrived yet.
volatile int32_t g_latest_audio_timestamp = 0;

} // namespace

//this next function is the dma interupt
void CaptureSamples() {
  //reset the interrupt request first so you do not lose an interupt
  // Clear the interrupt request.
  dma_hw->ints0 = 1u << dma_chan;
  // get the current capture index
  int read_index = capture_index;
  // get the next capture index to send the dma to start
  capture_index = (capture_index + 1) % 2;
  // Give the channel a new wave table entry to read from, and re-trigger it
  dma_channel_transfer_to_buffer_now(dma_chan,
    g_audio_sample_buffer[capture_index], NSAMP * 4);
  // data processing
  const int number_of_samples = NSAMP * 4;
  // Calculate what timestamp the last audio sample represents
  const int32_t time_in_ms = g_latest_audio_timestamp + (number_of_samples / (kAudioSampleFrequency / 1000));
  // Determine the index, in the history of all samples, of the last sample
  const int32_t start_sample_offset = g_latest_audio_timestamp * (kAudioSampleFrequency / 1000);
  // Determine the index of this sample in our ring buffer
  const int array_capture_index = start_sample_offset % kAudioCaptureBufferSize;
  // Read the data to the correct place in our buffer
  memcpy(g_audio_capture_buffer + array_capture_index,
    (void *)g_audio_sample_buffer[read_index],
    sizeof(int16_t)*number_of_samples);
  // when this changes the nn runs
  g_latest_audio_timestamp = time_in_ms;
}

void setup() {
  adc_gpio_init(ADC_PIN + CAPTURE_CHANNEL);

  adc_init();
  adc_select_input(CAPTURE_CHANNEL);
  adc_fifo_setup(
		 true,    // Write each completed conversion to the sample FIFO
		 true,    // Enable DMA data request (DREQ)
		 1,       // DREQ (and IRQ) asserted when at least 1 sample present
		 false,   // We won't see the ERR bit because of 8 bit reads; disable.
		 false     // Shift each sample to 8 bits when pushing to FIFO
		 );

  // set sample rate
  adc_set_clkdiv((48000000 / CAPTURE_FREQUENCY) - 1);

  sleep_ms(1000);
  // Set up the DMA to start transferring data as soon as it appears in FIFO
  uint dma_chan = dma_claim_unused_channel(true);
  cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  // Pace transfers based on availability of ADC samples
  channel_config_set_dreq(&cfg, DREQ_ADC);

  dma_channel_configure(dma_chan, &cfg,
			NULL,    // dst
			&adc_hw->fifo,  // src
			NSAMP * 4,          // transfer count
			false            // start immediately
	);

  // Tell the DMA to raise IRQ line 0 when the channel finishes a block
  dma_channel_set_irq0_enabled(dma_chan, true);
  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_exclusive_handler(DMA_IRQ_0, CaptureSamples);
  irq_set_enabled(DMA_IRQ_0, true);

  adc_run(true); //start running the adc

  dma_channel_transfer_to_buffer_now(dma_chan, g_audio_sample_buffer[capture_index],
    NSAMP * 4);
}

TfLiteStatus InitAudioRecording(tflite::ErrorReporter* error_reporter) {
  // Hook up the callback that will be called with each sample
  setup();
  // Manually call the handler once, to trigger the first transfer
  CaptureSamples();
  // let first samples roll in
  sleep_ms(1000);
  // Block until we have our first audio sample
  while (!g_latest_audio_timestamp) {
  }

  return kTfLiteOk;
}

TfLiteStatus GetAudioSamples(tflite::ErrorReporter* error_reporter,
                             int start_ms, int duration_ms,
                             int* audio_samples_size, int16_t** audio_samples) {
  // Set everything up to start receiving audio
  if (!g_is_audio_initialized) {
    TfLiteStatus init_status = InitAudioRecording(error_reporter);
    if (init_status != kTfLiteOk) {
      return init_status;
    }
    g_is_audio_initialized = true;
  }
  // This next part should only be called when the main thread notices that the
  // latest audio sample data timestamp has changed, so that there's new data
  // in the capture ring buffer. The ring buffer will eventually wrap around and
  // overwrite the data, but the assumption is that the main thread is checking
  // often enough and the buffer is large enough that this call will be made
  // before that happens.

  const int16_t kAdcSampleDC = 0;
  const int16_t kAdcSampleGain = 1;

  // Determine the index, in the history of all samples, of the first
  // sample we want
  //printf("getting audio samples\n");
  const int start_offset = start_ms * (kAudioSampleFrequency / 1000);
  // Determine how many samples we want in total
  const int duration_sample_count = duration_ms * (kAudioSampleFrequency / 1000);
  for (int i = 0; i < duration_sample_count; ++i) {
    // For each sample, transform its index in the history of all samples into
    // its index in g_audio_capture_buffer
    const int array_capture_index = (start_offset + i) % kAudioCaptureBufferSize;
    const int32_t capture_value = g_audio_capture_buffer[array_capture_index];
    int32_t output_value = capture_value - 0x7ff;
    //
    output_value *= kAdcSampleGain;
    //printf("%d, \n", output_value);
    if (output_value < std::numeric_limits<int16_t>::min()) {
      output_value = std::numeric_limits<int16_t>::min();
    }
    if (output_value > std::numeric_limits<int16_t>::max()) {
      output_value = std::numeric_limits<int16_t>::max();
    }
    // Write the sample to the output buffer
    g_audio_output_buffer[i] = output_value;
    //printf("%d\n", output_value);
    //g_audio_output_buffer[i] = capture_value;
  }
  // Set pointers to provide access to the audio
  *audio_samples_size = kMaxAudioSampleSize;
  *audio_samples = g_audio_output_buffer;

  return kTfLiteOk;
}

int32_t LatestAudioTimestamp() { return g_latest_audio_timestamp; }
