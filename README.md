# TinyML Wake-Word Detection on Raspberry Pi Pico

The wake word example shows how to run a 20 kB neural network that can detect 2
keywords, "yes" and "no". More information about this example can be found on
the [Tensorflow Lite Micro examples folder](https://github.com/tensorflow/tensorflow/tree/master/tensorflow/lite/micro/examples/micro_speech).

## Contents

## Overview


## Before You Begin

### Hardware Requirements

- 1x Raspberry Pi Pico
- 1x Adafruit Electret Microphone with adjustable gain
- 1x MicroUSB to USB cable
- 3x Jumper wires

### Setup

- Setup the Raspberry Pi Pico SDK by following steps on [GitHub](https://github.com/raspberrypi/pico-sdk).
- Git clone this repository.
- (Alternatively) use the Dockerfile to setup and build the application.

## Assembly and Wiring

### Assembly

1. Solder headers onto your Raspberry Pi Pico
2. Solder headers onto your Adafruit Electret Microphone

### Wiring

The electret microphone breakout is a analog, this means we can connect it to
one of the ADC pins on the Raspberry Pi Pico. The following connections are to
made.

| __Adafruit Electret Microphone__ | __Raspberry Pi Pico__ |
|------------------------------|-------------------|
| OUT                          | ADC0 - Pin31      |
| GND                          | Any ground pin    |
| VCC                          | 3V3(OUT) - Pin36  |

![The Raspberry Pi Pico](images/pico_wake_word_bb.png)

## Wake-Word uf2 file

## Build with Docker

## Build Yourself

## Deep-Dive

### Raspberry Pi Pico

![Raspberry Pi Pico](imgs/pico-board3.png)

The Raspberry Pi Pico is the latest product by the Raspberry Pi foundation. It
is a low-cost microcontroller board (<£4) which features the new _RP2040_ chip
by Raspberry Pi.

The RP2040 is built on a "high-clocked" dual-core Cortex M0+ processor, making
it a "remarkably" good platform for endpoint AI. Find out more about the Pico
and the RP2040 from James Adams, COO, Raspberry Pi on arm.com
[here](https://www.arm.com/blogs/blueprint/raspberry-pi-rp2040).

### Tensorflow Lite for Microcontrollers

> TensorFlow Lite for Microcontrollers is designed to run machine learning models
> on microcontrollers and other devices with only few kilobytes of memory. The
> core runtime just fits in 16 KB on an Arm Cortex M3 and can run many basic
> models. It doesn't require operating system support, any standard C or C++
> libraries, or dynamic memory allocation.
Learn more [here](https://www.tensorflow.org/lite/microcontrollers).

### Code

We will now go through the changes made to the Tensorflow `micro_speech` example
to allow it to work with the Pico. The Tensorflow team have already done a port
of Tensorflow Lite Micro for the Pico which can be found
[here](https://github.com/raspberrypi/pico-tflmicro).

The two files that need to be edited for the `micro_speech` application are the
`audio_provider.cc` and the `command_responder.cc`. The `audio_provider.cc` is
what connects a device's microphone hardware to the application, and the
`command_responder.cc` takes the model output and produces an output to say
which word was suggested.


#### Audio Provider

The `audio_provider.cc` works by continuously collecting data from the
microphone and saving the updated data into an array. This means collecting
data while other parts of the code are running, so the current audio can be
analysed while new audio is collected. We need to implement two functions for
this to work with the rest of the application, these are `GetAudioSamples()`
and `LatestAudioTimestamp`.

How this works:
- DMA to collect data of CPU
- Interrupt function to clean up data and put into ring buffer
- Ring buffer which is an array which keeps updating

To do this, we first make a function, `setup()`, this initializes the ADC,
the DMA and an interrupt. For more examples on the Pico's DMA and ADC, please
take a look at the [Pico-Examples](https://github.com/raspberrypi/pico-examples)
repository.

Lets break down this function, the first step is setting up the ADC:

```cpp
#define CLOCK_DIV 3000

adc_gpio_init(26 + CAPTURE_CHANNEL);
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
adc_set_clkdiv(CLOCK_DIV);
```

The final line sets the rate at which data is collected from the ADC into the
FIFO. This is based on the 48MHz ADC clock. Because the `micro_speech` model
expects 16KHz input audio, it is important we are sampling at that rate.

With the ADC setup we can now setup the DMA to transfer the data from the ADC
FIFO into an array. To do this we claim a DMA channel and set the DMA to read
a set amount caled `NSAMP` from the ADC FIFO before completing. We do not set a
write location at this step as we do this during the callback.

```cpp
#define NSAMP 1024

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
		NSAMP,          // transfer count
		false            // do no start immediately
);
```

The last step is to create the interrupt callback and setup the callback to
trigger when the DMA is complete. To do this we first create the callback
function called `CaptureSamples()`.

A lot of this function is taken from the Tensorflow `micro_speech` examples, so
it can be helpful to get a breif understanding before reading this function.

We start of by defining a large array (the ring buffer), a smaller array
(the buffer the DMA will write to) and a timestamp (used to calculate the
index):

```cpp
uint16_t g_audio_sample_buffer[NSAMP]; // the dma write location
constexpr int kAudioCaptureBufferSize = NSAMP * 16;
int16_t g_audio_capture_buffer[kAudioCaptureBufferSize]; // the ring buffer
volatile int32_t g_latest_audio_timestamp = 0;
```

We can now define the interrupt function. When the DMA is complete we want to
calculate the index of the ring buffer to store the new data. This is done by
converting the current timestamp into an index and using memory copy to
transfer the bytes to that location.

```cpp
void CaptureSamples() {
  // data processing
  const int number_of_samples = NSAMP;
  // Calculate what timestamp the last audio sample represents
  const int32_t time_in_ms = g_latest_audio_timestamp + (number_of_samples / (kAudioSampleFrequency / 1000));
  // Determine the index, in the history of all samples, of the last sample
  const int32_t start_sample_offset = g_latest_audio_timestamp * (kAudioSampleFrequency / 1000);
  // Determine the index of this sample in our ring buffer
  const int capture_index = start_sample_offset % kAudioCaptureBufferSize;
  // Read the data to the correct place in our buffer
  memcpy(g_audio_capture_buffer + capture_index, (void *)g_audio_sample_buffer, sizeof(int16_t)*number_of_samples);

  // Clear the interrupt request.
  dma_hw->ints0 = 1u << dma_chan;
  // Give the channel a new wave table entry to read from, and re-trigger it
  dma_channel_set_write_addr(dma_chan, g_audio_sample_buffer, true);

  g_latest_audio_timestamp = time_in_ms;
}
```

We now add the interrupt callback onto the dma channel.

```cpp
dma_channel_set_irq0_enabled(dma_chan, true);
// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
irq_set_exclusive_handler(DMA_IRQ_0, CaptureSamples);
irq_set_enabled(DMA_IRQ_0, true);
```

Finally, with all this complete, we can now start the ADC and initialize the
DMA by manually calling the `CaptureSamples()` function.

```cpp
adc_run(true); //start running the adc
CaptureSamples();
```

#### Command Responder

The `command_responder.cc` implements one function, `RespondToCommand`. In this
implementation application will turn the onboard LED on when "yes" is said and
turn the onboard LED off when "no" is said, as well as writing the output to the
serial output.

The first part to this is initializing the onboard LED:

```cpp
// led settings
static bool is_initialized = false;
const uint LED_PIN = 25;
// if not initialized, setup
if(!is_initialized) {
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  is_initialized = true;
}
```

With the onboard LED initialized, we can now use the input variables to handle
the output. This has two steps, the first step is to log the output into the
`error_reporter`:

```cpp
if (is_new_command) {
  TF_LITE_REPORT_ERROR(error_reporter, "Heard %s (%d) @%dms", found_command,
                       score, current_time);
}
```

The next part to the output handler is to turn the LED on or off based on the
heard command:

```cpp
if (is_new_command) {
  if (found_command == "yes"){
    //turn led on
    gpio_put(LED_PIN, 1);
  }
  else {
    //turn led off
    gpio_put(LED_PIN, 0);
  }
}
```

Putting this all together we get the following function:

```cpp
void RespondToCommand(tflite::ErrorReporter* error_reporter,
                      int32_t current_time, const char* found_command,
                      uint8_t score, bool is_new_command) {

  // led settings
  static bool is_initialized = false;
  const uint LED_PIN = 25;
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
    else {
      //turn led off
      gpio_put(LED_PIN, 0);
    }
  }
}
```
