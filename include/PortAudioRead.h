#include <vector>
#include <string>

#include <portaudio.h>
#include <sndfile.h>
#include <pa_ringbuffer.h>

struct Playback
{
  SNDFILE* audioFile;
  int position;
};

class PortAudioRead
{
public:
  PortAudioRead(SNDFILE* audioFile, int num_frames, int num_channels, int (*lights)(uint8_t*), void (*cbFunc)()) throw(std::string);
  ~PortAudioRead();

  void Start();
  void Stop();

  static int Callback(const void *input,
                      void *output,
                      unsigned long fame_count,
                      const PaStreamCallbackTimeInfo *time_info,
                      PaStreamCallbackFlags status_flags,
                      void *user_data);

  float* window;
  uint8_t lightsBuffer[6*40];
  int (*lights)(uint8_t*);
  long offset = 0; 

private:
  const int CHANNEL_COUNT = 2;
  const int SAMPLE_RATE = 44100;
  const PaStreamParameters *NO_INPUT = nullptr;

  void (*cbFunc)();
  int num_frames;
  int num_channels;
  PaStream* pa_stream;
  SNDFILE* audioFile;
  int position = 0;
  float* buffer;
  // Pointer to the ring buffer memory.
  float* ringbuffer_;
  // Ring buffer wrapper used in PortAudio.
  PaUtilRingBuffer pa_ringbuffer_;
  // Wait for this number of samples in each Start() call.
  int min_read_samples_;
  bool stop = false;
};

