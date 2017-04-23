#include "PortAudioRead.h"

#include <iostream>
#include <sstream>
#include <cstring>
#include <iomanip>

#include <math.h>
#include <fftw3.h>

#include <unistd.h>

#include <pa_util.h>

#include "EPI.h"

void buildHanWindow(float *window, int size)
{
   for (int i = 0; i < size ; ++i)
      window[i] = .5 * ( 1 - cos( 2 * M_PI * i / (size-1.0) ) );
}

void buildHammingWindow(float *window, int size)
{
   for (int i=0; i < size; ++i)
      window[i] = .54 - .46 * cos( 2 * M_PI * i / (float) size );
}

void applyWindow( float *window, float *data, int size )
{
   for (int i = 0; i < size; ++i)
      data[i] *= window[i] ;
}

float rms(float *v, int n)
{
  int i;
  float sum = 0.0;

  for (i = 0; i < n; i++) {
    sum += v[i] * v[i];
  }

  return sqrt(sum / n);
}

void hsv2rgb(float h, float s, float v, uint8_t* rgb)
{
    double      p, q, t, ff, r, g, b;
    long        i;

    if(h >= 360.0) h = 0.0;
    h /= 60.0;
    i = (long)h;
    ff = h - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
    default:
        r = v;
        g = p;
        b = q;
        break;
    }
    rgb[0] = round(r * 255.f);
    rgb[1] = round(g * 255.f);
    rgb[2] = round(b * 255.f);
}

void computeLighting(float* buffer, int size, PortAudioRead* handler)
{
  float RMS = rms(buffer, size);
  applyWindow(handler->window, buffer, size);
  int outSize = ceil(size/2+1);
  fftwf_complex* out = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex)*outSize);
  fftwf_plan plan = fftwf_plan_dft_r2c_1d(size, buffer, out, FFTW_ESTIMATE);
  fftwf_execute(plan);
  // out is ready
  int stride = static_cast<int>((64.f - powf((64.f * RMS), 1.32f)) + 0.5f);
  uint8_t rgb[3];
  stride = (stride < 1 ? 1 : stride);
  for (int i = 0; i < outSize; i += stride) {
    // HSV
    float amplitude = sqrt(pow(out[i][0], 2) + pow(out[i][1], 2)) * 18.f;
    if (amplitude > 1) {
      if (i == 0) {
        //std::cerr << "error in data?" << std::endl;
        continue;
      } else {
        //std::cerr << "amplitude" << amplitude << std::endl;
        amplitude = 1;
      }
    }
    hsv2rgb(360.f * float(i) / float(outSize), amplitude, amplitude, rgb);
    //std::cout << std::hex << rgb[0] << "/" << rgb[1] << "/" << rgb[2] << std::endl;
    handler->lights[handler->offset] = 0xFF;
    handler->lights[++handler->offset] = rgb[2];
    handler->lights[++handler->offset] = rgb[0];
    handler->lights[++handler->offset] = rgb[1];
    handler->offset++;
    if (handler->offset % 160 == 0) {
      lights(handler->lights);
    }
    if (handler->offset >= 4*60) 
    {
      handler->offset = 0;
    }
  }
  fftwf_destroy_plan(plan);
  fftwf_free(out);
}

PortAudioRead::PortAudioRead(SNDFILE *audioFile, int num_frames, int num_channels, uint8_t* lights) throw(std::string)
    : audioFile(audioFile), num_frames(num_frames), num_channels(num_channels), lights(lights)
{
  int ringbuffer_size = 16384;
  ringbuffer_ = static_cast<float *>(
      PaUtil_AllocateMemory(ringbuffer_size));
  if (ringbuffer_ == NULL)
  {
    std::cerr << "Fail to allocate memory for ring buffer." << std::endl;
  }

  // Initializes PortAudio ring buffer.
  ring_buffer_size_t rb_init_ans =
      PaUtil_InitializeRingBuffer(&pa_ringbuffer_, 1,
                                  ringbuffer_size, ringbuffer_);

  if (rb_init_ans == -1)
  {
    std::cerr << "Ring buffer size is not power of 2." << std::endl << std::flush;
  }

    Pa_Initialize();
    PaError errorCode;
    PaStreamParameters outputParameters;

    outputParameters.device = Pa_GetDefaultOutputDevice();
    outputParameters.channelCount = num_channels;
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = 0.03;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    errorCode = Pa_OpenStream(&pa_stream,
                              NO_INPUT,
                              &outputParameters,
                              SAMPLE_RATE,
                              paFramesPerBufferUnspecified,
                              paNoFlag,
                              &Callback,
                              this);

    if (errorCode)
    {
        Pa_Terminate();

        std::stringstream error;
        error << "Unable to open stream for output. Portaudio error code: " << errorCode;
        throw error.str();
    }

    min_read_samples_ = SAMPLE_RATE * 0.1;

    window = (float*)malloc(sizeof(float) * min_read_samples_);
    buildHammingWindow(window, min_read_samples_);
}

PortAudioRead::~PortAudioRead()
{
    Pa_CloseStream(pa_stream);
    Pa_Terminate();
    sf_close(audioFile);
}

int PortAudioRead::Callback(const void *input,
                            void *output,
                            unsigned long frame_count,
                            const PaStreamCallbackTimeInfo *time_info,
                            PaStreamCallbackFlags status_flag,
                            void *user_data)
{
    PortAudioRead *handler = static_cast<PortAudioRead *>(user_data);

    unsigned long stereoFrameCount = frame_count * handler->num_channels;
    std::memset((float *)output, 0, stereoFrameCount * sizeof(float));

    float *outputBuffer = new float[stereoFrameCount];
    float *bufferCursor = outputBuffer;

    unsigned int framesLeft = (unsigned int)frame_count;
    unsigned int framesRead;
    sf_count_t r;
    bool playbackEnded;
    while (framesLeft > 0)
    {
        sf_seek(handler->audioFile, handler->position, SEEK_SET);

        if (framesLeft > (handler->num_frames - handler->position))
        {
            framesRead = (unsigned int)(handler->num_frames - handler->position);
            handler->position += framesRead;
            playbackEnded = true;
            framesLeft = framesRead;
        }
        else
        {
            framesRead = framesLeft;
            handler->position += framesRead;
        }

        r = sf_readf_float(handler->audioFile, bufferCursor, framesRead);

        if (r == 0) {
          return paComplete;
        }

        bufferCursor += framesRead;

        framesLeft -= framesRead;
    }

    float *outputCursor = (float *)output;
    if (handler->num_channels == 1)
    {
        for (unsigned long i = 0; i < stereoFrameCount; ++i)
        {
            *outputCursor += (0.5 * outputBuffer[i]);
            ++outputCursor;
            *outputCursor += (0.5 * outputBuffer[i]);
            ++outputCursor;
        }
    }
    else
    {
        for (unsigned long i = 0; i < stereoFrameCount; ++i)
        {
            *outputCursor += (0.5 * outputBuffer[i]);
            ++outputCursor;
        }
    }

    if (playbackEnded)
    {
        computeLighting(outputBuffer, framesRead, handler);

        //handler->position = 0;
        //return paComplete;
    }

    return paContinue;
}

void PortAudioRead::Start()
{
    std::cout << "Play Music" << std::endl;
    PaError err;
    if (!Pa_IsStreamStopped(pa_stream))
    {
        std::cout << "Stop stream" << std::endl;
        err = Pa_StopStream(pa_stream);
        if (err < 0)
        {
            std::cerr << "Error" << err << std::endl;
        }
    }
    err = Pa_StartStream(pa_stream);
    if (err < 0)
    {
        std::cerr << "Error" << err << std::endl;
    }

    ring_buffer_size_t num_available_samples = 0;
    while (!Pa_IsStreamStopped(pa_stream))
    {

      num_available_samples =
          PaUtil_GetRingBufferReadAvailable(&pa_ringbuffer_);

      if (num_available_samples >= min_read_samples_)
      {
        if (buffer == NULL){
          buffer = (float*)malloc(min_read_samples_ * sizeof(float));
        }  else {
          buffer = (float*)realloc(buffer, min_read_samples_ * sizeof(float));
        }
        ring_buffer_size_t num_read_samples = PaUtil_ReadRingBuffer(
            &pa_ringbuffer_, buffer, min_read_samples_);

        computeLighting(buffer, num_read_samples, this);
      }
      Pa_Sleep(5);
    }

    free(buffer);
}

