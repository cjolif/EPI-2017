/**
 * author: Christophe Jolif (cjolif@gmail.com)
 *
 * Apache License
 **/

#include <iostream>

#include <stdio.h>
#include <sndfile.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include <softPwm.h>

#include "include/PortAudioRead.h"

#define	LED1		        15
#define LED2		        16
#define BUTTON1	      	11
#define BUTTON2	      	18
#define MOTOR1_RIGHT  	29
#define MOTOR1_LEFT	    31
#define MOTOR1_ENABLE 	33
#define MOTOR2_RIGHT   	36
#define MOTOR2_LEFT     37
#define MOTOR2_ENABLE   32

#define START	0b00000001
#define MOVE	0b00000010
#define STOP  0b00000100
#define RUN	  0b10000000

static const char       *spiDev1  = "/dev/spidev1.0";
static const uint8_t     spiBPW   = 8 ;
static const uint32_t    spiSpeed = 8000000;
static int spiFd;

static uint8_t state = 0b00000000;

static struct spi_ioc_transfer xfer[3];

/* = {
 { .tx_buf        = 0, // Header (zeros)
   .rx_buf        = 0,
   .len           = 4,
   .speed_hz      = 8000000,
   .delay_usecs   = 0,
   .bits_per_word = 8,
   .cs_change     = 0 },
 { .rx_buf        = 0, // Color payload
   .speed_hz      = 8000000,
   .delay_usecs   = 0,
   .bits_per_word = 8,
   .cs_change     = 0 },
 { .tx_buf        = 0, // Footer (zeros)
   .rx_buf        = 0,
   .speed_hz      = 8000000,
   .delay_usecs   = 0,
   .bits_per_word = 8,
   .cs_change     = 0 }
};
*/

static pid_t pid;

static uint8_t buffer[60*4];

static PortAudioRead* pa_read;

void initLights()
{
  int i;
  for (i = 0; i < 60; i++) {
    buffer[i * 4] = 0xFF;
    buffer[i * 4 + 1] = 0xFF; // (b)
    buffer[i * 4 + 2] = 0x00; // (r)
    buffer[i * 4 + 3] = 0x00; // (g)
  }
  xfer[0].tx_buf = 0;
  xfer[0].rx_buf = 0;
  xfer[0].len = 4;
  xfer[0].speed_hz = 8000000;
  xfer[0].delay_usecs = 0;
  xfer[0].bits_per_word = 8;
  xfer[0].cs_change = 0;
  xfer[1].rx_buf = 0;
  xfer[1].speed_hz=  8000000;
  xfer[1].delay_usecs = 0;
  xfer[1].bits_per_word = 8;
  xfer[1].cs_change = 0;
  xfer[2].tx_buf = 0;
  xfer[2].rx_buf = 0;
  xfer[2].speed_hz = 8000000;
  xfer[2].delay_usecs = 0;
  xfer[2].bits_per_word = 8;
  xfer[2].cs_change = 0;
}

int lights(/*uint8_t ptr[]*/)
{
  xfer[1].tx_buf   = (unsigned long)buffer;
  xfer[1].len      = 60 * 4; // numLEDS* 4? (len/4)
  xfer[2].len      = (60 /*numLEDS*/ + 15) / 16;

  return ioctl(spiFd, SPI_IOC_MESSAGE(3), &xfer) ;
}

void scanButton(int button, int led, uint8_t mode)
{
  // TODO better debouncing

  // no button push let's skip
  if (digitalRead(button) == HIGH)
    return;

  // a button push has been detected
  // toggle the state mode on/off
  state ^= mode;

  // trigger the LED
  digitalWrite(led, HIGH);

  // wait until the button is released
  while (digitalRead (button) == LOW)
    delay(10);

  // some delay so that we don't switch off the LED immediately
  delay(200);
  digitalWrite(led, LOW);
}

void startMusic() {
  std::cout << "Start Music" << std::endl;
  pa_read->Start();
/*

  pid = fork();
  if (pid == 0) {
    execl("/usr/bin/aplay", "aplay", "file.mp3", NULL);
    printf("should not happen\n");
  } else {
    // parent just continue as is
  }*/
}

void stopMusic() {
  //if (pid != 0) {
  //  kill(pid, SIGINT);
 // }
}

void startLights() {
  lights();
}

void stopLights() {
}

void applyState() {
  if (((state & START) == START) && ((state & MOVE) == 0)) {
    state ^= MOVE;
    startMusic();
    startLights();
    digitalWrite(MOTOR1_RIGHT, HIGH);
    digitalWrite(MOTOR1_LEFT, LOW);
    softPwmWrite(MOTOR1_ENABLE, 1024);
  } else if (((state & START) == 0) && ((state & MOVE) == MOVE)) {
    // stop
    state ^= MOVE;
    softPwmWrite(MOTOR1_ENABLE, 0);
    stopMusic();
    stopLights();
  }
}

int spiSetup(int mode)
{
  int fd;

  if ((fd = open (spiDev1, O_RDWR)) < 0)
    return wiringPiFailure (WPI_ALMOST, "Unable to open SPI device: %s\n", strerror (errno)) ;

  spiFd = fd ;

  // Set SPI parameters.

  if (ioctl (fd, SPI_IOC_WR_MODE, &mode)            < 0)
    return wiringPiFailure (WPI_ALMOST, "SPI Mode Change failure: %s\n", strerror (errno)) ;

  if (ioctl (fd, SPI_IOC_WR_BITS_PER_WORD, &spiBPW) < 0)
    return wiringPiFailure (WPI_ALMOST, "SPI BPW Change failure: %s\n", strerror (errno)) ;

  if (ioctl (fd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeed)   < 0)
    return wiringPiFailure (WPI_ALMOST, "SPI Speed Change failure: %s\n", strerror (errno)) ;

  return fd ;
}

PortAudioRead* InitMusic()
{
   SF_INFO sf_info;
   SNDFILE *snd_file = sf_open("./resources/RunTheWorld.wav", SFM_READ, &sf_info);
   if (snd_file == NULL) {
     std::cerr << "Music file is not available" << std::endl;
     return NULL;
   } else {
    return new PortAudioRead(snd_file, sf_info.frames, sf_info.channels/*, buffer*/);
   }
}

int main(int argc, char *argv[])
{
  std::cout << "Starting initialization of Raspberry Pi EPI" << std::endl;

  wiringPiSetupPhys();
  
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  
  initLights();
  pa_read = InitMusic();

  std::cout << "Phase 1 Done" << std::endl;
  
  digitalWrite(LED2, HIGH);
  delay(300);
  digitalWrite(LED2, LOW);
 
  state ^= RUN;
  
  uint8_t mode = SPI_MODE_0 | SPI_NO_CS;
  spiSetup(mode);
  
  pinMode(BUTTON1, INPUT);
  pullUpDnControl(BUTTON1, PUD_UP);
  pinMode(BUTTON2, INPUT);
  pullUpDnControl(BUTTON2, PUD_UP);
  
  pinMode(MOTOR1_RIGHT, OUTPUT);
  pinMode(MOTOR1_LEFT, OUTPUT);
  softPwmCreate(MOTOR1_ENABLE, 0, 1024);
  
  std::cout << "Phase 2 Done" << std::endl;
  
  digitalWrite(LED1, HIGH);
  delay(300);
  digitalWrite(LED1, LOW);
  
  state ^= START;

  while ((state & RUN) == RUN) {
   // scanButton(BUTTON1, LED1, START);
    applyState();
    delay(1);
  }
  
  sleep(1000);
  return 0;
}

