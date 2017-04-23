DYNAMIC := True
CC = $(CXX)
CXX :=
LDFLAGS :=
LDLIBS :=
PORTAUDIOINC := portaudio/install/include
PORTAUDIOLIBS := portaudio/install/lib/libportaudio.a

ifeq ($(DYNAMIC), True)
  CXXFLAGS += -fPIC
endif

ifeq ($(shell uname -m | cut -c 1-3), x86)
  CXXFLAGS += -msse  -msse2
endif

ifeq ($(shell uname), Darwin)
  # By default Mac uses clang++ as g++, but people may have changed their
  # default configuration.
  CXX := clang++
  CXXFLAGS += -Wall -Wno-sign-compare -Winit-self\
      -DHAVE_POSIX_MEMALIGN -DHAVE_CLAPACK -I$(PORTAUDIOINC) -I/usr/local/include -Iinclude/
  LDLIBS += -ldl -lm -framework Accelerate -framework CoreAudio \
      -framework AudioToolbox -framework AudioUnit -framework CoreServices \
      $(PORTAUDIOLIBS) -L/usr/local/lib -lsndfile -lfftw3f -lwiringPi -lwiringPiDev 
else ifeq ($(shell uname), Linux)
  CXX := g++ 
  CXXFLAGS += -std=c++0x -Wall -Wno-sign-compare\
      -Wno-unused-local-typedefs -Winit-self -rdynamic\
      -DHAVE_POSIX_MEMALIGN -I$(PORTAUDIOINC) -Iinclude/
  LDLIBS += -ldl -lm -Wl,-Bstatic -Wl,-Bdynamic -lrt -lpthread $(PORTAUDIOLIBS)\
      -L/usr/lib/atlas-base -lf77blas -lcblas -llapack_atlas -latlas -lsndfile -lfftw3f -lwiringPi -lwiringPiDev 
  ifneq ($(wildcard $(PORTAUDIOINC)/pa_linux_alsa.h),)
    LDLIBS += -lasound
  endif
  ifneq ($(wildcard $(PORTAUDIOINC)/pa_jack.h),)
    LDLIBS += -ljack
  endif
endif

# Suppress clang warnings...
COMPILER = $(shell $(CXX) -v 2>&1 )
ifeq ($(findstring clang,$(COMPILER)), clang)
  CXXFLAGS += -Wno-mismatched-tags -Wno-c++11-extensions
endif

# Set optimization level.
CXXFLAGS += -O3
