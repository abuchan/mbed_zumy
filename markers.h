#ifndef MARKERS_H
#define MARKERS_H

#include "mbed.h"
#include "protocol.h"
#include "neopixel.h"

class Markers {
  
  public:
    
    Markers(
      PinName marker_pin
    );

    void update(marker_data_t* marker_data);

  private:

    DigitalIn temp_pulldown_;
    neopixel::PixelArray pixel_array_;
    neopixel::Pixel pixel_buffer_[MARKER_N_PIXEL];

};

#endif // MARKERS_H
