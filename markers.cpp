#include "markers.h"

Markers::Markers(
    PinName marker_pin
  ) :
    temp_pulldown_(marker_pin),
    pixel_array_(marker_pin)
  {
  marker_data_t* temp_data = (marker_data_t*)calloc(1, sizeof(marker_data_t));
  update(temp_data);
  free(temp_data);
}

void Markers::update(marker_data_t* marker_data) {
  int i, color;
  for (i = 0; i < MARKER_N_PIXEL; i++) {
    color = marker_data->colors[i];
    pixel_buffer_[i].red = (color>>16) & 0xff;
    pixel_buffer_[i].green = (color>>8) & 0xff;
    pixel_buffer_[i].blue = color & 0xff;
  }
  pixel_array_.update(pixel_buffer_, MARKER_N_PIXEL);
}
