#include "Encoder.hpp"

void IRAM_ATTR Encoder::isr() {
  counter++;
}

