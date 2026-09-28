#include "dart_mode.h"

const char *mode_name(int m)
{
    switch (m) {
    case MODE_COMPETITION:      return "COMPETITION";
    case MODE_DECOUPLED_MANUAL: return "DECOUPLED_MANUAL";
    case MODE_FULL_MANUAL:      return "FULL_MANUAL";
    default:                    return "INVALID";
    }
}

int mode_valid(int m)
{
    return (m >= MODE_COMPETITION && m <= MODE_FULL_MANUAL);
}

void mode_led_color(int m, unsigned char *r, unsigned char *g, unsigned char *b)
{
    switch (m) {
    case MODE_COMPETITION:      *r = 0;   *g = 0;   *b = 255; break;
    case MODE_DECOUPLED_MANUAL: *r = 0;   *g = 255; *b = 0;   break;
    case MODE_FULL_MANUAL:      *r = 255; *g = 0;   *b = 0;   break;
    default:                    *r = 255; *g = 255; *b = 255; break;
    }
}
