/* dart_mode.h — 三种飞行模式定义 (移植自 dart_fc/App/mode.h) */
#ifndef DART_MODE_H
#define DART_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODE_COMPETITION      = 0,
    MODE_DECOUPLED_MANUAL = 1,
    MODE_FULL_MANUAL      = 2,
    MODE_INVALID          = 0xFF
} dart_mode_t;

const char *mode_name(int m);
int  mode_valid(int m);
void mode_led_color(int m, unsigned char *r, unsigned char *g, unsigned char *b);

#ifdef __cplusplus
}
#endif

#endif /* DART_MODE_H */
