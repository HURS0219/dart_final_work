/*
 * dart_proto.h — ESP32 <-> C板 串口协议 (ASCII, '\n' 结尾)
 *
 * ESP32 -> C:
 *   PING                         -> PONG
 *   SAVE                         -> SAVED / SAVEERR
 *   H                            心跳
 *   MODE,<m>                     0待机 1手动 2混控 3自检
 *   ANG,<ch>,<deg10>             手动角
 *   MIX,<p1000>,<y1000>,<r1000>  混控指令
 *   TRIM,<ch>,<deg10>
 *   SCALE,<ch>,<scale1000>
 *   DIR,<ch>                     方向取反
 *   ZERO,<ch>                    当前位置记为 0°
 *   RSTCAL,<ch>                  清除标定
 *
 * C -> ESP32 (100ms):
 *   T,<mode>,<link>,
 *     <a0>,<p0>,<d0>,<t0>,<s0>, ... x4,
 *     <mp>,<my>,<mr>
 *   a=角度x10  p=脉宽us  d=方向  t=trim x10  s=scale x1000  m=mix x1000
 */
#ifndef DART_V2_PROTO_H
#define DART_V2_PROTO_H

void DartProtoInit(void);
void DartProtoTask(void);
void DartProtoSend(const char *s);

#endif /* DART_V2_PROTO_H */
