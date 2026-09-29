/**
 * @file syscalls.c
 * @author ai
 * @brief dart_final_test_app 专用的 newlib 系统调用桩
 *
 * @note  F4 只有 sysmem.c(_sbrk), 缺少 stdio 所需的 syscall 桩; app 用 snprintf 会链接失败。
 *        按团队规则(底层库不改), 在 app 目录内补最小实现。输出统一走 RTT, 不经 _write。
 */
#include <errno.h>
#include <sys/stat.h>

int _getpid(void) { return 1; }

int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

void _exit(int status) {
  (void)status;
  while (1) {
  }
}

int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  return len; /* 丢弃(不使用 _write 输出, 统一走 RTT) */
}

int _close(int file) {
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st) {
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}
