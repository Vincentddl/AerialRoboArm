#include "usart.h"

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

extern char _end;

int _close(int file)
{
    (void)file;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    (void)file;
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

off_t _lseek(int file, off_t offset, int whence)
{
    (void)file;
    (void)offset;
    (void)whence;
    return 0;
}

int _read(int file, char *buffer, int length)
{
    (void)file;
    (void)buffer;
    (void)length;
    errno = EIO;
    return -1;
}

int _write(int file, char *buffer, int length)
{
    (void)file;
    if ((buffer == NULL) || (length <= 0)) {
        return 0;
    }
    if (HAL_UART_Transmit(&huart2, (uint8_t *)buffer, (uint16_t)length, 100U)
        != HAL_OK) {
        errno = EIO;
        return -1;
    }
    return length;
}

void *_sbrk(ptrdiff_t increment)
{
    static char *heap_end;
    register char *stack_pointer asm("sp");

    if (heap_end == NULL) {
        heap_end = &_end;
    }
    if ((heap_end + increment) > stack_pointer) {
        errno = ENOMEM;
        return (void *)-1;
    }

    char *previous = heap_end;
    heap_end += increment;
    return previous;
}

int _getpid(void)
{
    return 1;
}

int _kill(int pid, int signal)
{
    (void)pid;
    (void)signal;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    (void)status;
    while (1) {
    }
}
