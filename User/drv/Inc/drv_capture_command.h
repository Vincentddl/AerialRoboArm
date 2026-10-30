#ifndef DRV_CAPTURE_COMMAND_H
#define DRV_CAPTURE_COMMAND_H
#include <stddef.h>
#include <stdint.h>
typedef struct { char key; int32_t value; } CaptureCommand_t;
typedef enum { CAP_CMD_OK=0, CAP_CMD_BAD_KEY, CAP_CMD_BAD_NUMBER, CAP_CMD_TRAILING } CaptureCommandResult_t;
CaptureCommandResult_t DrvCaptureCommand_Parse(const char *line,size_t length,CaptureCommand_t *out);
#endif
