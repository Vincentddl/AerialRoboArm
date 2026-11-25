#include "drv_capture_command.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void good(const char *s,char key,int value)
{
    CaptureCommand_t c={0};assert(DrvCaptureCommand_Parse(s,strlen(s),&c)==CAP_CMD_OK);
    assert(c.key==key && c.value==value);
}
int main(void)
{
    good("g -20",'g',-20);good("g-20",'g',-20);good(" \tG -20 \r\n",'g',-20);
    good("g +85",'g',85);good("g 0",'g',0);good("v 220",'v',220);
    good("w 1000",'w',1000);good("a 100",'a',100);good(" x ",'x',0);good(" m ",'m',0);good("?",'?',0);
    good("h",'h',0);
    good("r 1",'r',1);good("r 0",'r',0);
    good("\xEF\xBB\xBFg -20",'g',-20);
    good("g\xC2\xA0-20",'g',-20);
    good("\xE3\x80\x80g \xE2\x88\x92" "20",'g',-20);
    good("g \xEF\xBC\x8D\xEF\xBC\x92\xEF\xBC\x90",'g',-20);
    CaptureCommand_t c;
    const char *bad[]={"g","g -","gg -20","g 20abc","g 20 30","x 20","m 20","h 75","r","g 999999999999999", "g 2.5"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(DrvCaptureCommand_Parse(bad[i],strlen(bad[i]),&c)!=CAP_CMD_OK);
    assert(DrvCaptureCommand_Parse("g\0-20",6,&c)!=CAP_CMD_OK);
    puts("Capture command parsing passed: ASCII, UTF-8 whitespace/sign/digits, bounds and malformed input.");
    return 0;
}
