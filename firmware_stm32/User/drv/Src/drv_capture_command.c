#include "drv_capture_command.h"

static int token(const unsigned char *s,size_t n,size_t *used)
{
    *used=1;
    if (s[0]<128U) return s[0];
    if (n>=2U && s[0]==0xC2U && s[1]==0xA0U) { *used=2;return ' '; }
    if (n>=3U) {
        if (s[0]==0xE3U && s[1]==0x80U && s[2]==0x80U) { *used=3;return ' '; }
        if (s[0]==0xE2U && s[1]==0x88U && s[2]==0x92U) { *used=3;return '-'; }
        if (s[0]==0xEFU && s[1]==0xBCU) {
            if (s[2]==0x8DU) { *used=3;return '-'; }
            if (s[2]==0x8BU) { *used=3;return '+'; }
            if (s[2]>=0x90U && s[2]<=0x99U) { *used=3;return '0'+s[2]-0x90; }
        }
    }
    return -1;
}
static int whitespace(int c) { return c==' ' || c=='\t' || c=='\r' || c=='\n'; }

CaptureCommandResult_t DrvCaptureCommand_Parse(const char *s,size_t length,CaptureCommand_t *out)
{
    if (!s || !out || length>=32U) return CAP_CMD_BAD_KEY;
    char normalized[32];size_t n=0,i=0;
    if (length>=3U && (unsigned char)s[0]==0xEFU && (unsigned char)s[1]==0xBBU && (unsigned char)s[2]==0xBFU) i=3;
    while (i<length) {
        size_t used;int c=token((const unsigned char*)s+i,length-i,&used);
        if (c<0 || c==0) return CAP_CMD_BAD_NUMBER;
        normalized[n++]=(char)c;i+=used;
    }
    i=0;while (i<n && whitespace(normalized[i])) i++;
    if (i==n) return CAP_CMD_BAD_KEY;
    char key=normalized[i++];if (key>='A' && key<='Z') key=(char)(key-'A'+'a');
    if (key!='g' && key!='v' && key!='a' && key!='w' && key!='r' && key!='x' && key!='m' && key!='h' && key!='?') return CAP_CMD_BAD_KEY;
    while (i<n && whitespace(normalized[i])) i++;
    int32_t sign=1,value=0;
    if (key!='x' && key!='m' && key!='h' && key!='?') {
        if (i<n && (normalized[i]=='-' || normalized[i]=='+')) { sign=normalized[i]=='-' ? -1:1;i++; }
        size_t first=i;
        while (i<n && normalized[i]>='0' && normalized[i]<='9') {
            if (value>10000) return CAP_CMD_BAD_NUMBER;
            value=value*10+normalized[i++]-'0';
        }
        if (i==first) return CAP_CMD_BAD_NUMBER;
    }
    while (i<n && whitespace(normalized[i])) i++;
    if (i!=n) return CAP_CMD_TRAILING;
    out->key=key;out->value=value*sign;return CAP_CMD_OK;
}
