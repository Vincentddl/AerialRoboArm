#include "drv_jy60.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void packet(uint8_t out[11], uint8_t type, int16_t x, int16_t y, int16_t z)
{
    memset(out,0,11);out[0]=0x55;out[1]=type;
    int16_t v[3]={x,y,z};
    for (unsigned i=0;i<3;i++) { out[2+i*2]=(uint8_t)v[i]; out[3+i*2]=(uint8_t)((uint16_t)v[i]>>8); }
    for (unsigned i=0;i<10;i++) out[10]=(uint8_t)(out[10]+out[i]);
}
static void feed(Jy60Parser_t *p,const uint8_t *bytes,unsigned n,uint32_t time)
{ for (unsigned i=0;i<n;i++) (void)DrvJy60_Push(p,bytes[i],time); }

int main(void)
{
    Jy60Parser_t p;uint8_t b[11];DrvJy60_Init(&p);
    packet(b,0x53,-16384,8192,0);
    feed(&p,b,4,0);assert(p.good_frames==0);
    feed(&p,b+4,7,100);assert(p.good_frames==1);
    assert(p.angle.milli[0]==-90000 && p.angle.milli[1]==45000 && p.angle.milli[2]==0);
    feed(&p,b,11,150);assert(p.angle_period_ms==50);
    packet(b,0x51,0,-32768,2048);feed(&p,b,11,160);
    assert(p.acceleration.milli[1]==-16000 && p.acceleration.milli[2]==1000);
    packet(b,0x52,32767,-32768,0);feed(&p,b,11,170);
    assert(p.gyro.milli[0]==1999938 && p.gyro.milli[1]==-2000000);
    unsigned good=p.good_frames;b[10]^=1;feed(&p,b,11,180);
    assert(p.good_frames==good && p.checksum_errors==1);
    uint8_t noise[]={0x00,0xFF,0x55,0x99,0x55};feed(&p,noise,sizeof(noise),190);
    packet(b,0x53,85,0,-32768);feed(&p,b,11,200);
    assert(p.angle.count==3 && p.angle.milli[2]==-180000);
    /* Loss of one byte followed by a good frame recovers without reset. */
    feed(&p,b,5,210);feed(&p,b+6,5,210);feed(&p,b,11,250);
    assert(p.angle.count==4 && p.angle.timestamp_ms==250);
    DrvJy60_Init(&p);packet(b,0x53,0,0,0);feed(&p,b,11,0xFFFFFFF0U);
    feed(&p,b,11,0x22);assert(p.angle_period_ms==50);
    /* JY61P-compatible 200 Hz stream, all three packet types for 10 s. */
    DrvJy60_Init(&p);
    for (unsigned tick=0;tick<2000;tick++) {
        for (unsigned type=0x51;type<=0x53;type++) {
            packet(b,(uint8_t)type,2048,-4096,8192);
            feed(&p,b,11,tick*5U);
        }
    }
    assert(p.good_frames==6000 && p.acceleration.count==2000 && p.gyro.count==2000 && p.angle.count==2000);
    assert(p.checksum_errors==0 && p.discarded_bytes==0 && p.angle_period_ms==5);
    puts("JY60/JY61P parser passed: scales, noise recovery, wrap, and 200 Hz three-packet stream.");
    return 0;
}
