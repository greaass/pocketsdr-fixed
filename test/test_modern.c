/* L3 外交层：独立构造 ICD 位域，覆盖非零参数、坏帧、跨周和偏差来源。 */
#include "rtklib.h"
#include <assert.h>
#include <stdint.h>

/* 直接逐位写入，避免依赖生产代码的位读取辅助函数。 */
static void put(uint8_t *d, int p, int n, int64_t value)
{
    int i;
    for (i=0;i<n;i++) {
        int mask=1<<(7-(p+i)%8);
        if (((uint64_t)value>>(n-1-i))&1) d[(p+i)/8]|=mask;
        else d[(p+i)/8]&=~mask;
    }
}

/* 从 ICD 的生成多项式计算校验字；翻转内容后故意保留旧 CRC。 */
static void seal(uint8_t *d, int start, int bits)
{
    uint32_t crc=0;
    int i;
    for (i=start;i<start+bits-24;i++) {
        crc=(crc<<1)^((((crc>>23)&1)^((d[i/8]>>(7-i%8))&1))?0x1864CFB:0);
    }
    put(d,start+bits-24,24,crc&0xFFFFFF);
}

/* 使用负半长轴变化率和超出有符号 32 位的角度，检验最易截断的字段。 */
static void orbit(uint8_t *a, int p, uint8_t *b, int q, int bds)
{
    int n=bds?19:17;
    put(a,p,26,-1024); put(a,p+26,25,-65536);
    put(a,p+51,17,-12345); put(a,p+68,23,34567);
    put(a,p+91,33,-3000000000LL); put(a,p+124,33,500000000);
    put(a,p+157,33,3000000000LL);
    put(b,q,33,-3000000000LL); put(b,q+33,33,1500000000);
    put(b,q+66,n,-12345); q+=66+n;
    put(b,q,15,-2345); put(b,q+15,16,-1234); put(b,q+31,16,2345);
    put(b,q+47,24,-54321); put(b,q+71,24,65432);
    put(b,q+95,21,-23456); put(b,q+116,21,34567);
}

/* 检查解析值和尺度，而不是只验证返回成功。 */
static void numeric(const eph_t *e, int bds)
{
    assert(fabs(e->A-((bds?27906100.0:26559710.0)-2.0))<1E-9);
    assert(e->Adot==-0.03125);
    assert(fabs(e->M0-ldexp(-3000000000.0,-32)*SC2RAD)<1E-12);
    assert(fabs(e->ndot-ldexp(34567.0,-57)*SC2RAD)<1E-22);
    assert(fabs(e->OMG0-ldexp(-3000000000.0,-32)*SC2RAD)<1E-12);
    assert(e->crs==ldexp(-54321.0,-8));
}

/* CNAV 覆盖匹配参考时刻、CRC、PRN 和下一帧时间跨周。 */
static void gps_cnav(void)
{
    uint8_t a[38]={0},b[38]={0},c[38]={0};
    uint8_t *frames[]={a,b,c};
    int i,types[]={10,11,30},sat=satid2no("G02");
    eph_t e;
    for (i=0;i<3;i++) {
        put(frames[i],0,8,0x8B); put(frames[i],8,6,2);
        put(frames[i],14,6,types[i]); put(frames[i],20,17,50000+i*2);
    }
    put(a,38,13,2200); put(a,54,11,1000); put(c,38,11,1000);
    put(a,70,11,1000); put(b,38,11,1000); put(c,60,11,1000);
    orbit(a,81,b,49,0);
    put(c,127,13,20); put(c,153,13,-50);
    for (i=0;i<3;i++) seal(frames[i],0,300);
    assert(decode_gps_cnav(a,b,c,sat,&e,NULL)); numeric(&e,0);
    assert(e.tgd[2]==ldexp(-50.0,-35));
    b[10]^=1; assert(!decode_gps_cnav(a,b,c,sat,&e,NULL)); b[10]^=1;
    assert(!decode_gps_cnav(a,b,c,satid2no("G03"),&e,NULL));
    put(c,60,11,1001); seal(c,0,300); assert(!decode_gps_cnav(a,b,c,sat,&e,NULL));
    put(c,60,11,1000); put(a,54,11,999); seal(a,0,300); seal(c,0,300);
    assert(!decode_gps_cnav(a,b,c,sat,&e,NULL));
    put(a,54,11,1000);
    for (i=0;i<3;i++) {
        put(frames[i],20,17,0);
        put(frames[i],i==0?70:i==1?38:60,11,2015); seal(frames[i],0,300);
    }
    assert(decode_gps_cnav(a,b,c,sat,&e,NULL));
    assert(fabs(timediff(e.toe,gpst2time(2200,604500)))<1E-9);
}

/* CNAV-2 两子帧非字节对齐，导频和数据分量 ISC 不能交换。 */
static void gps_cnav2(void)
{
    uint8_t d[111]={0}; eph_t e;
    put(d,0,9,1); put(d,9,13,2200); put(d,22,8,40); put(d,47,11,960);
    orbit(d,58,d,248,0); put(d,609,8,2); put(d,617,6,1);
    put(d,548,13,23); put(d,561,13,-45);
    seal(d,9,600); seal(d,609,274);
    assert(decode_gps_cnav2(d,satid2no("G02"),&e,NULL)); numeric(&e,0);
    assert(e.tgd[1]==ldexp(23.0,-35)&&e.tgd[2]==ldexp(-45.0,-35));
    d[70]^=1; assert(!decode_gps_cnav2(d,satid2no("G02"),&e,NULL));
}

/* B-CNAV1 保留广播 IOD，拒绝同帧内部不一致和无定义卫星类型。 */
static void bds_bcnav1(void)
{
    uint8_t d[110]={0}; eph_t e;
    put(d,0,6,28); put(d,6,8,0); put(d,14,13,844); put(d,27,8,80);
    put(d,35,10,263); put(d,45,8,7); put(d,53,11,960); put(d,64,2,3);
    orbit(d,66,d,256,1); put(d,478,11,960); put(d,614,6,1);
    put(d,571,12,-23); put(d,559,12,45);
    seal(d,14,600); seal(d,614,264);
    assert(decode_bds_bcnav1(d,satid2no("C28"),&e,NULL)); numeric(&e,1);
    assert(e.iodc==263&&e.iode==7&&e.tgd[4]==ldexp(45.0,-34));
    put(d,45,8,8); seal(d,14,600);
    assert(!decode_bds_bcnav1(d,satid2no("C28"),&e,NULL));
}

/* B-CNAV2 验证相邻轨道帧、匹配 IOD 以及 B2a 独立群延迟。 */
static void bds_bcnav2(void)
{
    uint8_t a[36]={0},b[36]={0},c[36]={0},*frames[]={a,b,c};
    int i,types[]={10,11,30}; eph_t e;
    for (i=0;i<3;i++) {
        put(frames[i],0,6,28); put(frames[i],6,6,types[i]);
        put(frames[i],12,18,96000+i);
    }
    put(a,30,13,844); put(a,53,8,7); put(a,61,11,960); put(a,72,2,3);
    orbit(a,74,b,42,1); put(c,42,11,960); put(c,111,10,263);
    put(c,121,12,-23); put(c,133,12,45);
    for (i=0;i<3;i++) seal(frames[i],0,288);
    assert(decode_bds_bcnav2(a,b,c,satid2no("C28"),&e,NULL)); numeric(&e,1);
    assert(e.tgd[3]==ldexp(-23.0,-34)&&e.tgd[5]==ldexp(45.0,-34));
    put(b,12,18,96002); seal(b,0,288);
    assert(!decode_bds_bcnav2(a,b,c,satid2no("C28"),&e,NULL));
}

/* 每个协议都覆盖成功和拒绝路径，进程失败会阻止安装。 */
int main(void)
{
    gps_cnav(); gps_cnav2(); bds_bcnav1(); bds_bcnav2();
    puts("PASS: modern ICD fields, CRC, PRN, IOD, references, week rollover");
    return 0;
}
