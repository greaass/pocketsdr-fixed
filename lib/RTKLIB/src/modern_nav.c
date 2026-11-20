/* L1 积木层：跨协议复用数值读取，所有位偏移均以零为起点。 */
#include "modern_nav.h"

/* 33 位字段必须拆分读取，避免 RTKLIB 的 32 位位移截断。 */
double nav_u(const uint8_t *d, int p, int n, int scale)
{
    double value=n==33?getbitu(d,p,1)*4294967296.0+getbitu(d,p+1,32):getbitu(d,p,n);
    return ldexp(value,scale);
}

/* 保持补码符号扩展，支持超过有符号整型宽度的轨道角。 */
double nav_s(const uint8_t *d, int p, int n, int scale)
{
    double value=n==33?getbits(d,p,1)*4294967296.0+getbitu(d,p+1,32):getbits(d,p,n);
    return ldexp(value,scale);
}

/* CRC 可从任意位开始，不把子帧开头误当作字节边界。 */
int nav_crc(const uint8_t *d, int start, int n)
{
    uint32_t crc=0;
    int i,bit;
    for (i=0;i<n-24;i++) {
        bit=((crc>>23)&1)^getbitu(d,start+i,1);
        crc=(crc<<1)&0xFFFFFF;
        if (bit) crc^=0x864CFB;
    }
    return crc==getbitu(d,start+n-24,24);
}

/* 用半周规则比较电文时间，避免跨周时把相邻帧判断为过期。 */
double nav_wrap(double dt)
{
    if (dt>302400.0) dt-=604800.0;
    if (dt< -302400.0) dt+=604800.0;
    return dt;
}

/* 参考时间只使用广播周数，不依据系统日期补全。 */
gtime_t nav_ref(int week, double sow, double ref, int bds)
{
    if (ref<sow-302400.0) week++;
    if (ref>sow+302400.0) week--;
    return bds?bdt2gpst(bdt2time(week,ref)):gpst2time(week,ref);
}

/* 从半长轴差开始的字段布局由 GPS 与北斗现代电文共享。 */
void nav_orbit1(const uint8_t *d, int p, eph_t *e, double reference)
{
    e->A=reference+nav_s(d,p,26,-9);
    e->Adot=nav_s(d,p+26,25,-21);
    e->deln=nav_s(d,p+51,17,-44)*SC2RAD;
    e->ndot=nav_s(d,p+68,23,-57)*SC2RAD;
    e->M0=nav_s(d,p+91,33,-32)*SC2RAD;
    e->e=nav_u(d,p+124,33,-34);
    e->omg=nav_s(d,p+157,33,-32)*SC2RAD;
}

/* 北斗升交点角速度为 19 位；GPS 为带参考值的 17 位差。 */
void nav_orbit2(const uint8_t *d, int p, eph_t *e, int bds)
{
    int n=bds?19:17;
    e->OMG0=nav_s(d,p,33,-32)*SC2RAD;
    e->i0=nav_s(d,p+33,33,-32)*SC2RAD;
    e->OMGd=(nav_s(d,p+66,n,-44)+(bds?0.0:-2.6E-9))*SC2RAD;
    p+=66+n;
    e->idot=nav_s(d,p,15,-44)*SC2RAD;
    e->cis=nav_s(d,p+15,16,-30); e->cic=nav_s(d,p+31,16,-30);
    e->crs=nav_s(d,p+47,24,-8); e->crc=nav_s(d,p+71,24,-8);
    e->cus=nav_s(d,p+95,21,-30); e->cuc=nav_s(d,p+116,21,-30);
}

/* 起点为钟差常数项，参考时刻由各电文的外层布局处理。 */
void nav_clock(const uint8_t *d, int p, eph_t *e, int bds)
{
    e->f0=nav_s(d,p,bds?25:26,bds?-34:-35);
    p+=bds?25:26;
    e->f1=nav_s(d,p,bds?22:20,bds?-50:-48);
    p+=bds?22:20;
    e->f2=nav_s(d,p,bds?11:10,bds?-66:-60);
}

/* 两类 GPS 电文均广播八个 Klobuchar 系数。 */
void nav_klobuchar(const uint8_t *d, int p, double *ion)
{
    static const int scale[]={-30,-27,-24,-24,11,14,16,16};
    int i;
    for (i=0;i<8;i++) ion[i]=nav_s(d,p+8*i,8,scale[i]);
}

/* BDGIM 的符号约定不能统一按补码处理。 */
void nav_bdgim(const uint8_t *d, int p, double *ion)
{
    int i,n;
    for (i=0;i<9;i++,p+=n) {
        n=i==0?10:8;
        ion[i]=(i==1||i>=5?nav_s(d,p,n,-3):nav_u(d,p,n,-3))*(i==4?-1.0:1.0);
    }
}

/* 拒绝保留值和过期轨道；健康标志保留给定位器，及时替换旧健康状态。 */
int nav_valid(eph_t *e)
{
    double age=satsys(e->sat,NULL)==SYS_GPS?5400.0:7200.0;
    return e->toes<604800.0&&e->A>2.0E7&&e->A<5.0E7&&e->e<1.0&&
        fabs(timediff(e->ttr,e->toe))<=age&&
        fabs(timediff(e->ttr,e->toc))<=age;
}
