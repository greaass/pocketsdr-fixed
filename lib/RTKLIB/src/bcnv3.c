/* L1 积木层：按 BDS-SIS-ICD-B2b-1.0 图 6-3～6-8 解析完整自主导航星历。 */
#include "rtklib.h"

/* 33 位轨道字段超出 getbits 的 32 位范围，保留符号位后合成双精度值。 */
static double bits33(const uint8_t *data, int pos)
{
    return getbits(data,pos,1)*4294967296.0+getbitu(data,pos+1,32);
}

/* B-CNAV3 的 CRC 位于非字节边界，按 462 个信息位直接校验。 */
static int valid_crc(const uint8_t *data)
{
    uint32_t crc=0;
    int i;
    for (i=0;i<462;i++) {
        int bit=((crc>>23)&1)^getbitu(data,i,1);
        crc=(crc<<1)&0xFFFFFF;
        if (bit) crc^=0x864CFB;
    }
    return crc==getbitu(data,462,24);
}

/* 参考时间采用电文周数，按半周规则处理跨周，不借用电脑日期。 */
static gtime_t ref_time(int week, double sow, double reference)
{
    if (reference<sow-302400.0) week++;
    else if (reference>sow+302400.0) week--;
    return bdt2gpst(bdt2time(week,reference));
}

/* 返回 1 表示两帧完整且时间有效；B-CNAV3 未定义 IODE，不能伪造广播版本号。 */
extern int decode_bds_bcnv3(const uint8_t *orbit, const uint8_t *clock,
                          int sat, eph_t *eph)
{
    eph_t e={0};
    double sow1,sow2,dt,toc;
    int week,type,prn;
    if (!orbit||!clock||!eph||satsys(sat,&prn)!=SYS_CMP||prn<6||prn>58) return 0;
    if (getbitu(orbit,0,6)!=10||getbitu(clock,0,6)!=30) return 0;
    if (!valid_crc(orbit)||!valid_crc(clock)) return 0;
    sow1=getbitu(orbit,6,20); sow2=getbitu(clock,6,20);
    if (sow1>=604800.0||sow2>=604800.0) return 0;
    dt=sow1-sow2;
    if (dt>302400.0) dt-=604800.0;
    if (dt< -302400.0) dt+=604800.0;
    if (fabs(dt)>120.0) return 0;
    week=getbitu(clock,26,13);
    type=getbitu(orbit,41,2);
    if (type!=2&&type!=3) return 0;
    e.sat=sat; e.code=EPH_BDS_BCNAV3; e.flag=type;
    e.iode=e.iodc=-1; e.week=week;
    e.toes=getbitu(orbit,30,11)*300.0;
    toc=getbitu(clock,43,11)*300.0;
    if (e.toes>=604800.0||toc>=604800.0) return 0;
    e.toe=ref_time(week,sow2,e.toes);
    e.toc=ref_time(week,sow2,toc);
    e.ttr=bdt2gpst(bdt2time(week,sow2));
    if (fabs(timediff(e.ttr,e.toe))>7200.0||fabs(timediff(e.ttr,e.toc))>7200.0) return 0;
    e.A=(type==3?27906100.0:42162200.0)+ldexp(getbits(orbit,43,26),-9);
    e.Adot=ldexp(getbits(orbit,69,25),-21);
    e.deln=ldexp(getbits(orbit,94,17),-44)*SC2RAD;
    e.ndot=ldexp(getbits(orbit,111,23),-57)*SC2RAD;
    e.M0=ldexp(bits33(orbit,134),-32)*SC2RAD;
    e.e=ldexp(getbitu(orbit,167,1)*4294967296.0+getbitu(orbit,168,32),-34);
    e.omg=ldexp(bits33(orbit,200),-32)*SC2RAD;
    e.OMG0=ldexp(bits33(orbit,233),-32)*SC2RAD;
    e.i0=ldexp(bits33(orbit,266),-32)*SC2RAD;
    e.OMGd=ldexp(getbits(orbit,299,19),-44)*SC2RAD;
    e.idot=ldexp(getbits(orbit,318,15),-44)*SC2RAD;
    e.cis=ldexp(getbits(orbit,333,16),-30);
    e.cic=ldexp(getbits(orbit,349,16),-30);
    e.crs=ldexp(getbits(orbit,365,24),-8);
    e.crc=ldexp(getbits(orbit,389,24),-8);
    e.cus=ldexp(getbits(orbit,413,21),-30);
    e.cuc=ldexp(getbits(orbit,434,21),-30);
    e.f0=ldexp(getbits(clock,54,25),-34);
    e.f1=ldexp(getbits(clock,79,22),-50);
    e.f2=ldexp(getbits(clock,101,11),-66);
    e.tgd[1]=ldexp(getbits(clock,112,12),-34);
    e.svh=getbitu(clock,460,2)|getbitu(orbit,455,1)|getbitu(orbit,456,1);
    e.fit=2.0;
    if (e.A<2.0E7||e.A>5.0E7||e.e>=1.0) return 0;
    *eph=e;
    return 1;
}

/* 参数 5 使用负比例因子；参数 3、4 是无符号数，不能统一按补码读取。 */
extern void decode_bds_bdgim(const uint8_t *clock, double *ion)
{
    int i,pos=124;
    for (i=0;i<9;i++) {
        int len=i==0?10:8;
        double value=(i==1||i>=5)?(double)getbits(clock,pos,len):(double)getbitu(clock,pos,len);
        ion[i]=value*(i==4?-0.125:0.125);
        pos+=len;
    }
}
