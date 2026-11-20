/* L1 积木层：IS-GPS-200N 的 CNAV 与 IS-GPS-800J 的 CNAV-2 星历。 */
#include "modern_nav.h"

/* 转为既有 URA 方差上界，额外覆盖两小时内的 NED 一阶增长。 */
static int gps_ura(int ed, int ned)
{
    static const double legacy[]={2.4,3.4,4.85,6.85,9.65,13.65,24,48,96,192,384,768,1536,3072,6144};
    double a,b,error;
    int i;
    if (ed==-16||ed==15||ned==-16||ned==15) return -1;
    a=ed<=6?pow(2.0,1.0+ed/2.0):pow(2.0,ed-2.0);
    b=ned<=6?pow(2.0,1.0+ned/2.0):pow(2.0,ned-2.0);
    if (ed==1||ed==3||ed==5) a=ed==1?2.8:ed==3?5.7:11.3;
    if (ned==1||ned==3||ned==5) b=ned==1?2.8:ned==3?5.7:11.3;
    error=sqrt(a*a+(b+0.5)*(b+0.5));
    for (i=0;i<14&&legacy[i]<error;i++) {}
    return i;
}

/* CNAV 必须同时满足轨道参考时刻、钟差参考时刻及预测时刻一致。 */
int decode_gps_cnav(const uint8_t *a, const uint8_t *b, const uint8_t *c,
                    int sat, eph_t *eph, double *ion)
{
    eph_t e={0};
    int prn,i,week;
    double sow[3];
    const uint8_t *frames[3];
    const int types[]={10,11,30};
    frames[0]=a; frames[1]=b; frames[2]=c;
    if (satsys(sat,&prn)!=SYS_GPS) return 0;
    for (i=0;i<3;i++) {
        if (!nav_crc(frames[i],0,300)||getbitu(frames[i],0,8)!=0x8B||
            getbitu(frames[i],8,6)!=prn||getbitu(frames[i],14,6)!=types[i]) return 0;
        sow[i]=getbitu(frames[i],20,17)*6.0;
        if (sow[i]>=604800.0) return 0;
        if (getbitu(frames[i],37,1)) e.svh|=8;
    }
    if (fabs(nav_wrap(sow[0]-sow[1]))>600.0||
        fabs(nav_wrap(sow[0]-sow[2]))>600.0) return 0;
    if (getbitu(a,70,11)!=getbitu(b,38,11)||
        getbitu(a,70,11)!=getbitu(c,60,11)||
        getbitu(a,54,11)!=getbitu(c,38,11)) return 0;
    e.sat=sat; e.code=EPH_GPS_CNAV; e.iode=e.iodc=-1;
    e.week=week=getbitu(a,38,13);
    e.toes=getbitu(a,70,11)*300.0;
    e.ttr=gpst2time(week,sow[0]==0.0?604800.0:sow[0]);
    e.toe=e.toc=nav_ref(week,sow[0]==0.0?604800.0:sow[0],e.toes,0);
    e.svh|=getbitu(a,51,3);
    e.sva=gps_ura(getbits(a,65,5),getbits(c,49,5));
    if (e.sva<0) { e.sva=15; e.svh|=8; }
    nav_orbit1(a,81,&e,26559710.0);
    nav_orbit2(b,49,&e,0);
    nav_clock(c,71,&e,0);
    for (i=0;i<5;i++) e.tgd[i]=nav_s(c,127+13*i,13,-35);
    e.fit=3.0;
    if (!nav_valid(&e)) return 0;
    if (ion&&!e.svh) nav_klobuchar(c,192,ion);
    *eph=e;
    return 1;
}

/* CNAV-2 的 SF2 独立包含完整星历，SF3 页一提供电离层参数。 */
int decode_gps_cnav2(const uint8_t *d, int sat, eph_t *eph, double *ion)
{
    eph_t e={0};
    int prn,toi,itow;
    double sow;
    if (satsys(sat,&prn)!=SYS_GPS||!nav_crc(d,9,600)||
        !nav_crc(d,609,274)||getbitu(d,609,8)!=prn) return 0;
    toi=getbitu(d,0,9); itow=getbitu(d,22,8);
    if (toi>399||itow>83) return 0;
    sow=itow*7200.0+(toi>0?toi-1:399)*18.0;
    e.sat=sat; e.code=EPH_GPS_CNAV2; e.iode=e.iodc=-1;
    e.week=getbitu(d,9,13); e.toes=getbitu(d,47,11)*300.0;
    e.toe=e.toc=nav_ref(e.week,sow,e.toes,0);
    e.ttr=gpst2time(e.week,sow);
    e.svh=getbitu(d,41,1);
    e.sva=gps_ura(getbits(d,42,5),getbits(d,468,5));
    if (e.sva<0) { e.sva=15; e.svh|=8; }
    nav_orbit1(d,58,&e,26559710.0);
    nav_orbit2(d,248,&e,0);
    nav_clock(d,479,&e,0);
    e.tgd[0]=nav_s(d,535,13,-35);
    e.tgd[1]=nav_s(d,548,13,-35);
    e.tgd[2]=nav_s(d,561,13,-35);
    e.fit=3.0;
    if (!nav_valid(&e)) return 0;
    if (ion&&!e.svh&&getbitu(d,617,6)==1) nav_klobuchar(d,721,ion);
    *eph=e;
    return 1;
}
