/* L1 积木层：按北斗 B1C/B2a ICD 1.0 解析 B-CNAV1 与 B-CNAV2。 */
#include "modern_nav.h"

/* 两类北斗电文共享轨道块；IODE 必须与 IODC 的低八位一致。 */
static int bds_orbit(const uint8_t *a, int p, const uint8_t *b, int q, eph_t *e)
{
    e->iode=getbitu(a,p,8); e->toes=getbitu(a,p+8,11)*300.0;
    e->flag=getbitu(a,p+19,2);
    if ((e->iodc&255)!=e->iode||(e->flag!=2&&e->flag!=3)) return 0;
    nav_orbit1(a,p+21,e,e->flag==3?27906100.0:42162200.0);
    nav_orbit2(b,q,e,1);
    return 1;
}

/* B-CNAV1 的 SF2、SF3 各自校验，页零没有可用的健康状态。 */
int decode_bds_bcnav1(const uint8_t *d, int sat, eph_t *eph, double *ion)
{
    eph_t e={0};
    int prn,soh,how,page;
    double sow,toc;
    if (satsys(sat,&prn)!=SYS_CMP||prn<6||prn>58||
        getbitu(d,0,6)!=prn||!nav_crc(d,14,600)||!nav_crc(d,614,264)) return 0;
    soh=getbitu(d,6,8); how=getbitu(d,27,8); page=getbitu(d,614,6);
    if (soh>199||how>167||page<1||page>4) return 0;
    sow=how*3600.0+soh*18.0;
    e.sat=sat; e.code=EPH_BDS_BCNAV1; e.week=getbitu(d,14,13);
    e.iodc=getbitu(d,35,10);
    if (!bds_orbit(d,45,d,256,&e)) return 0;
    toc=getbitu(d,478,11)*300.0;
    if (toc>=604800.0) return 0;
    e.toe=nav_ref(e.week,sow,e.toes,1); e.toc=nav_ref(e.week,sow,toc,1);
    e.ttr=bdt2gpst(bdt2time(e.week,sow));
    nav_clock(d,489,&e,1);
    e.tgd[3]=nav_s(d,547,12,-34);
    e.tgd[4]=nav_s(d,559,12,-34);
    e.tgd[2]=nav_s(d,571,12,-34);
    e.svh=getbitu(d,620,2)|getbitu(d,622,1)|getbitu(d,623,1);
    e.fit=2.0;
    if (!nav_valid(&e)) return 0;
    if (ion&&!e.svh&&page==1) nav_bdgim(d,656,ion);
    *eph=e;
    return 1;
}

/* 消息 10、11 必须相邻且按顺序播发，防止更新边界混合两套轨道。 */
int decode_bds_bcnav2(const uint8_t *a, const uint8_t *b, const uint8_t *c,
                     int sat, eph_t *eph, double *ion)
{
    eph_t e={0};
    int prn,i;
    double sow[3],toc;
    const uint8_t *frames[3];
    const int types[]={10,11,30};
    frames[0]=a; frames[1]=b; frames[2]=c;
    if (satsys(sat,&prn)!=SYS_CMP||prn<6||prn>58) return 0;
    for (i=0;i<3;i++) {
        if (!nav_crc(frames[i],0,288)||getbitu(frames[i],0,6)!=prn||
            getbitu(frames[i],6,6)!=types[i]) return 0;
        sow[i]=getbitu(frames[i],12,18)*3.0;
        if (sow[i]>=604800.0) return 0;
    }
    if (nav_wrap(sow[1]-sow[0])!=3.0||fabs(nav_wrap(sow[2]-sow[0]))>120.0) return 0;
    e.sat=sat; e.code=EPH_BDS_BCNAV2; e.week=getbitu(a,30,13);
    e.iodc=getbitu(c,111,10);
    if (!bds_orbit(a,53,b,42,&e)) return 0;
    toc=getbitu(c,42,11)*300.0;
    if (toc>=604800.0) return 0;
    e.toe=nav_ref(e.week,sow[0],e.toes,1); e.toc=nav_ref(e.week,sow[0],toc,1);
    e.ttr=bdt2gpst(bdt2time(e.week,sow[0]));
    nav_clock(c,53,&e,1);
    e.tgd[3]=nav_s(c,121,12,-34); e.tgd[5]=nav_s(c,133,12,-34);
    e.tgd[2]=nav_s(c,219,12,-34);
    e.svh=getbitu(a,43,1)|getbitu(a,44,1)|getbitu(b,30,4)|getbitu(c,30,4);
    e.fit=2.0;
    if (!nav_valid(&e)) return 0;
    if (ion&&!e.svh) nav_bdgim(c,145,ion);
    *eph=e;
    return 1;
}
