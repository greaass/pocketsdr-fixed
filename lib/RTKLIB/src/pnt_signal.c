/* L1 积木层：选择有效单频观测、匹配导航来源并执行对应码偏差改正。 */
#include "rtklib.h"

/* 现代信号码必须匹配具有该信号偏差的电文来源。 */
static int source_code(int sys, int code)
{
    if (sys==SYS_CMP) {
        if (code==CODE_L7D) return EPH_BDS_BCNAV3;
        if (code==CODE_L1D||code==CODE_L1P) return EPH_BDS_BCNAV1;
        if (code==CODE_L5D||code==CODE_L5P) return EPH_BDS_BCNAV2;
    }
    if (sys==SYS_GPS) {
        if (code==CODE_L2S||code==CODE_L2L||code==CODE_L5I||code==CODE_L5Q) return EPH_GPS_CNAV;
        if (code==CODE_L1S||code==CODE_L1L) return EPH_GPS_CNAV2;
    }
    return 0;
}

/* 第一槽优先，补充当前已实现偏差改正的 GPS 与北斗非首频段。 */
int pnt_obs_slot(const obsd_t *obs)
{
    int i,sys=satsys(obs->sat,NULL);
    if (obs->P[0]>0.0&&obs->code[0]) return 0;
    for (i=1;i<NFREQ+NEXOBS;i++) {
        if (obs->P[i]<=0.0) continue;
        if (source_code(sys,obs->code[i])||(sys==SYS_CMP&&
            (obs->code[i]==CODE_L7I||obs->code[i]==CODE_L6I))) return i;
    }
    return -1;
}

/* 仅调整求解副本，原始观测输出仍保留真实信号码和频率槽。 */
void pnt_select_obs(obsd_t *target, const obsd_t *source, int n)
{
    int i,j;
    memcpy(target,source,sizeof(obsd_t)*n);
    for (i=0;i<n;i++) {
        if ((j=pnt_obs_slot(source+i))<=0) continue;
        target[i].P[0]=source[i].P[j]; target[i].L[0]=source[i].L[j];
        target[i].D[0]=source[i].D[j]; target[i].SNR[0]=source[i].SNR[j];
        target[i].LLI[0]=source[i].LLI[j]; target[i].code[0]=source[i].code[j];
    }
}

/* 保持既有公共接口名；现在同时为 GPS 与北斗筛选独立导航来源。 */
int pnt_bds_nav(const obsd_t *obs, int n, const nav_t *nav, nav_t **output)
{
    nav_t *selected;
    int i,j,modern=0;
    for (j=0;j<nav->n;j++) if (IS_MODERN_EPH(nav->eph+j)) modern=1;
    *output=NULL;
    if (!modern) return 1;
    if (!(selected=(nav_t *)malloc(sizeof(nav_t)))) return 0;
    *selected=*nav;
    selected->eph=(eph_t *)malloc(sizeof(eph_t)*nav->n);
    if (!selected->eph) { free(selected); return 0; }
    memcpy(selected->eph,nav->eph,sizeof(eph_t)*nav->n);
    for (i=0;i<n;i++) {
        int sys=satsys(obs[i].sat,NULL),source=source_code(sys,obs[i].code[0]);
        if (sys!=SYS_CMP&&sys!=SYS_GPS) continue;
        for (j=0;j<nav->n;j++) {
            eph_t *e=selected->eph+j;
            if (e->sat!=obs[i].sat) continue;
            if (source?(e->code!=source):IS_MODERN_EPH(e)) e->sat=0;
            if (e->sat&&e->code==EPH_GPS_CNAV) {
                e->svh=(e->svh&8)|(obs[i].code[0]==CODE_L2S||obs[i].code[0]==CODE_L2L?
                    ((e->svh>>1)&1):(e->svh&1));
            }
        }
    }
    *output=selected;
    return 1;
}

/* 偏差必须来自定位器已经筛选的同一星历来源，且遵守现代参考时效。 */
static const eph_t *range_eph(const obsd_t *obs, const nav_t *nav)
{
    const eph_t *e=NULL;
    double age=1E99;
    int j;
    for (j=0;j<nav->n;j++) {
        const eph_t *candidate=nav->eph+j;
        double dt=fabs(timediff(obs->time,candidate->toe));
        double limit=satsys(obs->sat,NULL)==SYS_GPS&&IS_MODERN_EPH(candidate)?5400.0:7200.0;
        if (candidate->sat!=obs->sat) continue;
        if (IS_MODERN_EPH(candidate)&&(dt>limit||fabs(timediff(obs->time,candidate->toc))>limit)) continue;
        if (dt<=7200.0&&dt<=age) { age=dt; e=candidate; }
    }
    return e;
}

/* 北斗的广播钟差以 B3I 为基准，数据分量还需叠加相应 ISC。 */
int pnt_bds_range(const obsd_t *obs, const nav_t *nav, double *range)
{
    const eph_t *e=range_eph(obs,nav);
    double bias=0.0;
    int code=obs->code[0];
    if (!e) return 0;
    if (code==CODE_L2I) bias=e->tgd[0];
    else if (code==CODE_L7I||code==CODE_L7D) bias=e->tgd[1];
    else if (code==CODE_L6I) bias=0.0;
    else if (code==CODE_L1P) bias=e->tgd[2];
    else if (code==CODE_L1D) bias=e->tgd[2]+e->tgd[4];
    else if (code==CODE_L5P) bias=e->tgd[3];
    else if (code==CODE_L5D) bias=e->tgd[3]+e->tgd[5];
    else return 0;
    if (source_code(SYS_CMP,code)&&e->code!=source_code(SYS_CMP,code)) return 0;
    *range=obs->P[0]-CLIGHT*bias;
    return *range>0.0;
}

/* GPS 民码 ISC 已以 L1 P(Y) 为基准，单频改正为负 TGD 加 ISC。 */
int pnt_gps_range(const obsd_t *obs, const nav_t *nav, double *range)
{
    const eph_t *e=range_eph(obs,nav);
    double isc=0.0;
    int code=obs->code[0],source=source_code(SYS_GPS,code);
    if (!e||(source&&e->code!=source)) return 0;
    if (e->code==EPH_GPS_CNAV) {
        if (code==CODE_L2S||code==CODE_L2L) isc=e->tgd[2];
        else if (code==CODE_L5I) isc=e->tgd[3];
        else if (code==CODE_L5Q) isc=e->tgd[4];
        else return 0;
    }
    else if (e->code==EPH_GPS_CNAV2) {
        if (code==CODE_L1L) isc=e->tgd[1];
        else if (code==CODE_L1S) isc=e->tgd[2];
        else return 0;
    }
    *range=obs->P[0]+(!source&&code==CODE_L1C?nav->cbias[obs->sat-1][1]:0.0)-CLIGHT*(e->tgd[0]-isc);
    return *range>0.0;
}
