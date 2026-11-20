/* L3 外交层：以含错误导航来源的合成观测验证真实单频求解器。 */
#include "rtklib.h"
#include <assert.h>

/* 每颗卫星使用不同非零偏差，避免被共同接收机钟差吸收。 */
static double bias(const eph_t *e, int code)
{
    if (e->code==EPH_GPS_CNAV) {
        return e->tgd[0]-e->tgd[code==CODE_L2S?2:code==CODE_L5I?3:4];
    }
    if (e->code==EPH_GPS_CNAV2) return e->tgd[0]-e->tgd[code==CODE_L1S?2:1];
    if (code==CODE_L1D) return e->tgd[2]+e->tgd[4];
    if (code==CODE_L1P) return e->tgd[2];
    if (code==CODE_L5D) return e->tgd[3]+e->tgd[5];
    if (code==CODE_L5P) return e->tgd[3];
    if (code==CODE_L6I) return 0.0;
    if (code==CODE_L2I) return e->tgd[0];
    if (code==CODE_L7I||code==CODE_L7D) return e->tgd[1];
    return e->tgd[0];
}

/* 用完整发射时刻和地球自转生成观测，求解端只接收码量和广播星历。 */
static void scenario(int sys, int code, int source)
{
    nav_t nav={0}; eph_t good[8]={{0}},eph[16]={{0}};
    obsd_t obs[8]={{0}}; prcopt_t opt=prcopt_default; sol_t sol={0};
    double receiver[]={-2763924.0,4775027.0,3196180.0},rs[3],clk,var,los[3];
    double error[3]; char msg[128]=""; int i,k,slot=code2idx(sys,code),sv;
    gtime_t time=gpst2time(2200,300000.0);
    assert(slot>=0&&slot<NFREQ+NEXOBS);
    nav.n=16; nav.eph=eph;
    for (i=0;i<8;i++) {
        eph_t *e=good+i;
        e->sat=satno(sys,sys==SYS_CMP?19+i:1+i); e->code=source;
        e->toe=e->toc=time; e->ttr=time; e->toes=300000.0-(sys==SYS_CMP?14.0:0.0);
        e->A=2.7E7; e->e=0.01; e->i0=0.96;
        e->OMG0=i*0.8; e->M0=i*1.5; e->omg=i*0.4; e->OMGd=-8E-9;
        e->f0=(i-3)*2E-5; e->flag=3;
        if (source) { e->Adot=-0.02; e->ndot=2E-13; }
        for (k=0;k<6;k++) e->tgd[k]=(i+1)*(k-2)*1E-8;
        eph[i]=*e; eph[i+8]=*e;
        eph[i].code=source?0:(sys==SYS_CMP?EPH_BDS_BCNAV3:EPH_GPS_CNAV);
        eph[i].f0+=1E-3;
        nav.cbias[e->sat-1][1]=i*0.3;
        obs[i].sat=e->sat; obs[i].time=time; obs[i].code[slot]=code;
        obs[i].SNR[slot]=45000; obs[i].P[slot]=2E7;
        for (k=0;k<6;k++) {
            double p=obs[i].P[slot];
            eph2pos(timeadd(time,-p/CLIGHT),e,rs,&clk,&var);
            obs[i].P[slot]=geodist(rs,receiver,los)-CLIGHT*clk+CLIGHT*bias(e,code);
            if (!source&&sys==SYS_GPS) obs[i].P[slot]-=nav.cbias[e->sat-1][1];
        }
    }
    opt.navsys=sys; opt.ionoopt=IONOOPT_OFF; opt.tropopt=TROPOPT_OFF;
    opt.elmin=-PI/2.0; opt.maxgdop=100.0; opt.posopt[4]=0;
    for (i=0;i<3;i++) sol.rr[i]=receiver[i]+100.0;
    sv=pntpos(obs,8,&nav,&opt,&sol,NULL,NULL,msg);
    if (!sv) printf("FAIL source=%d code=%d %s\n",source,code,msg);
    assert(sv);
    for (i=0;i<3;i++) error[i]=sol.rr[i]-receiver[i];
    printf("SPP source=%d code=%d error=%.6fm ns=%d\n",source,code,norm(error,3),sol.ns);
    assert(norm(error,3)<0.05);
}

/* 覆盖数据、导频观测，以及现代星历并存时的传统信号回归。 */
int main(void)
{
    scenario(SYS_GPS,CODE_L2S,EPH_GPS_CNAV);
    scenario(SYS_GPS,CODE_L5I,EPH_GPS_CNAV);
    scenario(SYS_GPS,CODE_L5Q,EPH_GPS_CNAV);
    scenario(SYS_GPS,CODE_L1S,EPH_GPS_CNAV2);
    scenario(SYS_GPS,CODE_L1L,EPH_GPS_CNAV2);
    scenario(SYS_CMP,CODE_L1D,EPH_BDS_BCNAV1);
    scenario(SYS_CMP,CODE_L1P,EPH_BDS_BCNAV1);
    scenario(SYS_CMP,CODE_L5D,EPH_BDS_BCNAV2);
    scenario(SYS_CMP,CODE_L5P,EPH_BDS_BCNAV2);
    scenario(SYS_CMP,CODE_L7D,EPH_BDS_BCNAV3);
    scenario(SYS_GPS,CODE_L1C,0);
    scenario(SYS_CMP,CODE_L7I,0);
    scenario(SYS_CMP,CODE_L6I,0);
    puts("PASS: source isolation, nonzero biases, true frequency slots, legacy regression");
    return 0;
}
