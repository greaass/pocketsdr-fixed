/* L3 外交层：验证协议边界、跨周、观测槽与星历来源隔离。 */
#include "rtklib.h"
#include <assert.h>

/* 测试帧以字节左对齐 CRC 算法编码，与生产端逐位实现相互核对。 */
static void checksum(uint8_t *data)
{
    uint8_t bytes[61]={0};
    int i;
    for (i=0;i<462;i++) setbitu(bytes,i+2,1,getbitu(data,i,1));
    setbitu(data,462,24,rtk_crc24q(bytes,58));
}

/* 构造可控轨道与钟差，不依赖真实电文中恰好出现所有边界字段。 */
static void fixture(uint8_t *orbit, uint8_t *clock)
{
    memset(orbit,0,61); memset(clock,0,61);
    setbitu(orbit,0,6,10); setbitu(clock,0,6,30);
    setbitu(orbit,6,20,10000); setbitu(clock,6,20,10001);
    setbitu(orbit,30,11,33); setbitu(clock,43,11,33);
    setbitu(orbit,41,2,3); setbitu(clock,26,13,1066);
    setbitu(orbit,134,1,1); setbitu(orbit,135,32,0x80000000u);
    setbitu(clock,124,10,400); setbits(clock,134,8,-8);
    setbitu(clock,142,8,20); setbitu(clock,150,8,30);
    setbitu(clock,158,8,40); setbits(clock,166,8,-16);
    checksum(orbit); checksum(clock);
}

/* 必须拒绝坏 CRC、过期和缺帧；允许电文自身标记非健康，但不能伪装健康。 */
static void protocol_tests(void)
{
    uint8_t orbit[61],clock[61],bad[61];
    eph_t e;
    double ion[9];
    int sat=satno(SYS_CMP,24);
    fixture(orbit,clock);
    assert(decode_bds_bcnv3(orbit,clock,sat,&e));
    assert(e.iode==-1&&e.iodc==-1&&fabs(e.M0+PI/2)<1E-12);
    decode_bds_bdgim(clock,ion);
    assert(ion[0]==50&&ion[1]==-1&&ion[2]==2.5&&ion[3]==3.75);
    assert(ion[4]==-5&&ion[5]==-2);
    assert(!decode_bds_bcnv3(NULL,clock,sat,&e));
    assert(!decode_bds_bcnv3(orbit,clock,satno(SYS_CMP,1),&e));
    memcpy(bad,orbit,61); bad[10]^=1;
    assert(!decode_bds_bcnv3(bad,clock,sat,&e));
    setbitu(clock,6,20,10122); checksum(clock);
    assert(!decode_bds_bcnv3(orbit,clock,sat,&e));
    fixture(orbit,clock); setbitu(orbit,30,11,0); checksum(orbit);
    assert(!decode_bds_bcnv3(orbit,clock,sat,&e));
    fixture(orbit,clock); setbitu(clock,460,2,1); checksum(clock);
    assert(decode_bds_bcnv3(orbit,clock,sat,&e)&&e.svh);
    fixture(orbit,clock); setbitu(orbit,41,2,1); checksum(orbit);
    assert(!decode_bds_bcnv3(orbit,clock,sat,&e));
    fixture(orbit,clock);
    setbitu(orbit,6,20,604790); setbitu(clock,6,20,10);
    setbitu(clock,26,13,1067); setbitu(orbit,30,11,2015);
    setbitu(clock,43,11,0); checksum(orbit); checksum(clock);
    assert(decode_bds_bcnv3(orbit,clock,sat,&e));
    assert(fabs(timediff(e.toc,e.toe)-300)<1E-9);
    assert(fabs(timediff(e.ttr,e.toe)-310)<1E-9);
}

/* 相同卫星同时有旧与现代星历时，按观测码隔离；原始频率槽不受修改。 */
static void observation_tests(void)
{
    nav_t nav={0},*selected=NULL;
    eph_t e[2]={{0}};
    obsd_t obs={0},copy;
    double range;
    int codes[3]={CODE_L7I,CODE_L6I,CODE_L7D},k;
    obs.sat=e[0].sat=e[1].sat=satno(SYS_CMP,24);
    obs.time=e[0].toe=e[1].toe=bdt2gpst(bdt2time(1066,10000));
    e[0].toc=e[1].toc=obs.time;
    e[0].tgd[1]=1E-8; e[1].tgd[1]=-2E-8;
    e[1].code=EPH_BDS_BCNAV3; nav.n=2; nav.eph=e;
    for (k=0;k<3;k++) {
        memset(obs.P,0,sizeof(obs.P)); memset(obs.code,0,sizeof(obs.code));
        obs.P[4]=25000000; obs.code[4]=(uint8_t)codes[k]; obs.D[4]=123;
        assert(pnt_obs_slot(&obs)==4);
        pnt_select_obs(&copy,&obs,1);
        assert(copy.P[0]==obs.P[4]&&copy.D[0]==123&&copy.code[0]==codes[k]);
        assert(obs.P[0]==0&&obs.code[0]==0);
        assert(pnt_bds_nav(&copy,1,&nav,&selected)&&selected);
        assert(selected->eph[k==2?0:1].sat==0);
        assert(pnt_bds_range(&copy,selected,&range));
        assert(fabs(range-(25000000-CLIGHT*(k==0?1E-8:k==2?-2E-8:0)))<1E-7);
        free(selected->eph); free(selected);
    }
    nav.n=1; copy.code[0]=CODE_L7D;
    assert(!pnt_bds_range(&copy,&nav,&range));
    copy.time=timeadd(copy.time,7201); nav.n=2;
    assert(pnt_bds_nav(&copy,1,&nav,&selected));
    assert(!pnt_bds_range(&copy,selected,&range));
    free(selected->eph); free(selected);
}

/* 以独立科学库数值核对所需的电离层样本覆盖高纬、低仰角和日期分界。 */
static void ionosphere_samples(void)
{
    double ion[]={50,-1,2.5,3.75,-5,-2,.5,-.25,.125};
    double ep[]={2026,6,8,10,17,20},pos[3],azel[2];
    int j;
    for (j=0;j<6;j++) {
        pos[0]=(-80+j*30)*D2R; pos[1]=(-170+j*65)*D2R; pos[2]=100;
        azel[0]=(j*67)*D2R; azel[1]=(5+j*15)*D2R;
        printf("ION,%d,%.15g\n",j,bdgim_delay(utc2gpst(epoch2time(ep)),ion,pos,azel));
    }
}

/* 断言失败直接终止，不能以仅编译成功代替协议和适配验收。 */
int main(void)
{
    protocol_tests(); observation_tests(); ionosphere_samples();
    puts("PASS protocol and observation boundaries"); return 0;
}
