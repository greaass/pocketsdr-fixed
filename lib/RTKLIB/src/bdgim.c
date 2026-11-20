/* L1 积木层：实现 BDS-SIS-ICD-B2b-1.0 第 7.7 节的 BDGIM 单频延迟。 */
#include "rtklib.h"
#include "bdgim_coeff.h"
#define SQR(x) ((x)*(x))

/* 用无 Condon-Shortley 相位的递推计算完全归一化球谐函数。 */
static double harmonic(int n, int order, double lat, double lon)
{
    double p=1.0,prev=0.0,x=sin(lat),root=cos(lat),ratio=1.0;
    int j,m=abs(order);
    for (j=1;j<=m;j++) p*=(2*j-1)*root;
    for (j=m+1;j<=n;j++) {
        double next=((2*j-1)*x*p-(j+m-1)*prev)/(j-m);
        prev=p; p=next;
    }
    for (j=n-m+1;j<=n+m;j++) ratio/=j;
    p*=sqrt((2*n+1)*(m?2.0:1.0)*ratio);
    return p*(order<0?sin(m*lon):cos(m*lon));
}

/* 非广播系数按最近奇数整点计算；所有预测常数随代码固定，运行无需联网。 */
static double background(double mjd, double lat, double lon)
{
    static const int degrees[17]={3,3,3,3,3,3,3,4,4,4,4,4,5,5,5,5,5};
    static const int orders[17]={0,1,-1,2,-2,3,-3,0,1,-1,2,-2,0,1,-1,2,-2};
    static const double periods[12]={1,.5,.33,14.6,27,121.6,182.51,365.25,
                                     4028.71,2014.35,1342.90,1007.18};
    double tp=floor(mjd)+(floor((mjd-floor(mjd))*12.0)*2.0+1.0)/24.0;
    double total=0.0;
    int j,k;
    for (j=0;j<17;j++) {
        double beta=bdgim_coeff[0][j];
        for (k=0;k<12;k++) {
            double phase=2.0*PI*tp/periods[k];
            beta+=bdgim_coeff[k*2+1][j]*cos(phase)+bdgim_coeff[k*2+2][j]*sin(phase);
        }
        total+=beta*harmonic(degrees[j],orders[j],lat,lon);
    }
    return total;
}

/* 返回 L1 等效斜延迟，定位器随后按当前观测的真实载频平方进行缩放。 */
extern double bdgim_delay(gtime_t time, const double *ion, const double *pos,
                         const double *azel)
{
    static const double epoch[]={1970,1,1,0,0,0};
    static const int degrees[9]={0,1,1,1,2,2,2,2,2};
    static const int orders[9]={0,0,1,-1,0,1,-1,2,-2};
    const double pole_lat=80.27*D2R,pole_lon=-72.58*D2R;
    const double ratio=6378000.0/6778000.0;
    double psi,lat,lon,mlat,mlon,sun,mjd,vtec,mapping,delta;
    int j;
    if (azel[1]<=0.0) return 0.0;
    psi=PI/2.0-azel[1]-asin(ratio*cos(azel[1]));
    lat=asin(sin(pos[0])*cos(psi)+cos(pos[0])*sin(psi)*cos(azel[0]));
    lon=pos[1]+atan2(sin(psi)*sin(azel[0])*cos(pos[0]),cos(psi)-sin(pos[0])*sin(lat));
    delta=lon-pole_lon;
    mlat=asin(sin(pole_lat)*sin(lat)+cos(pole_lat)*cos(lat)*cos(delta));
    mlon=atan2(cos(lat)*sin(delta),sin(pole_lat)*cos(lat)*cos(delta)-cos(pole_lat)*sin(lat));
    /* RTKLIB 不接受 1970 年以前的日历，Unix 起点的 MJD 固定为 40587。 */
    mjd=40587.0+timediff(gpst2utc(time),epoch2time(epoch))/86400.0;
    sun=PI*(1.0-2.0*(mjd-floor(mjd)))-pole_lon;
    mlon-=atan2(sin(sun),sin(pole_lat)*cos(sun));
    vtec=background(mjd,mlat,mlon);
    for (j=0;j<9;j++) vtec+=ion[j]*harmonic(degrees[j],orders[j],mlat,mlon);
    mapping=1.0/sqrt(1.0-SQR(ratio*cos(azel[1])));
    return 40.28E16/SQR(FREQ1)*mapping*(vtec>0.0?vtec:0.0);
}
