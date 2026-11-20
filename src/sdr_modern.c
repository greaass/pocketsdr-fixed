/* L2 流程层：隔离各通道的现代电文组帧状态，向定位器提交完整星历。 */
#include "pocket_sdr.h"

/* 缓存位于原始帧之后，失锁时既有导航复位会清除帧锁定计数。 */
static int cache_frames(sdr_ch_t *ch, int bytes, double age)
{
    int i,type=ch->nav->type,slot=type==10?0:type==11?1:type==30?2:-1;
    sdr_nav_t *n=ch->nav;
    if (slot<0) return 0;
    memcpy(n->data+128+64*slot,n->data,bytes);
    n->lock_sf[slot]=ch->lock;
    for (i=0;i<3;i++) {
        if (!n->lock_sf[i]||fabs((ch->lock-n->lock_sf[i])*ch->T)>age) return 0;
    }
    return 1;
}

/* 导频不携带星历；这里只接受具备 CRC 校验电文的数据分量。 */
int sdr_modern_update(sdr_ch_t *ch, nav_t *nav, eph_t *eph)
{
    int ok=0,sat=satid2no(ch->sat);
    uint8_t *d=ch->nav->data;
    if (!strcmp(ch->sig,"L2CM")||!strcmp(ch->sig,"L5I")) {
        if (!cache_frames(ch,38,600.0)) return 0;
        ok=decode_gps_cnav(d+128,d+192,d+256,sat,eph,nav->ion_gps);
    }
    else if (!strcmp(ch->sig,"L1CD")) {
        ok=decode_gps_cnav2(d,sat,eph,nav->ion_gps);
    }
    else if (!strcmp(ch->sig,"B1CD")) {
        ok=decode_bds_bcnav1(d,sat,eph,nav->ion_bdgim);
        if (ok&&ch->nav->type==1&&!eph->svh) nav->bdgim_time=eph->ttr;
    }
    else if (!strcmp(ch->sig,"B2AD")) {
        if (!cache_frames(ch,36,120.0)) return 0;
        ok=decode_bds_bcnav2(d+128,d+192,d+256,sat,eph,nav->ion_bdgim);
        if (ok&&!eph->svh) nav->bdgim_time=eph->ttr;
    }
    return ok;
}
