/* L2 流程层：按通道缓存 B-CNAV3 两类消息，向本接收机会话提交完整星历。 */
#include "pocket_sdr.h"

/* 只处理已通过纠错与 CRC 的开放服务消息，PPP-B2b 不生成自主星历。 */
int sdr_bcnv3_update(sdr_ch_t *ch, nav_t *nav, eph_t *eph)
{
    sdr_nav_t *state=ch->nav;
    int type=state->type, slot=type==10?0:1;
    if (ch->prn<6||ch->prn>58||(type!=10&&type!=30)) return 0;
    memcpy(state->data+64+slot*64,state->data,61);
    state->lock_sf[slot]=ch->lock;
    if (!state->lock_sf[0]||!state->lock_sf[1]) return 0;
    if (fabs((state->lock_sf[0]-state->lock_sf[1])*ch->T)>120.0) return 0;
    if (!decode_bds_bcnv3(state->data+64,state->data+128,
                          satid2no(ch->sat),eph)) {
        sdr_log(3,"$LOG,%.3f,%s,B2BI,BCNAV3 REJECT TIME OR DATA",ch->time,ch->sat);
        return 0;
    }
    if (type==30&&eph->svh==0) {
        decode_bds_bdgim(state->data+128,nav->ion_bdgim);
        nav->bdgim_time=eph->ttr;
    }
    if (eph->svh) {
        sdr_log(3,"$LOG,%.3f,%s,B2BI,BCNAV3 UNHEALTHY",ch->time,ch->sat);
    }
    return 1;
}
