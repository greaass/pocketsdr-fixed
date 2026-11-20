/* L0 公理层：现代导航解析的位读取、时标及共享轨道接口。 */
#ifndef MODERN_NAV_H
#define MODERN_NAV_H
#include "rtklib.h"
double nav_u(const uint8_t *, int, int, int);
double nav_s(const uint8_t *, int, int, int);
int nav_crc(const uint8_t *, int, int);
double nav_wrap(double);
gtime_t nav_ref(int, double, double, int);
void nav_orbit1(const uint8_t *, int, eph_t *, double);
void nav_orbit2(const uint8_t *, int, eph_t *, int);
void nav_clock(const uint8_t *, int, eph_t *, int);
void nav_klobuchar(const uint8_t *, int, double *);
void nav_bdgim(const uint8_t *, int, double *);
int nav_valid(eph_t *);
#endif
