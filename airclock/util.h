#ifndef UTIL_H
#define UTIL_H

#include <time.h>

#define EPOCH_YR 1970
#define LEAPYEAR(year) (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define YEARSIZE(year) (LEAPYEAR(year) ? 366 : 365)
#define SECS_DAY (24*3600)
#define YEAR0 0

const int _ytab[2][12] = {
                { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 },
                { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }
        };

// Converting Unix Epoch to date and time
// e.g. epochToDateTime(1577808000, 8, &dt) --> dt = 2020-1-1 0:0:0
static void epochToDateTime(unsigned long epoch, int timezone, struct tm *timep)
{
  epoch += 3600 * timezone;   // adjust for timezone
  register unsigned long dayclock, dayno;
  int year = EPOCH_YR;

  dayclock = epoch % SECS_DAY;
  dayno = epoch / SECS_DAY;

  timep->tm_sec = dayclock % 60;
  timep->tm_min = (dayclock % 3600) / 60;
  timep->tm_hour = dayclock / 3600;
  timep->tm_wday = (dayno + 4) % 7;       /* day 0 was a thursday */
  while (dayno >= YEARSIZE(year)) {
          dayno -= YEARSIZE(year);
          year++;
  }
  timep->tm_year = year - YEAR0;
  timep->tm_yday = dayno;
  timep->tm_mon = 0;
  while (dayno >= _ytab[LEAPYEAR(year)][timep->tm_mon]) {
          dayno -= _ytab[LEAPYEAR(year)][timep->tm_mon];
          timep->tm_mon++;
  }
  timep->tm_mday = dayno + 1;
  timep->tm_isdst = 0;
}

#endif
