#include "myTime.h"

void clock_Init(void) {
	_RTC = RTC;
	_PWR = PWR;
}

void getCurrentTime(Time *time) {
	uint32_t RTC_TR = _RTC->TR;

	uint8_t secBCD = (RTC_TR & (RTC_TR_ST_Msk | RTC_TR_SU_Msk)) >> RTC_TR_SU_Pos;
	time->sec.MSD = (RTC_TR & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos;
	time->sec.LSD = (RTC_TR & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos;
	time->sec.BIN = BCDtoBinary(secBCD);

	uint8_t minBCD = (RTC_TR & (RTC_TR_MNT_Msk | RTC_TR_MNU_Msk)) >> RTC_TR_MNU_Pos;
	time->min.MSD = (RTC_TR & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos;
	time->min.LSD = (RTC_TR & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos;
	time->min.BIN = BCDtoBinary(minBCD);

	uint8_t houBCD = (RTC_TR & (RTC_TR_HT_Msk | RTC_TR_HU_Msk)) >> RTC_TR_HU_Pos;
	time->hou.MSD = (RTC_TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos;
	time->hou.LSD = (RTC_TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos;
	time->hou.BIN = BCDtoBinary(houBCD);

	uint8_t format = (RTC_TR & (RTC_TR_PM_Msk)) >> RTC_TR_PM_Pos;
	if (format) {
		time->format = AM;
	} else {
		time->format = PM24;
	}
}

void setCurrentTime(Time *time) {
	RTC_TimeTypeDef tm;
	tm.Hours = time->hou.BIN;
	tm.Minutes = time->min.BIN;
	tm.Seconds = time->sec.BIN;
	tm.TimeFormat = (time->format == AM) ? RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM;
	tm.SubSeconds = 0;
	tm.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	tm.StoreOperation = RTC_STOREOPERATION_RESET;

	HAL_RTC_SetTime(&hrtc, &tm, RTC_FORMAT_BIN);
}

void getCurrentDate(Date *date) {
	uint32_t RTC_DR = _RTC->DR;

	uint8_t yeaBCD = (RTC_DR & (RTC_DR_YT_Msk | RTC_DR_YU_Msk)) >> RTC_DR_YU_Pos;
	date->yea.MSD = (RTC_DR & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos;
	date->yea.LSD = (RTC_DR & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos;
	date->yea.BIN = BCDtoBinary(yeaBCD);

	uint8_t monBCD = (RTC_DR & (RTC_DR_MT_Msk | RTC_DR_MU_Msk)) >> RTC_DR_MU_Pos;
	date->mon.MSD = (RTC_DR & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos;
	date->mon.LSD = (RTC_DR & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos;
	date->mon.BIN = BCDtoBinary(monBCD);

	uint8_t dayBCD = (RTC_DR & (RTC_DR_DT_Msk | RTC_DR_DU_Msk)) >> RTC_DR_DU_Pos;
	date->day.MSD = (RTC_DR & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos;
	date->day.LSD = (RTC_DR & RTC_DR_DU_Msk) >> RTC_DR_DU_Pos;
	date->day.BIN = BCDtoBinary(dayBCD);

	uint8_t dayf = (RTC_DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos;
	if (dayf == 0) {
		date->dayName = Undef;
	} else if (dayf > 7) {
		date->dayName = Undef;
	} else {
		date->dayName = dayf - 1;
	}
}

void setCurrentDate(Date *date) {
	RTC_DateTypeDef dm;
	dm.WeekDay = date->dayName + 1;
	dm.Month = date->mon.BIN;
	dm.Date = date->day.BIN;
	dm.Year = date->yea.BIN;

	HAL_RTC_SetDate(&hrtc, &dm, RTC_FORMAT_BIN);
}

uint8_t BCDtoBinary(uint8_t value) {
	uint8_t MS4B = (value & 0xF0) >> 4U;
	uint8_t LS4B = (value & 0x0F) >> 0U;
	uint8_t ret = MS4B * 10 + LS4B;
	return ret;
}

uint8_t BinarytoBCD(uint8_t value) {
	uint8_t LSB = value % 10;
	uint8_t MSB = (value - LSB) /10;
	uint8_t ret = MSB<<4U | LSB<<0U;
	return ret;
}

Time CreateTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
	Time ret;
	uint8_t BCDhou = BinarytoBCD(hours);
	ret.hou.BIN = hours;
	ret.hou.MSD = (BCDhou & 0xF0) >> 4U;
	ret.hou.LSD = (BCDhou & 0x0F) >> 0U;

	uint8_t BCDmin = BinarytoBCD(minutes);
	ret.min.BIN = minutes;
	ret.min.MSD = (BCDmin & 0xF0) >> 4U;
	ret.min.LSD = (BCDmin & 0x0F) >> 0U;

	uint8_t BCDsec = BinarytoBCD(seconds);
	ret.sec.BIN = seconds;
	ret.sec.MSD = (BCDsec & 0xF0) >> 4U;
	ret.sec.LSD = (BCDsec & 0x0F) >> 0U;

	ret.format = PM24;
	return ret;
}

Date CreateDate(uint8_t day, uint8_t month, uint16_t year) {
	Date ret;
	uint8_t BCDday = BinarytoBCD(day);
	ret.day.BIN = day;
	ret.day.MSD = (BCDday & 0xF0) >> 4U;
	ret.day.LSD = (BCDday & 0x0F) >> 0U;

	uint8_t BCDmon = BinarytoBCD(month);
	ret.mon.BIN = month;
	ret.mon.MSD = (BCDmon & 0xF0) >> 4U;
	ret.mon.LSD = (BCDmon & 0x0F) >> 0U;

	uint8_t yearF = year%100;
	uint8_t BCDyea = BinarytoBCD(yearF);
	ret.yea.BIN = yearF;
	ret.yea.MSD = (BCDyea & 0xF0) >> 4U;
	ret.yea.LSD = (BCDyea & 0x0F) >> 0U;

	ret.dayName = Mon;
	return ret;
}
