//! The daylight of a world: the local date and hour, the clock rate and the
//! weather of the scene's site, each a setting of `scene_daylight`.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::Error;

/// The seconds of a day.
const SECONDS_PER_DAY: u32 = 86_400;

/// The daylight settings of the scene's site: each the last value a
/// `daylight.set_*` call gave it, the date and the hour from
/// `set_daylight_time`, the rate from `set_daylight_rate` and the weather
/// from `set_weather`. A setting left out is the scene's own.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "DaylightFields")]
pub struct Daylight {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub date: Option<Date>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub time: Option<TimeOfDay>,
    /// Daylight seconds per simulated second.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rate: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weather: Option<Weather>,
}

#[derive(Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a daylight, { date, time, rate, weather }"
)]
struct DaylightFields {
    date: Option<Date>,
    time: Option<TimeOfDay>,
    rate: Option<f64>,
    weather: Option<Weather>,
}

impl TryFrom<DaylightFields> for Daylight {
    type Error = Error;

    fn try_from(fields: DaylightFields) -> Result<Daylight, Error> {
        let daylight = Daylight {
            date: fields.date,
            time: fields.time,
            rate: fields.rate,
            weather: fields.weather,
        };
        if daylight == Daylight::EMPTY {
            return Err(Error::rule(
                "a daylight entry names at least one setting: date, time, rate or weather",
            ));
        }
        Ok(daylight)
    }
}

impl Daylight {
    pub const EMPTY: Daylight = Daylight {
        date: None,
        time: None,
        rate: None,
        weather: None,
    };
}

/// The weather, as `daylight.set_weather` takes it: the share of the sky the
/// clouds cover and the rain rate in mm/h.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "a weather, { cloud_cover, rain_mm_h }"
)]
pub struct Weather {
    pub cloud_cover: f64,
    pub rain_mm_h: f64,
}

/// A day of the Gregorian calendar, written `YYYY-MM-DD`: a year from 1 to
/// 9999, a month from 1 to 12 and a day the month has.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct Date {
    year: i32,
    month: u32,
    day: u32,
}

/// Whether `year` of the Gregorian calendar has a February 29.
fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

/// The days of `month` (1 to 12) of `year`.
fn days_in_month(year: i32, month: u32) -> u32 {
    match month {
        2 if is_leap_year(year) => 29,
        2 => 28,
        4 | 6 | 9 | 11 => 30,
        _ => 31,
    }
}

const MONTH_NAMES: [&str; 12] = [
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December",
];

impl Date {
    /// The day `year`-`month`-`day`. Refuses a year outside 1 to 9999, a
    /// month outside 1 to 12 and a day the month has not.
    pub fn new(year: i32, month: u32, day: u32) -> Result<Date, Error> {
        if !(1..=9999).contains(&year) {
            return Err(Error::rule(format!(
                "the year {year} is outside 1 to 9999, the years a date writes in four digits"
            )));
        }
        if !(1..=12).contains(&month) {
            return Err(Error::rule(format!("the month {month} is outside 1 to 12")));
        }
        let days = days_in_month(year, month);
        if !(1..=days).contains(&day) {
            return Err(Error::rule(format!(
                "{} {year} has {days} days, so it has no day {day}",
                MONTH_NAMES[month as usize - 1]
            )));
        }
        Ok(Date { year, month, day })
    }

    /// Reads `YYYY-MM-DD`: four digits, a hyphen, two digits, a hyphen and
    /// two digits, a day that [`Date::new`] takes.
    pub fn parse(text: &str) -> Result<Date, Error> {
        let malformed = || Error::rule(format!("the date `{text}` must be written YYYY-MM-DD"));
        let parts: Vec<&str> = text.split('-').collect();
        let [year, month, day] = parts.as_slice() else {
            return Err(malformed());
        };
        let widths_match = year.len() == 4 && month.len() == 2 && day.len() == 2;
        let all_digits = [year, month, day]
            .iter()
            .all(|part| part.bytes().all(|byte| byte.is_ascii_digit()));
        if !(widths_match && all_digits) {
            return Err(malformed());
        }
        let number = |part: &str| part.parse::<u32>().map_err(|_| malformed());
        let year = i32::try_from(number(year)?).map_err(|_| malformed())?;
        Date::new(year, number(month)?, number(day)?).map_err(|reason| {
            Error::rule(format!(
                "the date `{text}` is no day of the calendar: {reason}"
            ))
        })
    }

    pub fn year(self) -> i32 {
        self.year
    }

    pub fn month(self) -> u32 {
        self.month
    }

    pub fn day(self) -> u32 {
        self.day
    }
}

impl fmt::Display for Date {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:04}-{:02}-{:02}", self.year, self.month, self.day)
    }
}

impl TryFrom<String> for Date {
    type Error = Error;

    fn try_from(text: String) -> Result<Date, Error> {
        Date::parse(&text)
    }
}

impl From<Date> for String {
    fn from(date: Date) -> String {
        date.to_string()
    }
}

/// A time of the day to the second, on a 24-hour clock: read as `HH:MM` or
/// `HH:MM:SS`, written as `HH:MM:SS`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct TimeOfDay {
    seconds: u32,
}

impl TimeOfDay {
    /// The time `seconds` after midnight. Refuses a whole day or more.
    pub fn from_seconds(seconds: u32) -> Result<TimeOfDay, Error> {
        if seconds >= SECONDS_PER_DAY {
            return Err(Error::rule(format!(
                "a time of the day is under {SECONDS_PER_DAY} seconds after midnight, and {seconds} is not"
            )));
        }
        Ok(TimeOfDay { seconds })
    }

    /// Reads `HH:MM` or `HH:MM:SS`, two digits each: hours from 00 to 23,
    /// minutes and seconds from 00 to 59.
    pub fn parse(text: &str) -> Result<TimeOfDay, Error> {
        let malformed = || {
            Error::rule(format!(
                "the time `{text}` must be written HH:MM or HH:MM:SS, from 00:00 to 23:59:59"
            ))
        };
        let parts: Vec<&str> = text.split(':').collect();
        if !(2..=3).contains(&parts.len()) {
            return Err(malformed());
        }
        let fields = parts
            .iter()
            .map(|part| {
                let two_digits = part.len() == 2 && part.bytes().all(|byte| byte.is_ascii_digit());
                two_digits
                    .then(|| part.parse::<u32>().ok())
                    .flatten()
                    .ok_or_else(malformed)
            })
            .collect::<Result<Vec<u32>, Error>>()?;
        let (hours, minutes, seconds) = (fields[0], fields[1], fields.get(2).copied().unwrap_or(0));
        if hours > 23 || minutes > 59 || seconds > 59 {
            return Err(malformed());
        }
        TimeOfDay::from_seconds(hours * 3600 + minutes * 60 + seconds)
    }

    /// The seconds since midnight.
    pub fn seconds(self) -> u32 {
        self.seconds
    }
}

impl fmt::Display for TimeOfDay {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:02}:{:02}:{:02}",
            self.seconds / 3600,
            self.seconds / 60 % 60,
            self.seconds % 60
        )
    }
}

impl TryFrom<String> for TimeOfDay {
    type Error = Error;

    fn try_from(text: String) -> Result<TimeOfDay, Error> {
        TimeOfDay::parse(&text)
    }
}

impl From<TimeOfDay> for String {
    fn from(time: TimeOfDay) -> String {
        time.to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::tests::refusal_of;

    #[test]
    fn a_daylight_reads_its_settings_and_writes_the_hour_with_its_seconds() {
        let daylight: Daylight = crate::text::from_json5(
            r#"{ date: "2026-11-24", time: "22:00", rate: 0, weather: { cloud_cover: 1, rain_mm_h: 4 } }"#,
        )
        .unwrap();
        assert_eq!(
            daylight,
            Daylight {
                date: Some(Date::new(2026, 11, 24).unwrap()),
                time: Some(TimeOfDay::from_seconds(22 * 3600).unwrap()),
                rate: Some(0.0),
                weather: Some(Weather {
                    cloud_cover: 1.0,
                    rain_mm_h: 4.0
                }),
            }
        );
        let text = crate::text::to_json5(&daylight).unwrap();
        assert!(text.contains("time: \"22:00:00\""), "{text}");
        assert_eq!(
            crate::text::from_json5::<Daylight>(&text).unwrap(),
            daylight
        );
    }

    #[test]
    fn a_daylight_entry_names_at_least_one_setting() {
        assert_eq!(
            refusal_of::<Daylight>("{}"),
            "a daylight entry names at least one setting: date, time, rate or weather"
        );
        let hour_alone: Daylight = crate::text::from_json5(r#"{ time: "06:30" }"#).unwrap();
        assert_eq!(
            (hour_alone.date, hour_alone.rate, hour_alone.weather),
            (None, None, None)
        );
    }

    #[test]
    fn a_date_is_a_day_of_the_calendar_written_yyyy_mm_dd() {
        assert_eq!(Date::parse("2028-02-29").unwrap().to_string(), "2028-02-29");
        assert_eq!(Date::parse("0001-01-01").unwrap().year(), 1);
        for (text, refusal) in [
            (
                "2026-11-5",
                "the date `2026-11-5` must be written YYYY-MM-DD",
            ),
            ("26-11-05", "the date `26-11-05` must be written YYYY-MM-DD"),
            (
                "2026/11/05",
                "the date `2026/11/05` must be written YYYY-MM-DD",
            ),
            (
                "+026-11-05",
                "the date `+026-11-05` must be written YYYY-MM-DD",
            ),
            (
                "2026-02-29",
                "the date `2026-02-29` is no day of the calendar: February 2026 has 28 days, so it has no day 29",
            ),
            (
                "1900-02-29",
                "the date `1900-02-29` is no day of the calendar: February 1900 has 28 days, so it has no day 29",
            ),
            (
                "2026-13-01",
                "the date `2026-13-01` is no day of the calendar: the month 13 is outside 1 to 12",
            ),
            (
                "0000-01-01",
                "the date `0000-01-01` is no day of the calendar: the year 0 is outside 1 to 9999, the years a date writes in four digits",
            ),
        ] {
            assert_eq!(Date::parse(text).unwrap_err().to_string(), refusal);
        }
    }

    #[test]
    fn a_time_reads_hh_mm_or_hh_mm_ss_and_writes_hh_mm_ss() {
        assert_eq!(
            TimeOfDay::parse("06:30").unwrap().seconds(),
            6 * 3600 + 30 * 60
        );
        assert_eq!(
            TimeOfDay::parse("23:59:59").unwrap().to_string(),
            "23:59:59"
        );
        assert_eq!(TimeOfDay::parse("00:00").unwrap().to_string(), "00:00:00");
        for text in [
            "24:00",
            "12:60",
            "12:00:60",
            "6:30",
            "06:30:00:00",
            "06-30",
            "noon",
        ] {
            assert_eq!(
                TimeOfDay::parse(text).unwrap_err().to_string(),
                format!(
                    "the time `{text}` must be written HH:MM or HH:MM:SS, from 00:00 to 23:59:59"
                )
            );
        }
        assert_eq!(
            TimeOfDay::from_seconds(86_400).unwrap_err().to_string(),
            "a time of the day is under 86400 seconds after midnight, and 86400 is not"
        );
    }

    #[test]
    fn a_daylight_refuses_a_setting_that_does_not_read_naming_it() {
        assert_eq!(
            refusal_of::<Daylight>(r#"{ date: "2026-02-30" }"#),
            "date: the date `2026-02-30` is no day of the calendar: February 2026 has 28 days, so it has no day 30"
        );
        assert_eq!(
            refusal_of::<Daylight>(r#"{ weather: { cloud_cover: 1 } }"#),
            "weather: missing field `rain_mm_h`"
        );
        assert_eq!(
            refusal_of::<Daylight>(r#"{ hour: "10:00" }"#),
            "hour: unknown field `hour`, expected one of `date`, `time`, `rate`, `weather`"
        );
    }
}
