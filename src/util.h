#pragma once

#include <chrono>
#include <string>
#include <sstream>

inline std::string formatDuration(std::chrono::seconds duration) {
    using namespace std::chrono;

    long long secs = duration.count();

    const int seconds_per_minute = 60;
    const int seconds_per_hour   = 3600;
    const int seconds_per_day    = 86400;
    const int seconds_per_month  = 2592000;  // ~30 days
    const int seconds_per_year   = 31536000; // ~365 days

    long long years   = secs / seconds_per_year; secs %= seconds_per_year;
    long long months  = secs / seconds_per_month; secs %= seconds_per_month;
    long long days    = secs / seconds_per_day; secs %= seconds_per_day;
    long long hours   = secs / seconds_per_hour; secs %= seconds_per_hour;
    long long minutes = secs / seconds_per_minute; secs %= seconds_per_minute;
    long long seconds = secs;

    std::stringstream ss;
    if(years)   ss << years   << " years ";
    if(months)  ss << months  << " months ";
    if(days)    ss << days    << " days ";
    // if(hours)   ss << hours   << " hours ";
    // if(minutes) ss << minutes << " minutes ";
    // if(seconds) ss << seconds << " seconds";

    std::string result = ss.str();
    if(result.empty()) result = "0 seconds";
    return result;
}
