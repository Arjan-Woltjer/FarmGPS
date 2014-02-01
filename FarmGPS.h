/*
  FarmGPS - a small GPS library for Arduino providing basic NMEA parsing.
Based on work by Maarten Lamers and Mikal Hart.
Copyright (C) 2011-2013 J.A. Woltjer.
All rights reserved.
 
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FarmGPS_h
#define FarmGPS_h

#if ARDUINO <= 22
#include "WProgram.h"
#else
#include "Arduino.h"
#endif

// software version of this library
#define GPS_VERSION 0.7

// conversion constants
#define GPS_MPH_PER_KNOT 1.15077945
#define GPS_MS_PER_KNOT 0.51444444
#define GPS_KMH_PER_KNOT 1.852
#define GPS_MILES_PER_METER 0.00062137112
#define GPS_KM_PER_METER 0.001

#define GPGGA_TERM   "GPGGA"
#define GPVTG_TERM   "GPVTG"
#define GPXTE_TERM   "GPXTE"
#define ROXTE_TERM   "ROXTE"

#define GPS_INVALID_FLOAT 999999.9
#define GPS_INVALID_LONG 0xFFFFFFFF

#define GPS_NO_STATS

class FarmGPS {
private:
  //-------------
  // data members
  //-------------

  // nmea items
  float time, new_time;
  unsigned long date, new_date;
  float latitude, new_latitude;
  float longitude, new_longitude;
  float altitude, new_altitude;
  float speed, new_speed;
  float course, new_course;
  float xte, new_xte;
  int quality, new_quality;

  // timekeepers
  unsigned long last_GGA_fix, new_GGA_fix;
  unsigned long last_VTG_fix, new_VTG_fix;
  unsigned long last_XTE_fix, new_XTE_fix;

  // flags for usage monitoring
  boolean new_GGA_data;
  boolean new_VTG_data;
  boolean new_XTE_data;

  // parsing state variables
  char term[20];
  byte term_number;
  byte term_offset;
  byte parity;
  byte checksum;
  int sum;
  bool is_checksum_term;

  // sentence type of decoded message
  enum types{
    GGA, VTG, XTE, XTE2, OTHER
  };
  types sentence_type;

#ifndef _GPS_NO_STATS
  // statistics
  unsigned long encoded_characters;
  unsigned long good_sentences;
  unsigned long failed_checksum;
  unsigned long passed_checksum;
#endif
  //----------------------------------------------------
  // private member functions implemented in FarmGPS.cpp
  //----------------------------------------------------

  // Convert string to decimal or degrees
  float parse_decimal(const char *s);
  float parse_degrees(const char *s);
  int parse_integer(const char *s);
  
  // Compares two strings, returns true when the same
  bool strcmp(const char *str1, const char *str2);

  // Convert ascii hexadecimal to integer
  int hex_to_int(char c);
  
  // Checks whether nmea term is a complete term
  bool parse_term();


public:
  //--------------------------------------------------
  //public member functions implemented in FarmGPS.cpp
  //--------------------------------------------------

  //Constructor
  FarmGPS();

  // Processes characters received from GPS
  bool decode(char c);

  // Calculates distance between two geographical points
  static float distance_between(float lat1, float long1, float lat2, float long2);

  // Provides statistics
#ifndef GPS_NO_STATS
  void stats(unsigned long *chars, unsigned short *sentences, unsigned short *failed_cs);
#endif

  //------------------------------
  //public inline member functions
  //------------------------------
  
  //in general all getters for this class

  // date as ddmmyy, time as hhmmsscc, and age in milliseconds
  inline void get_datetime(unsigned long *outdate, unsigned long *outtime) {
    if (outdate) *outdate = date;
    if (outtime) *outtime = time;
  }

  // date as dd, mm, yyyy, time as hh, mm, ss, cc, and age in milliseconds
  inline void get_datetime_details(int *outyear, byte *outmonth, byte *outday,
  byte *outhour, byte *outminute, byte *outsecond, byte *outhundredths = 0) {
    unsigned long _d, _t;
    get_datetime(&_d, &_t);
    if (outyear) {
      *outyear = _d % 100;
      *outyear += *outyear > 80 ? 1900 : 2000;
    }
    if (outmonth) *outmonth = (_d / 100) % 100;
    if (outday) *outday = _d / 10000;
    if (outhour) *outhour = _t / 1000000;
    if (outminute) *outminute = (_t / 10000) % 100;
    if (outsecond) *outsecond = (_t / 100) % 100;
    if (outhundredths) *outhundredths = _t % 100;
  }

  // lat/long in degrees and age of fix in milliseconds
  inline void get_position(float *outlatitude, float *outlongitude) {
    if (outlatitude) *outlatitude = latitude;
    if (outlongitude) *outlongitude = longitude;
  }

  // altitude in last full GPGGA sentence in centimeters
  inline float get_altitude() {
    return altitude;
  }

  // quality of the GPS data from GGA string
  inline int get_quality() {
    return quality;
  }

  // course in last full GPVTG sentence in degrees
  inline float get_course() {
    return course;
  }

  // speed in last full GPVTG sentence in knots
  inline float get_speed() {
    return speed;
  }

  // xte in last full GPXTE sentence in meters
  inline float get_xte() {
    return xte;
  }

  //-------------------
  //special conversions
  //-------------------

  // altitude in centimeters
  inline int get_altitude_cm(){
    return int(altitude * 100);
  }

  // speed in miles per hour
  inline float get_speed_mph() {
    return GPS_MPH_PER_KNOT * speed;
  }

  // speed in meters per second
  inline float get_speed_ms() {
    return GPS_MS_PER_KNOT * speed;
  }

  // speed in kilometers per hour
  inline float get_speed_kmh() {
    return GPS_KMH_PER_KNOT * speed;
  }

  // cross track error in centimeters
  inline int get_xte_cm() {
    return xte * 100;
  }
  
  //-----------
  //age & usage
  //-----------
  
  //returns age of sentence
  inline unsigned long get_GGA_fix_age(){
    return last_GGA_fix;
  }

  //returns age of sentence
  inline unsigned long get_VTG_fix_age(){
    return last_VTG_fix;
  }

  //returns age of sentence
  inline unsigned long get_XTE_fix_age(){
    return last_XTE_fix;
  }
  
  //returns true if data has not been used
  inline boolean got_new_GGA_data(){
    boolean new_data  = new_GGA_data;
    new_GGA_data = false;
    return new_data;
  }
  
  //returns true if data has not been used
  inline boolean got_new_VTG_data(){
    boolean new_data  = new_VTG_data;
    new_VTG_data = false;
    return new_data;
  }
  
  //returns true if data has not been used
  inline boolean got_new_XTE_data(){
    boolean new_data  = new_XTE_data;
    new_XTE_data = false;
    return new_data;
  }

  // library version
  inline static float library_version() {
    return GPS_VERSION;
  }
};

#endif


