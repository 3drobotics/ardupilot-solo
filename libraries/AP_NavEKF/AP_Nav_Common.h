/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  AP_Nav_Common holds definitions shared by inertial and ekf nav filters

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Nav_Common
#define AP_Nav_Common

union nav_filter_status {
    struct {
        uint16_t attitude           : 1; // 0 - true if attitude estimate is valid
        uint16_t horiz_vel          : 1; // 1 - true if horizontal velocity estimate is valid
        uint16_t vert_vel           : 1; // 2 - true if the vertical velocity estimate is valid
        uint16_t horiz_pos_rel      : 1; // 3 - true if the relative horizontal position estimate is valid
        uint16_t horiz_pos_abs      : 1; // 4 - true if the absolute horizontal position estimate is valid
        uint16_t vert_pos           : 1; // 5 - true if the vertical position estimate is valid
        uint16_t terrain_alt        : 1; // 6 - true if the terrain height estimate is valid
        uint16_t const_pos_mode     : 1; // 7 - true if we are in const position mode
        uint16_t pred_horiz_pos_rel : 1; // 8 - true if filter expects it can produce a good relative horizontal position estimate - used before takeoff
        uint16_t pred_horiz_pos_abs : 1; // 9 - true if filter expects it can produce a good absolute horizontal position estimate - used before takeoff
        uint16_t takeoff_detected   : 1; // 10 - true if optical flow takeoff has been detected
        uint16_t takeoff            : 1; // 11 - true if filter is compensating for baro errors during takeoff
        uint16_t touchdown          : 1; // 12 - true if filter is compensating for baro errors during touchdown
        uint16_t using_gps          : 1; // 13 - true if we are using GPS position
        uint16_t gps_glitching      : 1; // 14 - true if the the GPS is glitching
    } flags;
    uint16_t value;
};

#endif // AP_Nav_Common
