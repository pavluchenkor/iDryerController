/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

const short temptable_148[][2] PROGMEM = {
// These values have been calculated and tested over many days.  See https://docs.google.com/spreadsheets/d/1MJXa6feEe0mGVCT2TrBwLxVOMoLDkJlvfQ4JXhAdV_E
// Values that are missing from the 5C gap are missing due to resolution limits.
{OV(19.00000),  0},
{OV(19.25000),  5},
{OV(19.50000), 10},
{OV(19.87500), 15},
{OV(20.25000), 20},
{OV(21.00000), 25},
{OV(21.75000), 35},
{OV(22.00000), 40},
{OV(23.00000), 50},  // 55C is more commonly used.
{OV(23.75000), 60},
{OV(24.00000), 65},
{OV(24.06250), 70},
{OV(25.00000), 75},
{OV(25.50000), 85},
{OV(26.00000), 90},
{OV(26.93750),100},
{OV(27.00000),105},
{OV(27.37500),110},
{OV(28.00000),115},
{OV(29.00000),125},
{OV(29.25000),135},
{OV(30.00000),140},
{OV(35.50000),150},
{OV(31.00000),155},
{OV(32.00000),165},
{OV(32.18750),175},
{OV(33.00000),180},
{OV(33.62500),190},
{OV(34.00000),195},
{OV(35.00000),205},
{OV(35.50000),215},
{OV(36.00000),220},
{OV(36.75000),230},
{OV(37.00000),235},
{OV(37.75000),245},
{OV(38.00000),250},
{OV(38.12500),255},
{OV(39.00000),260},
{OV(40.00000),275},
{OV(40.25000),285},
{OV(41.00000),290},
{OV(41.25000),300},
{OV(42.00000),305},
{OV(43.00000),315},
{OV(43.25000),325},
{OV(44.00000),330},
{OV(44.18750),340},
{OV(45.00000),345},
{OV(45.25000),355},
{OV(46.00000),360},
{OV(46.62500),370},
{OV(47.00000),375},
{OV(47.25000),385},
{OV(48.00000),390},
{OV(48.75000),400},
{OV(49.00000),405},
};