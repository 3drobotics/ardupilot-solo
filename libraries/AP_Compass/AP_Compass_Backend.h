// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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

/*
  Compass driver backend class. Each supported compass sensor type
  needs to have an object derived from this class.
 */
#ifndef __AP_COMPASS_BACKEND_H__
#define __AP_COMPASS_BACKEND_H__

#include "Compass.h"

class Compass;  // forward declaration
class AP_Compass_Backend
{
public:
    AP_Compass_Backend(Compass &compass);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_Compass_Backend(void) {}

    // initialize the magnetometers
    virtual bool init(void) = 0;

    // read sensor data
    virtual void read(void) = 0;

    // accumulate a reading from the magnetometer. Optional in
    // backends
    virtual void accumulate(void) {};

protected:

    /*
     * A compass measurement is expected to pass through the following functions:
     * 1. rotate_field - this rotates the measurement in-place from sensor frame
     *      to body frame
     * 2. publish_raw_field - this provides an uncorrected point-sample for
     *      calibration libraries
     * 3. correct_field - this corrects the measurement in-place for hard iron,
     *      soft iron, motor interference, and non-orthagonality errors
     * 4. publish_unfiltered_field - this (optionally) provides a corrected
     *      point sample for fusion into the EKF
     * 5. publish_filtered_field - legacy filtered magnetic field
     */

    void rotate_field(Vector3f &mag, uint8_t instance);
    void publish_raw_field(const Vector3f &mag, uint32_t time_us, uint8_t instance);
    void correct_field(Vector3f &mag, uint8_t i);
    void publish_unfiltered_field(const Vector3f &mag, uint32_t time_us, uint8_t instance);
    void publish_filtered_field(const Vector3f &mag, uint8_t instance);
    void update_saturated(const bool is_saturated, uint8_t instance);

    // register a new compass instance with the frontend
    uint8_t register_compass(void) const;

    // set dev_id for an instance
    void set_dev_id(uint8_t instance, uint32_t dev_id);

    // set external state for an instance
    void set_external(uint8_t instance, bool external);

    // access to frontend
    Compass &_compass;

private:
    void apply_corrections(Vector3f &mag, uint8_t i);
};

#endif // __AP_COMPASS_BACKEND_H__
