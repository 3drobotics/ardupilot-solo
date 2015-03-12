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
  IMU driver backend class. Each supported gyro/accel sensor type
  needs to have an object derived from this class.

  Note that drivers can implement just gyros or just accels, and can
  also provide multiple gyro/accel instances.
 */
#ifndef __AP_INERTIALSENSOR_BACKEND_H__
#define __AP_INERTIALSENSOR_BACKEND_H__

class AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_Backend(AP_InertialSensor &imu);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_InertialSensor_Backend(void) {}

    /* 
     * Update the sensor data. Called by the frontend to transfer
     * accumulated sensor readings to the frontend structure via the
     * _publish_gyro() and _publish_accel() functions
     */
    virtual bool update() = 0;

    /* 
     * return true if at least one accel sample is available in the backend
     * since the last call to update()
     */
    virtual bool accel_sample_available() = 0;

    /* 
     * return true if at least one gyro sample is available in the backend
     * since the last call to update()
     */
    virtual bool gyro_sample_available() = 0;

    /*
      return the product ID
     */
    int16_t product_id(void) const { return _product_id; }

protected:
    // access to frontend
    AP_InertialSensor &_imu;

    void _rotate_and_correct_accel(uint8_t instance, Vector3f &accel);
    void _rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro);

    void _publish_delta_velocity(uint8_t instance, const Vector3f &delta_velocity);
    void _publish_delta_angle(uint8_t instance, const Vector3f &delta_angle);

    // rotate gyro vector, offset and publish
    void _publish_gyro(uint8_t instance, const Vector3f &gyro, bool rotate_and_correct = true);

    // rotate accel vector, scale, offset and publish
    void _publish_accel(uint8_t instance, const Vector3f &accel, bool rotate_and_correct = true);

    // set accelerometer error_count
    void _set_accel_error_count(uint8_t instance, uint32_t error_count);

    // set gyro error_count
    void _set_gyro_error_count(uint8_t instance, uint32_t error_count);

    // backend should fill in its product ID from AP_PRODUCT_ID_*
    int16_t _product_id;

    // return the default filter frequency in Hz for the sample rate
    uint8_t _accel_filter_cutoff(void) const { return _imu._accel_filter_cutoff; }

    // return the default filter frequency in Hz for the sample rate
    uint8_t _gyro_filter_cutoff(void) const { return _imu._gyro_filter_cutoff; }

    // return the requested sample rate in Hz
    uint16_t get_sample_rate_hz(void) const;

    // note that each backend is also expected to have a static detect()
    // function which instantiates an instance of the backend sensor
    // driver if the sensor is available
};

#endif // __AP_INERTIALSENSOR_BACKEND_H__
