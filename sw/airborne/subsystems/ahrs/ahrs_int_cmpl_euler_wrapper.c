/*
 * Copyright (C) 2015 Felix Ruess <felix.ruess@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file subsystems/ahrs/ahrs_int_cmpl_euler_wrapper.c
 *
 * Paparazzi specific wrapper to run floating point complementary filter.
 */

#include "subsystems/ahrs/ahrs_int_cmpl_euler_wrapper.h"
#include "subsystems/ahrs.h"
#include "subsystems/abi.h"
#include "state.h"

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_filter(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FILTER(trans, dev, AC_ID,
                       &ahrs_ice.ltp_to_imu_euler.phi,
                       &ahrs_ice.ltp_to_imu_euler.theta,
                       &ahrs_ice.ltp_to_imu_euler.psi,
                       &ahrs_ice.measure.phi,
                       &ahrs_ice.measure.theta,
                       &ahrs_ice.measure.psi,
                       &ahrs_ice.hi_res_euler.phi,
                       &ahrs_ice.hi_res_euler.theta,
                       &ahrs_ice.hi_res_euler.psi,
                       &ahrs_ice.residual.phi,
                       &ahrs_ice.residual.theta,
                       &ahrs_ice.residual.psi,
                       &ahrs_ice.gyro_bias.p,
                       &ahrs_ice.gyro_bias.q,
                       &ahrs_ice.gyro_bias.r);
}

static void send_euler(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Eulers *eulers = stateGetNedToBodyEulers_i();
  pprz_msg_send_AHRS_EULER_INT(trans, dev, AC_ID,
                               &ahrs_ice.ltp_to_imu_euler.phi,
                               &ahrs_ice.ltp_to_imu_euler.theta,
                               &ahrs_ice.ltp_to_imu_euler.psi,
                               &(eulers->phi),
                               &(eulers->theta),
                               &(eulers->psi));
}

static void send_bias(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_AHRS_GYRO_BIAS_INT(trans, dev, AC_ID,
                                   &ahrs_ice.gyro_bias.p, &ahrs_ice.gyro_bias.q, &ahrs_ice.gyro_bias.r);
}
#endif

/** ABI binding for IMU data.
 * Used for gyro, accel and mag ABI messages.
 */
#ifndef AHRS_ICE_IMU_ID
#define AHRS_ICE_IMU_ID ABI_BROADCAST
#endif
static abi_event gyro_ev;
static abi_event accel_ev;
static abi_event mag_ev;
static abi_event aligner_ev;
static abi_event body_to_imu_ev;


static void gyro_cb(uint8_t sender_id __attribute__((unused)),
                    uint32_t stamp __attribute__((unused)),
                    struct Int32Rates *gyro)
{
  if (ahrs_ice.is_aligned) {
    ahrs_ice_propagate(gyro);
  }

}

static void accel_cb(uint8_t sender_id __attribute__((unused)),
                     uint32_t stamp __attribute__((unused)),
                     struct Int32Vect3 *accel)
{
  if (ahrs_ice.is_aligned) {
    ahrs_ice_update_accel(accel);
  }
}

static void mag_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct Int32Vect3 *mag)
{
  if (ahrs_ice.is_aligned) {
    ahrs_ice_update_mag(mag);
  }
}

static void aligner_cb(uint8_t __attribute__((unused)) sender_id,
                       uint32_t stamp __attribute__((unused)),
                       struct Int32Rates *lp_gyro, struct Int32Vect3 *lp_accel,
                       struct Int32Vect3 *lp_mag)
{
  if (!ahrs_ice.is_aligned) {
    ahrs_ice_align(lp_gyro, lp_accel, lp_mag);
  }
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  ahrs_ice_set_body_to_imu_quat(q_b2i_f);
}

void ahrs_ice_register(void)
{
  ahrs_register_impl(ahrs_ice_init, NULL);

  /*
   * Subscribe to scaled IMU measurements and attach callbacks
   */
  AbiBindMsgIMU_GYRO_INT32(AHRS_ICE_IMU_ID, &gyro_ev, gyro_cb);
  AbiBindMsgIMU_ACCEL_INT32(AHRS_ICE_IMU_ID, &accel_ev, accel_cb);
  AbiBindMsgIMU_MAG_INT32(AHRS_ICE_IMU_ID, &mag_ev, mag_cb);
  AbiBindMsgIMU_LOWPASSED(ABI_BROADCAST, &aligner_ev, aligner_cb);
  AbiBindMsgBODY_TO_IMU_QUAT(ABI_BROADCAST, &body_to_imu_ev, body_to_imu_cb);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "FILTER", send_filter);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_EULER_INT", send_euler);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_GYRO_BIAS_INT", send_bias);
#endif
}
