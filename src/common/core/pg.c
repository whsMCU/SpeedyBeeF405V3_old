/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "pg.h"

typedef struct
{
  uint32_t magic_number;
  char version_str[32];
} version_info_t;

__attribute__((section(".pg_registry"))) version_info_t verstion_info =
{
  .magic_number = 0x5555AAAA,
  .version_str = "V230331R1"
};

const version_info_t *p_verstion_info = (version_info_t *)0x8000400;

typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_LEVEL,
    PID_MAG,
    PID_ITEM_COUNT
} pidIndex_e;

typedef struct pidf_s {
    uint8_t P;
    uint8_t I;
    uint8_t D;
} pidf_t;

typedef struct pidPG_s {
    pidf_t  outer_pid[PID_ITEM_COUNT];
    pidf_t  inerer_pid[PID_ITEM_COUNT];
} pidPG_t;

__attribute__((section(".pg_registry"))) pidPG_t pid_pg =
{
    .outer_pid[PID_ROLL].P = 100,
    .outer_pid[PID_ROLL].I = 10,
    .outer_pid[PID_ROLL].D = 1,

    .outer_pid[PID_PITCH].P = 100,
    .outer_pid[PID_PITCH].I = 10,
    .outer_pid[PID_PITCH].D = 1,

    .outer_pid[PID_YAW].P = 100,
    .outer_pid[PID_YAW].I = 10,
    .outer_pid[PID_YAW].D = 1,


    .inerer_pid[PID_ROLL].P = 100,
    .inerer_pid[PID_ROLL].I = 10,
    .inerer_pid[PID_ROLL].D = 1,

    .inerer_pid[PID_PITCH].P = 100,
    .inerer_pid[PID_PITCH].I = 10,
    .inerer_pid[PID_PITCH].D = 1,

    .inerer_pid[PID_YAW].P = 100,
    .inerer_pid[PID_YAW].I = 10,
    .inerer_pid[PID_YAW].D = 1,
};

const pidPG_t *p_pid_pg = (pidPG_t *)(0x8000400+sizeof(version_info_t));
