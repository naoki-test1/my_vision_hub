/*******************************************************************************
 * Copyright 2025 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *	this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *	this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *	may be used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *	ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "offboard_trajectory.h"

#include <errno.h>
#include <modal_pipe.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

#include "config_file.h"
#include "geometry.h"
#include "macros.h"
#include "misc.h"
#include "pipe_channels.h"
#include "mavlink_io.h"
#include "autopilot_monitor.h"
#include "trajectory_monitor.h"

#define BENCHTOP_TESTING false
#define RATE 30 // loop rate hz
#define NUM_OF_STATES 7

typedef enum state_t
{
    WARMUP,
    WAITING_FOR_OB,
    WAITING_FOR_TRAJ, // remain in waiting state until we start following
    FOLLOWING_TRAJ,
    PAUSED_TRAJ,
    WAITING_FOR_ESTOP_RESET,
    EXITING
} state_t;

// Function prototypes for the state handlers
static void warmup_state_handler(int64_t *last_eval_time_ns);
static void waiting_for_ob_state_handler(int64_t *last_eval_time_ns);
static void waiting_for_traj_state_handler(int64_t *last_eval_time_ns);
static void following_traj_state_handler(int64_t *last_eval_time_ns);
static void paused_traj_state_handler(int64_t *last_eval_time_ns);
static void waiting_for_estop_reset_state_handler(int64_t *last_eval_time_ns);
static void exit_state_handler(__attribute__((unused))int64_t *last_eval_time_ns);

// Do nothing for now
static void exit_state_handler(__attribute__((unused))int64_t *last_eval_time_ns){
	return;
};

static void (*state_handler[NUM_OF_STATES])() = {warmup_state_handler,
                                          waiting_for_ob_state_handler,
                                          waiting_for_traj_state_handler,
                                          following_traj_state_handler,
                                          paused_traj_state_handler,
                                          waiting_for_estop_reset_state_handler,
                                          exit_state_handler};

// protect trajectory with this mutex
static pthread_mutex_t traj_handler_mutex = PTHREAD_MUTEX_INITIALIZER;
static state_t state;
static pthread_t thread_id;
static int en_debug = 0;
static mavlink_set_position_target_local_ned_t last_position;
static trajectory_handler traj_handler;

static int _eval_segment(poly_segment_t *s, double t,
                         mavlink_set_position_target_local_ned_t *out)
{
    if (t > s->duration_s + 0.0001)
    {
        fprintf(stderr, "WARNING, trying to evaluate segment after duration\n");
        fprintf(stderr, "%d/%f | cur = %d/%f\n", s->id, t, traj_handler.cur_segment_id, traj_handler.cur_segment_t);
    }

    // make sure yaw is wrapped neatly
    double yaw = eval_poly_at_t(s->n_coef, s->cyaw, t);
    WRAP_TO_NEGPI_TO_PI(yaw);
    static double last_yaw = 0.0;
    static double last_t = -1.0;

    // check if we are starting a new trajectory
    if (t < last_t)
    {
        last_t = -1.0;
        last_yaw = 0.0;
    }

    out->time_boot_ms = 0;
    out->coordinate_frame = MAV_FRAME_LOCAL_NED;

    // only thing we don't calculate is yaw rate
    out->type_mask = 0;

    out->x = eval_poly_at_t(s->n_coef, s->cx, t);
    out->y = eval_poly_at_t(s->n_coef, s->cy, t);
    out->z = eval_poly_at_t(s->n_coef, s->cz, t);
    out->vx = eval_vel_at_t(s->n_coef, s->cx, t);
    out->vy = eval_vel_at_t(s->n_coef, s->cy, t);
    out->vz = eval_vel_at_t(s->n_coef, s->cz, t);
    out->afx = eval_accel_at_t(s->n_coef, s->cx, t);
    out->afy = eval_accel_at_t(s->n_coef, s->cy, t);
    out->afz = eval_accel_at_t(s->n_coef, s->cz, t);

    // Disable for testig freaky trajectories
    // out->yaw = 0;

    // yaw just follows velocity
    if (fabs(out->vx) < 0.0001 && fabs(out->vy) < 0.0001)
    {
        out->yaw = last_position.yaw;
    }
    else
    {
        out->yaw = atan2(out->vy, out->vx);
    }

    // if we have calculated a position in the trajectory already,
    // calculate the yaw rate by dirty differentiation
    if (last_t >= 0.0)
    {
        double d_yaw = (double)out->yaw - last_yaw;
        WRAP_TO_NEGPI_TO_PI(d_yaw);
        if (d_yaw > 0.1)
        {
            d_yaw = 0.0;
        }
        out->yaw_rate = d_yaw / (t - last_t);
    }
    else
    {
        out->yaw_rate = 0.0;
    }

    // save yaw for next time
    last_yaw = out->yaw;
    last_t = t;

    // final values for mavlink packet
    out->target_system = 0; // will reset later when sending
    out->target_component = AUTOPILOT_COMPID;

    return 0;
}

/**
 * @brief Evaluates the trajectory dt_s seconds further along than the
 * previously evaluated time and saves the position in the out message.
 *
 * @param traj The trajectory to evaluate
 * @param dt_s The time step, in seconds_ further ahead than the previous time
 * to evaluate the trajectory at
 * @param out The output message for px4
 * @return int -1 if uncusseful, 0 if succesful, 1 if succesful and reached end
 * of trajectory
 */
static int _eval_trajectory(trajectory_handler *handler, double dt_s,
                            mavlink_set_position_target_local_ned_t *out)
{
    if (dt_s < 0.0)
    {
        fprintf(stderr, "ERROR in %s, t must be >=0\n", __FUNCTION__);
        return -1;
    }

    trajectory_t *traj = &handler->traj;
    int segment_idx = -1;
    handler->cur_segment_t += dt_s;

    // find the segment we were previously on
    for (int i = 0; i < traj->n_segments; i++)
    {
        if (handler->cur_segment_id == traj->segments[i].id)
        {
            segment_idx = i;
            break;
        }
    }

    if (segment_idx < 0)
    {
        fprintf(stderr, "ERROR in %s, Segment with id %d not found\n", __FUNCTION__, handler->cur_segment_id);
        return -1;
    }

    // Make sure to check if we need to move to the next segment
    if (handler->cur_segment_t >= traj->segments[segment_idx].duration_s)
    {
        // Check if we hit the last segment
        if (segment_idx + 1 >= traj->n_segments)
        {
            // Evalaute the segment at its end position then return with code 1
            _eval_segment(&traj->segments[segment_idx],
                          traj->segments[segment_idx].duration_s, out);
            return 1;
        }

        // Otherwise set our id for the next segment and fix the eval time
        handler->cur_segment_id = traj->segments[segment_idx + 1].id;
        handler->cur_segment_t -= traj->segments[segment_idx].duration_s;
        segment_idx++;
    }

    return _eval_segment(&traj->segments[segment_idx], handler->cur_segment_t,
                         out);
}

// update the last position with just xyz/yaw from a full pose
// this is meant to be the hold-still equivalent of whatever the current command
// is for when we need to stop in-place.
static void _update_last_position(mavlink_set_position_target_local_ned_t pos)
{
    last_position.time_boot_ms = 0;
    last_position.coordinate_frame = MAV_FRAME_LOCAL_NED;
    last_position.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                              POSITION_TARGET_TYPEMASK_VY_IGNORE |
                              POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                              POSITION_TARGET_TYPEMASK_AX_IGNORE |
                              POSITION_TARGET_TYPEMASK_AY_IGNORE |
                              POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                              POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    last_position.x = pos.x;
    last_position.y = pos.y;
    last_position.z = pos.z;
    last_position.vx = 0.0f;
    last_position.vy = 0.0f;
    last_position.vz = 0.0f;
    last_position.afx = 0.0f;
    last_position.afy = 0.0f;
    last_position.afz = 0.0f;
    last_position.yaw = pos.yaw;
    last_position.yaw_rate = 0.0f;
    last_position.target_system = 0; // will reset later when sending
    last_position.target_component = AUTOPILOT_COMPID;
    return;
}

static void _send_last_position_again(void)
{
    mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID,
                                    last_position);
    return;
}

static void _send_current_position(void)
{
    // TODO validate VIO is running before sending this

    // fetch latest position and attitude from px4 itself so the setpoint
    // we are about to send is a close as possible to where we currently are
    // TODO validate this data before actually sending!!!
    mavlink_odometry_t odom = autopilot_monitor_get_odometry();
    mavlink_attitude_t atti = autopilot_monitor_get_attitude();
    mavlink_set_position_target_local_ned_t pos;

    pos.time_boot_ms = 0;
    pos.coordinate_frame = MAV_FRAME_LOCAL_NED;
    pos.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                    POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    pos.x = odom.x;
    pos.y = odom.y;
    pos.z = odom.z;
    pos.vx = 0.0f;
    pos.vy = 0.0f;
    pos.vz = 0.0f;
    pos.afx = 0.0f;
    pos.afy = 0.0f;
    pos.afz = 0.0f;
    pos.yaw = atti.yaw;
    pos.yaw_rate = 0.0f;
    pos.target_system = 0; // will reset later when sending
    pos.target_component = AUTOPILOT_COMPID;

    _update_last_position(pos);
    mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

    if (en_debug)
    {
        printf("commanding: XYZ %6.1f %6.1f %6.1f  yaw: %6.1f\n", (double)pos.x,
               (double)pos.y, (double)pos.z, (double)pos.yaw);
    }
    return;
}

/**
 * @brief Gets the position that is dt_s seconds further along the trajectory
 * than the previously evaluated time and sends it to px4.
 *
 * @param dt_s
 * @return int -1 if unsuccessful, 0 if successful, 1 if successful but reached
 * end of trajectory
 */
static int _send_position_in_traj(double dt_s)
{
    mavlink_set_position_target_local_ned_t pos;
    pthread_mutex_lock(&traj_handler_mutex);
    int ret = _eval_trajectory(&traj_handler, dt_s, &pos);
    pthread_mutex_unlock(&traj_handler_mutex);
    if (ret < 0)
    {
        fprintf(stderr, "ERROR failed to send position in trajectory\n");
        return -1;
    }

    mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);
    _update_last_position(pos);

    pthread_mutex_lock(&traj_handler_mutex);
    traj_protocol_t eval_msg = create_evaluated_msg(traj_handler.cur_segment_id, traj_handler.cur_segment_t);
    pthread_mutex_unlock(&traj_handler_mutex);

    pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &eval_msg, sizeof(eval_msg));

    if (en_debug)
    {
        printf(
            "commanding: XYZ %6.1f %6.1f %6.1f yaw: %6.1f  V:%6.1f %6.1f %6.1f "
            "A: %6.1f %6.1f %6.1f\n",
            (double)pos.x, (double)pos.y, (double)pos.z, (double)pos.yaw,
            (double)pos.vx, (double)pos.vy, (double)pos.vz, (double)pos.afx,
            (double)pos.afy, (double)pos.afz);
    }
    return ret;
}

void trigger_estop()
{
    state = WAITING_FOR_ESTOP_RESET;

    pthread_mutex_lock(&traj_handler_mutex);
    traj_handler.is_ready = false;
    pthread_mutex_unlock(&traj_handler_mutex);

    _send_last_position_again();
    printf("EStop triggered. Trajectory stopped\n");

    traj_protocol_t estop_msg = create_estop_msg();
    pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &estop_msg, sizeof(estop_msg));
}

trajectory_handler get_trajectory_handler()
{
    pthread_mutex_lock(&traj_handler_mutex);
    trajectory_handler copy = traj_handler;
    pthread_mutex_unlock(&traj_handler_mutex);

    return copy;
}

static bool is_traj_ready()
{
    pthread_mutex_lock(&traj_handler_mutex);
    bool ready = traj_handler.is_ready;
    pthread_mutex_unlock(&traj_handler_mutex);

    return ready;
}

void warmup_state_handler(__attribute__((unused)) int64_t *last_eval_time_ns)
{
    static int counter = 0;
    _send_current_position();
    counter++;

    if (counter > (RATE * 2))
    {
        state = WAITING_FOR_OB;
    }
}

void waiting_for_ob_state_handler(__attribute__((unused)) int64_t *last_eval_time_ns)
{
    _send_current_position();

#if !BENCHTOP_TESTING
    if (!autopilot_monitor_is_armed_and_in_offboard_mode())
        return;
#endif

    // make sure we only follow trajectories received after we got
    // into offboard mode
    state = WAITING_FOR_TRAJ;

    pthread_mutex_lock(&traj_handler_mutex);
    traj_handler.is_ready = false;
    pthread_mutex_unlock(&traj_handler_mutex);
}

void waiting_for_traj_state_handler(int64_t *last_eval_time_ns)
{
    _send_last_position_again();

#if !BENCHTOP_TESTING
    if (!autopilot_monitor_is_armed_and_in_offboard_mode())
    {
        state = WAITING_FOR_OB;
        return;
    }
#endif

    if (is_traj_ready())
    {
        *last_eval_time_ns = my_time_monotonic_ns();
        state = FOLLOWING_TRAJ;
    }
}

void paused_traj_state_handler(int64_t *last_eval_time_ns)
{
    _send_last_position_again();

#if !BENCHTOP_TESTING
    if (!autopilot_monitor_is_armed_and_in_offboard_mode())
    {
        state = WAITING_FOR_OB;
        return;
    }
#endif

    if (is_traj_ready())
    {
        *last_eval_time_ns = my_time_monotonic_ns();
        state = FOLLOWING_TRAJ;
    }
}

void following_traj_state_handler(int64_t *last_eval_time_ns)
{
#if !BENCHTOP_TESTING
    if (!autopilot_monitor_is_armed_and_in_offboard_mode())
    {
        state = WAITING_FOR_OB;
        return;
    }
#endif

    // Go back to waiting if trajectory is not ready
    if (!is_traj_ready())
    {
        fprintf(stderr, "Exiting following traj state\n");
        state = WAITING_FOR_TRAJ;
        return;
    }

    int64_t dt_ns = my_time_monotonic_ns() - *last_eval_time_ns;
    *last_eval_time_ns = my_time_monotonic_ns();

    double dt_s = (double)dt_ns / 1e9;
    int ret = _send_position_in_traj(dt_s);
    if (ret == 1)
    {
        pthread_mutex_lock(&traj_handler_mutex);
        traj_handler.is_ready = false; // done with this traj
        pthread_mutex_unlock(&traj_handler_mutex);

        state = WAITING_FOR_TRAJ; // wait for a new one

        printf("FINISHED TRAJECTORY\n");
    }
}

void waiting_for_estop_reset_state_handler(__attribute__((unused)) int64_t *last_eval_time_ns)
{
    // The only way to exit this state is by receiving
    // a traj with command ESTOP_ACK, or exiting offboard mode.

#if !BENCHTOP_TESTING
    if (!autopilot_monitor_is_armed_and_in_offboard_mode())
    {
        state = WAITING_FOR_OB;
        return;
    }
#endif
}

static void *_thread_func(__attribute__((unused)) void *arg)
{

    int64_t last_eval_time_ns = my_time_monotonic_ns();
    int64_t next_time = 0;

    while (state != EXITING)
    {
        if(my_loop_sleep(RATE, &next_time)){
            fprintf(stderr, "WARNING offboard trajectory thread fell behind\n");
        }

        // Run the corresponding state handler
        (*state_handler[state])(&last_eval_time_ns);
    }

    printf("exiting offboard trajectory thread\n");
    return NULL;
}

static void _connect_cb(__attribute__((unused)) int ch,
                        __attribute__((unused)) void *context)
{
    printf("Connected to voxl-mapper\n");
    int size = pipe_client_get_pipe_size(TRAJECTORY_PIPE_CH);
    printf("voxl-mapper pipe size is: %d bytes\n", size);
    if (size < (64 * 1024))
    {
        printf("resetting pipe size to 64k\n");
        pipe_client_set_pipe_size(TRAJECTORY_PIPE_CH, 64 * 1024);
        size = pipe_client_get_pipe_size(TRAJECTORY_PIPE_CH);
        printf("voxl-mapper pipe size is: %d bytes\n", size);
    }
    return;
}

static void _disconnect_cb(__attribute__((unused)) int ch,
                           __attribute__((unused)) void *context)
{
    // On disconnect do not continue to follow a trajectory
    pthread_mutex_lock(&traj_handler_mutex);
    traj_handler.is_ready = false;
    pthread_mutex_unlock(&traj_handler_mutex);

    printf("Disconnected from voxl-mapper\n");
    return;
}

static void start_cmd_handler()
{
    fprintf(stderr, "Received start command.\n");

    // as long as we have something stored, follow it!
    pthread_mutex_lock(&traj_handler_mutex);
    if (traj_handler.traj.n_segments > 0)
    {
        traj_handler.is_ready = true;
        fprintf(stderr, "Following last trajectory!\n");

        traj_protocol_t ack_msg = create_ack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &ack_msg, sizeof(ack_msg));
    }
    else
    {
        traj_protocol_t nack_msg = create_nack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &nack_msg, sizeof(nack_msg));
    }
    pthread_mutex_unlock(&traj_handler_mutex);
}

static void pause_cmd_handler()
{
    fprintf(stderr, "Received pause command.\n");

    if (state == FOLLOWING_TRAJ)
    {
        pthread_mutex_lock(&traj_handler_mutex);
        traj_handler.is_ready = false;
        pthread_mutex_unlock(&traj_handler_mutex);

        state = PAUSED_TRAJ;
        fprintf(stderr, "Trajectory paused! Waiting for start command\n");

        traj_protocol_t ack_msg = create_ack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &ack_msg, sizeof(ack_msg));
    }
    else
    {
        fprintf(stderr, "Received pause command when not following anything.\n");

        traj_protocol_t nack_msg = create_nack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &nack_msg, sizeof(nack_msg));
    }
}

static void stop_cmd_handler()
{
    fprintf(stderr, "Received stop command.\n");

    if (state == FOLLOWING_TRAJ)
    {
        pthread_mutex_lock(&traj_handler_mutex);
        traj_handler.is_ready = false;
        pthread_mutex_unlock(&traj_handler_mutex);

        state = WAITING_FOR_TRAJ;
        fprintf(stderr, "Stopped following trajectory.\n");

        traj_protocol_t ack_msg = create_ack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &ack_msg, sizeof(ack_msg));
    }
    else
    {
        fprintf(stderr, "Received stop command when not following anything.\n");

        traj_protocol_t nack_msg = create_nack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &nack_msg, sizeof(nack_msg));
    }
}

static void estop_ack_cmd_handler()
{
    if (state == WAITING_FOR_ESTOP_RESET)
    {
        printf("Received ESTOP reset command. Transitioning to waiting for trajectory.\n");
        state = WAITING_FOR_TRAJ;
    }
}

static void load_and_start_handler(trajectory_t *new_traj)
{
    fprintf(stderr, "Received load and start command.\n");

    pthread_mutex_lock(&traj_handler_mutex);
    traj_handler.cur_segment_id = new_traj->segments[0].id;
    traj_handler.cur_segment_t = 0;
    traj_handler.traj = *new_traj;
    traj_handler.is_ready = true;
    pthread_mutex_unlock(&traj_handler_mutex);

    traj_protocol_t ack_msg = create_ack_msg();
    pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &ack_msg, sizeof(ack_msg));
}

static void load_handler(trajectory_t *new_traj)
{
    fprintf(stderr, "Received load command.\n");

    // A load command will wait for a start command before execution
    pthread_mutex_lock(&traj_handler_mutex);
    traj_handler.cur_segment_id = new_traj->segments[0].id;
    traj_handler.cur_segment_t = 0;
    traj_handler.traj = *new_traj;
    traj_handler.is_ready = false;
    pthread_mutex_unlock(&traj_handler_mutex);

    traj_protocol_t ack_msg = create_ack_msg();
    pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &ack_msg, sizeof(ack_msg));
}

static void insert_handler(trajectory_t *new_traj)
{
    fprintf(stderr, "Received insert command.\n");

    pthread_mutex_lock(&traj_handler_mutex);

    // Trim the trajectory to make space for the new trajectory
    trim_trajectory(&traj_handler.traj, traj_handler.cur_segment_id);

    // Insert the new trajectory in
    bool succ = insert_trajectory(&traj_handler.traj, new_traj);

    pthread_mutex_unlock(&traj_handler_mutex);

    if (succ)
    {
        traj_protocol_t ack_msg = create_ack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &ack_msg, sizeof(ack_msg));
    }
    else
    {
        traj_protocol_t nack_msg = create_nack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &nack_msg, sizeof(nack_msg));
    }
}

static void execute_trajectory_command(trajectory_t *traj)
{
    if (state == WAITING_FOR_ESTOP_RESET && traj->traj_command != TRAJ_CMD_ESTOP_RESET)
    {
        printf("Previous ESTOP not reset. Ignoring received trajectory.\n");
        return;
    }

    // before we do anything with this traj, see if its just a command
    switch (traj->traj_command)
    {
    case TRAJ_CMD_START:
        start_cmd_handler();
        return;

    case TRAJ_CMD_PAUSE:
        pause_cmd_handler();
        return;

    case TRAJ_CMD_STOP:
        stop_cmd_handler();
        return;

    case TRAJ_CMD_ESTOP:
        trigger_estop();
        return;

    case TRAJ_CMD_ESTOP_RESET:
        estop_ack_cmd_handler();
        return;
    }

    // Validate the trajectory
    double tmp = 0.0;
    for (int i = 0; i < traj->n_segments; i++)
    {
        tmp += traj->segments[i].duration_s;
    }
    printf("Received trajectory has duration %f seconds\n", tmp);

    if (tmp <= 0.0)
    {
        fprintf(stderr, "ERROR received trajectory has 0 duration\n");

        traj_protocol_t nack_msg = create_nack_msg();
        pipe_client_send_control_cmd_bytes(TRAJECTORY_PIPE_CH, &nack_msg, sizeof(nack_msg));

        return;
    }

    // Handle commands that involve trajectory data
    switch (traj->traj_command)
    {
    case TRAJ_CMD_LOAD_AND_START:
        load_and_start_handler(traj);
        break;

    case TRAJ_CMD_LOAD:
        load_handler(traj);
        break;

    case TRAJ_CMD_INSERT:
        insert_handler(traj);
        break;
    }
}

static void execute_setpoint_position_command(setpoint_position_t *msg)
{
    mavlink_set_position_target_local_ned_t pos;

    pos.time_boot_ms = 0;
    pos.coordinate_frame = msg->coordinate_frame;
    pos.type_mask = msg->type_mask;
    pos.x = msg->position[0];
    pos.y = msg->position[1];
    pos.z = msg->position[2];
    pos.vx = msg->velocity[0];
    pos.vy = msg->velocity[1];
    pos.vz = msg->velocity[2];
    pos.afx = msg->acceleration[0];
    pos.afy = msg->acceleration[1];
    pos.afz = msg->acceleration[2];
    pos.yaw = msg->yaw;
    pos.yaw_rate = msg->yaw_rate;
    pos.target_system = 0; // will reset later when sending
    pos.target_component = AUTOPILOT_COMPID;

    _update_last_position(pos);
    mavlink_io_send_fixed_setpoint(autopilot_monitor_get_sysid(), VOXL_COMPID, pos);

    if (en_debug)
    {
        printf("Setpoint position: XYZ %6.1f %6.1f %6.1f  yaw: %6.1f\n", (double)pos.x,
               (double)pos.y, (double)pos.z, (double)pos.yaw);
    }
    return;
}

static void _trajectory_pipe_helper_cb(__attribute__((unused)) int ch,
                                       char *data, int bytes,
                                       __attribute__((unused)) void *context)
{
    // validate that the data makes sense
    int n_packets;

    if (data == NULL)
        return;

    int32_t magic_number = ((int32_t *)data)[0];

    if (magic_number == TRAJECTORY_MAGIC_NUMBER)
    {

        trajectory_t *data_array = pipe_validate_trajectory_t(data, bytes, &n_packets);

        // Check valid data
        if (data_array == NULL)
            return;
        if (n_packets < 1)
            return;

        // Ensure in valid state to receive trajectory
        if (state < WAITING_FOR_TRAJ)
        {
            fprintf(stderr, "Received trajectory while not waiting for one\n");
            return;
        }

        // Execute all received commands
        for (int i = 0; i < n_packets; i++)
            execute_trajectory_command(&data_array[i]);
    }
    else if (magic_number == SETPOINT_POSITION_MAGIC_NUMBER)
    {

        setpoint_position_t *data_array = pipe_validate_setpoint_position_t(data, bytes, &n_packets);

        // Check valid data
        if (data_array == NULL)
            return;
        if (n_packets < 1)
            return;

        // Only process setpoint when in waiting for traj state
        if (state != WAITING_FOR_TRAJ)
        {
            fprintf(stderr, "Received setpoint position while in invalid state\n");
            return;
        }

        execute_setpoint_position_command(&data_array[n_packets - 1]);
    }
    else
    {
        fprintf(stderr, "Received unrecognized magic number\n");
    }
}

int offboard_trajectory_init(void)
{

#if BENCHTOP_TESTING
    printf("####################################################################\n"
           "#                   BENCHTOP TESTING FLAG IS ON                    #\n"
           "#                   DO NOT ATTEMPT TO FLY                          #\n"
           "#                   BAD THINGS ARE GUARANTEED TO                   #\n"
           "#                   TO HAPPEN. REBUILD WITHOUT                     #\n"
           "#                   THE FLAG TO AVOID ISSUES                       #\n"
           "####################################################################\n");
#endif

    state = WARMUP;
    pipe_pthread_create(&thread_id, _thread_func, NULL, OFFBOARD_THREAD_PRIORITY);

    pipe_client_set_connect_cb(TRAJECTORY_PIPE_CH, _connect_cb, NULL);
    pipe_client_set_disconnect_cb(TRAJECTORY_PIPE_CH, _disconnect_cb, NULL);
    pipe_client_set_simple_helper_cb(TRAJECTORY_PIPE_CH, _trajectory_pipe_helper_cb, NULL);
    pipe_client_open(TRAJECTORY_PIPE_CH, TRAJECTORY_PIPE_LOCATION,
                     PIPE_CLIENT_NAME, EN_PIPE_CLIENT_SIMPLE_HELPER,
                     TRAJECTORY_RECOMMENDED_READ_BUF_SIZE);

    trajectory_monitor_init();

    return 0;
}

int offboard_trajectory_stop(int blocking)
{
    state = EXITING;
    if (blocking)
    {
        pthread_join(thread_id, NULL);
    }
    pipe_client_close(TRAJECTORY_PIPE_CH);
    trajectory_monitor_stop();
    return 0;
}

void offboard_trajectory_en_print_debug(int debug)
{
    if (debug)
        en_debug = 1;
    return;
}
