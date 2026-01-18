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

#include <float.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C"
{
#include <modal_pipe.h>
#include <rc_math.h>

#include "config_file.h"
#include "geometry.h"
#include "misc.h"
#include "offboard_trajectory.h"
}

#include <vector>

#include "octree.hpp"
#include "pipe_channels.h"
#include "voxl_trajectory.h"
#include "trajectory_monitor.h"
#include "voxl_vision_hub.h"

#include <string>

#define A_MAX 2.0
#define COLLISION_POINT_COUNT 5

// big read buffer for point clouds, auto-expanded by helper if needed
#define VOA_READ_BUF_SIZE (12 * 320 * 240)

struct Point3f
{
    float x;
    float y;
    float z;
};

// Need to keep points for octree alive with octree
// (otherwise we would need to set copyPoints to true in octree params)
unibn::Octree<Point3f> octree;
std::vector<Point3f> points;
int64_t voa_pc_timestamp;
int voa_ch;
bool prev_in_col = false;

bool debug = false;
std::string log_str;

static void _setup_octree(float *data, int n)
{
    points.clear();
    points.reserve(n);

    if (debug)
    {
        log_str += "-------------\n";
        log_str += std::to_string(n) + "\n";
    }

    // check every point in the input for bounding box
    for (int i = 0; i < n; i++)
    {
        float x = data[(i * 3)];
        float y = data[(i * 3) + 1];
        float z = data[(i * 3) + 2];

        if (debug)
            log_str += std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ", ";

        points.push_back(Point3f{.x = x, .y = y, .z = z});
    }

    if (debug)
        log_str += "\n\n";

    octree.initialize(points);
}

static bool _is_footprint_in_collision(float x, float y, float z)
{
    // Find all points within robot radius
    const Point3f query_point{.x = x, .y = y, .z = z};
    std::vector<uint32_t> results;

    octree.radiusNeighbors<unibn::L2Distance<Point3f>>(query_point,
                                                       robot_radius, results);

    // If there are no points within robot radius then we are clear
    if (results.size() < COLLISION_POINT_COUNT)
        return false;

    if (debug)
    {
        fprintf(stderr, "%d Collisions\n", (int)results.size());

        for (uint32_t i = 0; i < results.size(); ++i)
        {
            const Point3f &p = points[results[i]];
            fprintf(stderr, "%d: (%f, %f, %f) => %f\n", results[i], p.x, p.y, p.z, sqrt(unibn::L2Distance<Point3f>::compute(p, query_point)));
        }
    }

    return true;
}

static bool _is_segment_in_collision(poly_segment_t *segment, double start_time,
                                     double end_time, float R_fixed_to_level[3][3],
                                     float T_fixed_wrt_level[3])
{
    for (double t = start_time; t <= end_time; t += collision_sampling_dt)
    {
        float x = eval_poly_at_t(segment->n_coef, segment->cx, t);
        float y = eval_poly_at_t(segment->n_coef, segment->cy, t);
        float z = eval_poly_at_t(segment->n_coef, segment->cz, t);

        // Transform polynomial points from fixed frame to level frame
        float tf_x = R_fixed_to_level[0][0] * x + R_fixed_to_level[0][1] * y + R_fixed_to_level[0][2] * z + T_fixed_wrt_level[0];
        float tf_y = R_fixed_to_level[1][0] * x + R_fixed_to_level[1][1] * y + R_fixed_to_level[1][2] * z + T_fixed_wrt_level[1];
        float tf_z = R_fixed_to_level[2][0] * x + R_fixed_to_level[2][1] * y + R_fixed_to_level[2][2] * z + T_fixed_wrt_level[2];

        if (debug)
        {
            log_str += std::to_string(segment->id) + ", " + std::to_string(start_time) + ", " + std::to_string(end_time) + ", " + std::to_string(t) + " | ";
            log_str += std::to_string(tf_x) + ", " + std::to_string(tf_y) + ", " + std::to_string(tf_z) + "\n";
        }

        if (_is_footprint_in_collision(tf_x, tf_y, tf_z))
            return true;
    }

    return false;
}

static bool _is_trajectory_in_collision(trajectory_handler *traj_handler,
                                        int64_t voa_pc_timestamp)
{
    double t = 0;

    // Get robot pose
    static rc_vector_t T_body_wrt_fixed = RC_VECTOR_INITIALIZER;
    geometry_get_T_body_wrt_fixed(&T_body_wrt_fixed);

    // Compute transform to get polynomial from fixed frame to level frame
    static rc_matrix_t R_fixed_to_level = RC_MATRIX_INITIALIZER;
    static rc_vector_t T_fixed_wrt_level = RC_VECTOR_INITIALIZER;

    int ret = geometry_get_RT_fixed_to_level_at_time(
        voa_pc_timestamp, &R_fixed_to_level, &T_fixed_wrt_level);

    // convert to float for faster math
    float T[3];
    float R[3][3];
    for (int i = 0; i < 3; i++)
    {
        T[i] = (float)T_fixed_wrt_level.d[i];
        for (int j = 0; j < 3; j++)
        {
            R[i][j] = (float)R_fixed_to_level.d[i][j];
        }
    }

    int segment_idx = get_segment_idx(&traj_handler->traj, traj_handler->cur_segment_id);

    // If we couldnt find the segment then we cant check collision, so be safe
    // and consider it to be in collision
    if (segment_idx < 0)
    {
        fprintf(stderr, "ERROR in %s, Segment with id %d not found\n",
                __FUNCTION__, traj_handler->cur_segment_id);
        return true;
    }

    // Only check for collisions up to the point where we can no longer stop
    poly_segment_t cur_seg = traj_handler->traj.segments[segment_idx];
    double vx = eval_poly_at_t(cur_seg.n_coef, cur_seg.cx, traj_handler->cur_segment_t);
    double vy = eval_poly_at_t(cur_seg.n_coef, cur_seg.cy, traj_handler->cur_segment_t);
    double vz = eval_poly_at_t(cur_seg.n_coef, cur_seg.cz, traj_handler->cur_segment_t);
    double v = pow(pow(vx, 2) + pow(vy, 2) + pow(vz, 2), 0.5);

    // Add 20% margin for safety and make sure we are at least within one collision sample
    double stopping_time = (v / A_MAX) * 1.2;
    stopping_time = collision_sampling_dt > stopping_time ? collision_sampling_dt : stopping_time;

    if (debug)
        log_str += "\nStopping time = " + std::to_string(stopping_time) + "\n\n";

    // Iterate over segments until we have reached stopped time and check for collision
    double start_time = traj_handler->cur_segment_t;
    double total_time = -traj_handler->cur_segment_t;
    double end_time;

    for (int i = segment_idx; i < traj_handler->traj.n_segments; i++)
    {
        total_time += traj_handler->traj.segments[i].duration_s;

        if (total_time >= stopping_time)
        {
            end_time = traj_handler->traj.segments[i + 1].duration_s - (total_time - stopping_time);

            // Check collision
            if (_is_segment_in_collision(&traj_handler->traj.segments[i],
                                         start_time, end_time, R, T))
                return true;

            break;
        }
        else
        {
            // Check collision
            if (_is_segment_in_collision(&traj_handler->traj.segments[i],
                                         start_time, traj_handler->traj.segments[i].duration_s, R, T))
                return true;
        }

        // Reset start time to 0 for all segments after the first
        start_time = 0;
    }

    return false;
}

static void _trajectory_connect_cb(__attribute__((unused)) int ch,
                                   __attribute__((unused)) void *context)
{
    printf("Trajectory Monitor connected to voxl-mapper trajectory\n");
}

static void _trajectory_disconnect_cb(__attribute__((unused)) int ch,
                                      __attribute__((unused)) void *context)
{
    printf("Trajectory Monitor disconnected from voxl-mapper trajectory\n");
}

static void _voa_connect_cb(__attribute__((unused)) int ch,
                            __attribute__((unused)) void *context)
{
    printf("Trajectory Monitor connected to voa pointcloud\n");
}

static void _voa_disconnect_cb(__attribute__((unused)) int ch,
                               __attribute__((unused)) void *context)
{
    printf("Trajectory Monitor disconnected from voa pointcloud\n");
}

void print_traj(trajectory_t *new_traj)
{
    fprintf(stderr, "%d\n", new_traj->n_segments);

    for (int i = 0; i < new_traj->n_segments; i++)
    {
        fprintf(stderr, "%f | %d\n", new_traj->segments->duration_s, new_traj->segments[i].id);
        for (int j = 0; j < new_traj->segments[i].n_coef; j++)
        {
            fprintf(stderr, "%f, ", new_traj->segments[i].cx[j]);
        }
        fprintf(stderr, "\n");

        for (int j = 0; j < new_traj->segments[i].n_coef; j++)
        {
            fprintf(stderr, "%f, ", new_traj->segments[i].cy[j]);
        }
        fprintf(stderr, "\n");

        for (int j = 0; j < new_traj->segments[i].n_coef; j++)
        {
            fprintf(stderr, "%f, ", new_traj->segments[i].cz[j]);
        }
        fprintf(stderr, "\n");
    }

    fprintf(stderr, "\n\n");
}

static void _voa_pipe_helper_cb(__attribute__((unused)) int ch,
                                point_cloud_metadata_t meta, void *data,
                                __attribute__((unused)) void *context)
{
    if (meta.format != POINT_CLOUD_FORMAT_FLOAT_XYZ)
    {
        fprintf(stderr, "ERROR: voa point cloud not of correct type\n");
        return;
    }

    // Get trajectory from the offboard_trajectroy file (returns a copy so no
    // need to worry about mutex locking)
    trajectory_handler traj_handler = get_trajectory_handler();

    // We dont care if the trajectory isnt ready/being used so skip
    if (!traj_handler.is_ready)
        return;

    _setup_octree(static_cast<float *>(data), meta.n_points);

    bool in_col = _is_trajectory_in_collision(&traj_handler, meta.timestamp_ns);

    if (in_col && prev_in_col)
    {
        // Robot is going to crash. Send estop
        fprintf(stderr,
                "Trajectory Monitor detected a future collision. Trigerring "
                "emergency stop\n");
        trigger_estop();
        prev_in_col = false;

        if (debug)
        {
            fprintf(stderr, "%s\n", log_str.c_str());
            print_traj(&traj_handler.traj);
        }
    }
    else if (in_col)
    {
        if (debug)
        {
            fprintf(stderr, "%s\n", log_str.c_str());
            print_traj(&traj_handler.traj);
        }

        prev_in_col = true;
    }
    else
        prev_in_col = false;

    if (debug)
        log_str = "";
}

int trajectory_monitor_init()
{
    voa_ch = pipe_client_get_next_available_channel();
    pipe_client_set_connect_cb(voa_ch, _voa_connect_cb, nullptr);
    pipe_client_set_disconnect_cb(voa_ch, _voa_disconnect_cb, nullptr);
    pipe_client_set_point_cloud_helper_cb(voa_ch, _voa_pipe_helper_cb, nullptr);
    pipe_client_open(voa_ch, VOA_PC_OUT_LOCATION, PIPE_CLIENT_NAME,
                     CLIENT_FLAG_EN_POINT_CLOUD_HELPER, VOA_READ_BUF_SIZE);

    return 0;
}

int trajectory_monitor_stop()
{
    pipe_client_close(voa_ch);
    return 0;
}
