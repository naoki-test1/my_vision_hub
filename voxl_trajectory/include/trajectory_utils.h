#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

#include "trajectory_interface.h"
#include "stdbool.h"
#include <string.h>

static void trim_trajectory(trajectory_t *traj, int segment_id)
{
    int first_idx_to_keep = 0;
    for (int i = 0; i < traj->n_segments; i++)
    {
        // Skip segments until we reach the one that contains our current id
        if (segment_id == traj->segments[i].id)
        {
            first_idx_to_keep = i;
            break;
        }
    }

    // Do nothing if no segments to remove
    if (first_idx_to_keep == 0)
        return;

    // Fix number of segments then move all unevaluated segments to the front of
    // the array
    traj->n_segments -= first_idx_to_keep;
    memmove(traj->segments, traj->segments + first_idx_to_keep, sizeof(poly_segment_t) * traj->n_segments);
}

static bool insert_trajectory(trajectory_t *dst, trajectory_t *src)
{
    // Check that there is enough space to put our segment in
    if (dst->n_segments + src->n_segments >= TRAJ_MAX_SEGMENTS)
    {
        fprintf(stderr, "ERROR: Cannot insert trajectory. Total segments = %d, Max segments = %d\n", dst->n_segments + src->n_segments, TRAJ_MAX_SEGMENTS);
        return false;
    }

    // Check that the segment id to split at actually exists
    int segment_idx = -1;
    for (int i = 0; i < dst->n_segments; i++)
    {
        if (dst->segments[i].id == src->segment_split_id)
        {
            segment_idx = i;
            break;
        }
    }

    if (segment_idx < 0)
    {
        fprintf(stderr, "ERROR: Cannot insert trajectory. Segment id %d does not exist\n", src->segment_split_id);
        return false;
    }

    // Check that the segment time to split at is valid for the splitting segment
    if (dst->segments[segment_idx].duration_s < src->segment_split_time)
    {
        fprintf(stderr, "ERROR: Cannot insert trajectory. Segment split time %f occurs past end of splitting segment (duration = %f)\n", src->segment_split_time, dst->segments[segment_idx].duration_s);
        return false;
    }

    // Fix the duration of the split segment, n_segments in dst, and the actual segments themselves
    dst->segments[segment_idx].duration_s = src->segment_split_time;
    dst->n_segments = segment_idx + 1 + src->n_segments;
    memcpy(dst->segments + segment_idx + 1, src->segments, sizeof(poly_segment_t) * src->n_segments);

    return true;
}

static int get_segment_idx(trajectory_t *traj, int segment_id)
{
    // Iterate over all segments and return the segment that matches the id
    for (int i = 0; i < traj->n_segments; i++)
    {
        // Skip segments until we reach the one that contains the cur segment id
        if (traj->segments[i].id == segment_id)
        {
            return i;
        }
    }

    return -1;
}

#endif