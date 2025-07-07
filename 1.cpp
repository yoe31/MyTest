#define MAX_TRAJECTORIES 100

typedef struct {
    uint32_t obj_id;  // 객체 ID
    int active;       // 1: 유효, 0: 삭제됨
    int miss_count;
    double last_timestamp;
    TrajectoryPoint_t points[MAX_HISTORY];
    int length;
} Trajectory_t;

Trajectory_t trajectories[MAX_TRAJECTORIES];
int num_trajectories = 0;

int find_trajectory_index(uint32_t obj_id) {
    for (int i = 0; i < num_trajectories; ++i) {
        if (trajectories[i].obj_id == obj_id && trajectories[i].active)
            return i;
    }
    return -1;  // 없음
}

int create_trajectory(uint32_t obj_id, double timestamp) {
    if (num_trajectories >= MAX_TRAJECTORIES)
        return -1;  // 초과

    trajectories[num_trajectories].obj_id = obj_id;
    trajectories[num_trajectories].active = 1;
    trajectories[num_trajectories].miss_count = 0;
    trajectories[num_trajectories].last_timestamp = timestamp;
    trajectories[num_trajectories].length = 0;
    return num_trajectories++;
}

void process_objects(const DR_t* dr, const Object_t* objs, int num_obj) {
    int found_indices[MAX_TRAJECTORIES] = {0};

    for (int i = 0; i < num_obj; ++i) {
        if (objs[i].confidence < 70) continue;
        if (objs[i].obj_type == 2 || objs[i].obj_type == 3 || objs[i].obj_type == 4 || objs[i].obj_type == 8) continue;

        uint32_t obj_id = objs[i].obj_id;

        int idx = find_trajectory_index(obj_id);
        if (idx == -1) {
            idx = create_trajectory(obj_id, dr->timestamp);
            if (idx == -1) continue;  // 용량 초과
        }

        found_indices[idx] = 1;
        trajectories[idx].miss_count = 0;
        trajectories[idx].last_timestamp = dr->timestamp;

        float gx, gy;
        transform_to_global(dr, objs[i].x, objs[i].y, &gx, &gy);

        int len = trajectories[idx].length;
        if (len < MAX_HISTORY) {
            trajectories[idx].points[len].timestamp = dr->timestamp;
            trajectories[idx].points[len].obj_id = obj_id;
            trajectories[idx].points[len].global_x = gx;
            trajectories[idx].points[len].global_y = gy;
            trajectories[idx].points[len].heading_deg = objs[i].heading_deg;
            trajectories[idx].length++;
        }
    }

    // === 인식되지 않은 객체 처리 ===
    for (int i = 0; i < num_trajectories; ++i) {
        if (!trajectories[i].active) continue;
        if (found_indices[i]) continue;

        trajectories[i].miss_count++;

        if (trajectories[i].length > 0) {
            TrajectoryPoint_t* last = &trajectories[i].points[trajectories[i].length - 1];
            float lx = last->global_x - dr->posX;
            float ly = last->global_y - dr->posY;

            if (is_out_of_range(dr, lx, ly)) {
                trajectories[i].active = 0;
                trajectories[i].length = 0;
                continue;
            }
        }

        if (trajectories[i].miss_count >= MAX_MISS_COUNT) {
            trajectories[i].active = 0;
            trajectories[i].length = 0;
        }
    }
}
