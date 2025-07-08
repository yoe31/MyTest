#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_TRAJECTORIES 100
#define MAX_HISTORY 10000
#define MAX_CLUSTER 50

typedef struct {
    double timestamp;
    uint32_t obj_id;
    float global_x;
    float global_y;
    float heading_deg;
    float velocity; // optional
} TrajectoryPoint_t;

typedef struct {
    uint32_t obj_id;
    int active;
    int miss_count;
    double last_timestamp;
    TrajectoryPoint_t points[MAX_HISTORY];
    int length;
} Trajectory_t;

typedef struct {
    float mean_x, mean_y;
    float mean_heading;
    float mean_velocity;
    int traj_index;
} TrajFeature_t;

typedef enum {
    ROAD_CITY,
    ROAD_HIGHWAY
} RoadType;

typedef struct {
    float max_heading_diff;
    float max_lateral_dist;
    float max_velocity_diff;
} MergeThresholds;

Trajectory_t trajectories[MAX_TRAJECTORIES];
int num_trajectories = 0;

TrajFeature_t features[MAX_TRAJECTORIES];
int cluster_id[MAX_TRAJECTORIES];  // -1로 초기화 필요

// === [1] 고속도로/도심 병합 기준 설정 ===
MergeThresholds get_thresholds(RoadType type) {
    MergeThresholds t;
    if (type == ROAD_HIGHWAY) {
        t.max_heading_diff = 10.0f;
        t.max_lateral_dist = 1.5f;
        t.max_velocity_diff = 3.0f;
    } else {
        t.max_heading_diff = 15.0f;
        t.max_lateral_dist = 2.0f;
        t.max_velocity_diff = 5.0f;
    }
    return t;
}

// === [2] 궤적에서 특징 추출 ===
void compute_features() {
    for (int i = 0; i < num_trajectories; ++i) {
        Trajectory_t* traj = &trajectories[i];
        if (!traj->active || traj->length < 2) continue;

        float sum_x = 0, sum_y = 0, sum_h = 0, sum_v = 0;
        for (int j = 0; j < traj->length; ++j) {
            sum_x += traj->points[j].global_x;
            sum_y += traj->points[j].global_y;
            sum_h += traj->points[j].heading_deg;
            sum_v += traj->points[j].velocity;
        }

        features[i].mean_x = sum_x / traj->length;
        features[i].mean_y = sum_y / traj->length;
        features[i].mean_heading = sum_h / traj->length;
        features[i].mean_velocity = sum_v / traj->length;
        features[i].traj_index = i;
    }
}

// === [3] 차선 변경 감지 ===
int is_lane_change(const Trajectory_t* traj) {
    if (traj->length < 3) return 0;

    for (int i = 2; i < traj->length; ++i) {
        float y1 = traj->points[i - 2].global_y;
        float y2 = traj->points[i].global_y;
        float dy = fabs(y2 - y1);

        float h1 = traj->points[i - 2].heading_deg;
        float h2 = traj->points[i].heading_deg;
        float dh = fabs(h2 - h1);
        if (dh > 180) dh = 360 - dh;

        if (dy > 2.5 || dh > 15.0) {
            return 1;
        }
    }

    return 0;
}

// === [4] 유사 궤적 판단 ===
int is_similar_trajectory(
    const TrajFeature_t* a, const TrajFeature_t* b,
    MergeThresholds t
) {
    float dx = a->mean_x - b->mean_x;
    float dy = a->mean_y - b->mean_y;
    float dist = sqrt(dx * dx + dy * dy);

    float dh = fabs(a->mean_heading - b->mean_heading);
    if (dh > 180) dh = 360 - dh;

    float dv = fabs(a->mean_velocity - b->mean_velocity);

    return (dist < t.max_lateral_dist &&
            dh < t.max_heading_diff &&
            dv < t.max_velocity_diff);
}

// === [5] 병합 (클러스터 ID 설정) ===
int merge_trajectories(RoadType road_type) {
    MergeThresholds t = get_thresholds(road_type);
    int cluster_count = 0;

    for (int i = 0; i < num_trajectories; ++i) {
        cluster_id[i] = -1;
    }

    for (int i = 0; i < num_trajectories; ++i) {
        if (!trajectories[i].active || is_lane_change(&trajectories[i]))
            continue;

        if (cluster_id[i] != -1) continue;
        cluster_id[i] = cluster_count;

        for (int j = i + 1; j < num_trajectories; ++j) {
            if (!trajectories[j].active || is_lane_change(&trajectories[j]))
                continue;
            if (cluster_id[j] != -1) continue;

            if (is_similar_trajectory(&features[i], &features[j], t)) {
                cluster_id[j] = cluster_count;
            }
        }

        cluster_count++;
    }

    return cluster_count;
}

// === [6] 대표 중심선 계산 ===
void print_representative_trajectories(int total_clusters) {
    for (int cid = 0; cid < total_clusters; ++cid) {
        printf("Cluster %d: Center line (x, y):\n", cid);

        // 평균 길이보다 짧은 건 패스
        float sum_x[MAX_HISTORY] = {0};
        float sum_y[MAX_HISTORY] = {0};
        int count[MAX_HISTORY] = {0};
        int max_len = 0;

        for (int i = 0; i < num_trajectories; ++i) {
            if (cluster_id[i] != cid) continue;

            Trajectory_t* traj = &trajectories[i];
            if (traj->length > max_len) max_len = traj->length;

            for (int j = 0; j < traj->length; ++j) {
                sum_x[j] += traj->points[j].global_x;
                sum_y[j] += traj->points[j].global_y;
                count[j]++;
            }
        }

        for (int j = 0; j < max_len; ++j) {
            if (count[j] > 0) {
                float avg_x = sum_x[j] / count[j];
                float avg_y = sum_y[j] / count[j];
                printf("    %.2f, %.2f\n", avg_x, avg_y);
            }
        }
        printf("\n");
    }
}