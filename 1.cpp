#define MAX_OBJ 40
#define MAX_HISTORY 10000

typedef struct {
    double timestamp;
    double lat, lon, heading;  // deg
    float posX, posY, posTheta;
} DR_t;

typedef struct {
    uint32_t obj_id;
    uint8_t moving_status;
    uint8_t confidence;
    float update_age;
    uint8_t obj_type;
    float x;  // 자차 기준
    float y;
    float heading_deg;
    float velocity;
    float width;
    float length;
} Object_t;

typedef struct {
    double timestamp;
    uint32_t obj_id;
    float global_x;
    float global_y;
    float heading_deg;
} TrajectoryPoint_t;

TrajectoryPoint_t traj_history[MAX_OBJ][MAX_HISTORY];
int traj_length[MAX_OBJ] = {0};

// 자차 기준 좌표 → 전역 좌표 변환 (단순 변환 예시)
void transform_to_global(const DR_t* dr, float local_x, float local_y, float* out_x, float* out_y) {
    float theta = dr->heading * M_PI / 180.0;  // Heading은 CW 기준
    *out_x = dr->posX + local_x * cos(theta) - local_y * sin(theta);
    *out_y = dr->posY + local_x * sin(theta) + local_y * cos(theta);
}

// 유효 객체만 필터링 및 궤적 저장
void process_objects(const DR_t* dr, const Object_t* objs, int num_obj) {
    for (int i = 0; i < num_obj; ++i) {
        if (objs[i].confidence < 70) continue;
        if (objs[i].obj_type == 2 || objs[i].obj_type == 3 || objs[i].obj_type == 4 || objs[i].obj_type == 8) continue;

        uint32_t id = objs[i].obj_id;
        if (id >= MAX_OBJ) continue;

        float gx, gy;
        transform_to_global(dr, objs[i].x, objs[i].y, &gx, &gy);

        int len = traj_length[id];
        if (len < MAX_HISTORY) {
            traj_history[id][len].timestamp = dr->timestamp;
            traj_history[id][len].obj_id = id;
            traj_history[id][len].global_x = gx;
            traj_history[id][len].global_y = gy;
            traj_history[id][len].heading_deg = objs[i].heading_deg;
            traj_length[id]++;
        }
    }
}