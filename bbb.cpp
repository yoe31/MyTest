#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_OBJECTS 100
#define MAX_TRAJECTORY_LENGTH 100
#define MAX_MISSING_COUNT 10
#define DEG2RAD(x) ((x) * 3.14159265f / 180.0f)
#define NOISE_ALPHA 0.2f
#define TRACKING_DISTANCE_LIMIT 50.0f

typedef struct {
    int id;
    float x;
    float y;
    float heading;
    float velocity;
} DetectedObject;

typedef struct {
    int id;
    float trajX[MAX_TRAJECTORY_LENGTH];
    float trajY[MAX_TRAJECTORY_LENGTH];
    int length;
    int missing_count;
    float last_heading;
} TrackedObject;

TrackedObject tracked[MAX_OBJECTS];
int tracked_count = 1;  // tracked[0]는 자차 궤적용

// ------------------------------------------------------------
float angle_diff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return fabsf(d);
}

void apply_low_pass(float *old_value, float new_value) {
    *old_value = NOISE_ALPHA * new_value + (1.0f - NOISE_ALPHA) * (*old_value);
}

// ------------------------------------------------------------
// 자차 이동 반영
void update_ego_motion(float deltaX, float deltaY, float deltaTheta) {
    float angle = DEG2RAD(deltaTheta);
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);

    for (int i = 0; i < tracked_count; i++) {
        for (int j = 0; j < tracked[i].length; j++) {
            float x = tracked[i].trajX[j];
            float y = tracked[i].trajY[j];
            float new_x = cos_a * x - sin_a * y - deltaX;
            float new_y = sin_a * x + cos_a * y - deltaY;
            tracked[i].trajX[j] = new_x;
            tracked[i].trajY[j] = new_y;
        }
    }
}

// ------------------------------------------------------------
// 자차 궤적 최신 포인트 추가 (최신이 0번 인덱스)
void update_ego_trajectory() {
    TrackedObject *ego = &tracked[0];
    if (ego->length < MAX_TRAJECTORY_LENGTH) ego->length++;
    for (int i = ego->length - 1; i > 0; i--) {
        ego->trajX[i] = ego->trajX[i - 1];
        ego->trajY[i] = ego->trajY[i - 1];
    }
    ego->trajX[0] = 0.0f;
    ego->trajY[0] = 0.0f;
}

// ------------------------------------------------------------
// 인식된 객체 처리 및 tracking 관리
void process_detected_objects(DetectedObject *objs, int num_objs, float ego_heading) {
    int updated_ids[MAX_OBJECTS] = {0};

    for (int i = 0; i < num_objs; i++) {
        DetectedObject *obj = &objs[i];
        if (angle_diff(obj->heading, ego_heading) > 30.0f) continue;

        int found = 0;
        for (int j = 1; j < tracked_count; j++) {
            if (tracked[j].id == obj->id || (tracked[j].id == -1 && tracked[j].missing_count >= MAX_MISSING_COUNT)) {
                found = 1;

                if (tracked[j].id == -1) {
                    tracked[j].id = obj->id; // 재활성화
                    tracked[j].missing_count = 0;
                }

                apply_low_pass(&tracked[j].trajX[0], obj->x);
                apply_low_pass(&tracked[j].trajY[0], obj->y);

                // 뒤로 밀고 0번에 새 점 추가
                if (tracked[j].length < MAX_TRAJECTORY_LENGTH) tracked[j].length++;
                for (int k = tracked[j].length - 1; k > 0; k--) {
                    tracked[j].trajX[k] = tracked[j].trajX[k - 1];
                    tracked[j].trajY[k] = tracked[j].trajY[k - 1];
                }
                tracked[j].trajX[0] = obj->x;
                tracked[j].trajY[0] = obj->y;

                tracked[j].last_heading = obj->heading;
                tracked[j].missing_count = 0;
                updated_ids[j] = 1;
                break;
            }
        }

        if (!found && tracked_count < MAX_OBJECTS) {
            TrackedObject *newObj = &tracked[tracked_count++];
            memset(newObj, 0, sizeof(TrackedObject));
            newObj->id = obj->id;
            newObj->trajX[0] = obj->x;
            newObj->trajY[0] = obj->y;
            newObj->length = 1;
            newObj->last_heading = obj->heading;
        }
    }

    // 미갱신 객체 처리
    for (int i = 1; i < tracked_count; i++) {
        if (!updated_ids[i]) {
            tracked[i].missing_count++;
            if (tracked[i].missing_count >= MAX_MISSING_COUNT) {
                tracked[i].id = -1;  // 추적 중단 표시
            }
        }
    }
}

// ------------------------------------------------------------
// 자차 기준 ±50m 벗어난 점 제거, 궤적 길이 0이면 삭제 (자차 제외)
void filter_trajectories() {
    int i = 1;
    while (i < tracked_count) {
        TrackedObject *obj = &tracked[i];
        int k = 0;
        for (int j = 0; j < obj->length; j++) {
            if (fabsf(obj->trajX[j]) <= TRACKING_DISTANCE_LIMIT) {
                obj->trajX[k] = obj->trajX[j];
                obj->trajY[k] = obj->trajY[j];
                k++;
            }
        }
        obj->length = k;

        if (obj->length == 0 && obj->id != -1) {
            for (int j = i; j < tracked_count - 1; j++) {
                tracked[j] = tracked[j + 1];
            }
            tracked_count--;
        } else {
            i++;
        }
    }
}

// ------------------------------------------------------------
// 주기 실행 함수
void cycle(float deltaX, float deltaY, float deltaTheta,
           DetectedObject *objs, int obj_num, float ego_heading) {
    update_ego_motion(deltaX, deltaY, deltaTheta);
    update_ego_trajectory();
    process_detected_objects(objs, obj_num, ego_heading);
    filter_trajectories();
}

// ------------------------------------------------------------
// 디버그 출력
void print_tracked() {
    printf("=== Tracked Vehicles ===\n");
    for (int i = 0; i < tracked_count; i++) {
        printf("ID %d: ", tracked[i].id);
        for (int j = 0; j < tracked[i].length; j++) {
            printf("(%.1f, %.1f) ", tracked[i].trajX[j], tracked[i].trajY[j]);
        }
        printf("\n");
    }
}

// ------------------------------------------------------------
int main() {
    // 자차 궤적 초기화
    tracked[0].id = 0;
    tracked[0].length = 0;

    DetectedObject frame1[] = {
        {1, 10.0f, 1.0f, 0.0f, 3.0f},
        {2, -5.0f, -2.0f, 180.0f, 2.0f}  // 반대 방향 → 무시
    };
    cycle(0.0f, 0.0f, 0.0f, frame1, 2, 0.0f);
    print_tracked();

    DetectedObject frame2[] = {
        {1, 11.0f, 1.2f, 1.0f, 3.2f}
    };
    cycle(1.0f, 0.0f, 0.0f, frame2, 1, 0.0f);
    print_tracked();

    // 10회 이상 미검출 → 추적 중단(id = -1)
    for (int i = 0; i < 11; i++) {
        cycle(1.0f, 0.0f, 0.0f, NULL, 0, 0.0f);
    }
    print_tracked();

    return 0;
}