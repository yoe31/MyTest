#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_OBJECTS 100
#define MAX_TRAJECTORY_LENGTH 100
#define DETECTION_LOST_THRESHOLD 10
#define TRACKING_DISTANCE_LIMIT 50.0f

typedef struct {
    int id;
    float x;
    float y;
    float heading;
    float velocity;
} DetectedObject;

typedef struct {
    int id; // -1이면 tracking 멈춤 상태
    float trajX[MAX_TRAJECTORY_LENGTH];
    float trajY[MAX_TRAJECTORY_LENGTH];
    int length;
    int lost_count;
    float last_heading;
} TrackedObject;

TrackedObject tracked[MAX_OBJECTS];
int tracked_count = 1; // index 0은 자차

// 자차의 이동량을 모든 객체에 반영
void update_ego_motion(float deltaX, float deltaY, float deltaTheta) {
    for (int i = 0; i < tracked_count; i++) {
        TrackedObject *obj = &tracked[i];
        for (int j = 0; j < obj->length; j++) {
            float x = obj->trajX[j];
            float y = obj->trajY[j];
            // 회전 및 이동 변환
            float nx = cosf(deltaTheta) * x - sinf(deltaTheta) * y - deltaX;
            float ny = sinf(deltaTheta) * x + cosf(deltaTheta) * y - deltaY;
            obj->trajX[j] = nx;
            obj->trajY[j] = ny;
        }
    }
}

// 자차 궤적 업데이트 (최신이 0번 인덱스)
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

// 탐지 결과 기반으로 트래킹 객체 갱신
void update_tracking(DetectedObject *objs, int num_objs) {
    for (int i = 0; i < num_objs; i++) {
        DetectedObject *obj = &objs[i];
        int found = 0;
        for (int j = 1; j < tracked_count; j++) {
            if (tracked[j].id == obj->id) {
                found = 1;
                if (fabsf(tracked[j].last_heading - obj->heading) < M_PI / 6) {
                    if (tracked[j].length < MAX_TRAJECTORY_LENGTH) tracked[j].length++;
                    for (int k = tracked[j].length - 1; k > 0; k--) {
                        tracked[j].trajX[k] = tracked[j].trajX[k - 1];
                        tracked[j].trajY[k] = tracked[j].trajY[k - 1];
                    }
                    tracked[j].trajX[0] = obj->x;
                    tracked[j].trajY[0] = obj->y;
                }
                tracked[j].lost_count = 0;
                tracked[j].last_heading = obj->heading;
                break;
            }
        }
        if (!found && tracked_count < MAX_OBJECTS) {
            if (tracked_count < MAX_OBJECTS) {
                TrackedObject *newObj = &tracked[tracked_count++];
                memset(newObj, 0, sizeof(TrackedObject));
                newObj->id = obj->id;
                newObj->trajX[0] = obj->x;
                newObj->trajY[0] = obj->y;
                newObj->length = 1;
                newObj->last_heading = obj->heading;
            }
        }
    }

    // 미탐지 카운트 증가 및 ID -1 처리
    for (int i = 1; i < tracked_count; i++) {
        int still_detected = 0;
        for (int j = 0; j < num_objs; j++) {
            if (tracked[i].id == objs[j].id) {
                still_detected = 1;
                break;
            }
        }
        if (!still_detected) {
            tracked[i].lost_count++;
            if (tracked[i].lost_count >= DETECTION_LOST_THRESHOLD) {
                tracked[i].id = -1; // 추적 정지 상태
            }
        }
    }
}

// ±50m x축 필터 적용
void filter_trajectories() {
    for (int i = 0; i < tracked_count; i++) {
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
    }

    // Trajectory가 0개가 되면 삭제 (자차 제외)
    int new_count = 1;
    for (int i = 1; i < tracked_count; i++) {
        if (tracked[i].length > 0) {
            if (i != new_count) {
                tracked[new_count] = tracked[i];
            }
            new_count++;
        }
    }
    tracked_count = new_count;
}

// 디버깅 출력
void print_tracked_objects() {
    printf("Tracked objects: %d\n", tracked_count);
    for (int i = 0; i < tracked_count; i++) {
        printf("ID: %d, Points: %d\n", tracked[i].id, tracked[i].length);
        for (int j = 0; j < tracked[i].length; j++) {
            printf("  (%.2f, %.2f)\n", tracked[i].trajX[j], tracked[i].trajY[j]);
        }
    }
}

int main() {
    // 자차 초기화
    tracked[0].id = 0;
    tracked[0].length = 0;

    // 예시: 한 사이클
    float deltaX = 1.0f, deltaY = 0.0f, deltaTheta = 0.0f;
    update_ego_motion(deltaX, deltaY, deltaTheta);
    update_ego_trajectory();

    DetectedObject objects[2] = {
        {101, 5.0f, 1.0f, 0.1f, 10.0f},
        {102, 10.0f, -1.5f, 0.2f, 12.0f}
    };

    update_tracking(objects, 2);
    filter_trajectories();
    print_tracked_objects();

    return 0;
}