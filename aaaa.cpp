#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_OBJECTS 100
#define MAX_TRAJECTORY_LENGTH 50
#define MAX_MISSING_COUNT 10
#define DEG2RAD(x) ((x) * 3.14159265f / 180.0f)
#define NOISE_ALPHA 0.2f  // low-pass filter weight

// 차량 인식 정보
typedef struct {
    int id;
    float x;
    float y;
    float heading;
    float velocity;
} DetectedObject;

// 추적 중인 차량 정보
typedef struct {
    int id;
    float trajX[MAX_TRAJECTORY_LENGTH];
    float trajY[MAX_TRAJECTORY_LENGTH];
    int length;
    int missing_count;
    float last_heading;
} TrackedObject;

// 글로벌 추적 목록
TrackedObject tracked[MAX_OBJECTS];
int tracked_count = 0;

// ------------------------------------------------------------
// 유틸리티 함수

// 각도 차이 (0~180도 사이 절댓값)
float angle_diff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return fabsf(d);
}

// 노이즈 완화 (1차 저역통과필터)
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

            // 좌표 회전 및 ego 이동 보정
            float new_x = cos_a * x - sin_a * y - deltaX;
            float new_y = sin_a * x + cos_a * y - deltaY;

            tracked[i].trajX[j] = new_x;
            tracked[i].trajY[j] = new_y;
        }
    }
}

// ------------------------------------------------------------
// 인식 차량 처리 및 추적 목록 업데이트
void process_detected_objects(DetectedObject *objs, int num_objs, float ego_heading) {
    int updated_ids[MAX_OBJECTS] = {0};

    for (int i = 0; i < num_objs; i++) {
        DetectedObject *obj = &objs[i];

        // 진행 방향이 너무 다르면 무시
        if (angle_diff(obj->heading, ego_heading) > 30.0f)
            continue;

        int found = 0;
        for (int j = 0; j < tracked_count; j++) {
            if (tracked[j].id == obj->id) {
                found = 1;

                // 기존 좌표 갱신 (노이즈 필터)
                apply_low_pass(&tracked[j].trajX[0], obj->x);
                apply_low_pass(&tracked[j].trajY[0], obj->y);

                // 새로운 위치 추가
                if (tracked[j].length < MAX_TRAJECTORY_LENGTH) {
                    tracked[j].trajX[tracked[j].length] = obj->x;
                    tracked[j].trajY[tracked[j].length] = obj->y;
                    tracked[j].length++;
                }

                tracked[j].missing_count = 0;
                tracked[j].last_heading = obj->heading;
                updated_ids[j] = 1;
                break;
            }
        }

        // 신규 객체 등록
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

    // 누락된 객체 누락 횟수 증가
    for (int i = 0; i < tracked_count; i++) {
        if (!updated_ids[i]) {
            tracked[i].missing_count++;
        }
    }

    // 10번 연속 누락된 객체 삭제
    int i = 0;
    while (i < tracked_count) {
        if (tracked[i].missing_count >= MAX_MISSING_COUNT) {
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
// 주기적 업데이트 호출 함수
void cycle(float deltaX, float deltaY, float deltaTheta,
           DetectedObject *objs, int obj_num, float ego_heading) {
    update_ego_motion(deltaX, deltaY, deltaTheta);
    process_detected_objects(objs, obj_num, ego_heading);
}

// ------------------------------------------------------------
// 디버그용: 궤적 출력
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
// 테스트용 메인 함수 (예시)
int main() {
    DetectedObject frame1[] = {
        {1, 10.0f, 2.0f, 0.0f, 5.0f},
        {2, 5.0f, -1.0f, 5.0f, 4.0f},
        {3, -3.0f, 1.0f, 180.0f, 3.0f}  // 반대 방향: 무시됨
    };

    cycle(0.0f, 0.0f, 0.0f, frame1, 3, 0.0f);
    print_tracked();

    // 다음 주기, 자차가 오른쪽(진행방향)으로 1.0 이동
    DetectedObject frame2[] = {
        {1, 11.0f, 2.2f, 1.0f, 5.0f},
        {2, 6.0f, -1.0f, 3.0f, 4.2f}
    };

    cycle(1.0f, 0.0f, 0.0f, frame2, 2, 0.0f);
    print_tracked();

    return 0;
}