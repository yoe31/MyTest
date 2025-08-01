#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define UNCLASSIFIED -1
#define NOISE -2
#define MAX_POINTS 1000  // 필요시 조정

typedef struct {
    double x, y;
    int cluster_id;
} Point;

int minPts = 3;
double eps = 2.0;

double distance(Point* a, Point* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    return sqrt(dx * dx + dy * dy);
}

int regionQuery(Point* points, int num_points, int index, int* neighbors) {
    int count = 0;
    for (int i = 0; i < num_points; ++i) {
        if (distance(&points[index], &points[i]) <= eps) {
            neighbors[count++] = i;
        }
    }
    return count;
}

int expandCluster(Point* points, int num_points, int index, int cluster_id) {
    int neighbors[MAX_POINTS];
    int neighbor_count = regionQuery(points, num_points, index, neighbors);

    if (neighbor_count < minPts) {
        points[index].cluster_id = NOISE;
        return 0;
    }

    points[index].cluster_id = cluster_id;

    for (int i = 0; i < neighbor_count; ++i) {
        int neighbor_index = neighbors[i];

        if (points[neighbor_index].cluster_id == UNCLASSIFIED || points[neighbor_index].cluster_id == NOISE) {
            points[neighbor_index].cluster_id = cluster_id;

            int neighbor_neighbors[MAX_POINTS];
            int nn_count = regionQuery(points, num_points, neighbor_index, neighbor_neighbors);

            if (nn_count >= minPts) {
                for (int j = 0; j < nn_count; ++j) {
                    int n_idx = neighbor_neighbors[j];
                    // 중복 방지
                    int already_in = 0;
                    for (int k = 0; k < neighbor_count; ++k) {
                        if (neighbors[k] == n_idx) {
                            already_in = 1;
                            break;
                        }
                    }
                    if (!already_in && neighbor_count < MAX_POINTS) {
                        neighbors[neighbor_count++] = n_idx;
                    }
                }
            }
        }
    }

    return 1;
}

void dbscan(Point* points, int num_points) {
    int cluster_id = 1;

    for (int i = 0; i < num_points; ++i) {
        if (points[i].cluster_id == UNCLASSIFIED) {
            if (expandCluster(points, num_points, i, cluster_id)) {
                cluster_id++;
            }
        }
    }
}

int main() {
    Point points[MAX_POINTS] = {
        {0.0, 0.0, UNCLASSIFIED}, {0.2, 0.1, UNCLASSIFIED}, {0.3, -0.2, UNCLASSIFIED},
        {10.0, 10.0, UNCLASSIFIED}, {10.1, 10.2, UNCLASSIFIED}, {9.9, 10.1, UNCLASSIFIED},
        {50.0, 50.0, UNCLASSIFIED}
    };

    int num_points = 7;

    eps = 1.0;
    minPts = 2;

    dbscan(points, num_points);

    for (int i = 0; i < num_points; ++i) {
        printf("Point (%.2f, %.2f) -> Cluster %d\n", points[i].x, points[i].y, points[i].cluster_id);
    }

    return 0;
}