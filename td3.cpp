#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>
#include <tuple>
#include <algorithm>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* new_array = new double[new_size];
    for (int i = 0; i < length; i++)
    {
        new_array[i] = array[i];
    }
    delete[] array;
    return new_array;
}

double* shrink_array(double* array, int length, int new_size) {
    double* new_array = new double[new_size];
    for (int i = 0; i < new_size; i++)
    {
        new_array[i] = array[i];
    }
    delete[] array;
    return new_array;
}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if (current_size == max_size)
    {
        max_size += 5;
        array = extend_array(array, current_size, max_size);
    }
    array[current_size++] = element;
    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    if (current_size == 0)
    {
        return nullptr;
    }
    current_size--;
    array[current_size] = 0;
    if (max_size - current_size >= 5)
    {
        array = shrink_array(array, max_size, current_size);
        max_size -= 5;
    }
    return array;
}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
    bool hit_target, hit_obstacle;
    double v0_x, v0_y, x, y, t;
    double PI = 3.14159265;
    double g = 9.8;

    v0_x = magnitude * cos(angle * PI / 180);
    v0_y = magnitude * sin(angle * PI / 180);

    t = 0;
    x = 0;
    y = 0;

    hit_target = false;
    hit_obstacle = false;
    while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
        telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

        double * target_coordinates = find_collision(x, y, targets, tot_targets);
        if (target_coordinates != NULL) {
            remove_target(targets, tot_targets, target_coordinates);
            hit_target = true;
        }
        else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
            hit_obstacle = true;
        }
        else {
            t = t + simulation_interval;
            y = v0_y * t  - 0.5 * g * t * t;
            x = v0_x * t;
        }
    }

    return hit_target;
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
    int tot_entries = 0;
    for (int i = 0; i < tot_telemetries; i++)
    {
        tot_entries += telemetries_sizes[i];
    }
    if (tot_entries == 0)
    {
        return;
    }
    tuple<double, double, double> temp[tot_entries];
    tot_entries = 0;
    for (int i = 0; i < tot_telemetries; i++)
    {
        for (int j = 0; j < telemetries_sizes[i]; j += 3)
        {
            temp[tot_entries++] = make_tuple(telemetries[i][j], telemetries[i][j+1], telemetries[i][j+2]);
        }
    }
    sort(temp, temp + tot_entries);
    for (int i = 0; i < tot_entries; i++)
    {
        global_telemetry = append_to_array(get<0>(temp[i]), global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
        global_telemetry = append_to_array(get<1>(temp[i]), global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
        global_telemetry = append_to_array(get<2>(temp[i]), global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
    }
}
