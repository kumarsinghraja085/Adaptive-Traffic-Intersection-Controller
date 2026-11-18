#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MIN_GREEN 10
#define MAX_GREEN 60
#define DEFAULT_CYCLE_TIME 120
#define VEHICLE_PASS_RATE 2 // vehicles passing per green second

// ==========================================
// Data Structures
// ==========================================

typedef enum {
    RED,
    YELLOW,
    GREEN
} SignalState;

typedef struct {
    int lane_id;
    int vehicle_count;
    int max_capacity;
} Lane;

typedef struct {
    int sensor_id;
    int detected_queue;
    float avg_speed;
} Sensor;

typedef struct {
    int approach_id;
    char name[30];
    int num_lanes;
    Lane *lanes;             // Dynamic array of lanes
    Sensor sensor;
    SignalState signal;
    int green_time_allocated;
    int vehicles_passed_total;
    int total_waiting_time;  // Sum of waiting steps across all vehicles
} Approach;

typedef struct {
    int timestamp;
    int approach_id;
    char phase_name[10];
    int vehicles_cleared;
} EventLog;

typedef struct {
    int num_approaches;
    Approach **approaches;   // Dynamic pointer array for runtime configuration
    int **lane_map;          // 2D Array for lane-to-intersection routing matrix
    EventLog *logs;
    int log_count;
    int log_capacity;
} Intersection;

// ==========================================
// Function Prototypes
// ==========================================

Intersection *init_intersection(int num_approaches);
void free_intersection(Intersection *inter);
void add_approach(Intersection *inter, const char *name, int num_lanes);
void remove_approach(Intersection *inter, int approach_id);
void log_event(Intersection *inter, int time, int approach_id, const char *phase, int cleared);
void calculate_adaptive_timings(Intersection *inter, int total_cycle);
void generate_sensor_traffic(Intersection *inter);
void run_simulation(Intersection *inter, int cycles, int mode, const char *scenario_name);
void display_metrics(Intersection *inter, int total_simulation_time, const char *controller_type);
void save_report_to_file(const char *filename, Intersection *inter, int sim_time, const char *mode_str);

// ==========================================
// Memory & Intersection Management
// ==========================================

Intersection *init_intersection(int num_approaches) {
    Intersection *inter = (Intersection *)malloc(sizeof(Intersection));
    if (!inter) return NULL;

    inter->num_approaches = 0;
    inter->approaches = NULL;
    inter->log_count = 0;
    inter->log_capacity = 100;
    inter->logs = (EventLog *)malloc(inter->log_capacity * sizeof(EventLog));

    // Dynamic 2D routing matrix initialization
    inter->lane_map = (int **)malloc(num_approaches * sizeof(int *));
    for (int i = 0; i < num_approaches; i++) {
        inter->lane_map[i] = (int *)calloc(num_approaches, sizeof(int));
    }

    return inter;
}

void add_approach(Intersection *inter, const char *name, int num_lanes) {
    int idx = inter->num_approaches;
    inter->approaches = (Approach **)realloc(inter->approaches, (idx + 1) * sizeof(Approach *));
    
    Approach *app = (Approach *)malloc(sizeof(Approach));
    app->approach_id = idx + 1;
    strncpy(app->name, name, sizeof(app->name) - 1);
    app->name[sizeof(app->name) - 1] = '\0';
    app->num_lanes = num_lanes;
    app->signal = RED;
    app->green_time_allocated = MIN_GREEN;
    app->vehicles_passed_total = 0;
    app->total_waiting_time = 0;

    // Allocate lane memory
    app->lanes = (Lane *)malloc(num_lanes * sizeof(Lane));
    for (int j = 0; j < num_lanes; j++) {
        app->lanes[j].lane_id = j + 1;
        app->lanes[j].vehicle_count = 0;
        app->lanes[j].max_capacity = 50;
    }

    app->sensor.sensor_id = 100 + idx;
    app->sensor.detected_queue = 0;
    app->sensor.avg_speed = 30.0f;

    inter->approaches[idx] = app;
    inter->num_approaches++;
}

void remove_approach(Intersection *inter, int approach_id) {
    int target = -1;
    for (int i = 0; i < inter->num_approaches; i++) {
        if (inter->approaches[i]->approach_id == approach_id) {
            target = i;
            break;
        }
    }
    if (target == -1) {
        printf("[Warning] Approach ID %d not found.\n", approach_id);
        return;
    }

    // Free internal lane array and structure
    free(inter->approaches[target]->lanes);
    free(inter->approaches[target]);

    // Shift pointer array left
    for (int i = target; i < inter->num_approaches - 1; i++) {
        inter->approaches[i] = inter->approaches[i + 1];
    }
    inter->num_approaches--;
    inter->approaches = (Approach **)realloc(inter->approaches, inter->num_approaches * sizeof(Approach *));
    printf("Approach %d removed successfully.\n", approach_id);
}

void free_intersection(Intersection *inter) {
    if (!inter) return;
    for (int i = 0; i < inter->num_approaches; i++) {
        free(inter->approaches[i]->lanes);
        free(inter->approaches[i]);
    }
    free(inter->approaches);
    for (int i = 0; i < inter->num_approaches; i++) {
        free(inter->lane_map[i]);
    }
    free(inter->lane_map);
    free(inter->logs);
    free(inter);
}

void log_event(Intersection *inter, int time, int approach_id, const char *phase, int cleared) {
    if (inter->log_count == inter->log_capacity) {
        inter->log_capacity *= 2;
        inter->logs = (EventLog *)realloc(inter->logs, inter->log_capacity * sizeof(EventLog));
    }
    inter->logs[inter->log_count].timestamp = time;
    inter->logs[inter->log_count].approach_id = approach_id;
    strncpy(inter->logs[inter->log_count].phase_name, phase, 9);
    inter->logs[inter->log_count].phase_name[9] = '\0';
    inter->logs[inter->log_count].vehicles_cleared = cleared;
    inter->log_count++;
}

// ==========================================
// Adaptive Algorithm Logic (CO1)
// ==========================================

void calculate_adaptive_timings(Intersection *inter, int total_cycle) {
    int total_queue = 0;
    for (int i = 0; i < inter->num_approaches; i++) {
        total_queue += inter->approaches[i]->sensor.detected_queue;
    }

    for (int i = 0; i < inter->num_approaches; i++) {
        if (total_queue == 0) {
            inter->approaches[i]->green_time_allocated = total_cycle / inter->num_approaches;
        } else {
            float ratio = (float)inter->approaches[i]->sensor.detected_queue / (float)total_queue;
            int calculated = (int)(ratio * total_cycle);

            // Enforce minimum and maximum constraints
            if (calculated < MIN_GREEN) calculated = MIN_GREEN;
            if (calculated > MAX_GREEN) calculated = MAX_GREEN;
            inter->approaches[i]->green_time_allocated = calculated;
        }
    }
}

void generate_sensor_traffic(Intersection *inter) {
    for (int i = 0; i < inter->num_approaches; i++) {
        int incoming = rand() % 15 + 5; // 5 to 20 incoming vehicles
        Approach *app = inter->approaches[i];
        
        // Distribute incoming traffic across approach lanes
        int per_lane = incoming / app->num_lanes;
        for (int l = 0; l < app->num_lanes; l++) {
            app->lanes[l].vehicle_count += per_lane;
        }

        // Sensor aggregates total queued vehicles
        int total_app_queue = 0;
        for (int l = 0; l < app->num_lanes; l++) {
            total_app_queue += app->lanes[l].vehicle_count;
        }
        app->sensor.detected_queue = total_app_queue;
    }
}

// ==========================================
// Simulation Engine (CO2, CO3)
// ==========================================

void run_simulation(Intersection *inter, int cycles, int mode, const char *scenario_name) {
    printf("\n============================================================\n");
    printf(" Starting Scenario: %s (%s Strategy)\n", scenario_name, mode == 1 ? "ADAPTIVE" : "FIXED-TIME");
    printf("============================================================\n");

    int current_time = 0;

    for (int c = 1; c <= cycles; c++) {
        printf("\n--- Cycle %d / %d ---\n", c, cycles);
        generate_sensor_traffic(inter);

        if (mode == 1) {
            calculate_adaptive_timings(inter, DEFAULT_CYCLE_TIME);
        } else {
            // Fixed-time strategy: equal division
            for (int i = 0; i < inter->num_approaches; i++) {
                inter->approaches[i]->green_time_allocated = DEFAULT_CYCLE_TIME / inter->num_approaches;
            }
        }

        for (int i = 0; i < inter->num_approaches; i++) {
            Approach *app = inter->approaches[i];
            app->signal = GREEN;

            int capacity_to_clear = app->green_time_allocated * VEHICLE_PASS_RATE * app->num_lanes;
            int actual_cleared = (app->sensor.detected_queue < capacity_to_clear) ? app->sensor.detected_queue : capacity_to_clear;

            // Update lanes
            int remaining_to_remove = actual_cleared;
            for (int l = 0; l < app->num_lanes && remaining_to_remove > 0; l++) {
                if (app->lanes[l].vehicle_count <= remaining_to_remove) {
                    remaining_to_remove -= app->lanes[l].vehicle_count;
                    app->lanes[l].vehicle_count = 0;
                } else {
                    app->lanes[l].vehicle_count -= remaining_to_remove;
                    remaining_to_remove = 0;
                }
            }

            app->sensor.detected_queue -= actual_cleared;
            app->vehicles_passed_total += actual_cleared;
            app->total_waiting_time += (app->sensor.detected_queue * app->green_time_allocated);

            log_event(inter, current_time, app->approach_id, "GREEN", actual_cleared);
            current_time += app->green_time_allocated;

            printf("Approach: %-12s | Queue: %3d | Green Assigned: %2ds | Cleared: %3d | Remaining: %3d\n",
                   app->name, app->sensor.detected_queue + actual_cleared, app->green_time_allocated, actual_cleared, app->sensor.detected_queue);

            app->signal = RED;
        }
    }

    display_metrics(inter, current_time, mode == 1 ? "Adaptive" : "Fixed-Time");
    save_report_to_file("simulation_results.txt", inter, current_time, mode == 1 ? "Adaptive" : "Fixed-Time");
}

// ==========================================
// Statistics & Output Reporting
// ==========================================

void display_metrics(Intersection *inter, int total_simulation_time, const char *controller_type) {
    int total_throughput = 0;
    int total_wait = 0;

    for (int i = 0; i < inter->num_approaches; i++) {
        total_throughput += inter->approaches[i]->vehicles_passed_total;
        total_wait += inter->approaches[i]->total_waiting_time;
    }

    float avg_wait = (total_throughput > 0) ? ((float)total_wait / (float)total_throughput) : 0.0f;
    float throughput_per_min = (total_simulation_time > 0) ? ((float)total_throughput / ((float)total_simulation_time / 60.0f)) : 0.0f;

    printf("\n------------------------------------------------------------\n");
    printf(" Performance Summary [%s Controller]\n", controller_type);
    printf("------------------------------------------------------------\n");
    printf(" Total Simulation Duration : %d seconds\n", total_simulation_time);
    printf(" Total Throughput (Vehicles): %d\n", total_throughput);
    printf(" Throughput Rate           : %.2f vehicles/min\n", throughput_per_min);
    printf(" Average Wait Time         : %.2f seconds/vehicle\n", avg_wait);
    printf("------------------------------------------------------------\n");
}

void save_report_to_file(const char *filename, Intersection *inter, int sim_time, const char *mode_str) {
    FILE *fp = fopen(filename, "a");
    if (!fp) {
        printf("[Error] Unable to open %s for writing.\n", filename);
        return;
    }

    int total_throughput = 0;
    int total_wait = 0;
    for (int i = 0; i < inter->num_approaches; i++) {
        total_throughput += inter->approaches[i]->vehicles_passed_total;
        total_wait += inter->approaches[i]->total_waiting_time;
    }
    float avg_wait = (total_throughput > 0) ? ((float)total_wait / (float)total_throughput) : 0.0f;

    fprintf(fp, "============================================================\n");
    fprintf(fp, "SIMULATION LOG - Mode: %s\n", mode_str);
    fprintf(fp, "Simulated Time: %d s | Total Cleared: %d | Avg Wait: %.2f s\n", sim_time, total_throughput, avg_wait);
    fprintf(fp, "------------------------------------------------------------\n");
    fprintf(fp, "%-10s %-15s %-10s %-12s\n", "Time", "Approach ID", "Phase", "Cleared");
    for (int i = 0; i < inter->log_count; i++) {
        fprintf(fp, "%-10d %-15d %-10s %-12d\n",
                inter->logs[i].timestamp,
                inter->logs[i].approach_id,
                inter->logs[i].phase_name,
                inter->logs[i].vehicles_cleared);
    }
    fprintf(fp, "============================================================\n\n");
    fclose(fp);
    printf(">> Event log and metrics successfully appended to '%s'.\n", filename);
}

// ==========================================
// Interactive Console Menu
// ==========================================

int main(void) {
    srand((unsigned int)time(NULL));

    Intersection *inter = init_intersection(4);
    add_approach(inter, "Northbound", 2);
    add_approach(inter, "Eastbound", 3);
    add_approach(inter, "Southbound", 2);
    add_approach(inter, "Westbound", 1);

    int choice = 0;
    while (1) {
        printf("\n=========================================\n");
        printf("  Adaptive Traffic Intersection System   \n");
        printf("=========================================\n");
        printf("1. Run Adaptive Simulation\n");
        printf("2. Run Fixed-Time Baseline Simulation\n");
        printf("3. Compare Adaptive vs Fixed Strategy\n");
        printf("4. Add Approach (Dynamic Allocation)\n");
        printf("5. Remove Approach (Dynamic Free)\n");
        printf("6. Exit\n");
        printf("Selection: ");

        if (scanf("%d", &choice) != 1) break;

        if (choice == 1) {
            run_simulation(inter, 3, 1, "Standard Peak Traffic");
        } else if (choice == 2) {
            run_simulation(inter, 3, 2, "Standard Peak Traffic");
        } else if (choice == 3) {
            printf("\n--- Step 1/2: Running Adaptive System ---");
            run_simulation(inter, 4, 1, "Benchmark Test");
            printf("\n--- Step 2/2: Running Baseline System ---");
            run_simulation(inter, 4, 2, "Benchmark Test");
        } else if (choice == 4) {
            char name[30];
            int lanes;
            printf("Enter Approach Name: ");
            scanf(" %29[^\n]", name);
            printf("Enter Number of Lanes: ");
            scanf("%d", &lanes);
            add_approach(inter, name, lanes);
            printf("Added '%s' with %d lanes.\n", name, lanes);
        } else if (choice == 5) {
            int app_id;
            printf("Enter Approach ID to remove: ");
            scanf("%d", &app_id);
            remove_approach(inter, app_id);
        } else if (choice == 6) {
            printf("Cleaning heap memory and terminating...\n");
            break;
        } else {
            printf("Invalid selection.\n");
        }
    }

    free_intersection(inter);
    return 0;
}
