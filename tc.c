#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdbool.h>

// --- Project Constants ---
#define DELTA_L 5  // Turning Left
#define DELTA_S 4  // Going Straight
#define DELTA_R 3  // Turning Right

// --- Global Synchronization ---
sem_t lane_lock[4];        // Head-of-line lock for N, S, E, W

// --- Intersection Monitor ---
// Replaces the global lock with a centralized monitor for collision and sequence tracking
pthread_mutex_t monitor_lock;
pthread_cond_t monitor_cond;

int global_seq = 0;        // Deli-ticket counter for arrival sequence
int front_seq[4] = {0};    // The ticket number held by the car currently at the stop sign
bool stuck[4] = {false, false, false, false}; // True if the car at the stop sign is waiting to cross

typedef struct {
    int id;
    char orig;
    int mask;
} ActiveCar;

ActiveCar active_cars[10];
int num_active = 0;

struct timeval start_time;

typedef struct _directions {
    char dir_original;
    char dir_target;
} directions;

typedef struct {
    int cid;
    double arrival_delay;
    directions dir;
} car_info;

// Helper to get elapsed simulation time
double current_time() {
    struct timeval now;
    gettimeofday(&now, NULL);
    return (now.tv_sec - start_time.tv_sec) + (now.tv_usec - start_time.tv_usec) / 1000000.0;
}

// Simulates crossing time
void Spin(int seconds) {
    sleep(seconds); 
}

// Map char direction to index
int dir_to_idx(char d) {
    if (d == '^') return 0; // Northbound (comes from South)
    if (d == 'v') return 1; // Southbound (comes from North)
    if (d == '>') return 2; // Eastbound (comes from West)
    if (d == '<') return 3; // Westbound (comes from East)
    return -1;
}

// Determine duration based on turn type
int get_turn_duration(char orig, char targ) {
    if (orig == targ) return DELTA_S;
    
    // Left Turns
    if ((orig == '^' && targ == '<') || 
        (orig == '<' && targ == 'v') || 
        (orig == 'v' && targ == '>') || 
        (orig == '>' && targ == '^')) {
        return DELTA_L; 
    }
    
    // Right Turns
    if ((orig == '^' && targ == '>') || 
        (orig == '>' && targ == 'v') || 
        (orig == 'v' && targ == '<') || 
        (orig == '<' && targ == '^')) {
        return DELTA_R; 
    }
    
    return DELTA_S;
}

// Determine the intersection quadrants a specific path sweeps through
// Quadrants: NW=1, NE=2, SE=4, SW=8
int get_path_mask(char orig, char targ) {
    if (orig == '^') {
        if (targ == '^') return 6; // SE, NE
        if (targ == '<') return 7; // SE, NE, NW (Sweeps wide)
        if (targ == '>') return 4; // SE
    } else if (orig == 'v') {
        if (targ == 'v') return 9; // NW, SW
        if (targ == '>') return 13; // NW, SW, SE (Sweeps wide)
        if (targ == '<') return 1; // NW
    } else if (orig == '>') {
        if (targ == '>') return 12; // SW, SE
        if (targ == '^') return 14; // SW, SE, NE (Sweeps wide)
        if (targ == 'v') return 8; // SW
    } else if (orig == '<') {
        if (targ == '<') return 3; // NE, NW
        if (targ == 'v') return 11; // NE, NW, SW (Sweeps wide)
        if (targ == '^') return 2; // NE
    }
    return 0;
}

void ArriveIntersection(car_info *car) {
    // 1. Initial arrival wait
    usleep(car->arrival_delay * 1000000);
    printf("Time %.1f: Car %d (%c%c) arriving\n", current_time(), car->cid, car->dir.dir_original, car->dir.dir_target);

    // 2. Stop sign wait (2 seconds)
    usleep(2000000);

    // 3. Move up in lane. The thread blocks here until it is at the front of the line
    int lane = dir_to_idx(car->dir.dir_original);
    sem_wait(&lane_lock[lane]);

    // 4. We are at the stop sign! Grab a sequence ticket to timestamp our exact arrival
    pthread_mutex_lock(&monitor_lock);
    front_seq[lane] = ++global_seq;
    stuck[lane] = true;
    pthread_mutex_unlock(&monitor_lock);
}

void CrossIntersection(car_info *car) {
    int lane = dir_to_idx(car->dir.dir_original);
    int my_mask = get_path_mask(car->dir.dir_original, car->dir.dir_target);

    pthread_mutex_lock(&monitor_lock);
    
    while (1) {
        bool safe = true;
        
        // CONDITION 1: Check if any car at OTHER stop signs arrived earlier and is waiting
        for (int i = 0; i < 4; i++) {
            if (i == lane) continue;
            if (stuck[i] && front_seq[i] < front_seq[lane]) {
                safe = false; // We must yield to the earlier car
                break;
            }
        }
        
        // CONDITION 2: Check for physical collision path with currently crossing cars
        if (safe) {
            for (int i = 0; i < num_active; i++) {
                if (active_cars[i].orig == car->dir.dir_original) {
                    continue; // Readers-Writers logic: Flow from same origin can go back-to-back
                }
                if ((active_cars[i].mask & my_mask) != 0) {
                    safe = false; // Paths intersect!
                    break;
                }
            }
        }
        
        if (safe) {
            break; // Safe to cross, exit while loop
        } else {
            pthread_cond_wait(&monitor_cond, &monitor_lock); // Wait for the intersection state to change
        }
    }
    
    // We are officially crossing
    stuck[lane] = false; 
    ActiveCar ac = {car->cid, car->dir.dir_original, my_mask};
    active_cars[num_active++] = ac;
    
    // Broadcast immediately: Us crossing means we are no longer "stuck", which might unblock someone waiting on us
    pthread_cond_broadcast(&monitor_cond); 
    pthread_mutex_unlock(&monitor_lock);

    // Release the lane lock so the car behind can move up to the stop sign
    sem_post(&lane_lock[lane]);

    printf("Time %.1f: Car %d (%c%c) crossing\n", current_time(), car->cid, car->dir.dir_original, car->dir.dir_target);

    // Determine and execute crossing duration
    int duration = get_turn_duration(car->dir.dir_original, car->dir.dir_target);
    Spin(duration);
}

void ExitIntersection(car_info *car) {
    pthread_mutex_lock(&monitor_lock);
    
    // Remove car from active crossing list
    for (int i = 0; i < num_active; i++) {
        if (active_cars[i].id == car->cid) {
            for (int j = i; j < num_active - 1; j++) {
                active_cars[j] = active_cars[j+1];
            }
            num_active--;
            break;
        }
    }
    
    // Wake up all waiting cars to re-evaluate if it is safe to cross
    pthread_cond_broadcast(&monitor_cond); 
    pthread_mutex_unlock(&monitor_lock);

    printf("Time %.1f: Car %d (%c%c) exiting\n", current_time(), car->cid, car->dir.dir_original, car->dir.dir_target);
}

void* CarThread(void* arg) {
    car_info *car = (car_info*)arg;
    ArriveIntersection(car);
    CrossIntersection(car);
    ExitIntersection(car);
    return NULL;
}

int main() {
    gettimeofday(&start_time, NULL);

    // Initialize Synchronization Primitives
    pthread_mutex_init(&monitor_lock, NULL);
    pthread_cond_init(&monitor_cond, NULL);
    
    for(int i = 0; i < 4; i++) {
        sem_init(&lane_lock[i], 0, 1);
    }

    // Testing Data
    car_info cars[8] = {
        {1, 1.1, {'^', '^'}},
        {2, 2.2, {'^', '^'}},
        {3, 3.3, {'^', '<'}},
        {4, 4.4, {'v', 'v'}},
        {5, 5.5, {'v', '>'}},
        {6, 6.6, {'^', '^'}},
        {7, 7.7, {'>', '^'}},
        {8, 8.8, {'<', '^'}}
    };

    pthread_t threads[8];
    for(int i = 0; i < 8; i++) {
        pthread_create(&threads[i], NULL, CarThread, &cars[i]);
    }

    for(int i = 0; i < 8; i++) {
        pthread_join(threads[i], NULL);
    }

    // Clean up
    pthread_mutex_destroy(&monitor_lock);
    pthread_cond_destroy(&monitor_cond);
    for(int i = 0; i < 4; i++) {
        sem_destroy(&lane_lock[i]);
    }

    return 0;
}