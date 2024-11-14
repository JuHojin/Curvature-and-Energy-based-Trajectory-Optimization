#!/usr/bin/env python
import csv
import os

# 고정된 x 좌표
START_X = -16.0
GOAL_X = 16.0

# y 좌표의 범위와 분할 개수
START_MIN_Y = -9.0
START_MAX_Y = 9.0
GOAL_MIN_Y = -9.0
GOAL_MAX_Y = 9.0
NUM_DIVISIONS = 15

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, '../data')
POINTS_FILE = os.path.join(SCRIPT_DIR, '../data/start_goal_pair.csv')

def generate_start_goal_points(num_divisions):
    points = []
    start_y_values = [START_MIN_Y + i * (START_MAX_Y - START_MIN_Y) / (num_divisions - 1) for i in range(num_divisions)]
    goal_y_values = [GOAL_MIN_Y + i * (GOAL_MAX_Y - GOAL_MIN_Y) / (num_divisions - 1) for i in range(num_divisions)]
    
    for start_y in start_y_values:
        for goal_y in goal_y_values:
            points.append([START_X, start_y, GOAL_X, goal_y])
    
    return points

def save_start_goal_points(filename, points):
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)
    
    with open(filename, 'w') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(['start_x', 'start_y', 'goal_x', 'goal_y'])
        csv_writer.writerows(points)
    print("Saved start and goal points to {}".format(filename))

def main():
    points = generate_start_goal_points(NUM_DIVISIONS)
    save_start_goal_points(POINTS_FILE, points)

if __name__ == '__main__':
    main()
