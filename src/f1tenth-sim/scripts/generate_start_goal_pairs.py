#!/usr/bin/env python
import csv
import random
import os

START_MIN_X = -17.0
START_MAX_X = -14.0
START_MIN_Y = -9.0
START_MAX_Y = 9.0

GOAL_MIN_X = 14.0
GOAL_MAX_X = 17.0
GOAL_MIN_Y = -9.0
GOAL_MAX_Y = 9.0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, '../data')
POINTS_FILE = os.path.join(SCRIPT_DIR, '../data/start_goal_pair.csv')

def get_random_position(min_x, max_x, min_y, max_y):
    return random.uniform(min_x, max_x), random.uniform(min_y, max_y)

def generate_start_goal_points(iterations):
    points = []
    for _ in range(iterations):
        start_x, start_y = get_random_position(START_MIN_X, START_MAX_X, START_MIN_Y, START_MAX_Y)
        goal_x, goal_y = get_random_position(GOAL_MIN_X, GOAL_MAX_X, GOAL_MIN_Y, GOAL_MAX_Y)
        points.append([start_x, start_y, goal_x, goal_y])
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
    iterations = int(input("Enter the number of iterations: "))

    points = generate_start_goal_points(iterations)
    save_start_goal_points(POINTS_FILE, points)

if __name__ == '__main__':
    main()