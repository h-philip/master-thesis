import datetime
import random
import math
import os

class RandomScenario:

    font_size = 20
    
    def __init__(self, width, depth, height, p):
        self.width = width
        self.depth = depth
        self.height = height
        self.p = p
        self.obstacles = set()
        self.route_points = []
        self.route_point_counter = 0
        self.collision_point_counter = 0

    def generate_obstacles(self):
        self.obstacles = set()
        for x in range(self.width):
            for y in range(self.depth):
                for z in range(self.height):
                    if random.random() < self.p:
                        self.obstacles.add((x, y, z))
        return self.obstacles
    
    def get_obstacles(self):
        return self.obstacles
    
    def generate_route_points(self):
        self.route_points = []
        # Generate first two points
        x = random.randint(0, self.width - 1)
        y = random.randint(0, self.depth - 1)
        self.route_points.append((x, y, 0))
        z = random.randint(10, 20)
        self.route_points.append((x, y, z))

        # Generate the middle points
        p = self.route_points[-1]
        for i in range(25):
            p = self.get_random_nearby_route_point(p[0], p[1], p[2], 20, 35)
            self.route_points.append(p)

        # Generate the last two points
        x = p[0]
        y = p[1]
        z = random.randint(10, 20)
        self.route_points.append((x, y, z))
        self.route_points.append((x, y, 0))

        self.remove_obstacles_between_points()

        return self.route_points
    
    def remove_obstacles_between_points(self, radius = 5):
        # Remove obstacles between first two points
        x, y, z = self.route_points[1]
        for obstacle in self.obstacles.copy():
            ox, oy, oz = obstacle
            if oz <= z + 5:
                distance = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
                if distance <= radius:
                    self.obstacles.remove(obstacle)
            
        # Remove obstacles between last two points
        x, y, z = self.route_points[-2]
        for obstacle in self.obstacles.copy():
            ox, oy, oz = obstacle
            if oz <= z + 5:
                distance = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
                if distance <= radius:
                    self.obstacles.remove(obstacle)
    
    def get_route_points(self):
        return self.route_points
    
    def get_random_nearby_route_point(self, x, y, z, min_distance, max_distance):
        while True:
            new_x = random.randint(0, self.width - 1)
            new_y = random.randint(0, self.depth - 1)
            new_z = random.randint(10, self.height - 1)
            distance = math.sqrt((new_x - x) ** 2 + (new_y - y) ** 2 + (new_z - z) ** 2)
            if min_distance <= distance and distance <= max_distance:
                return (new_x, new_y, new_z)

    def route_point_cell(self, x, y)->str:
        cell = f'''        <mxCell id="routepoint-{self.route_point_counter}" value="" style="shape=waypoint;sketch=0;fillStyle=solid;size=6;pointerEvents=1;points=[];fillColor=none;resizable=0;rotatable=0;perimeter=centerPerimeter;snapToPoint=1;fontSize={self.font_size};" vertex="1" parent="1">
          <mxGeometry x="{x*10 - 10}" y="{y*10 - 10}" width="20" height="20" as="geometry" />
        </mxCell>
        <mxCell id="routepointtext-{self.route_point_counter}" value="\(p_{{{self.route_point_counter}}}\)" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;fontSize={self.font_size};" vertex="1" parent="1">
          <mxGeometry x="{x*10 - 15}" y="{y*10 - 10}" width="70" height="30" as="geometry" />
        </mxCell>'''
        self.route_point_counter += 1
        return cell

    def collision_point_cell(self, x, y)->str:
        cell = f'''        <mxCell id="collsionpoint-{self.collision_point_counter}" value="" style="ellipse;whiteSpace=wrap;html=1;aspect=fixed;fontSize={self.font_size};fillColor=#eeeeee;strokeColor=#36393d;" vertex="1" parent="1">
          <mxGeometry x="{x*10 - 15}" y="{y*10 - 15}" width="30" height="30" as="geometry" />
        </mxCell>'''
        self.collision_point_counter += 1
        return cell

    def write_drawio_xml_file(self):
        with open(f"template.drawio", "r") as file:
            template_string = file.read()

        cells = ""

        for point in self.obstacles:
            cells += self.collision_point_cell(point[0], point[1]) + "\n"
        for point in self.route_points:
            cells += self.route_point_cell(point[0], point[1]) + "\n"

        replacements = {
            "INSERT_MODIFIED": str(datetime.datetime.now()),
            "INSERT_ID": "random",
            "INSERT_MX_CELLS": cells
        }

        for key, value in replacements.items():
            template_string = template_string.replace(key, value)

        with open(f"random/random.drawio", "w") as file:
            file.write(template_string)
        
    def save(self):
        # Make sure directory exists
        if not os.path.exists("random"):
            os.makedirs("random")

        # Save obstacles
        with open("random/random.collisions", "w") as f:
            for obstacle in self.obstacles:
                f.write(f"{obstacle[0]} {obstacle[1]} {obstacle[2]}\n")

        # Save route points
        with open("random/random.route", "w") as f:
            for point in self.route_points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")
def main():
    # Example usage
    scenario = RandomScenario(100, 100, 100, 0.001)
    scenario.generate_obstacles()
    scenario.generate_route_points()
    scenario.write_drawio_xml_file()
    scenario.save()

if __name__ == "__main__":
    main()