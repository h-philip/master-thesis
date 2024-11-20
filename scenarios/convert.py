import argparse
from PIL import Image
import numpy as np
import datetime
import os

class Converter:
        
    route_point = 9 #FF0000
    tree = -1#6        #008080
    bridge = 6      #008080
    wall = 0        #000000
    air = 15        #FFFFFF

    bridge_height = 20
    obstacle_height = (0,20)
    point_height = 10

    font_size = 20

    def __init__(self, input_file):
        # Initialize any necessary variables or resources here
        self.input_file = input_file
        self.route:list = []
        self.trees:list = []
        self.bridges:list = []
        self.collisions:list = []
        self.route_point_counter = 0
        self.collision_point_counter = 0

        self.name = input_file.split("/")[-1].split("\\")[-1].split(".")[-2]
        print(f"Creating files for {self.name}...")
        os.makedirs(self.name, exist_ok=True)

    def optimize_lines(self, image) -> list:
        all_pixels = 0
        optimized_pixels = 0
        p = np.array(image)
        # Create copy of image to optimize
        optimized = np.copy(p)

        for y in range(p.shape[0]):
            for x in range(p.shape[1]):
                pixel = optimized[y, x]
                all_pixels += 1
                # if left and right or top and bottom are same color, set this to air in optimized, else copy the pixel
                if x > 0 and x < optimized.shape[1] - 1 and y > 0 and y < optimized.shape[0] - 1:
                    if (pixel == optimized[y, x - 1] and pixel == optimized[y, x + 1]) or (pixel == optimized[y - 1, x] and pixel == optimized[y + 1, x]):
                        optimized[y, x] = self.air
                        optimized_pixels += 1
                        continue
                optimized[y, x] = pixel

        print(f"Optimized {optimized_pixels} out of {all_pixels} pixels. ({optimized_pixels/all_pixels*100:.2f}%)")
        return optimized

    def read_image(self):
        image = Image.open(self.input_file)

        p = self.optimize_lines(image)
        route_points:list = []
            
        for y in range(p.shape[0]):
            for x in range(p.shape[1]):
                pixel = p[y, x]
                if pixel == self.route_point:
                    route_points.append((x, y))
                elif pixel == self.wall:
                    self.collisions.append((x, y))
                elif pixel == self.tree:
                    self.trees.append((x, y))
                elif pixel == self.bridge:
                    self.bridges.append((x, y))

        self.create_ordered_route(route_points)

    def create_ordered_route(self, route_points):
        print("Route points:")
        for i, point in enumerate(route_points):
            print(f"{i}: {point}")
        
        order = input("Enter the correct order of the route points (indices separated by a space): ")
        order_indices = order.split()
        
        self.route = [route_points[int(index)] for index in order_indices]
        self.route.insert(0, self.route[0])
        self.route.append(self.route[-1])
        
        self.route[0] = (self.route[0][0], self.route[0][1], 0)
        for i in range(1, len(self.route)-1):
            self.route[i] = (self.route[i][0], self.route[i][1], self.point_height)
        self.route[-1] = (self.route[-1][0], self.route[-1][1], 0)
        
        print("Ordered route:")
        for i, point in enumerate(self.route):
            print(f"{i}: {point}")

    def create_collision(self, x: int, y: int) -> list:
        collisions = []
        for z in range(self.obstacle_height[0], self.obstacle_height[1] + 1, 2):
            collisions.append((x, y, z))
        return collisions

    def create_tree(self, x: int, y: int) -> list:
        collisions = self.create_collision(x, y)
        z = self.obstacle_height[1]
        collisions.append((x - 2, y, z))
        collisions.append((x + 2, y, z))
        collisions.append((x, y - 2, z))
        collisions.append((x, y + 2, z))
        collisions.append((x, y, z + 2))
        return collisions

    def write_route_file(self):
        with open(f"{self.name}/{self.name}.route", "w") as file:
            for point in self.route:
                file.write(f"{point[0]} {-point[1]} {point[2]}\n")

    def write_collision_cloud_file(self):
        all_collisions = []
        for point in self.collisions:
            all_collisions += self.create_collision(point[0], -point[1])
        for point in self.trees:
            all_collisions += self.create_tree(point[0], -point[1])
        for point in self.bridges:
            all_collisions.append((point[0], -point[1], self.bridge_height))


        with open(f"{self.name}/{self.name}.collisions", "w") as file:
            for point in all_collisions:
                file.write(f"{point[0]} {point[1]} {point[2]}\n")

    def route_point_cell(self, x, y)->str:
        cell = f'''        <mxCell id="routepoint-{self.route_point_counter}" value="" style="shape=waypoint;sketch=0;fillStyle=solid;size=6;pointerEvents=1;points=[];fillColor=none;resizable=0;rotatable=0;perimeter=centerPerimeter;snapToPoint=1;fontSize={self.font_size};" vertex="1" parent="1">
          <mxGeometry x="{x*10 - 10}" y="{y*10 - 10}" width="20" height="20" as="geometry" />
        </mxCell>
        <mxCell id="routepointtext-{self.route_point_counter}" value="\(p_{{{self.route_point_counter}}}\)" style="text;html=1;align=center;verticalAlign=middle;resizable=0;points=[];autosize=1;strokeColor=none;fillColor=none;fontSize={self.font_size};" vertex="1" parent="1">
          <mxGeometry x="{x*10 - 15}" y="{y*10 - 10}" width="70" height="30" as="geometry" />
        </mxCell>'''
        self.route_point_counter += 1
        return cell

    def collision_point_cell(self, x, y, color = "#eeeeee")->str:
        cell = f'''        <mxCell id="collsionpoint-{self.collision_point_counter}" value="" style="ellipse;whiteSpace=wrap;html=1;aspect=fixed;fontSize={self.font_size};fillColor={color};strokeColor=#36393d;" vertex="1" parent="1">
          <mxGeometry x="{x*10 - 15}" y="{y*10 - 15}" width="30" height="30" as="geometry" />
        </mxCell>'''
        self.collision_point_counter += 1
        return cell

    def write_drawio_xml_file(self):
        with open(f"template.drawio", "r") as file:
            template_string = file.read()

        cells = ""

        for point in self.collisions:
            cells += self.collision_point_cell(point[0], point[1]) + "\n"
        for point in self.trees:
            cells += self.collision_point_cell(point[0], point[1]) + "\n"
        for point in self.bridges:
            cells += self.collision_point_cell(point[0], point[1], "#888888") + "\n"
        for point in self.route:
            cells += self.route_point_cell(point[0], point[1]) + "\n"

        replacements = {
            "INSERT_MODIFIED": str(datetime.datetime.now()),
            "INSERT_ID": self.name,
            "INSERT_MX_CELLS": cells
        }

        for key, value in replacements.items():
            template_string = template_string.replace(key, value)

        with open(f"{self.name}/{self.name}.drawio", "w") as file:
            file.write(template_string)

def main():
    parser = argparse.ArgumentParser(description='Converter')
    parser.add_argument('input_file', help='Path to 16 bit bitmap input file')
    args = parser.parse_args()

    converter = Converter(args.input_file)
    converter.read_image()
    converter.write_route_file()
    converter.write_collision_cloud_file()
    converter.write_drawio_xml_file()

if __name__ == "__main__":
    main()