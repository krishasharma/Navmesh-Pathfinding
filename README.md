# Navmesh Pathfinding

This project involves implementing a bidirectional A* search algorithm in Python to solve the problem of finding paths in navigation meshes (navmeshes) created from user-provided images. The goal is to compute and visualize a path between a source and destination point, interactively selected by the user, on a navmesh derived from the input image.

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [File Structure](#file-structure)
- [A* Pathfinding Implementation](#a-pathfinding-implementation)
- [Creating Custom Maps](#creating-custom-maps)
- [Submission Requirements](#submission-requirements)
- [References](#references)

## Introduction

In this project, I implement a bidirectional A* search algorithm that computes a path between a source and destination on a navigation mesh. Navmeshes are built from user-provided images, and my implementation will visualize the shortest path between two points.

The program provides the ability to interactively define the source and destination points on an image and visualizes the computed path using the `nm_interactive.py` tool. I also build and test navmeshes from custom images.

## Prerequisites

- Python 3.x
- `numpy` and `scipy` Python libraries (for image processing and navmesh generation)
