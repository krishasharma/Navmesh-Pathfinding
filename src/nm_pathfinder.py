# Function to calculate Euclidean distance between two points
def calculate_distance(pointA, pointB):
    # Euclidean distance formula
    return ((pointA[1] - pointB[1]) ** 2 + (pointA[0] - pointB[0]) ** 2) ** 0.5

# Function to get neighbors of a given box from the navmesh
def get_adjacent_boxes(navmesh, current_box):
    # Retrieve adjacent boxes from the navmesh adjacency list
    return navmesh['adj'].get(current_box, [])

# Function to find the box that contains the given point
def find_box_containing_point(navmesh, point):
    # Iterate through all boxes in the navmesh to find the one containing the point
    for box in navmesh['boxes']:
        top, bottom, left, right = box
        if top <= point[0] <= bottom and left <= point[1] <= right:
            return box
    return None

# Simple priority queue implementation using a list
class SimplePriorityQueue:
    def __init__(self):
        self.elements = []

    def is_empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        self.elements.append((priority, item))
        self.elements.sort()  # Sort by priority

    def get(self):
        return self.elements.pop(0)[1]

# Function to clamp a point to be within the bounds of a box
def clamp_point_to_box(point, box):
    top, bottom, left, right = box
    clamped_y = max(top, min(point[0], bottom))
    clamped_x = max(left, min(point[1], right))
    return (clamped_y, clamped_x)

# Function to reconstruct the path once the meeting point is found
def reconstruct_path(forward_path_prev, backward_path_prev, meeting_point, source, destination):
    # Build the forward path from the source to the meeting point
    forward_path = []
    current = meeting_point
    while current is not None:
        forward_path.append(current)
        current = forward_path_prev[current]
    forward_path.reverse()

    # Build the backward path from the meeting point to the destination
    backward_path = []
    current = meeting_point
    while current is not None:
        backward_path.append(current)
        current = backward_path_prev[current]

    # Combine both paths to form the complete path
    complete_path = forward_path + backward_path[1:]

    # Convert boxes to their center points and clamp points to box bounds
    path_points = [source]
    for box in complete_path:
        point = clamp_point_to_box(path_points[-1], box)
        path_points.append(point)
    path_points.append(destination)
    
    return path_points

# Main pathfinding function
def find_path(source_point, destination_point, navmesh):
    
    # Initialize path and visited boxes
    path = []
    visited_boxes = {}

    # Find the boxes containing the source and destination points
    source_box = find_box_containing_point(navmesh, source_point)
    destination_box = find_box_containing_point(navmesh, destination_point)

    # If either box is not found, there is no path
    if not source_box or not destination_box:
        print("No path!")
        return path, visited_boxes.keys()

    # Initialize the priority queues for both searches
    forward_queue = SimplePriorityQueue()
    backward_queue = SimplePriorityQueue()

    forward_queue.put(source_box, 0)
    backward_queue.put(destination_box, 0)
    
    # Initialize distances and previous nodes dictionaries
    forward_distance = {source_box: 0}
    backward_distance = {destination_box: 0}
    
    forward_path_prev = {source_box: None}
    backward_path_prev = {destination_box: None}
    
    # Initialize visited sets for both searches
    visited_forward = set()
    visited_backward = set()
    meeting_point = None
    
    # While there are still nodes to explore in both queues
    while not forward_queue.is_empty() and not backward_queue.is_empty():
        # Expand the forward search
        if not forward_queue.is_empty():
            current_box = forward_queue.get()
            if current_box in visited_backward:
                meeting_point = current_box
                break
            visited_forward.add(current_box)
            for neighbor in get_adjacent_boxes(navmesh, current_box):
                current_center = ((current_box[0] + current_box[1]) // 2, (current_box[2] + current_box[3]) // 2)
                neighbor_center = ((neighbor[0] + neighbor[1]) // 2, (neighbor[2] + neighbor[3]) // 2)
                cost = forward_distance[current_box] + calculate_distance(current_center, neighbor_center)
                if neighbor not in forward_distance or cost < forward_distance[neighbor]:
                    forward_distance[neighbor] = cost
                    priority = cost + calculate_distance(neighbor_center, destination_point)
                    forward_queue.put(neighbor, priority)
                    forward_path_prev[neighbor] = current_box

        # Expand the backward search
        if not backward_queue.is_empty():
            current_box = backward_queue.get()
            if current_box in visited_forward:
                meeting_point = current_box
                break
            visited_backward.add(current_box)
            for neighbor in get_adjacent_boxes(navmesh, current_box):
                current_center = ((current_box[0] + current_box[1]) // 2, (current_box[2] + current_box[3]) // 2)
                neighbor_center = ((neighbor[0] + neighbor[1]) // 2, (neighbor[2] + neighbor[3]) // 2)
                cost = backward_distance[current_box] + calculate_distance(current_center, neighbor_center)
                if neighbor not in backward_distance or cost < backward_distance[neighbor]:
                    backward_distance[neighbor] = cost
                    priority = cost + calculate_distance(neighbor_center, source_point)
                    backward_queue.put(neighbor, priority)
                    backward_path_prev[neighbor] = current_box

        # If a meeting point is found, break the loop
        if meeting_point:
            break
    
    # If no meeting point was found, return no path
    if not meeting_point:
        print("No path!")
        return path, visited_boxes.keys()
    
    # Reconstruct the path from the meeting point
    path = reconstruct_path(forward_path_prev, backward_path_prev, meeting_point, source_point, destination_point)
    visited_boxes = {**forward_path_prev, **backward_path_prev}
    return path, visited_boxes.keys()