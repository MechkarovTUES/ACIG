
def get_points(distances, image_arr):
    points = []
    # Iterate through every pixel.
    for row in range(distances.shape[0]): 
        for col in range(distances.shape[1]):
            [r, g, b] = image_arr[row, col]
            [X, Y, Z] = distances[row, col]

            if Z is None:
                continue
            points.append([X, Y, Z, r, g, b])  
    return points  

