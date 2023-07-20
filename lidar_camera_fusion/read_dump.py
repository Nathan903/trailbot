import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cv2
import random

import human_detection_node
from human_detection_node import Person

human_detection_node.SAVED_MODEL_PATH = "multipose_model"
human_detection_node.parse_global_matrix()
human_detection_node.model = human_detection_node.load_saved_model()
p = [Person(),]

def track_person(image):
	return random.randint(0,100),10

def find_closest_point_depth(x, y, np_2d_array,k):
	global closest_index
	# Calculate the distance between each point and the target coordinates (x, y)
	distances_sq = (np_2d_array[0,:] - x) ** 2 + (np_2d_array[1,:] - y) ** 2

	# Find the index of the point with the minimum distance want
	k = k	 # Number of nearest neighbors we want
	indices = np.argpartition(distances_sq, k)[:k]
	
	# estimation_mean_distance = np.median(distances_sq[indices]**0.5)

	# Get the depth value of the closest point
	closest_depths = np_2d_array[2,indices]
	closest_points = np_2d_array[0:2,indices]
	return np.median(closest_depths),closest_points,np.min(closest_depths)


fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
plt.subplots_adjust(wspace=0.1)
ax1.set_xlabel('time (frames)')
ax1.set_ylabel('distance (m)')
ax3 = ax2.twinx()

class internalState:
	human_max_speed = 2.8 # m/s
	fps = 10
	buffer_ratio = 1.5 # allow fluctuation of up to 1.5 times 
	max_movement_per_frame = human_max_speed / fps * buffer_ratio
	moving_average_weights = [10,5,3,2,1]

	def __init__(self,depth_history_length):
		# Instance attributes (unique to each instance)
		self.depth_history = []
		self.discarded_depth_history = []
		self.missing_frame_count = 0 
		self.depth_history_length =depth_history_length  

	def weighted_moving_average(data, weights):
		num_points = min(len(data), len(weights))
		weights_sum = 0
		weighted_data_sum = 0
		for i in range(num_points):
			weighted_data_sum += weights[i]*data[-1-i] 
			weights_sum += weights[i]
		return weighted_data_sum/weights_sum

	def get_average(self):
		return self.weighted_moving_average(self.depth_history,internalState.moving_average_weights)


	def append(self, new_depth):
		if len(self.depth_history) < self.depth_history_length:
			self.depth_history.append(new_depth)
			return 0

		avg = self.get_average()
		if abs(avg-new_depth) < internalState.max_movement_per_frame*self.missing_frame_count:
			self.depth_history.pop(0)
			self.depth_history.append(new_depth)
			self.missing_frame_count = 0
			return 0
		self.missing_frame_count +=1
		self.discarded_depth_history.append(new_depth)
		return 1





images = []
for k in range(5,20):
	depths = []
	estimation_mean_distances=[]

	state = internalState(5)
	state2 = internalState(5)

	for i in range(50  , 716):
		image, points = np.load(f'dump2/{i}.npz').values()

		there_is_person = human_detection_node.process_frame(image,p,)
		test_x, test_y = p[0].x -15 ,p[0].y -15

		points2d = human_detection_node.convert_to_camera_frame(points)
		uv_x, uv_y, uv_z = points2d

		median, closest_points,test2 = find_closest_point_depth(test_x,test_y, points2d,k)

		state.append(median)
		depth = state.get_average()
		depths.append(depth)

		state2.append(test2)
		depth2 = state.get_average()
		estimation_mean_distances.append(depth2)

		if show_animation:=0 :
			
			ax1.cla()
			ax1.axis('off')
			ax1.scatter(uv_x,uv_y,s=1,c=uv_z, cmap='jet')
			if there_is_person:
				ax1.scatter(test_x,test_y, s=100, color='green', marker='x')  # Adding the red point at (100, 100)
				ax1.scatter(closest_points[0],closest_points[1],s=10,color="red")
			rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
			ax1.imshow(rgb_image, extent=[0, 1280, 0, 1024])

			# ax2.plot(depths, color='red')
			ax3.plot(depths, color='green')
			plt.pause(0.001)

		# Add a red point to the image
		# x, y = track_person(image)
		# cv2.circle(rgb_image, (x, y), 10, (255, 0, 0), -1)
		
		# images.append([plt.imshow(rgb_image, animated=True)])

		if testnow:=1:
			ax1.cla()
			ax2.cla()
			ax3.cla()
			ax3.plot(estimation_mean_distances, color='green')
			ax1.plot(depths, color ="red")
			plt.pause(0.001)

	ax1.cla()
	ax2.cla()
	ax3.cla()
	name = f"k={k}"
	ax2.annotate(name, xy=(len(depths) - 1, depths[-1]), xytext=(1, 0), color='red',textcoords='offset points', ha='left', va='center',fontsize=6)
	ax3.plot(estimation_mean_distances, color='green')
	ax1.plot(depths, color ="red")
	plt.savefig(name+"greenIsMin.png")

	print(name)
plt.show()

ani = animation.ArtistAnimation(fig, images, interval=1, blit=True,repeat=False)
plt.axis('off')
plt.show()
