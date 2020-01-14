#!/usr/bin/python3
import os
import copy
import numpy
import open3d
import pyquaternion


def change_background_to_black(vis):
    opt = vis.get_render_option()
    opt.background_color = numpy.asarray([0, 0, 0])
    opt.point_size = 1.
    opt.light_on = True
    opt.show_coordinate_frame = True
    #     opt.load_from_json('./opt.json')
    return False

key_to_callback = {ord("K"): change_background_to_black}


class Keyframe(object):
	def __init__(self, id, cloud, odom):
		self.id = id
		self.odom = copy.deepcopy(odom)
		self.cloud = copy.deepcopy(cloud)
		self.transformed = copy.deepcopy(cloud)

		self.node = open3d.registration.PoseGraphNode(odom)
		self.update_transformed()

	def update_transformed(self):
		self.transformed.points = self.cloud.points
		self.transformed.normals = self.cloud.normals
		self.transformed.transform(self.node.pose)


class GraphSLAM(object):
	def __init__(self):
		self.graph = open3d.registration.PoseGraph()

		self.keyframes = []
		self.last_frame_transformation = numpy.identity(4)

		self.odom = numpy.identity(4)

		self.keyframe_angle_thresh_deg = 15.0
		self.keyframe_trans_thresh_m = 1.0

		self.vis = open3d.visualization.Visualizer()
		self.vis.create_window()

	def update(self, cloud):
		cloud = cloud.voxel_down_sample(voxel_size=0.05)

		if not len(self.keyframes):
			self.keyframes.append(Keyframe(0, cloud, numpy.identity(4)))
			self.vis.add_geometry(self.keyframes[-1].transformed)
			self.graph.nodes.append(self.keyframes[-1].node)
			return

		if not self.update_keyframe(cloud):
			return

		print('optimizing...')
		option = open3d.registration.GlobalOptimizationOption(max_correspondence_distance=1.0,
															  edge_prune_threshold=0.25, reference_node=0)
		print(open3d.registration.global_optimization(self.graph,
													  open3d.registration.GlobalOptimizationLevenbergMarquardt(),
													  open3d.registration.GlobalOptimizationConvergenceCriteria(),
													  option))

		for keyframe in self.keyframes:
			keyframe.update_transformed()

		self.vis.poll_events()
		self.vis.update_renderer()

	def update_keyframe(self, cloud):
		criteria = open3d.registration.ICPConvergenceCriteria(max_iteration=100)
		reg = open3d.registration.registration_icp(cloud,
												   self.keyframes[-1].cloud,
												   1.0,
												   self.last_frame_transformation,
												   open3d.registration.TransformationEstimationPointToPlane(),
												   criteria=criteria)

		angle = pyquaternion.Quaternion(matrix=reg.transformation[:3, :3]).degrees
		trans = numpy.linalg.norm(reg.transformation[:3, 3])

		if abs(angle) < self.keyframe_angle_thresh_deg and abs(trans) < self.keyframe_trans_thresh_m:
			self.last_frame_transformation = reg.transformation
			return False

		self.odom = numpy.dot(self.keyframes[-1].odom, reg.transformation)
		self.keyframes.append(Keyframe(len(self.keyframes), cloud, self.odom))
		self.graph.nodes.append(self.keyframes[-1].node)
		self.vis.add_geometry(self.keyframes[-1].transformed)

		self.last_frame_transformation = numpy.identity(4)

		information = open3d.registration.get_information_matrix_from_point_clouds(self.keyframes[-1].cloud,
																				   self.keyframes[-2].cloud,
																				   1.0,
																				   reg.transformation)
		edge = open3d.registration.PoseGraphEdge(self.keyframes[-1].id,
												 self.keyframes[-2].id,
												 reg.transformation,
												 information,
												 uncertain=False)

		self.graph.edges.append(edge)

		return True

	def generate_map(self):
		map_cloud = open3d.geometry.PointCloud()
		for keyframe in self.keyframes:
			transformed = copy.deepcopy(keyframe.cloud)
			transformed.transform(keyframe.node.pose)
			map_cloud += transformed

		return map_cloud.voxel_down_sample(voxel_size=0.05)


def main():
	dataset_path = '/home/koide/datasets/kit/scenario1'
	cloud_files = sorted([dataset_path + '/' + x for x in os.listdir(dataset_path) if '.pcd' in x])

	graph_slam = GraphSLAM()

	for cloud_file in cloud_files[:100]:
		print(cloud_file)
		cloud = open3d.io.read_point_cloud(cloud_file)
		graph_slam.update(cloud)
	graph_slam.vis.destroy_window()
	open3d.visualization.draw_geometries_with_key_callbacks([graph_slam.generate_map(), ], key_to_callback)


if __name__ == '__main__':
	main()
