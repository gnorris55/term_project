#ifndef PLANE_H
#define PLANE_H

#define _USE_MATH_DEFINES

#include <learnopengl/model.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>
#include <string>
#include <map>
#include <math.h>
#include <tuple>
#include <limits>
#include <headers/ragdoll.h>

class Plane {

public:
	Shader* program;

	glm::vec4 position;
	glm::vec4 normal;
	Ragdoll* ragdoll;
	std::vector<RawModel> plane_mesh;
	int num_bones;

	Plane(glm::vec4 position, glm::vec4 normal, Ragdoll* ragdoll, Shader* program) {
		this->position = position;
		this->normal = normal;
		this->ragdoll = ragdoll;
		this->num_bones = ragdoll->get_num_bones();
		this->program = program;
		create_mesh();
	}


	// detect collision
	void detect_collisions() {
		for (int i = 0; i < num_bones; i++) {
			Bone* temp_bone = ragdoll->get_bone(i);
			if (bone_in_range(temp_bone)) {
				
				int collision_type = bone_collision(temp_bone);
				// vertex on face collision
				if (collision_type == 1) {
					
				}
				// edge on face collision
				else if (collision_type == 2) {

				}
				
			}
				
			//std::cout << glm::to_string(temp_bone->local_coord_matrix) << "\n";
		}
		draw_plane();
	}

private:
	Loader loader;
	Renderer renderer;
	void draw_plane() {
		glUniformMatrix4fv(glGetUniformLocation(program->ID, "model"), 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f)));
		
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		renderer.render(plane_mesh[0], GL_TRIANGLES);
	}

	bool bone_in_range(Bone* bone) {
		//how do we do this
		// get the center of the box
		glm::vec4 max = bone->box_max * glm::inverse(bone->local_coord_matrix);
		glm::vec4 min = bone->box_min * glm::inverse(bone->local_coord_matrix);
		glm::vec4 bone_center = (max + min) * 0.5f;

		glm::vec4 difference = bone_center - position;
		float box_height = glm::dot(difference.xyz(), normal.xyz());
		//std::cout << "diff: " << box_height << ", target: " << glm::length(bone->box_max) <<  "\n";
		if (box_height < glm::length(bone->box_max)) {
			std::cout << "in range\n";
			return true;
		}

		return false;
	}

	int bone_collision(Bone* bone) {
		

		std::vector<int> collision_points;
		std::vector<float> point_heights;
		for (int i = 0; i < bone->bone_mesh.size(); i++) {
			glm::vec4 point = bone->bone_mesh[i];
			glm::vec4 difference = point*glm::inverse(bone->local_coord_matrix) - position;
			float point_height = glm::dot(difference.xyz(), normal.xyz());

			std::cout << "point height" << point_height << "\n";
			point_heights.push_back(point_height);
			
			if (point_height < 0.0f) {
				collision_points.push_back(i);
			}
		}

		if (collision_points.size() > 0)
			collision_response(bone, collision_points, point_heights);

		return 0;
	}


	glm::vec4 get_collision_normal(Bone *bone, int collision_index, std::vector<float> point_heights) {
	
		int ref_pointA;
		int ref_pointB;
		int count = 0;
		float large = 1000;

		//need to find the face closest to the gorund and collision point
		// and calculate the normal of that face
		std::tuple<float, float> closest_heights = std::tuple<int, int>(large, large);
		std::tuple<int, int> closest_points = std::tuple<int, int>(0, 0);
		for (int i = 0; i < point_heights.size(); i++) {
			float height = point_heights[i];
			float tuple1 = std::get<0>(closest_heights);
			float tuple2 = std::get<1>(closest_heights);
			if ((height < tuple1 || height < tuple2) && i != collision_index) {
				//if tuple2 is greater than set new val
				if (tuple2 < tuple1) {
					std::get<0>(closest_heights) = height;
					std::get<0>(closest_points) = i;
				}
				else {
					std::get<1>(closest_heights) = height;
					std::get<1>(closest_points) = i;
				}

			}	
		}
		
		// creating the box normal
		// get the points in world coords than create normal
		glm::vec4 collision_point = bone->bone_mesh[collision_index]*glm::inverse(bone->local_coord_matrix);
		glm::vec4 A = bone->bone_mesh[std::get<0>(closest_points)]*glm::inverse(bone->local_coord_matrix);
		glm::vec4 B = bone->bone_mesh[std::get<1>(closest_points)]*glm::inverse(bone->local_coord_matrix);
		//std::cout << glm::to_string(collision_point)<< "\n";
		//std::cout << glm::to_string(A)<< "\n";
		//std::cout << glm::to_string(B)<< "\n";


		glm::vec3 box_normal = glm::normalize(glm::cross((collision_point - A).xyz(), (collision_point - B).xyz()));
		//std::cout << "BOX NORMAL" << glm::to_string(box_normal) << "\n";
		glm::vec3 collision_normal = glm::normalize(glm::cross(box_normal, normal.xyz()))*box_normal;
		//std::cout << "COLLISION NORMAL" << glm::to_string(collision_normal) << "\n";

		return glm::vec4(collision_normal, 0);

	}

	void collision_response(Bone* bone, std::vector<int> collision_points, std::vector<float> point_heights) {
		//want to get the normal of the box
		glm::vec4 bone_particle1 = ragdoll->get_particle(bone->particles[0]);
		glm::vec4 bone_particle2 = ragdoll->get_particle(bone->particles[1]);
		glm::vec4 bone_particle3 = ragdoll->get_particle(bone->particles[2]);
		glm::vec4 bone_particle4 = ragdoll->get_particle(bone->particles[3]);
		
		glm::vec4 v1 = ragdoll->get_force(bone->particles[0]);
		glm::vec4 v2 = ragdoll->get_force(bone->particles[1]);
		glm::vec4 v3 = ragdoll->get_force(bone->particles[2]);
		glm::vec4 v4 = ragdoll->get_force(bone->particles[3]);



		glm::vec4 collision_point = bone->bone_mesh[collision_points[0]];

		//need to find where it actually collided
		glm::vec4 p_entry = collision_point - (point_heights[collision_points[0]]) * normal;

		float linear_combination = glm::length((p_entry - bone_particle1) + (p_entry - bone_particle2) 
										     + (p_entry - bone_particle3) + (p_entry - bone_particle4));
		
		float c1 = (1.0f - (glm::length(p_entry - bone_particle1) / linear_combination)) / 3.0f;
		float c2 = (1.0f - (glm::length(p_entry - bone_particle2) / linear_combination)) / 3.0f;
		float c3 = (1.0f - (glm::length(p_entry - bone_particle3) / linear_combination)) / 3.0f;
		float c4 = (1.0f - (glm::length(p_entry - bone_particle4) / linear_combination)) / 3.0f;

		float lambda = (1 / (pow(c1, 2) + pow(c2, 2) + pow(c3, 2) + pow(c4, 2)));
		float penetration_depth = abs(point_heights[collision_points[0]]);
		//glm::vec4 collision_normal = get_collision_normal(bone, collision_points[0], point_heights);
		glm::vec4 delta_vec = (0.99f)*normal * penetration_depth;
		
		float friction_scalar = 1000.0f;
		// need to also factor in friction

		glm::vec4 new_p1 = bone_particle1 + c1 * lambda * delta_vec;
		glm::vec4 new_p2 = bone_particle2 + c2 * lambda * delta_vec;
		glm::vec4 new_p3 = bone_particle3 + c3 * lambda * delta_vec;
		glm::vec4 new_p4 = bone_particle4 + c4 * lambda * delta_vec;


		ragdoll->set_particle(bone->particles[0], new_p1);
		ragdoll->set_particle(bone->particles[1], new_p2);
		ragdoll->set_particle(bone->particles[2], new_p3);
		ragdoll->set_particle(bone->particles[3], new_p4);
		

		//ragdoll->set_force(bone->particles[0], v1 - (v1 * -penetration_depth*friction_scalar));
		//ragdoll->set_force(bone->particles[1], v2 - (v2 * -penetration_depth*friction_scalar));
		//ragdoll->set_force(bone->particles[2], v3 - (v3 * -penetration_depth*friction_scalar));
		//ragdoll->set_force(bone->particles[3], v4 - (v4 * -penetration_depth*friction_scalar));

		//std::cout << "old\n";
		//std::cout << "P1: " << glm::to_string(bone_particle1) << "\n";
		//std::cout << "P2: " << glm::to_string(bone_particle2) << "\n";
		//std::cout << "P3: " << glm::to_string(bone_particle3) << "\n";
		//std::cout << "P4: " << glm::to_string(bone_particle4) << "\n";

	}
	
	void create_mesh() {
		float sideLength = 10000;
		//glm::vec3 p1 = glm::cross(normal.xyz(), position.xyz());	
		//glm::vec3 p2 = p1*-1.0f;

		glm::vec3 v1 = normalize(cross(normal.xyz(), glm::vec3(1.0f, 0.0f, 0.0f)));
		glm::vec3 v2 = normalize(cross(normal.xyz(), v1));

		// Calculate the coordinates of the other three points
		glm::vec3 p1 = position.xyz() + sideLength / 2.0f * v1 + sideLength / 2.0f * v2;
		glm::vec3 p2 = position.xyz() + sideLength / 2.0f * v1 - sideLength / 2.0f * v2;
		glm::vec3 p3 = position.xyz() - sideLength / 2.0f * v1 + sideLength / 2.0f * v2;
	
		std::cout << "p0" << glm::to_string(position) << "p1: " << glm::to_string(p1) << ",p2: " << glm::to_string(p2) << ", p3: " << glm::to_string(p3) << "\n";
		float plane_vertices[6 * 3];
		float normals[6 * 3];
		for (int j = 0; j < 6; j++) {
			normals[(j * 3)] = normal.x;
			normals[(j * 3)+1] = normal.y;
			normals[(j * 3)+1] = normal.z;
		}

		int i = 0;

		// first triangle
		plane_vertices[i++] = p3.x;
		plane_vertices[i++] = p3.y;
		plane_vertices[i++] = p3.z;

		plane_vertices[i++] = p1.x;
		plane_vertices[i++] = p1.y;
		plane_vertices[i++] = p1.z;
		
		plane_vertices[i++] = p2.x;
		plane_vertices[i++] = p2.y;
		plane_vertices[i++] = p2.z;

		//second triangle
		plane_vertices[i++] = p3.x;
		plane_vertices[i++] = p3.y;
		plane_vertices[i++] = p3.z;

		plane_vertices[i++] = p2.x;
		plane_vertices[i++] = p2.y;
		plane_vertices[i++] = p2.z;
		
		plane_vertices[i++] = position.x;
		plane_vertices[i++] = position.y;
		plane_vertices[i++] = position.z;


		
		plane_mesh.push_back(loader.loadToVAO(plane_vertices, normals, (18) * sizeof(float)));
	}



};
#endif
