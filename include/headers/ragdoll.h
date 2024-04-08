#ifndef RAGDOLL_H
#define RAGDOLL_H
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

// TODO: add constraints to particles

struct Spring_Constraints {
	int index1;
	int index2;
	float length;
};

struct Joint {
	std::vector<int> particles;
	std::vector<int> bones;
	int ref_a;
	int ref_b;

	// variables for hinge
	std::vector<glm::vec4> plane1;
	std::vector<glm::vec4> plane2;
	float positive_angle;
	float negative_angle;
	std::vector<Spring_Constraints> particle_constraints;
	std::vector<float> stick_constraints= std::vector<float>(2);

	// variables for ball-joint
	glm::vec4 cone_origin;
	glm::vec4 axis_direction;
	float cone_angle;
	float height;
};

struct Particle {
	glm::vec4 position;
	glm::vec4 velocity;
	std::vector<int> bones;
	std::vector<int> joints;
	float mass;
};

struct Bone {
	// should always have 4  particles so that the bone forms a tetrahedron
	std::vector<int> particles;
	std::vector<Spring_Constraints> particle_constraints;
	glm::mat4 local_coord_matrix;
	std::vector<glm::vec4> bone_mesh= std::vector<glm::vec4>(8);
	glm::vec4 box_min;
	glm::vec4 box_max;
};

class Ragdoll {
public:
	glm::vec4 start_pos;
	Shader* program;

	Ragdoll(glm::vec4 starting_pos, std::string filename, Shader *program) {
		this->start_pos = starting_pos;
		this->program = program;
		load_ragdoll(filename);
	}
	
	void draw_bones(float delta_time) {
		
		
		//for (int i = 0; i < particles.size(); i++) {
			
			//Particle particle = particles[i];

			//if (i == 2) continue;
			//handle_joint_bone_constraints();
			//glm::vec4 strut_forces = get_strut_forces_on_particle(particles[i], i);
			//forces[i] = strut_forces;
		//}
		for (int i = 0; i < bones.size(); i++) {
			satisfy_bone_constraints(i);
		}

		for (int i = 0; i < particles.size(); i++) {
			Particle& particle = particles[i];
			forces[i] = forces[i] + glm::vec4(0, -9.8f, 0, 0) * particles[i].mass + particle.velocity*-0.25f;
			glm::vec4 force_value = forces[i];
			//std::cout << "our force: " << glm::to_string(force_value) << "\n";
			verlet_integration(particle, last_positions[i], force_value);
			forces[i] = glm::vec4(0, 0, 0, 0);
		}

		for (int i = 0; i < bones.size(); i++) {

			glm::mat4 model = glm::mat4(1.0f);
			model = glm::transpose(glm::translate(model, glm::vec3(0, 0, -15)));
		
			//model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0, 1, 0));
			glUniform3fv(glGetUniformLocation(program->ID, "objectColor"), 1, glm::value_ptr(glm::vec3(1.0, 0.0, 0.0)));
			draw_bone_mesh(i, model);
			update_bone_models();
			glUniformMatrix4fv(glGetUniformLocation(program->ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			renderer.render(bone_models[i], GL_TRIANGLES);
			glUniformMatrix4fv(glGetUniformLocation(program->ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
		}
	}

	void draw_bone_mesh(int bone_index, glm::mat4 curr_matrix) {
		Bone curr_bone = bones[bone_index];
		RawModel bone_mesh = bone_meshes[bone_index];
		glm::mat4 model = glm::mat4(1.0f);
		curr_matrix = glm::inverse(curr_bone.local_coord_matrix)*curr_matrix;
		//model = glm::translate(model, glm::vec3(0, 0, -15));
		glUniformMatrix4fv(glGetUniformLocation(program->ID, "model"), 1, GL_FALSE, glm::value_ptr(curr_matrix));
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		//glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);

		renderer.render(bone_mesh, GL_TRIANGLES);
	}

	int get_num_bones() {
		return bones.size();
	}
	Bone *get_bone(int i) {
		return &bones[i];
	}

	glm::vec4 get_particle(int i) {
		return particles[i].position;
	}

	glm::vec4 get_force(int i) {
		return forces[i];
	}

	void set_force(int i, glm::vec4 new_force) {
		std::cout << "before force: " << glm::to_string(forces[i]) << "\n";
		forces[i] = new_force;
		std::cout << "after force: " << glm::to_string(forces[i]) << "\n";
	}
	void set_particle(int i, glm::vec4 new_pos) {
		particles[i].position = new_pos;
	}

private:
	Loader loader;
	Renderer renderer;
	std::vector<Particle> particles;
	std::vector<RawModel> bone_models;
	std::vector<RawModel> bone_meshes;
	std::vector<glm::vec4> forces;
	std::vector<glm::vec4> last_positions;
	std::vector<Bone> bones;
	std::vector<Joint> joints;
	float time_step = 0.001;
	
	void symplectic_integration(Particle& particle, glm::vec4 force_value) {
		glm::vec4 new_velocity = particle.velocity + time_step * force_value;
		glm::vec4 new_pos = particle.position + (float)(time_step)*new_velocity;
		particle.velocity = new_velocity;
		particle.position = new_pos;

	}

	void verlet_integration(Particle& particle, glm::vec4& last_position, glm::vec4 force_value) {
		
		glm::vec4 new_pos = 2.0f * particle.position - last_position + force_value * (float)(pow(time_step, 2)) ;
		glm::vec4 new_velocity = (new_pos - last_position) / (2 * time_step);

		last_position = particle.position;
		particle.velocity = new_velocity;
		particle.position = new_pos;

	}

	void draw_plane(glm::vec4 p1, glm::vec4 p2, glm::vec4 p3, glm::mat4 local_coord_matrix) {
		glm::vec4 pw0 = p1 * glm::inverse(local_coord_matrix);
		glm::vec4 pw1 = p2 * glm::inverse(local_coord_matrix);
		glm::vec4 pw2 = p3 * glm::inverse(local_coord_matrix);
		float vertices[3 * 3] = {
			pw0.x, pw0.y, pw0.z,			
			pw1.x, pw1.y, pw1.z,
			pw2.x, pw2.y, pw2.z,
		};
		
		float normals[4 * 3];
	
		glUniform3fv(glGetUniformLocation(program->ID, "objectColor"), 1, glm::value_ptr(glm::vec3(0.0, 1.0, 0.0)));
		RawModel plane1 = loader.loadToVAO(vertices, normals, (9) * sizeof(float));
		renderer.render(plane1, GL_TRIANGLES);

	}
	
	glm::vec4 world_coord(glm::vec4 pos, glm::mat4 localMatrix) {
		return pos * glm::inverse(localMatrix);
	}

	void draw_cone(Joint joint, float cone_radius, glm::mat4 curr_matrix, float curr_height) {
		float delta_angle = 15.0f;
		//std::cout << glm::to_string(joint.axis_direction * glm::inverse(curr_matrix)) << "\n";
		float num_segments = 360.0f / delta_angle;
		float circle[24 * 3];
		for (int i = 0; i < num_segments; i++) {
			float theta = 2.0f * M_PI * i / num_segments;
			float x = cone_radius * cos(theta);
			float z = cone_radius * sin(theta);

			glm::vec4 point = glm::vec4((joint.cone_origin.x + x), (joint.cone_origin.y + curr_height), (joint.cone_origin.z + z), 1.0f) * glm::inverse(curr_matrix);
			//std::cout <<"cone point" << glm::to_string(point) << "\n";
			circle[i * 3] = point.x;
			circle[(i * 3) + 1] = point.y;
			circle[(i * 3) + 2] = point.z;
		}
		circle[23 * 3] = circle[0];
		circle[(23 * 3) + 1] = circle[1];
		circle[(23 * 3) + 2] = circle[2];
		glUniform3fv(glGetUniformLocation(program->ID, "objectColor"), 1, glm::value_ptr(glm::vec3(0.0, 0.0, 1.0)));
		RawModel circle_model = loader.loadToVAO(circle, {}, (24*3)*sizeof(float));
		renderer.render(circle_model, GL_LINES);
	}

	void handle_ball_joint_constraints(Joint& joint) {
	
		Bone boneA = bones[joint.bones[0]];
		Bone boneB = bones[joint.bones[1]];

		glm::vec4 b = particles[joint.ref_b].position * boneA.local_coord_matrix;

		float base_radius = tan(glm::radians(joint.cone_angle)) * joint.height;
		float cone_dist = glm::dot(b - joint.cone_origin, joint.axis_direction);
		float cone_radius = (cone_dist / joint.height) * base_radius;
		float orthogonal_dis = glm::length((b - joint.cone_origin) - cone_dist * joint.axis_direction);
		
		//for (int i = 0; i < 5; i++) {
			//float perc = (float)i / 5;
			//draw_cone(joint, perc*base_radius, boneA.local_coord_matrix, perc*joint.height);
		//}

		float a = glm::length(joint.cone_origin - b);
		glm::vec4 p_c_diff = b- joint.cone_origin;
		float p_distance = glm::dot(p_c_diff.xyz(), glm::normalize(joint.axis_direction.xyz()));
		float c_distance = (a*sin(M_PI/2.0f - glm::radians(joint.cone_angle)));
		float error = abs(glm::length(c_distance));
		float perc = (float) abs(p_distance) / joint.height;
		//draw_cone(joint, perc*base_radius, boneA.local_coord_matrix, perc*joint.height);
		//std::cout << "C DISTANCE: " << (c_distance) << "!!\n";
		//std::cout << "P DISTANCE: " << (p_distance) << "!!\n";
		// should I use projection instead? all problems come from a the length of the constraints increasing expoenetially

		if (c_distance > p_distance) {
			//project_ball_joint_violation(particles[joint.ref_b], joint);
			glm::vec4 hyp = -1.0f * p_c_diff;
			glm::vec3 adjacent = glm::cross(joint.axis_direction.xyz(), joint.cone_origin.xyz());
			Particle curr_a = particles[joint.ref_a];

				//std::cout << "satisfying ball joint constraints" << "\n";
				//satisfy_constraints(particles[joint.ref_b], curr_a,
					//glm::length(particles[joint.ref_b].position - curr_a.position) + (c_distance - p_distance));
			//	std::cout << "particle pos: " << glm::to_string(particles[joint.ref_b].position) << "\n";
				glm::vec4 temp_force = get_force_for_spring(particles[joint.ref_b], curr_a, glm::length(particles[joint.ref_b].position - curr_a.position) + (c_distance - p_distance), 600, 200);
				forces[joint.ref_b] = forces[joint.ref_b] + temp_force;
				forces[joint.ref_a] = forces[joint.ref_a] - temp_force;

				//std::cout << "CONSTRAINT SIZE: " << glm::length(particles[joint.ref_b].position - curr_a.position) + abs(abs(c_distance) - abs(p_distance)+0.01) << "\n";
				//std::cout << "DIFFERENCE SIZE: " << glm::length(particles[joint.ref_b].position - curr_a.position) << "\n";
				//std::cout << "LENGTH: " << a << "\n";

		}
	
	}
	
	void project_ball_joint_violation(Particle &particle, Joint joint) {
		
	}

	void handle_hinge_constraints(Joint& joint) {
		Bone boneA = bones[joint.bones[0]];
		Bone boneB = bones[joint.bones[1]];

		glm::vec4 b  = particles[joint.ref_b].position * boneA.local_coord_matrix;
		draw_plane(joint.plane2[0], joint.plane2[1], joint.plane2[2], boneA.local_coord_matrix);
		draw_plane(joint.plane1[0], joint.plane1[1], joint.plane1[2], boneA.local_coord_matrix);
		
		glm::vec3 diff = glm::normalize(b - joint.plane1[0]);
		float plane_val1 = glm::dot(diff.xyz(), joint.plane1[3].xyz());
		
		diff = glm::normalize(b - joint.plane2[0]);
		float plane_val2 = glm::dot(diff.xyz(), joint.plane2[3].xyz());


		if (plane_val1 < 0 || plane_val2 < 0) {
			std::cout << "violated constraint\n";
			float length;
			Particle line_constraint_particle1;
			Particle line_constraint_particle2;
			if (plane_val1 < 0) {
				satisfy_constraints(particles[joint.ref_b], particles[joint.ref_a], joint.stick_constraints[0]);
				satisfy_constraints(particles[joint.ref_a], particles[joint.ref_b], joint.stick_constraints[0]);
			}
			else {
				satisfy_constraints(particles[joint.ref_b], particles[joint.ref_a], joint.stick_constraints[1]);
				satisfy_constraints(particles[joint.ref_a], particles[joint.ref_b], joint.stick_constraints[1]);
			}
		}


	}

	void handle_joint_bone_constraints() {
		for (Bone& boneA : bones) {
			glm::vec4 p1 = particles[boneA.particles[0]].position;
			glm::vec4 p2 = particles[boneA.particles[1]].position;
			glm::vec4 p3 = particles[boneA.particles[2]].position;
			glm::vec4 p4 = particles[boneA.particles[3]].position;

			glm::vec3 x_axis = glm::normalize(p2 - p1).xyz();
			glm::vec3 y_axis = glm::normalize(glm::cross(x_axis, (p3 - p1).xyz()));
			glm::vec3 z_axis = glm::cross(x_axis, y_axis);

			glm::mat4 local_coords = glm::mat4(glm::vec4(x_axis.x, y_axis.x, z_axis.x, p1.x),
				glm::vec4(x_axis.y, y_axis.y, z_axis.y, p1.y),
				glm::vec4(x_axis.z, y_axis.z, z_axis.z, p1.z),
				glm::vec4(0, 0, 0, 1));

			boneA.local_coord_matrix = glm::inverse(local_coords);
		}
		
		for (Joint& joint : joints) {
			if (joint.particles.size() == 2) {
				handle_hinge_constraints(joint);
			}
			else if (joint.particles.size() == 1) {
			//	std::cout << "handling ball joint\n";
				handle_ball_joint_constraints(joint);
			}
		}
	}
	
	glm::vec4 get_strut_forces_on_particle(Particle &particle, int index) {
		glm::vec4 total_strut_force = glm::vec4(0, -9.8, 0, 0)*particle.mass /* + particle.velocity * -0.05f*/;
		bool immovable = false;
		//glm::vec4 total_strut_force = glm::vec4(0, 0, 0, 0);
		float stiffness = 100000;
		float dampness = 500;

		for (int bone_index : particle.bones) {
			//std::cout << "curr_index: " << index << "\n";
			for (auto constraint : bones[bone_index].particle_constraints) {
				if (constraint.index1 == index) {
					//total_strut_force = total_strut_force + get_force_for_spring(particle, particles[constraint.index2], constraint.length, stiffness, dampness);
					satisfy_constraints(particle, particles[constraint.index2], constraint.length);
				}
				else if (constraint.index2 == index) {
					//total_strut_force = total_strut_force + get_force_for_spring(particle, particles[constraint.index1], constraint.length, stiffness, dampness);
					satisfy_constraints(particle, particles[constraint.index1], constraint.length);
				}
			}


		}
		return total_strut_force;
	}
	
	void satisfy_bone_constraints(int index) {
		for (auto constraint : bones[index].particle_constraints) {
			handle_joint_bone_constraints();
			//std::cout << glm::to_string(particles[constraint.index1].position) << "\n";
			satisfy_constraints(particles[constraint.index1], particles[constraint.index2], constraint.length);
		}
	}

	//TODO: credit paper: Thomas Jakobsen
	void satisfy_constraints(Particle &particle1, Particle& particle2, float rest_length, bool move_completely = false) {
	
		float epsilon = 0.01;
		glm::vec4 old_delta = glm::vec4(0, 0, 0, 0);
		glm::vec4 delta= particle2.position - particle1.position;
		while (epsilon <= abs(glm::length(delta - old_delta))) {
			//std::cout << "p1: " << glm::to_string(particle1.position) << ", p2: " << glm::to_string(particle2.position) << "\n";
			//std::cout << glm::to_string(delta) << "\n";
			
			float delta_length = sqrt(glm::dot(delta.xyz(), delta.xyz()));
			float diff = (delta_length - rest_length) / (delta_length * ((1 / particle1.mass) + (1 / particle2.mass)));
			//if (abs(delta_length - rest_length) > 0 &&) {
			particle1.position = particle1.position + (1 / particle1.mass) * delta * diff;
			//if (!move_completely)
			particle2.position = particle2.position - (1 / particle2.mass) * delta * diff;

			old_delta = delta;
			delta  = particle2.position - particle1.position;
		}


	}
	
	glm::vec4 get_force_for_spring(Particle particle1, Particle particle2, float rest_length, float stiffness_val, float damping_val) {
	
		glm::vec4 length_vector;
		if (particle1.position == particle2.position) {
			length_vector = glm::vec4(0.01, 0.01, 0.01, 0);
		}
		else {
			length_vector = particle1.position - particle2.position;
			//std::cout << glm::to_string(length_vector) << "\n";
		}
		glm::vec4 spring_force = (stiffness_val) * (rest_length - glm::length(length_vector)) * glm::normalize(length_vector);
		glm::vec4 damping_force = (-damping_val) * (particle1.velocity - particle2.velocity) * glm::normalize(length_vector) * glm::normalize(length_vector);
		std::cout << glm::to_string(spring_force) << "\n";
		return (spring_force + damping_force) * (float)((1 / (particle1.mass +particle2.mass)/2));
	}
	
	//loads the particles and starting positions of a ragdoll skeleton
	void load_ragdoll(std::string filename) {
		std::string currLine;
		std::ifstream myFile;
		myFile.open(filename);

		// create some temperary variables
		int load_state;
		if (myFile.is_open()) {
			std::getline(myFile, currLine);
	
			//getting file contents line by line
			while (std::getline(myFile, currLine)) {
				std::stringstream ss(currLine);
				std::string tokenStrs;
				std::vector<float> inputNums;

				std::getline(ss, tokenStrs, ' ');
				std::string curr_item = tokenStrs;
				std::cout << curr_item << "\n";
				if (strcmp(curr_item.c_str(), "particle") == 0)
					load_state = 0;
				else if (strcmp(curr_item.c_str(), "bone") == 0)
					load_state = 1;
				else if (strcmp(curr_item.c_str(), "joint") == 0)
					load_state = 2;
				else
					continue;
				
				while (std::getline(ss, tokenStrs, ' ')) {
					inputNums.push_back(stof(tokenStrs));
				}

				if (load_state == 0 && inputNums.size() == 3) {
					Particle new_particle;
					std::cout << inputNums[0] << "\n";
					std::cout << inputNums[1] << "\n";
					std::cout << inputNums[2] << "\n";
					new_particle.position = glm::vec4(inputNums[0], inputNums[1], inputNums[2], 1.0);
					new_particle.velocity = glm::vec4(0.0, 0.0, 0.0, 0.0);
					std::cout << glm::to_string(new_particle.position) << "\n";
					last_positions.push_back(((new_particle.position - new_particle.velocity * time_step) + new_particle.velocity * time_step));
					if (particles.size() == 3)
						new_particle.mass = 200;
					else
						new_particle.mass = 200;
	
					particles.push_back(new_particle);
				}
				if (load_state == 1 && inputNums.size() == 10) {
					particles[inputNums[0]].bones.push_back(bones.size());
					particles[inputNums[1]].bones.push_back(bones.size());
					particles[inputNums[2]].bones.push_back(bones.size());
					particles[inputNums[3]].bones.push_back(bones.size());
					create_bone(inputNums);
				}
				if (load_state == 2 && inputNums.size() >= 3) {
						//do hinge stuff
					std::cout << "setting a joint\n";
					create_joint(inputNums);
				}
			}
			forces = std::vector<glm::vec4>(particles.size());
		}
		else {
			std::cout << "could not open file\n";
		}
	}

	int get_referance_particle(Joint joint, Bone bone) {

		for (int i = 0; i < bone.particles.size(); i++) {
			bool is_hinge = false;
			for (int j = 0; j < joint.particles.size(); j++) {
				if (bone.particles[i] == joint.particles[j]) {
					is_hinge = true;
					break;
				}
			}
			if (!is_hinge) {
				return bone.particles[i];
			}
		}

	}
	
	void create_hinge_rotation_planes(Joint &joint) {
		std::cout << "creating hinge plane constraints\n";
		Bone boneA = bones[joint.bones[0]];
		Bone boneB = bones[joint.bones[1]];

		glm::vec4 a;
		glm::vec4 b;
		
		for (int i = 0; i < boneA.particles.size(); i++) {
			bool is_hinge = false;
			for (int j = 0; j < joint.particles.size(); j++) {
				if (boneA.particles[i] == joint.particles[j]) {
					is_hinge = true;
					break;
				}
			}
			if (!is_hinge) {
				a = particles[boneA.particles[i]].position * boneA.local_coord_matrix;
				joint.ref_a = boneA.particles[i];
			}
		}
		
		for (int i = 0; i < boneB.particles.size(); i++) {
			bool is_hinge = false;
			for (int j = 0; j < joint.particles.size(); j++) {
				if (boneB.particles[i] == joint.particles[j]) {
					is_hinge = true;
					break;
				}
			}
			if (!is_hinge) {
				b = particles[boneB.particles[i]].position * boneA.local_coord_matrix;
				joint.ref_b = boneB.particles[i];
			}
		}

			//	glm::vec4 b = particles[boneB.particles[i]].position * boneA.local_coord_matrix;
		glm::vec4 hinge_particle1 = particles[joint.particles[0]].position * boneA.local_coord_matrix;
		glm::vec4 hinge_particle2 = particles[joint.particles[1]].position * boneA.local_coord_matrix;

		glm::vec4 hinge_axis = (hinge_particle1 - hinge_particle2);
		glm::vec4 rotation_b = (b - hinge_particle1);
		glm::vec3 rotation_axis = glm::normalize(glm::cross(hinge_axis.xyz(), glm::normalize(b).xyz()));
		
		glm::mat4 rot_positive = glm::mat4(1.0f);
		rot_positive = glm::rotate(rot_positive, glm::radians(joint.positive_angle), glm::normalize(hinge_axis.xyz()));
		
		glm::mat4 rot_negative = glm::mat4(1.0f);
		rot_negative = glm::rotate(rot_negative, glm::radians(joint.negative_angle), glm::normalize(hinge_axis.xyz()));
		
		glm::vec4 plane_positive_point = (rotation_b * rot_positive) + hinge_particle1;
		glm::vec4 plane_negative_point = (rotation_b * rot_negative) + hinge_particle1;
		glm::vec3 positive_normal = glm::normalize(glm::cross((plane_positive_point - hinge_particle1).xyz(), (plane_positive_point - hinge_particle2).xyz()));
		glm::vec3 negative_normal = glm::normalize(glm::cross((plane_negative_point - hinge_particle1).xyz(), (plane_negative_point - hinge_particle2).xyz()));

		joint.plane1.push_back(plane_positive_point);
		joint.plane1.push_back(hinge_particle1);
		joint.plane1.push_back(hinge_particle2);
		joint.plane1.push_back(-1.0f*glm::vec4(positive_normal, 0));
		
		joint.plane2.push_back(plane_negative_point);
		joint.plane2.push_back(hinge_particle1);
		joint.plane2.push_back(hinge_particle2);
		joint.plane2.push_back(glm::vec4(negative_normal, 0));

		joint.stick_constraints[0] = (abs(glm::length(particles[joint.ref_a].position * boneA.local_coord_matrix - plane_positive_point)));
		joint.stick_constraints[1] = (abs(glm::length(particles[joint.ref_a].position * boneA.local_coord_matrix - plane_negative_point)));
	}
	void create_ball_joint_rotation(Joint& joint) {
	
		Bone boneA = bones[joint.bones[0]];
		Bone boneB = bones[joint.bones[1]];

		glm::vec4 a;
		glm::vec4 b;
	
		glm::vec4 ref_joint = particles[joint.particles[0]].position;
		std::cout << joint.particles.size() << "\n";
		for (int i = 0; i < boneA.particles.size(); i++) {
			bool is_hinge = false;
			for (int j = 0; j < joint.particles.size(); j++) {
				if (boneA.particles[i] == joint.particles[j]) {
					is_hinge = true;
					break;
				}
			}
			if (!is_hinge) {
				a = particles[boneA.particles[i]].position * boneA.local_coord_matrix;
				joint.ref_a = boneA.particles[i];
			}
		}
		
		for (int i = 0; i < boneB.particles.size(); i++) {
			bool is_hinge = false;
			for (int j = 0; j < joint.particles.size(); j++) {
				if (boneB.particles[i] == joint.particles[j]) {
					is_hinge = true;
					break;
				}
			}
			if (!is_hinge) {
				glm::vec4 temp_position = particles[boneB.particles[i]].position - ref_joint;
				if (temp_position.x == 0 && temp_position.z == 0 || temp_position.y == 0 && temp_position.z == 0 || temp_position.y == 0 && temp_position.x == 0) {
					b = particles[boneB.particles[i]].position * boneA.local_coord_matrix;
					joint.ref_b = boneB.particles[i];
				}
			}
		}
		std::cout << "ref b: " << joint.ref_b << "\n";
		joint.cone_origin = ref_joint * boneA.local_coord_matrix;
		joint.axis_direction = glm::normalize((b - joint.cone_origin));
		std::cout << "cone driection: " << glm::to_string(joint.axis_direction * glm::inverse(boneA.local_coord_matrix)) << "\n";
		joint.height = glm::length(b - joint.cone_origin);
	}

	void create_joint(std::vector<float> inputNums) {
		int bone_index1 = (int)inputNums[0];
		int bone_index2 = (int)inputNums[1];
		int connections = (int)inputNums[2];
		
		Bone bone1 = bones[bone_index1];
		Bone bone2 = bones[bone_index2];

		std::vector<int> particle_index;
		Joint new_joint;
		new_joint.bones.push_back(bone_index1);
		new_joint.bones.push_back(bone_index2);
		if (connections == 2) {
			std::cout << "creating hinge\n";
			new_joint.positive_angle = inputNums[3];
			new_joint.negative_angle = inputNums[4];
		}

		if (connections == 1) {
			new_joint.cone_angle = inputNums[3];
		}

		int joint_count = 0;
		for (int i = 0; i < bone1.particles.size(); i++) {
			for (int j = 0; j < bone2.particles.size(); j++) {
				int particle_index1 = bone1.particles[i];
				int particle_index2 = bone2.particles[j];
				std::cout << "p1: " << particle_index1 << " p2: " << particle_index2 << "\n";
				if (particle_index1 == particle_index2) {
					std::cout << "bingo\n";
					new_joint.particles.push_back(particle_index1);
				
					joint_count++;
				}
			}
		}

		if (joint_count != connections) {
			std::cout << "error: wrong number of connections\n";
		}

		if (new_joint.particles.size() == 2)
			create_hinge_rotation_planes(new_joint);

		if (new_joint.particles.size() == 1)
			create_ball_joint_rotation(new_joint);
		
		joints.push_back(new_joint);
		std::cout << "finished processing joint\n";
	}
	
	void form_triangle(float arr[], float normals[], int triangle_index, glm::vec4 p1, glm::vec4 p2, glm::vec4 p3) {
		int arr_index = triangle_index * 9;
		glm::vec4 edge1 = p2 - p1;
		glm::vec4 edge2 = p3 - p1;
		glm::vec3 normal = glm::cross(edge1.xyz(), edge2.xyz());
		
		/*for (int i = arr_index; i < arr_index + 3; i++) {
			normals[i*3] = normal.x;
			normals[i*3+1] = normal.y;
			normals[i*3+2] = normal.z;
		}*/

		arr[arr_index]	 = p1.x;
		arr[++arr_index] = p1.y;
		arr[++arr_index] = p1.z;
		arr[++arr_index] = p2.x;
		arr[++arr_index] = p2.y;
		arr[++arr_index] = p2.z;
		arr[++arr_index] = p3.x;
		arr[++arr_index] = p3.y;
		arr[++arr_index] = p3.z;
	}

	void create_bone(std::vector<float> inputNums) {
		
		std::vector<int> particle_list = { (int)inputNums[0], (int)inputNums[1], (int)inputNums[2] , (int)inputNums[3]};
		glm::vec3 position = glm::vec3(inputNums[4], inputNums[5], inputNums[6]);
		std::cout <<  "Position: " << glm::to_string(position) << "\n";
		glm::vec3 scale = glm::vec3(inputNums[7], inputNums[8], inputNums[9]);
		
		const int num_vertices = 4 * 3 * 3;
		float input_vertices[num_vertices];
		float normals[num_vertices];
		


		//add the length constraints
		glm::vec4 p1 = particles[particle_list[0]].position;
		glm::vec4 p2 = particles[particle_list[1]].position;
		glm::vec4 p3 = particles[particle_list[2]].position;
		glm::vec4 p4 = particles[particle_list[3]].position;

		glm::vec3 x_axis = glm::normalize(p2 - p1).xyz();
		glm::vec3 y_axis = glm::normalize(glm::cross(x_axis, (p3 - p1).xyz()));
		glm::vec3 z_axis = glm::cross(x_axis, y_axis);

		glm::mat4 local_coords = glm::mat4(glm::vec4(x_axis.x, y_axis.x, z_axis.x, p1.x),
			glm::vec4(x_axis.y, y_axis.y, z_axis.y, p1.y),
			glm::vec4(x_axis.z, y_axis.z, z_axis.z, p1.z),
			glm::vec4(0, 0, 0, 1));
	
		Bone new_bone;
		new_bone.local_coord_matrix = glm::inverse(local_coords);
	    new_bone.particle_constraints.push_back(get_spring_constraint(p1 - p2, particle_list[0], particle_list[1]));
	    new_bone.particle_constraints.push_back(get_spring_constraint(p1 - p3, particle_list[0], particle_list[2]));
		new_bone.particle_constraints.push_back(get_spring_constraint(p2 - p3, particle_list[1], particle_list[2]));
		new_bone.particle_constraints.push_back(get_spring_constraint(p1 - p4, particle_list[0], particle_list[3]));
		new_bone.particle_constraints.push_back(get_spring_constraint(p2 - p4, particle_list[1], particle_list[3]));
		new_bone.particle_constraints.push_back(get_spring_constraint(p3 - p4, particle_list[2], particle_list[3]));
		new_bone.particles = particle_list;


		std::cout << glm::to_string(p1) << "\n";
		std::cout << glm::to_string(p2) << "\n";
		std::cout << glm::to_string(p3) << "\n";
		std::cout << glm::to_string(p4) << "\n";
		form_triangle(input_vertices, normals, 0, p1, p2, p3);
		form_triangle(input_vertices, normals, 1, p1, p2, p4);
		form_triangle(input_vertices, normals, 2, p1, p3, p4);
		form_triangle(input_vertices, normals, 3, p2, p3, p4);
		std::cout << glm::to_string(p1) << "\n";
		std::cout << glm::to_string(p2) << "\n";
		std::cout << glm::to_string(p3) << "\n";
		std::cout << glm::to_string(p4) << "\n";


		create_bone_mesh(new_bone, position, scale);

		RawModel new_rawModel = loader.loadToVAO(input_vertices, normals, num_vertices*sizeof(float));
		bone_models.push_back(new_rawModel);

		bones.push_back(new_bone);
	}

	void create_bone_mesh(Bone& bone, glm::vec3 displacement, glm::vec3 scale) {
		
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::translate(model, displacement);
		model = glm::transpose(glm::scale(model, scale));

		glm::vec4 pa = particles[bone.particles[0]].position*model;
		glm::vec4 pb = particles[bone.particles[1]].position*model;
		glm::vec4 pc = particles[bone.particles[2]].position*model;
		glm::vec4 pd = particles[bone.particles[3]].position*model;
	


		float max_x = std::max(pa.x, std::max(pb.x, std::max(pc.x, pd.x)));
		float max_y = std::max(pa.y, std::max(pb.y, std::max(pc.y, pd.y)));
		float max_z = std::max(pa.z, std::max(pb.z, std::max(pc.z, pd.z)));
	
		float min_x = std::min(pa.x, std::min(pb.x, std::min(pc.x, pd.x)));
		float min_y = std::min(pa.y, std::min(pb.y, std::min(pc.y, pd.y)));
		float min_z = std::min(pa.z, std::min(pb.z, std::min(pc.z, pd.z)));

	
		glm::vec4 maxes = glm::vec4(max_x, max_y, max_z, 1);
		glm::vec4 mins = glm::vec4(min_x, min_y, min_z, 1);
		std::cout << "max: " << glm::to_string(maxes) << ", min: " << glm::to_string(mins) << "\n";

		const int num_vertices = 12 * 3 * 3;
		float mesh_vertices[num_vertices];
		float normals[num_vertices];
	
		glm::vec4 p1 = glm::vec4(mins.x, mins.y, maxes.z, 1)*bone.local_coord_matrix;
		glm::vec4 p2 = glm::vec4(mins.x, maxes.y, maxes.z, 1)*bone.local_coord_matrix;
		glm::vec4 p3 = glm::vec4(maxes.x, maxes.y, maxes.z, 1)*bone.local_coord_matrix;
		glm::vec4 p4 = glm::vec4(maxes.x, mins.y, maxes.z, 1)*bone.local_coord_matrix;

		glm::vec4 p5 = glm::vec4(mins.x, mins.y, mins.z, 1)*bone.local_coord_matrix;
		glm::vec4 p6 = glm::vec4(mins.x, maxes.y, mins.z, 1)*bone.local_coord_matrix;
		glm::vec4 p7 = glm::vec4(maxes.x, maxes.y, mins.z, 1)*bone.local_coord_matrix;
		glm::vec4 p8 = glm::vec4(maxes.x, mins.y, mins.z, 1)*bone.local_coord_matrix;
		
		bone.bone_mesh[0] = p1;
		bone.bone_mesh[1] = p2;
		bone.bone_mesh[2] = p3;
		bone.bone_mesh[3] = p4;
		bone.bone_mesh[4] = p5;
		bone.bone_mesh[5] = p6;
		bone.bone_mesh[6] = p7;
		bone.bone_mesh[7] = p8;

		bone.box_max = maxes * bone.local_coord_matrix;
		bone.box_min = mins * bone.local_coord_matrix;

		//front side
		form_triangle(mesh_vertices, normals, 0, p1, p2, p3);
		form_triangle(mesh_vertices, normals, 1, p1, p3, p4);

		//left side  
		form_triangle(mesh_vertices, normals, 2, p1, p2, p6);
		form_triangle(mesh_vertices, normals, 3, p1, p6, p5);
		
		//right side
		form_triangle(mesh_vertices, normals, 4, p4, p3, p7);
		form_triangle(mesh_vertices, normals, 5, p4, p7, p8);

		//back side
		form_triangle(mesh_vertices, normals, 6, p5, p6, p7);
		form_triangle(mesh_vertices, normals, 7, p5, p7, p8);

		//top
		form_triangle(mesh_vertices, normals, 8, p2, p6, p7);
		form_triangle(mesh_vertices, normals, 9, p2, p7, p3);
		
		//bottom
		form_triangle(mesh_vertices, normals, 10, p1, p5, p8);
		form_triangle(mesh_vertices, normals, 11, p1, p8, p4);
		/*
		for (int i = 0; i < num_vertices; i++) {
			std::cout << mesh_vertices[i] << "\n";
		}*/

		RawModel new_rawModel = loader.loadToVAO(mesh_vertices, normals, num_vertices*sizeof(float));
		bone_meshes.push_back(new_rawModel);
	}

	Spring_Constraints get_spring_constraint(glm::vec4 length, int index1, int index2) {
		Spring_Constraints new_constraint;
		new_constraint.index1 = index1;
		new_constraint.index2 = index2;
		new_constraint.length = abs(glm::length(length));
		return new_constraint;
	}


	void update_bone_models() {
		for (int i = 0; i < bones.size(); i++) {

			glm::vec4 p1 = particles[bones[i].particles[0]].position;
			glm::vec4 p2 = particles[bones[i].particles[1]].position;
			glm::vec4 p3 = particles[bones[i].particles[2]].position;
			glm::vec4 p4 = particles[bones[i].particles[3]].position;
			const int num_vertices = 4 * 3 * 3;
			float input_vertices[num_vertices];
			float normals[num_vertices];
			form_triangle(input_vertices, normals, 0, p1, p2, p3);
			form_triangle(input_vertices, normals, 1, p1, p2, p4);
			form_triangle(input_vertices, normals, 2, p1, p3, p4);
			form_triangle(input_vertices, normals, 3, p2, p3, p4);
			RawModel new_rawModel = loader.loadToVAO(input_vertices, normals, num_vertices * sizeof(float));
			bone_models[i] = new_rawModel;
		}
	}

	void print_bones() {
		for (auto bone : bones) {
			for (auto vec : bone.particles)
				std::cout << vec << "\n";
		}
	}

	void print_particles() {
		for (auto particle : particles)
			std::cout << glm::to_string(particle.position) << "\n";
	}
};

#endif
