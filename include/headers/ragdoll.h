#ifndef RAGDOLL_H
#define RAGDOLL_H

#include <learnopengl/model.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>
#include <string>
#include <map>

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
		

		for (int i = 0; i < particles.size(); i++) {
			
			//Particle particle = particles[i];

			if (i < 4) continue;
			glm::vec4 strut_forces = get_strut_forces_on_particle(particles[i], i);
			forces[i] = strut_forces;
			handle_joint_bone_constraints();
		}

		for (int i = 0; i < particles.size(); i++) {
			Particle& particle = particles[i];
			glm::vec4 force_value = forces[i];
			//symplectic_integration(particle, force_value);
			verlet_integration(particle, last_positions[i], force_value);
			
		}

		for (int i = 0; i < bones.size(); i++) {

			glm::mat4 model = glm::mat4(1.0f);
			model = glm::translate(model, glm::vec3(0, 0, -15));
			//model = glm::rotate(model, glm::radians(90.0f), glm::vec3(0, 1, 0));
			glUniform3fv(glGetUniformLocation(program->ID, "objectColor"), 1, glm::value_ptr(glm::vec3(1.0, 0.0, 0.0)));
			glUniformMatrix4fv(glGetUniformLocation(program->ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
			update_bone_models();
			renderer.render(bone_models[i], GL_TRIANGLES);
		}
	}

private:
	Loader loader;
	Renderer renderer;
	std::vector<Particle> particles;
	std::vector<RawModel> bone_models;
	std::vector<glm::vec4> forces;
	std::vector<glm::vec4> last_positions;
	std::vector<Bone> bones;
	std::vector<Joint> joints;
	float time_step = 0.005;
	
	void symplectic_integration(Particle& particle, glm::vec4 force_value) {
		glm::vec4 new_velocity = particle.velocity + time_step * force_value;
		glm::vec4 new_pos = particle.position + (float)(time_step)*new_velocity;
		particle.velocity = new_velocity;
		particle.position = new_pos;

	}

	void verlet_integration(Particle& particle, glm::vec4& last_position, glm::vec4 force_value) {
		
		glm::vec4 new_pos = 2.0f * particle.position - last_position + force_value * (float)(pow(time_step, 2));
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

	void handle_ball_joint_constraints(Joint& joint) {
	
		Bone boneA = bones[joint.bones[0]];
		Bone boneB = bones[joint.bones[1]];

		glm::vec4 b = particles[joint.ref_b].position * boneA.local_coord_matrix;

		float base_radius = tan(glm::radians(joint.cone_angle)) * joint.height;
		float cone_dist = glm::dot(b - joint.cone_origin, joint.axis_direction);
		float cone_radius = (cone_dist / joint.height) * base_radius;
		float orthogonal_dis = glm::length((b - joint.cone_origin) - cone_dist * joint.axis_direction);

		//calculate world coords
		//float base_radius_w = tan(glm::radians(joint.cone_angle)));
		//float cone_dist_w = glm::dot(b * glm::inverse(boneA.local_coord_matrix) - joint.cone_origin * glm::inverse(boneA.local_coord_matrix), joint.axis_direction * glm::inverse(boneA.local_coord_matrix));
		//float cone_radius_w = (cone_dist / joint.height * glm::inverse(boneA.local_coord_matrix)) * base_radius;
		//float orthogonal_dis_w = glm::length((b - joint.cone_origin) - cone_dist * joint.axis_direction);




		std::cout << "cone radius: " << cone_radius << " ortho: " << orthogonal_dis << "\n";
		std::cout << "diff: " << (cone_radius - orthogonal_dis) << "\n";
		if (abs(orthogonal_dis) >= cone_radius) {
			//important: length from origin to point has to be constant
			std::cout << "ball joint violation\n";
			std::cout << "diff: " << (cone_radius - orthogonal_dis) << "\n";
			std::cout << "between a and b: " << glm::length(particles[joint.ref_b].position - particles[joint.ref_a].position) << "\n";
			//TODO: improve functionallity

			float result = sqrt((pow(b.z - joint.cone_origin.z, 2) / pow(cone_radius, 2) - pow(b.x - joint.cone_origin.x, 2)));
			float new_y_plus = joint.cone_origin.y + result;
			float new_y_negative = joint.cone_origin.y - result;
			std::cout << "p1: " << glm::to_string(b) << "\n";
			std::cout << "new y: " << new_y_plus << "\n";
			std::cout << "new y: " << new_y_negative << "\n";
			Particle joint_constraint_particle;
			joint_constraint_particle.position = world_coord(glm::vec4(b.x, new_y_plus+1, b.z, 1), boneA.local_coord_matrix);
			
			// cone_radius and orthogonal_distance are not in world coordinate scale?!

			//satisfy_constraints(particles[joint.ref_b], joint_constraint_particle,0);

			glm::vec4 refBForce = get_force_for_spring(	particles[joint.ref_b], particles[joint.ref_a], 
														glm::length(particles[joint.ref_b].position - particles[joint.ref_a].position) + abs(cone_radius-orthogonal_dis)/3.0f, 200, 20);
			forces[joint.ref_b] = forces[joint.ref_b] + refBForce;
		}
	
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
			float length;
			Particle line_constraint_particle;
			if (plane_val1 < 0) {
				//std::cout << "violated plane 1 constraint\n";
				line_constraint_particle.position = joint.plane1[0] * glm::inverse(boneA.local_coord_matrix);
				line_constraint_particle.velocity = glm::vec4(0, 0, 0, 0);
			}
			if (plane_val2 < 0) {
				//std::cout << "violated plane 2 constraint\n";
				line_constraint_particle.position = joint.plane2[0] * glm::inverse(boneA.local_coord_matrix);
				line_constraint_particle.velocity = glm::vec4(0, 0, 0, 0);
			}


			satisfy_constraints(particles[joint.ref_b], line_constraint_particle, 0/*glm::length(particles[joint.ref_a].position - line_constraint_particle.position) * 1.03*/);
			// todo fix ref A constraint violation
			//glm::vec4 refBForce = get_force_for_spring(	particles[joint.ref_b], line_constraint_particle, 
					//									0, 1000, 20);

			//glm::vec4 refAForce = get_force_for_spring(	particles[joint.ref_a], line_constraint_particle, 
														//abs(glm::length(particles[joint.ref_a].position - line_constraint_particle.position)), 1000, 20);
			//forces[joint.ref_b] = forces[joint.ref_b] + refBForce;
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
			if (joint.particles.size() == 4) {
				handle_hinge_constraints(joint);
			}
			else if (joint.particles.size() == 2) {
				std::cout << "handling ball joint\n";
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
		for (int joint_index : particle.joints) {
			Joint joint = joints[joint_index];
			for (auto constraint : joint.particle_constraints) {
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

	//TODO: credit paper: Thomas Jakobsen
	void satisfy_constraints(Particle &particle1, Particle& particle2, float rest_length) {
	
		
		glm::vec4 delta;
		if (particle1.position == particle2.position) {
			delta = glm::vec4(0.01, 0.01, 0.01, 0);
		}
		else {
			delta = particle1.position - particle2.position;
		}
		//std::cout << glm::to_string(delta) << "\n";
		float delta_length = sqrt(glm::dot(delta.xyz(), delta.xyz()));
		float diff = (delta_length - rest_length) / (delta_length * ((1 / particle1.mass) + (1 / particle2.mass)));
		//if (abs(delta_length - rest_length) > 0 &&) {
		if(particle2.mass == 0) {
			std::cout << "p1 pos: " << glm::to_string(particle1.position) << "\n";
			std::cout << "delta: " << glm::to_string(delta) << "\n";
			std::cout << "delta length: " << delta_length << "\n";
			std::cout << "rest length: " << rest_length << "\n";
			std::cout << diff << "\n";
			particle1.position = particle1.position - (1 / particle1.mass) *delta * diff;
			std::cout << "p1 pos after: " << glm::to_string(particle1.position) << "\n";
		}
		else {

			particle1.position = particle1.position - (1 / particle1.mass) * delta * diff;
		}

		//particle2.position = particle2.position - (1/particle2.mass)*delta * diff; 

	}
	
	glm::vec4 get_force_for_spring(Particle particle1, Particle particle2, float rest_length, float stiffness_val, float damping_val) {
	
		glm::vec4 length_vector;
		if (particle1.position == particle2.position) {
			length_vector = glm::vec4(0.01, 0.01, 0.01, 0);
		}
		else {
			length_vector = particle1.position - particle2.position;
		}
		glm::vec4 spring_force = (stiffness_val) * (rest_length - glm::length(length_vector)) * glm::normalize(length_vector);
		glm::vec4 damping_force = (-damping_val) * (particle1.velocity - particle2.velocity) * glm::normalize(length_vector) * glm::normalize(length_vector);
		return spring_force + damping_force;
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
					if (particles.size() == 1)
						new_particle.mass = 3;
					else
						new_particle.mass = 1;
	
					particles.push_back(new_particle);
				}
				if (load_state == 1 && inputNums.size() == 4) {
					particles[inputNums[0]].bones.push_back(bones.size());
					particles[inputNums[1]].bones.push_back(bones.size());
					particles[inputNums[2]].bones.push_back(bones.size());
					particles[inputNums[3]].bones.push_back(bones.size());
					create_bone({ (int)inputNums[0], (int)inputNums[1], (int)inputNums[2] , (int)inputNums[3]});
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
		glm::vec4 hinge_particle2 = particles[joint.particles[2]].position * boneA.local_coord_matrix;

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
	}
	void create_ball_joint_rotation(Joint& joint) {
	
		Bone boneA = bones[joint.bones[0]];
		Bone boneB = bones[joint.bones[1]];

		glm::vec4 a;
		glm::vec4 b;
	
		glm::vec4 ref_joint = particles[joint.particles[0]].position;
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
		joint.axis_direction = glm::normalize(b - joint.cone_origin);
		joint.height = glm::length(b - joint.cone_origin);
	}

	void create_joint(std::vector<float> inputNums) {
		int bone_index1 = (int)inputNums[0];
		int bone_index2 = (int)inputNums[1];
		int connections = (int)inputNums[2];
		
		Bone bone1 = bones[bone_index1];
		Bone bone2 = bones[bone_index2];

		std::vector<Spring_Constraints> constraints;
		std::vector<int> particle_index;
		Joint new_joint;
		new_joint.bones.push_back(bone_index1);
		new_joint.bones.push_back(bone_index2);
		if (connections == 2) {
			new_joint.positive_angle = inputNums[3];
			new_joint.negative_angle = inputNums[4];
		}
		if (connections == 1) {
			new_joint.cone_angle = inputNums[3];
		}

		std::map<int, bool> used;
		float smallest = 10000.0;
		int min_i = 0;
		int min_j = 0;

		for (int c = 0; c < connections; c++) {

			for (int i = 0; i < bone1.particles.size(); i++) {
				for (int j = 0; j < bone2.particles.size(); j++) {

					int particle_index1 = bone1.particles[i];
					int particle_index2 = bone2.particles[j];
					float difference = abs(glm::length(particles[particle_index1].position - particles[particle_index2].position));
					//std::cout << difference << "\n";
					
					if (difference < smallest && used.find(particle_index1) == used.end() && used.find(particle_index2) == used.end()) {
						min_i = particle_index1;
						min_j = particle_index2;
						smallest = difference;
					}
				}
			}

			particles[min_i].joints.push_back(joints.size());
			particles[min_j].joints.push_back(joints.size());
			new_joint.particles.push_back(min_i);
			new_joint.particles.push_back(min_j);
			new_joint.particle_constraints.push_back(get_spring_constraint(glm::vec4(0, 0, 0, 0), min_i, min_j));
			used[min_i] = true;
			used[min_j] = true;
			smallest = 10000;
		}

		if (new_joint.particles.size() == 4)
			create_hinge_rotation_planes(new_joint);

		if (new_joint.particles.size() == 2)
			create_ball_joint_rotation(new_joint);
		
		joints.push_back(new_joint);
		for (auto joint : joints) {
			for (int i : joint.particles)
				std::cout << i << "\n";
		}
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

	void create_bone(std::vector<int> particle_list) {
		
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
		
		RawModel new_rawModel = loader.loadToVAO(input_vertices, normals, num_vertices*sizeof(float));
		bone_models.push_back(new_rawModel);

		bones.push_back(new_bone);
	
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
