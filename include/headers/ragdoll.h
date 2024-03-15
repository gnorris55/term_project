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
	std::vector<Spring_Constraints> particle_constraints;
};

struct Particle {
	glm::vec4 position;
	glm::vec4 velocity;
	std::vector<int> bones;
	std::vector<int> joints;
};

struct Bone {
	// should always have 4  particles so that the bone forms a tetrahedron
	std::vector<int> particles;
	std::vector<Spring_Constraints> particle_constraints;
	
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
		
		std::vector<glm::vec4> forces(particles.size());
		for (int i = 0; i < particles.size(); i++) {
			
			Particle particle = particles[i];

			//get all springs attached to a particle and calculate the physics
			glm::vec4 strut_forces = get_strut_forces_on_particle(particle, i);
			forces[i] = strut_forces;
			//std::cout << glm::to_string(strut_forces) << "\n";
		}
		for (int i = 0; i < particles.size(); i++) {
			Particle& particle = particles[i];
			glm::vec4 force_value = forces[i];
			if (i == 17) continue;
			symplectic_integration(particle, force_value);

			//std::cout << glm::to_string(particles[i].position) << "\n";
		}
		for (int i = 0; i < bones.size(); i++) {

			glm::mat4 model = glm::mat4(1.0f);
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
	std::vector<Bone> bones;
	std::vector<Joint> joints;
	float time_step = 0.005;
	
	void symplectic_integration(Particle& particle, glm::vec4 force_value) {
		//std::cout << glm::to_string(force_value) << "\n";
		glm::vec4 new_velocity = particle.velocity + time_step * force_value;
		glm::vec4 new_pos = particle.position + (float)(time_step)*new_velocity;
		//std::cout << glm::to_string(new_velocity) << "\n";
		particle.velocity = new_velocity;
		particle.position = new_pos;

	}

	glm::vec4 get_strut_forces_on_particle(Particle particle, int index) {
		glm::vec4 total_strut_force = glm::vec4(0, -0.98, 0, 0) + particle.velocity * -0.05f;

		//glm::vec4 total_strut_force = glm::vec4(0, 0, 0, 0);
	
		for (int joint_index : particle.joints) {
			for (auto constraint : joints[joint_index].particle_constraints) {
				if (constraint.index1 == index) {
					total_strut_force = total_strut_force + get_force_for_spring(particle, particles[constraint.index2], constraint.length, 200, 20);
				}
				else if (constraint.index2 == index) {
					total_strut_force = total_strut_force + get_force_for_spring(particle, particles[constraint.index1], constraint.length, 200, 20);
				}
			}
		}

		for (int bone_index : particle.bones) {
			std::cout << "curr_index: " << index << "\n";
			for (auto constraint : bones[bone_index].particle_constraints) {
				if (constraint.index1 == index) {
					total_strut_force = total_strut_force + get_force_for_spring(particle, particles[constraint.index2], constraint.length, 200, 20);
				}
				else if (constraint.index2 == index) {
					total_strut_force = total_strut_force + get_force_for_spring(particle, particles[constraint.index1], constraint.length, 200, 20);
				}
			}
		}
		return total_strut_force;
	}
	
	glm::vec4 get_force_for_spring(Particle particle1, Particle particle2, float rest_length, float stiffness_val, float damping_val) {
	
		//std::cout << "ln: " << rest_length << "\n";
		glm::vec4 length_vector;
		if (particle1.position == particle2.position) {
			length_vector = glm::vec4(0, 0, 0, 0);
		}
		else {
			length_vector = particle1.position - particle2.position;
		}
		//std::cout << "distance from p: " << glm::to_string(particle1.position) << "to p2: " << glm::to_string(particle2.position) << glm::to_string(length_vector) << "\n";
		glm::vec4 spring_force = (stiffness_val) * (rest_length - glm::length(length_vector)) * glm::normalize(length_vector);
		glm::vec4 damping_force = (-damping_val) * (particle1.velocity - particle2.velocity) * glm::normalize(length_vector) * glm::normalize(length_vector);
		//std::cout << "stiffness: " << glm::to_string(damping_force) << "\n";
		//std::cout << "damping: " << glm::to_string(spring_force) << "\n";
		return spring_force + damping_force;
	}
	
	//loads the particles and starting positions of a ragdoll skeleton
	void load_ragdoll(std::string filename) {
		//std::cout << "loading file: " + filename << "\n";
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
					particles.push_back(new_particle);
				}
				if (load_state == 1 && inputNums.size() == 4) {
					particles[inputNums[0]].bones.push_back(bones.size());
					particles[inputNums[1]].bones.push_back(bones.size());
					particles[inputNums[2]].bones.push_back(bones.size());
					particles[inputNums[3]].bones.push_back(bones.size());
					create_bone({ (int)inputNums[0], (int)inputNums[1], (int)inputNums[2] , (int)inputNums[3]});
				}
				if (load_state == 2 && inputNums.size() == 3) {
					std::cout << "setting a joint\n";
					create_joint((int)inputNums[0], (int)inputNums[1], inputNums[2]);
				}
			}
		}
		else {
			std::cout << "could not open file\n";
		}
	}

	void create_joint(int bone_index1, int bone_index2, int connections) {
		Bone bone1 = bones[bone_index1];
		Bone bone2 = bones[bone_index2];

		std::vector<Spring_Constraints> constraints;
		std::vector<int> particle_index;
		Joint new_joint;
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
		joints.push_back(new_joint);
		for (auto joint : joints) {
			for (int i : joint.particles)
				std::cout << i << "\n";
		}
	}
	
	void form_triangle(float arr[], float normals[], int triangle_index, glm::vec4 p1, glm::vec4 p2, glm::vec4 p3) {
		int arr_index = triangle_index * 9;
		glm::vec4 edge1 = p2 - p1;
		glm::vec4 edge2 = p3 - p1;
		glm::vec3 normal = glm::cross(edge1.xyz(), edge2.xyz());
		
		/*
		for (int i = arr_index; i < arr_index + 3; i++) {
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
	
		Bone new_bone;
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
		new_constraint.length = abs(glm::length(length))*0.95f;
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
