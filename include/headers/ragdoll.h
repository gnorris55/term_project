#ifndef RAGDOLL_H
#define RAGDOLL_H

#include <learnopengl/model.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>
#include <string>

// TODO: add constraints to particles

struct Bone {
	// should always have 4  particles so that the bone forms a tetrahedron
	std::vector<int> particles;
	
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
	
	void draw_bones() {

		for (int i = 0; i < bones.size(); i++) {
			glm::mat4 model = glm::mat4(1.0f);
			glUniformMatrix4fv(glGetUniformLocation(program->ID, "model"), 1, GL_FALSE, glm::value_ptr(model));
			renderer.render(bone_models[i], GL_TRIANGLES);
		}
	}

private:
	Loader loader;
	Renderer renderer;
	std::vector<glm::vec4> particles;
	std::vector<RawModel> bone_models;
	std::vector<Bone> bones;
	
	//loads the particles and starting positions of a ragdoll skeleton
	void load_ragdoll(std::string filename) {
		std::cout << "loading file: " + filename << "\n";
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
				if (strcmp(curr_item.c_str(), "particle") == 0 )
					load_state = 0;
				else if (strcmp(curr_item.c_str(), "bone") == 0 )
					load_state = 1;
				else
					continue;
				
				while (std::getline(ss, tokenStrs, ' ')) {
					inputNums.push_back(stof(tokenStrs));
				}

				if (load_state == 0 && inputNums.size() == 3)
					particles.push_back(glm::vec4(inputNums[0], inputNums[1], inputNums[2], 1));
				if (load_state == 1 && inputNums.size() == 4) {
					create_bone({ (int)inputNums[0], (int)inputNums[1], (int)inputNums[2] , (int)inputNums[3]});
				}
			}
		}
		else {
			std::cout << "could not open file\n";
		}
	}
	
	void form_triangle(float arr[], float normals[], int triangle_index, int p1, int p2, int p3) {
		int arr_index = triangle_index * 9;
		glm::vec4 edge1 = particles[p2] - particles[p1];
		glm::vec4 edge2 = particles[p3] - particles[p1];
		glm::vec3 normal = glm::cross(edge1.xyz(), edge2.xyz());

		for (int i = arr_index; i < arr_index + 3; i++) {
			normals[i*3] = normal.x;
			normals[i*3+1] = normal.y;
			normals[i*3+2] = normal.z;
		}

		arr[arr_index]	 = particles[p1].x;
		arr[++arr_index] = particles[p1].y;
		arr[++arr_index] = particles[p1].z;
		arr[++arr_index] = particles[p2].x;
		arr[++arr_index] = particles[p2].y;
		arr[++arr_index] = particles[p2].z;
		arr[++arr_index] = particles[p3].x;
		arr[++arr_index] = particles[p3].y;
		arr[++arr_index] = particles[p3].z;
	}
	void create_bone(std::vector<int> particle_list) {
		Bone new_bone;
		new_bone.particles = particle_list;
		bones.push_back(new_bone);

		const int num_vertices = 4 * 3 * 3;
		float input_vertices[num_vertices];
		float normals[num_vertices];
	
		form_triangle(input_vertices, normals, 0, particle_list[0], particle_list[1], particle_list[2]);
		form_triangle(input_vertices, normals, 1, particle_list[0], particle_list[1], particle_list[3]);
		form_triangle(input_vertices, normals, 2, particle_list[0], particle_list[2], particle_list[3]);
		form_triangle(input_vertices, normals, 3, particle_list[1], particle_list[2], particle_list[3]);

		RawModel new_rawModel = loader.loadToVAO(input_vertices, normals, num_vertices*sizeof(float));
		bone_models.push_back(new_rawModel);
	
	}


	void update_bone_models() {

	}

	void print_bones() {
		for (auto bone : bones) {
			for (auto vec : bone.particles)
				std::cout << vec << "\n";
		}
	}

	void print_particles() {
		for (auto particle : particles)
			std::cout << glm::to_string(particle) << "\n";
	}
};

#endif
