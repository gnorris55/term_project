#ifndef RAGDOLL_SIMULATOR
#define RAGDOLL_SIMULATOR

#include <learnopengl/Shader.h>
#include <headers/ragdoll.h>

class RagdollSimulator {
public:
	Ragdoll *ragdoll;
	Shader* program;

	RagdollSimulator(Shader* program, Ragdoll *input_ragdoll) {
		this->program = program;
		ragdoll = input_ragdoll;
	}

	void simulate(double time) {
		time* time_step;
		//ragdoll->draw_bones();
	}

private:
	double time_step = 0.01;

};

#endif
