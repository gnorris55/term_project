#ifndef RAW_MODEL_H
#define RAW_MODEL_H

#include <learnopengl/shader.h>

class RawModel {

public:

	unsigned int vaoID;
	unsigned int vertexCount;

	RawModel(int vaoID, int vertexCount) {
		this->vaoID = vaoID;
		this->vertexCount = vertexCount;
	}
};

class Loader {

public:

	RawModel loadToVAO(float vertices[], float normals[], int numVertices) {
		unsigned int VAO = createVAO();
		storeDataInAttributeList(0, numVertices, 3, vertices);
		storeDataInAttributeList(1, numVertices, 3, normals);
		return RawModel(VAO, numVertices / 3 * sizeof(float));

	}

private:

	int createVAO() {
		unsigned int VAO;
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);
		return VAO;
	}

	void storeDataInAttributeList(int attributeNumber, int count, int verticesInAttribute, float vertices[]) {
		unsigned int VBO;
		glGenBuffers(1, &VBO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, count, vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(attributeNumber, verticesInAttribute, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(attributeNumber);
	}
};


class Renderer {

public:

	void prepare() {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	void render(RawModel model, GLenum type) {
		glBindVertexArray(model.vaoID);
		glDrawArrays(type, 0, model.vertexCount);
		//glDrawArrays(type, 0, 9*6*3);
		glBindVertexArray(model.vaoID);
	}
};

#endif
