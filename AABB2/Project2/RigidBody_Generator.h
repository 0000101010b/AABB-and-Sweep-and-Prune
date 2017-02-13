#include <GLFW/glfw3.h>

#include "Model.h"
#include "Camera.h"
#include "Shader.h"

// GLM Mathemtics
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glm/ext.hpp"

#include "Mesh.h"


#pragma once
struct EndPoint
{
	int owner;
	double value;
	bool isMin;
};
/*
struct Pair {
	int p1;
	int p2;
};
*/
struct BoundingBox {

	int box_id;
	EndPoint min[3];
	EndPoint max[3];

	BoundingBox(int bod_id) {
		this->box_id = bod_id;
	}

	BoundingBox() {
	}
};
struct BoundingSphere {
	GLfloat r;
};

std::pair<int, int> my_make_pair(int a, int b);
vector<EndPoint> InsertionSort(vector<EndPoint> a);
std::map<std::pair<int, int>, int> addToOverLapList(std::map<std::pair<int, int>, int> m, vector<EndPoint> axisPoints);


struct RigidBody
{

	GLint turnIndex;
	GLfloat mass;
	glm::mat3 Ibody;

	glm::vec3 position;
	glm::mat3 orientation;
	glm::vec3 linearMomentum;
	//glm::vec3 angularMomentum;

	glm::vec3 force;
	glm::vec3 torque;
	
	glm::vec4 color;
	
	glm::vec3 bbHWB;

	BoundingBox bb;

	BoundingSphere bs;

	Model model;

	RigidBody(Model model)
		:position(0.0f), mass(1.0f),color(0.5f,0.25f,0.3f,1.0f),bbHWB(1.25f),model(model){

		GLfloat r1 = (((rand() % 200) + 1.0f) / 100.0f) - 1.0f;
		GLfloat r2 = (((rand() % 200) + 1.0f) / 100.0f) - 1.0f;
		GLfloat r3 = (((rand() % 200) + 1.0f) / 100.0f) - 1.0f;

		this->turnIndex = ((rand() % 15))+0.0f;
	
		this->force = glm::vec3(r1,r2,r3)*0.001f;

	}
};


class RigidBodyGenerator
{
public:
	bool forceStop=false;

	glm::mat3 I_sphere;
	glm::mat3 I_cube;
	glm::mat3 w;
	
	vector<RigidBody> rigidbodies;
	
	//required for drawing
	Model model;
	Shader shader;
	Camera camera;

	//constructor
	RigidBodyGenerator(Shader shader, Model model,string IbodyStr);
	void init(string IbodyStr);
	void CreateRigidbody(string IbodyStr,int index);
	void SweepAndPrune();
	void BoundingSphereCheck();
	void Draw();
	void Update(GLfloat dt);
};


RigidBodyGenerator::RigidBodyGenerator(Shader shader, Model model,string  IbodyStr)
	: shader(shader), model(model)
{
	this->init(IbodyStr);
}



void RigidBodyGenerator::init(string IbodyStr)
{
	for (int i = 0; i < 10; i++)
	{
		this->CreateRigidbody(IbodyStr,i);
	}
}


vector<EndPoint> InsertionSort(vector<EndPoint> a)
{
	for (int i = 1; i < a.size(); i++)
	{
		int j = i;
		while (j > 0 && a[j - 1].value > a[j].value)
		{
			EndPoint temp = a[j];
			a[j] = a[j - 1];
			a[j - 1] = temp;

			j = j - 1;
		}
	}

	return a;
}

std::map<std::pair<int, int>, int> addToOverLapList(std::map<std::pair<int, int>, int> m, vector<EndPoint> axisPoints)
{

	vector<int> active;
	for (int i = 0; i < axisPoints.size(); i++)
	{
		if (axisPoints[i].isMin)
			active.push_back(axisPoints[i].owner);

		if (!axisPoints[i].isMin)
			active.erase(std::remove(active.begin(), active.end(), axisPoints[i].owner), active.end());

		for (int j = 0; j < active.size(); j++)
		{
			if (axisPoints[i].owner != active[j])
			{
				std::map<std::pair<int, int>, int>::iterator res;
				res = m.find(my_make_pair(axisPoints[i].owner, active[j]));
				if (res == m.end()) {
					if (active[j] < axisPoints[i].owner) {
						m.insert(std::make_pair(my_make_pair(active[j], axisPoints[i].owner), 1));
					}
				}
				else {
					if (active[j] < axisPoints[i].owner) {
						if (res->second == 2) {
							res->second = 3;
						}
						else if (res->second == 1) {
							res->second = 2;
						}
					}
				}
			}
		}

	}
	return m;
}


void RigidBodyGenerator::BoundingSphereCheck() {
	
	vector<pair< int, int>> overlapBoxes;

	for (int i = 0; i < rigidbodies.size(); i++)
	{
		rigidbodies[i].color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);

		for (int j = 0; j < rigidbodies.size(); j++)
		{
			if (i != j)
			{
				//if (glm::distance(rigidbodies[i].position, rigidbodies[j].position) < rigidbodies[i].bs.r + rigidbodies[j].bs.r)
				if (glm::distance(rigidbodies[i].position, rigidbodies[j].position) < 2.15f)
				{
					if(std::find(overlapBoxes.begin(), overlapBoxes.end(), my_make_pair(i, j)) == overlapBoxes.end())
						overlapBoxes.push_back(my_make_pair(i, j));
				}
			}
		}
	}
	
	//system("cls");
	//printf("\nBounding sphere overlap\n::::::::::::::::::::::::::::\n");
	for (int i = 0; i < overlapBoxes.size(); i++)
	{
		rigidbodies[overlapBoxes[i].first].color = glm::vec4(0.0f,1.0f,0.0f,1.0f);
		rigidbodies[overlapBoxes[i].second].color =glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
		//cout << "Boxes " << overlapBoxes[i].first << " and " << overlapBoxes[i].second << endl;

	}
	
}

void RigidBodyGenerator::SweepAndPrune() {
	
	vector<EndPoint> x;
	vector<EndPoint> y;
	vector<EndPoint> z;

	for (int i = 0; i < rigidbodies.size(); i++)
	{
		rigidbodies[i].color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
		x.push_back(rigidbodies[i].bb.max[0]);
		x.push_back(rigidbodies[i].bb.min[0]);

		y.push_back(rigidbodies[i].bb.max[1]);
		y.push_back(rigidbodies[i].bb.min[1]);

		z.push_back(rigidbodies[i].bb.max[2]);
		z.push_back(rigidbodies[i].bb.min[2]);
	
	}

	std::map<std::pair<int, int>, int> m;
	
	//insertion sort
	x = InsertionSort(x);
	y = InsertionSort(y);
	z = InsertionSort(z);

	m = addToOverLapList(m, x);
	m = addToOverLapList(m, y);
	m = addToOverLapList(m, z);

	/*
	cout << "---------------------------------------------------------------" << endl;
	
	for (int w = 0; w < x.size(); w++)
	{
		cout << "|" << w << "|";
		if (x[w].isMin)
			printf("S", x[w].value);
		else
			printf("E", x[w].value);

		printf("%d_", x[w].owner);
		printf("[%.2f]--", x[w].value);
	}
	printf("\n\n");

	for (int w = 0; w < y.size(); w++)
	{
		cout << "|" << w << "|";
		if (y[w].isMin)
			printf("S", y[w].value);
		else
			printf("E", y[w].value);

		printf("%d_", y[w].owner);
		printf("[%.2f]--", y[w].value);
	}	
	printf("\n\n");
	for (int w = 0; w < z.size(); w++)
	{
		cout << "|" << w << "|";
		if (z[w].isMin)
			printf("S", z[w].value);
		else
			printf("E", z[w].value);

		printf("%d_", z[w].owner);
		printf("[%.2f]--", z[w].value);
	}
	printf("\n\n");
	cout << endl << "sorted" <<endl;
	*/

	//printf("Sweep and prune overlaps\n::::::::::::::\n");
	for (std::map<std::pair<int, int>, int>::iterator iterator = m.begin(); iterator != m.end(); iterator++) {
		if (iterator->second == 3) {
			//cout << "Boxes: " << iterator->first.first << " and " << iterator->first.second << endl;
			rigidbodies[iterator->first.first].color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
			rigidbodies[iterator->first.second].color = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
		}
		//cout << "Boxes: " << iterator->first.first << " and " << iterator->first.second << " Axis overlap: " << iterator->second << endl;
	}
}
std::pair<int, int> my_make_pair(int a, int b)
{
	if (a < b) return std::pair<int, int>(a, b);
	else return std::pair<int, int>(b, a);
}

void RigidBodyGenerator::CreateRigidbody(string IbodyStr, int index) {



	RigidBody &rigidbody=RigidBody(Model(this->model));
	//this->mesh = this->model.meshes;
	//rigidbody.model = this->model;
	rigidbody.bs.r = 0.2f;

	for (int i = 0; i <rigidbody.model.meshes.size();i++)
	{
		rigidbody.model.meshes[i].setupMesh();
	}
	//set rotation matrix
	float R[9] = {
		glm::cos(.0f),-glm::sin(.0f),0,
		glm::sin(.0f),glm::cos(.0f),0,
		0,0,1
	};
	rigidbody.orientation = glm::make_mat3(R);

	//set Ibody
	float m = rigidbody.mass;

	if (IbodyStr == "sphere")
	{
		float r = 1.0f;

		float Isphere[9] = {
			2.0f / 5.0f * m * r *r,0			  ,0,
			0				,2.0f / 5.0f *m*r*r,0,
			0				,0			  ,2.0f / 5.0f * m*r*r
		};
		rigidbody.Ibody = glm::make_mat3(Isphere);
	}

	if (IbodyStr == "cube")
	{
		float h = 1.0f;
		float d = 1.0f;
		float w = 1.0f;

		float Icube[9] = {
			1.0f / 12.0f * m * (h * h + d * d),0			  ,0,
			0				,1.0f / 12.0f * m * (w * w + d * d),0,
			0				,0			  ,1.0f / 12.0f * m * (w * w + h * h)
		};
		rigidbody.Ibody = glm::make_mat3(Icube);

	}

	BoundingBox bb = BoundingBox(index);

	for (int j = 0; j < 3; j++)
	{
		bb.min[j].owner = index;
		bb.min[j].isMin = true;
		bb.max[j].owner = index;
		bb.max[j].isMin = false;
	}
	rigidbody.bb = bb;

	rigidbodies.push_back(RigidBody(rigidbody));
}

void RigidBodyGenerator::Update(GLfloat dt)
{

	for (int rb = 0; rb < rigidbodies.size(); rb++)
	{
		//setting the postion with linear momentum
		glm::vec3 acceleration = rigidbodies[rb].force / rigidbodies[rb].mass;
		rigidbodies[rb].linearMomentum += (acceleration*dt);
		rigidbodies[rb].position += rigidbodies[rb].linearMomentum;


		//get new orientation base on angular velocity
		glm::mat3 R = rigidbodies[rb].orientation;
		glm::mat3 I = R * rigidbodies[rb].Ibody * glm::transpose(R);
		glm::vec3 w = glm::inverse(I)*rigidbodies[rb].torque;

		float wf_matrix[9] = {
			0, -w.z,w.y,
			w.z,0,-w.x,
			-w.y,w.x,0
		};

		glm::mat3 wMat = glm::make_mat3(wf_matrix);
		rigidbodies[rb].orientation += rigidbodies[rb].orientation*wMat;

		//ortho normalization
		rigidbodies[rb].orientation = glm::orthonormalize(rigidbodies[rb].orientation);

		GLfloat maxX = 0.0f;
		GLfloat maxY = 0.0f;
		GLfloat maxZ = 0.0f;

		//apply rotation and get max dist
		for (int i = 0; i < model.meshes.size(); i++)
		{
			for (int j = 0; j < model.meshes[i].vertices.size(); j++)
			{
				Vertex &vertex = rigidbodies[rb].model.meshes[i].vertices[j];
				Vertex &vR = model.meshes[i].vertices[j];
				vertex.Position = rigidbodies[rb].position + (rigidbodies[rb].orientation*vR.Position);
				vertex.Normal = rigidbodies[rb].orientation*vR.Normal;
		

				GLfloat temp = glm::abs(vertex.Position.x-rigidbodies[rb].position.x);
				if (temp > maxX)
					maxX = temp;
				
				temp = glm::abs(vertex.Position.y-rigidbodies[rb].position.y);
				if (temp > maxY)
					maxY = temp;

				temp = glm::abs(vertex.Position.z-rigidbodies[rb].position.z);
				if (temp > maxZ)
					maxZ = temp;



				//GLfloat temp=glm::distance(vertex.Position, glm::vec3(0.0f));
				//if (temp > rigidbodies[rb].bs.r)
				//	rigidbodies[rb].bs.r = temp;
			}
			//rigidbodies[rb].model.meshes[i].setupMesh();
			rigidbodies[rb].model.meshes[i].UpdateMesh();
		}
		
		//get torque for next run
		for (int i = 0; i < model.meshes.size(); i++)
		{
			for (int j = 0; j < model.meshes[i].vertices.size(); j++)
			{
				Vertex &vertex = rigidbodies[rb].model.meshes[i].vertices[j];
				//torque force 1
				if (j == rigidbodies[rb].turnIndex && !forceStop) {
					rigidbodies[rb].torque += (vertex.Position - rigidbodies[rb].position) * glm::vec3(0.0000001f);
				}
				/*
				//torque force 2
				if (j == 8 && !forceStop) {
					forceStop = true;
					rigidbodies[rb].torque += (vertex.Position - rigidbodies[rb].position) * glm::vec3(0.001f);
				}*/
			}
		}

		
		if (glm::distance(rigidbodies[rb].position,glm::vec3(0.0f)) > 15.0f)
		{
			rigidbodies[rb].linearMomentum = -rigidbodies[rb].linearMomentum;
			rigidbodies[rb].force = -rigidbodies[rb].force;
		}
		
		rigidbodies[rb].bb.max[0].value = rigidbodies[rb].position.x + maxX;
		rigidbodies[rb].bb.min[0].value = rigidbodies[rb].position.x - maxX;

		rigidbodies[rb].bb.max[1].value = rigidbodies[rb].position.y + maxY;
		rigidbodies[rb].bb.min[1].value = rigidbodies[rb].position.y - maxY;

		rigidbodies[rb].bb.max[2].value = rigidbodies[rb].position.z + maxZ;
		rigidbodies[rb].bb.min[2].value = rigidbodies[rb].position.z - maxZ;
		
	}
	//BoundingSphereCheck();
	SweepAndPrune();


}


void RigidBodyGenerator::Draw() {

	this->shader.Use();

	for (int i = 0; i < rigidbodies.size(); i++)
	{
		//local view Position
		GLint viewPos = glGetUniformLocation(shader.Program, "viewPos");
		glUniform3f(viewPos, camera.Position.x, camera.Position.y, camera.Position.z);

		// Create camera transformations
		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 projection = glm::perspective(camera.Zoom, (GLfloat)800 / (GLfloat)600, 0.1f, 100.0f);

		// Get the uniform matrixes
		GLint modelLoc = glGetUniformLocation(shader.Program, "model");
		GLint viewLoc = glGetUniformLocation(shader.Program, "view");
		GLint projLoc = glGetUniformLocation(shader.Program, "projection");

		// Pass the matrices for view and projection
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
		glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

		glm::mat4 particleModel;
		particleModel = glm::scale(particleModel, glm::vec3(1.0f, 1.0f, 1.0f));
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(particleModel));


		glm::vec3 offset = glm::vec3(0.0f);
		glUniform3f(glGetUniformLocation(shader.Program, "offset"), (GLfloat)offset.x, (GLfloat)offset.y, (GLfloat)offset.z);


		GLfloat scale = 1.0f;
		glUniform1f(glGetUniformLocation(shader.Program, "scale"), scale);


		glm::vec4 color = rigidbodies[i].color;
		glUniform4f(glGetUniformLocation(shader.Program, "color"), (GLfloat)color.r, (GLfloat)color.g, (GLfloat)color.b, (GLfloat)color.a);

		glm::vec3 lightPos = glm::vec3(0.0f, 20.0f, 0.0f);
		glUniform3f(glGetUniformLocation(shader.Program, "lightPos"), (GLfloat)lightPos.x, (GLfloat)lightPos.y, (GLfloat)lightPos.z);

		rigidbodies[i].model.Draw(shader);



		glm::mat4 particleModel2;
		particleModel2 = glm::translate(rigidbodies[i].position);
		
		particleModel2=glm::scale(particleModel2, glm::vec3(
			rigidbodies[i].bb.max[0].value-rigidbodies[i].position.x,
			rigidbodies[i].bb.max[1].value-rigidbodies[i].position.y,
			rigidbodies[i].bb.max[2].value-rigidbodies[i].position.z
		));

		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(particleModel2));

		glm::vec4 c = glm::vec4(0.0f);
		glUniform4f(glGetUniformLocation(shader.Program, "color"), (GLfloat)color.r, (GLfloat)color.g, (GLfloat)color.b, (GLfloat)color.a);

		model.Draw2(shader);
		//cout << i<< " display" << endl;
		
		
	}
}



