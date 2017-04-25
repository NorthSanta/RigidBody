#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <time.h>
#include <stdio.h>
bool show_test_window = false;
using namespace glm;
float reset;
vec3 gravity = vec3(0, -9.8f, 0);
vec3 verticalForce;
vec3 linearMomentum;
vec3 angularMomentum;
vec3 torque;
vec3 velocity;
vec3 position;
mat4 rotatedMatrix;
mat4 translatedMatrix;

float mass = 1;
float xpos;
float ypos;
float zpos;

namespace Cube {

	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(glm::mat4x4);
	extern void drawCube();
}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit() {
	//TODO
	reset = 5;
	srand(time(NULL));
	glm::mat4x4 matriu = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
	};
	xpos = ((float)rand() / RAND_MAX) * 5 - 4;
	ypos = ((float)rand() / RAND_MAX) * 9;
	zpos = ((float)rand() / RAND_MAX) * 5 - 4;
	matriu = translate(matriu, vec3(xpos, ypos, zpos));

	float randAngle = (float)rand() / 360;
	int xrot = rand() % 1;
	int yrot = rand() % 1;
	int zrot = rand() % 1;
	if (xrot == 0 && yrot == 0 && zrot == 0) 
	{
		int alloc = rand() % 2;
		if (alloc == 0) 
		{
			xrot = 1;
		}
		else if (alloc == 1) 
		{
			yrot = 1;
		}
		else
		{
			zrot = 1;
		}
	}
	matriu = rotate(matriu, randAngle, glm::vec3(xrot, yrot, zrot));
	verticalForce = vec3(0, 20, 0);
	linearMomentum = vec3(0) + verticalForce;
	vec3 forcePoint = vec3(1,0,0);
	vec3 vecForce = forcePoint - vec3(0);
	torque = cross(vecForce, verticalForce);
	angularMomentum = vec3(0) + torque;
	velocity = linearMomentum / mass;
	position = vec3(xpos,ypos,zpos);
	Cube::updateCube(matriu);
}
void PhysicsUpdate(float dt) {
	//TODO
	reset -= dt;
	if (reset <= 0) {
		PhysicsInit();
	}
	linearMomentum += dt*gravity;
	angularMomentum += dt*torque;
	velocity = linearMomentum / mass;
	position += dt*velocity;
	printf("%f", position.y);
	mat4 tranlationMatrix = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		position.x,position.y,position.z,1
	};

	Cube::updateCube(tranlationMatrix);


}
void PhysicsCleanup() {
	//TODO
	Cube::cleanupCube();
}