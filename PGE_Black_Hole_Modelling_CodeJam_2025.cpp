
#define OLC_GFX_OPENGL33
#define OLC_PGE_APPLICATION
#define OLC_IMAGE_STB
#include "olcUTIL_Hardware3D.h"
#include "olcPixelGameEngine.h"
#include <immintrin.h>			
#define _USE_MATH_DEFINES 
#include <cmath>				
#include <vector>	
#include <mutex>
#include <thread>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // M_PI

const double C = 2.99792458e+8;			// Speed of light in m/s
const double G = 6.67430e-11;			// Gravitational constant
const double dAU = 1.496e+11;			// Astronomical Unit in meters
const double dPC = 3.086e+16;			// Parsec in meters
const double dLY = 9.461e+15;			// Light Year in meters
const double dSolarMass = 1.989e+30;	// Solar mass in kg
const double dEarthMass = 5.972e+24;	// Earth mass in kg
const double dEarthRadius = 6.371e+6;	// Earth radius in meters
const double dSunMass = dSolarMass;		// Sun mass in kg (1 Solar mass = our sun mass :))
const double dSunRadius = 6.9634e+8;	// Sun radius in meters
const double dSagittariusAMass = 4.1e+6 * dSolarMass; // Mass of Sagittarius A* in kg (approximately 4.154 million solar masses)

double dArbitraryfactor = 2.5;			// Arbitrary factor for visualization of event horizon

// The Light Ray initial position and direction that will loop around the black hole
olc::vd3d vd2dLoopyLoop = { -1e+11, 3.13106302719999999e+10, 1e+11};

olc::vd3d vd2dConstLightDir = { C, 0.0, 0.0 };// Const Initial direction of the light ray (pointing right along the x-axis)

olc::vd3d vd2dConstLightDirX = { C, 0.0, 0.0 };// Const Initial direction of the light ray (pointing right along the x-axis)
olc::vd3d vd2dConstLightDirY = { 0.0, C, 0.0 };// Const Initial direction of the light ray (pointing up along the y-axis)
olc::vd3d vd2dConstLightDirZ = { 0.0, 0.0, C };// Const Initial direction of the light ray (pointing up along the z-axis)

const double dKMtoMeters = 1e+3;		// Conversion factor from kilometers to meters
const double dMetersToKM = 1e-3;		// Conversion factor from meters to kilometers
const double dScreenMToWorldVM = 1e+11;	// Conversion factor from Screen meters to WorldView Meters(1e11 meters)
double WorldX = 0;					// Width of the viewport in meters  
double WorldY = 0;					// Height of the viewport in meters
double WorldZ = 0;					// Deph of the viewport in meters
         

// Mutex for thread safety
std::mutex draw_mutex;

#define OLC_PGEX_SPLASHSCREEN		// Manages the GPL-3.0 Licence requirements 
#include "olcPGEX_SplashScreen.h"


// Override base class with your custom functionality
class PGEBlackHoleDemo : public olc::PixelGameEngine
{

public:
	//olc::SplashScreen olcSplashScreen; //TODO add a splash screen

	// In Example's constructor, initialize PBH_SagittariusA after the class definition
	PGEBlackHoleDemo() {
		sAppName = "PGE Black Hole Modelling CodeJam2025";

	}

public:
	// 3D Stuff

	/* Matrices */
	olc::mf4d mf4dWorld;		// World Matrix
	olc::mf4d mf4dView;			// View Matrix
	olc::mf4d mf4dCube;			// Cube Matrix
	olc::mf4d mf4dSkyCube;		// Sky Cube Matrix
	olc::mf4d mf4dSphere;		// Matrix for Sphere (Sun, Stars etc)
	olc::mf4d mf4dBackGround;	// Matrix for Background Space Grid
	olc::mf4d mf4dEventHorizon; // Matrix for Event Horizon
	olc::mf4d mf4dGravityGrid;	// Matrix for Gravity Grid
	olc::mf4d mf4dProject;		// Projection Matrix

	/* Meshes */
	olc::utils::hw3d::mesh meshSpaceGrid;	// Space Grid Mesh
	olc::utils::hw3d::mesh meshSanityCube;	// Sanity Cube Mesh
	olc::utils::hw3d::mesh meshSkyCube;		// Sky Cube Mesh
	olc::utils::hw3d::mesh meshSphere;		// Sphere Mesh (black hole, Sun, Stars etc)
	olc::utils::hw3d::mesh meshEventHorizon; // Event Horizon Mesh
	olc::utils::hw3d::mesh meshBlackHole;	// Black Hole Mesh
	olc::utils::hw3d::mesh meshBackGround;	// Background Space Grid mesh
	olc::utils::hw3d::mesh meshGravityGrid;	// Gravity Grid Mesh

	// Camera vectors
	olc::vf3d vf3dUp = { 0.0f, 1.0f, 0.0f };         // vf3d up direction
	olc::vf3d vf3dCamera = { 0.0f, 0.0f, -20.0f };    // vf3d camera direction
	olc::vf3d vf3dLookDir = { 0.0f, 0.0f, 1.0f };    // vf3d look direction
	olc::vf3d vf3dForward = { 0.0f, 0.0f, 0.0f };    // vf3d Forward direction
	olc::vf3d vf3dOffset = { 0.0f, 0.0f, -20.0f };    // vf3d Offset

	// Camera angles
	float fYaw = 0.0f;		    // FPS Camera rotation in X plane
	float fYawRoC = 1.0f;	    // fYaw Rate of Change Look Up/Down 
	float fTheta = 0.0f;	    // Spins World transform
	float fThetaRoC = 1.5f;	    // fTheta Rate of Change Spin Left/Right
	float fStrifeRoC = 8.5f;    // Strife Rate of Change, thanks: #Boguslavv
	float fForwardRoC = 8.0f;   // Forward/Backwards Rate of Change
	float fJump = vf3dOffset.y;	// Monitors jump height so we can land again
	float fJumpRoC = 4.0f;		// fTheta Rate of Change

	// 3D Camera
	olc::utils::hw3d::Camera3D Cam3D;

	// Object vertors, locations and scales
	olc::vf3d vf3dSkyCubeScale = { 600.0f, 600.0f, 600.0f };    // vf3d SkyCube Scale (in sort its Size)
	olc::vf3d vf3dSkyCubeLocation = { 0.0f, 0.0f, 0.0f };		// vf3d SkyCube Location 
	olc::vf3d vf3dSkyCubeOffset = { -200.0f, -300.0f, -200.0f };// vf3d SkyCube Offset
	olc::vf3d vf3dCubeBLCorner = { 0.0f, 0.0f, 0.0f };		    // vf3d Cube bottom left corner

	olc::vf3d vf3dBlackHoleScale = { 1.0f, 1.0f, 1.0f };		// vf3d Black hole Scale (in sort its Size)
	olc::vf3d vf3dBlackHoleLocation = { 0.0f, 0.0f, 0.0f };		// vf3d Black hole Location 
	olc::vf3d vf3dBlackHoleOffset = { 0.0f, 0.0f, 0.0f };		// vf3d black hole Offset

	olc::vd3d vf3dEventHorizonScale = { 10.0f, 10.0f, 10.0f };		// vf3d Event Horizon Scale (in sort its Size)
	olc::vd3d vf3dEventHorizonLocation = { 0.0f, 0.0f, 0.0f };	// vf3d Event Horizon Location
	olc::vd3d vf3dEventHorizonOffset = { 0.0f, 0.0f, 0.0f };	// vf3d Event Horizon Offset

	olc::vf3d vf3dBackGroundScale = { 600.0f, 600.0f, 600.0f }; // vf3d BackGround Scale (in sort its Size)
	olc::vf3d vf3dSBackGroundLocation = { 0.0f, 0.0f, 0.0f };	// vf3d BackGround Location 
	olc::vf3d vf3dBackGroundOffset = { 0.0f, 0.0f, 0.0f };		// vf3d BackGround Offset

	olc::vf3d vf3dGravityGridScale = { 10.0f, 10.0f, 10.0f };		// vf3d Gravity Grid Scale (in sort its Size)
	olc::vf3d vf3dGravityGridLocation = { 0.0f, -2.5f, 0.0f };	// vf3d Gravity Grid Location
	olc::vf3d vf3dGravityGridOffset = { 0.0f, 0.0f, 0.0f };		// vf3d Gravity Grid Offset
	

	// Sphere default properties
	float fSphereRoC = 0.5f;				// Sphere Rate of Change
	float fSphereRotaotionY = -1.57079633;	// Sphere start Y rotation position

	// Event Horizon default properties
	float fEventHorizonRoC = 0.5f;			// Event Horizon Rate of Change
	float fEventHorizonRotaotionZ = 0.0f;	// Event Horizon start Z rotation position

public:
	// Other stuff

	/* Sprites */
	/* END Sprites*/

	/* Decals */
	/* End Decals */

	/* Renderables */
	olc::Renderable renOLCPGEMobLogo;			// OLC PGE Mob Logo Renderable
	olc::Renderable renCube;					// Sanity Cube Renderable 
	olc::Renderable renBlackHole;				// Black Hole Renderable
	olc::Renderable renStar;					// Star Renderable
	olc::Renderable renSkyCube;					// Sky Cube Renderable
	olc::Renderable renBlackHoleDecal;			// Black Hole Decal Renderable
	olc::Renderable renEventHorizon;			// Event Horizon Renderable
	olc::Renderable renBackGround;				// Background Renderable
	/* End Reneders */

	/* Vectors */
	std::vector<std::string> vecMessages;
	/* END Vectors*/

	/* Screen Messages */
	uint32_t nFrameCount = 0;
	float fStep = 20;
	olc::vf2d vf2MessPos = { 10.0f, 10.0f };
	/* END Screen Messages */

	olc::vi2d centreScreenPos;
	bool Gravity = false;

public:

	// Black hole structure
	struct PGEBlackHole
	{
		olc::vd3d vPosition = { 0.0, 0.0, 0.0 };
		double Mass;			// The mass of the black hole in kilograms (kg).... these are big numbers!
		double r_s;				// The Schwarzschild radius (r_s) can be calculated using the formula: \[r_s = \frac{ 2GM }{c\ ^ 2} \]
		double EventHorizon;	// Typically set at 2.5 times the Schwarzschild radius for visualization

		PGEBlackHole(olc::vf3d position, double mass) : vPosition(position), Mass(mass)
		{
			r_s = 2 * G * Mass / (C * C);			/* Schwarzschild radius in meters r_s = \frac{ 2GM }{c\ ^ 2} \ */
			EventHorizon = r_s * dArbitraryfactor;	// Arbitrary factor for visualization
		}
	};

	struct Ray2D {
		olc::vd3d Position;				// Current position in Cartesian coordinates
		olc::vd3d Direction;			// Direction vector (velocity in Cartesian)
		olc::vd3d Polar;				// Polar coordinates (r, phi)
		std::vector<olc::vd3d> trail;	// Trail of positions

		double r;		// Radius (magnitude of pos)
		double phi;		// Angle from origin
		double dr;		// Radial velocity
		double dphi;	// Angular velocity
		double E;		// Energy 
		double L;		// Angular momentum

		Ray2D(olc::vd3d position, olc::vd3d direction, PGEBlackHole blackhole)
			: Position(position), Direction(direction), Polar(position.polar())
		{
			// Convert to polar coordinates
			r = Polar.x;
			phi = Polar.y;

			// Convert direction to polar velocities
			dr = Direction.x * cos(phi) + Direction.y * sin(phi);
			dphi = (-Direction.x * sin(phi) + Direction.y * cos(phi)) / r;

			// Compute conserved quantities, engergy E and angular momentum L
			L = r * r * dphi;
			double f = 1.0 - blackhole.r_s / r;
			double dt_d = sqrt((dr * dr) / (f * f) + (r * r * dphi * dphi) / f);
			E = f * dt_d;

			// Initialize trail
			trail.push_back(Position);
		}
	};
	
	std::vector<Ray2D> rays2D;

	struct Ray3D {
		olc::vd3d WorldViewPosition;			// Current position in Cartesian coordinates
		olc::vf3d ViewPortPosition;				// Current position projected to 3D view port
		olc::vd3d Direction;					// Direction vector (velocity in Cartesian)
		olc::vd3d Polar;						// Polar coordinates (r, phi)
		std::vector<olc::vf3d> viewPortTrail;	// Trail of positions projected to 3D view port smaller numbers
		int32_t Colour;

		double r;		// Radius (magnitude of pos)
		double theta;	// Angle from z-axis
		double dtheta;	// Angle from z-axis velocity
		double phi;		// Angle from origin
		double dr;		// Radial velocity
		double dphi;	// Angular velocity
		double E;		// Energy 
		double L;		// Angular momentum

		Ray3D(olc::vd3d worldViewPosition, olc::vf3d viewportPosition, olc::vd3d direction, PGEBlackHole blackhole, int32_t colour = olc::WHITE.n)
			: WorldViewPosition(worldViewPosition), ViewPortPosition(viewportPosition), Direction(direction), Colour(colour), Polar(worldViewPosition.polar())
		{
			// Convert to polar coordinates
			r = Polar.x;
			theta = Polar.y;
			phi = Polar.z;

			// Convert direction to polar velocities
			float dx = Direction.x, dy = Direction.y, dz = Direction.z;
			
			dtheta = (cos(theta) * cos(phi) * dx + cos(theta) * sin(phi) * dy - sin(theta) * dz) / r;
			dphi = (-sin(phi) * dx + cos(phi) * dy) / (r * sin(theta));

			L = r * r * sin(theta) * dphi;
			double f = 1.0 - blackhole.r_s / r;
			double dt_dL = sqrt((dr * dr) / f + r * r * (dtheta * dtheta + sin(theta) * sin(theta) * dphi * dphi));
			E = f * dt_dL;

			// Initialize trail
			//worldVTrail.push_back(worldViewPosition);
			viewPortTrail.push_back(viewportPosition);
		}
	};

	std::vector<Ray3D> rays3D;

	std::vector<std::pair<olc::vf3d, int32_t>> finalRayPoints; // Points in the gravity grid

	PGEBlackHole SagittariusA = PGEBlackHole({ 0.0, 0.0, 0.0 }, dSagittariusAMass); // Sagittarius A* black hole


public:

	
	
	// Some drawing functions
	olc::Renderable CreateBlackHoleEventHorizon(float radius) {
		olc::Renderable ren;
		ren.Create((radius * 2) + 2, (radius * 2) + 2);
		SetDrawTarget(ren.Sprite());
		Clear(olc::BLANK);
		FillCircle(radius, radius, radius, olc::DARK_YELLOW);
		FillCircle(radius, radius, radius - 1.0f, olc::BLACK);
		SetDrawTarget(nullptr);
		ren.Decal()->Update();
		return ren;
	}

	void DrawRays2D(const std::vector<Ray2D>& rays) {
		// draw current ray positions as points
		float screenX = 0;
		float screenY = 0;
		float alpha = 1.0f;

		for (const auto& ray : rays) {
			screenX = int32_t((ray.Position.x / WorldX + 0.5) * ScreenWidth());
			screenY = int32_t((ray.Position.y / WorldZ + 0.5) * ScreenHeight());
			DrawLineDecal(centreScreenPos, { screenX, screenY }, olc::WHITE);
			DrawLineDecal({ screenX, screenY }, { screenX + 1, screenY }, olc::WHITE);
			//Draw(screenX, screenY, olc::WHITE);
		}

		// draw each trail with fading alpha
		for (const auto& ray : rays) {
			size_t N = ray.trail.size();
			if (N < 2) continue;

			for (size_t i = 0; i < N; ++i) {
				// older points (i=0) get alpha≈0, newer get alpha≈1
				alpha = float(i) / float(N - 1);
				// convert world coords to screen coords
				screenX = int32_t((ray.trail[i].x / WorldX + 0.5) * ScreenWidth());
				screenY = int32_t((ray.trail[i].y / WorldY + 0.5) * ScreenHeight());
				DrawLineDecal({ screenX, screenY }, { screenX + 1, screenY }, olc::PixelF(1.0f, 1.0f, 1.0f, std::max(alpha, 0.05f)));

			}

		}

	}
	void RayStep2D(Ray2D& ray, double d, double rs) {
		// 1) integrate (r,φ,dr,dφ)
		if (ray.r <= rs) return; // stop if inside the event horizon
		rk4Step2D(ray, d, rs);

		// 2) convert back to cartesian x,y, TODO rework this to use 2D vectors properly
		ray.Position.x = ray.r * cos(ray.phi);
		ray.Position.y = ray.r * sin(ray.phi);

		// 3) record the trail
		ray.trail.push_back(ray.Position);
	}


	// Draws a single ray in 3D space
	void DrawRay3D(const Ray3D& ray) {
		// draw current ray positions as points
		float screenX = 0;
		float screenY = 0;
		float screenZ = 0;
		float LastScreenX = 0;
		float LastScreenY = 0;
		float LastScreenZ = 0;
		float alpha = 1.0f;

		screenX = ray.ViewPortPosition.x; 
		screenY = ray.ViewPortPosition.y; 
		screenZ = ray.ViewPortPosition.z; 

		HW3D_DrawLine((mf4dWorld).m, { 0.0f, 0.0f, 0.0f }, { screenX, screenY, screenZ }, olc::YELLOW);


	}


	// Removes tail rays that are outside of the X/Y bounds to optimize rendering
	void RemoveTrailRays3D(std::vector<Ray3D>& rays) {
		for (auto& ray : rays) {
			size_t N = ray.viewPortTrail.size();
			if (N < 2) return;

			for (size_t i = 1; i < N; ++i) {
				
				const auto& p = ray.viewPortTrail[i];
				if (std::abs(p.x) < 1.0)
				{
					finalRayPoints.push_back(std::make_pair(olc::vf3d{ p.x, p.y, p.z }, olc::YELLOW.n));
				}
				if (std::abs(p.y) < 1.0)
				{
					finalRayPoints.push_back(std::make_pair(olc::vf3d{ p.x, p.y, p.z }, olc::RED.n));
				}
				
			}
		}
	}

	// Draws all final ray points in 3D space
	void DrawFinalRayPoints()
	{
		for (const auto& p : finalRayPoints)
		{
			HW3D_DrawLineBox((mf4dWorld).m, { p.first.x, p.first.y, p.first.z }, { 0.001f, 0.001f, 0.001f }, p.second);
		}
		
	}

	// Draws all rays and trail rays in 3D space using multithreading
    void DrawRays3Ds_Threaded(const std::vector<Ray3D>& rays) 
	{
		const float screenW = float(ScreenWidth());
		const float screenH = float(ScreenHeight());
		const float invWorldX = 1.0f / float(WorldX);
		const float invWorldY = 1.0f / float(WorldY);
		const float invWorldZ = 1.0f / float(WorldZ);
		const float scale = 0.01f;

		std::mutex draw_mutex;
		auto draw_trail = [&](const Ray3D& ray) {
			size_t N = ray.viewPortTrail.size();
			if (N < 2) return;

			for (size_t i = 1; i < N; ++i) {
				float alpha = float(i) / float(N - 1);

				const auto& p = ray.viewPortTrail[i];

				{
					std::lock_guard<std::mutex> lock(draw_mutex);
					if (std::abs(p.x) < 1.0)
					{

						HW3D_DrawLine((mf4dWorld).m, { p.x, p.y, p.z }, { p.x, p.y + 0.01f, p.z }, ray.Colour);
					}
					if (std::abs(p.y) < 1.0)
					{
						HW3D_DrawLine((mf4dWorld).m, { p.x, p.y, p.z }, { p.x + 0.01f, p.y, p.z }, olc::RED);
					}
					
					//HW3D_DrawLineBox((mf4dWorld).m, { lp.x, lp.y, lp.z }, { 0.01f, 0.01f, 0.01f }, olc::YELLOW);
					//HW3D_DrawLine((mf4dWorld).m, { 0.0f, 0.0f, 0.0f }, { lp.x, lp.y, lp.z }, olc::PixelF(1.0f, 1.0f, 1.0f, std::max(alpha, 0.05f)));
				}
			}

		};

		std::vector<std::thread> threads;
		for (const auto& ray : rays) {
			threads.emplace_back(draw_trail, std::ref(ray));
		}
		for (auto& t : threads) t.join();
    }
		

    void RayStep3D(Ray3D& ray, double d, double rs) {
		

		if (ray.r <= rs) return; // stop if inside the event horizon
		rk4Step3D(ray, d, rs);

		ray.WorldViewPosition.x = ray.r * sin(ray.theta) * cos(ray.phi);
		ray.WorldViewPosition.y = ray.r * sin(ray.theta) * sin(ray.phi);
		ray.WorldViewPosition.z = ray.r * cos(ray.theta);

		ray.viewPortTrail.push_back(ConvertWorldViewPosToViewPortPos(ray.WorldViewPosition));
    }

    olc::vd3d ConvertWorldViewPosToViewPortPos(const olc::vd3d& worldPos) {
		
		const float invWorldVM = 1.0f / float(dScreenMToWorldVM);
		const float screenW = float(ScreenWidth());
		const float screenH = float(ScreenHeight());
		const float scale = 0.01f;

		const float x = float(worldPos.x) * invWorldVM * screenW * scale;
		const float y = float(worldPos.y) * invWorldVM * screenH * scale;
		const float z = float(worldPos.z) * invWorldVM * screenW * scale;
		return { x, y, z };
    }

    olc::vd3d ConvertViewPortPosToWorldViewPos(const olc::vd3d& viewPortPos) {
		const float invScreenW = 1.0f / float(ScreenWidth());
		const float invScreenH = 1.0f / float(ScreenHeight());
		const float scale = 0.01f;
		const double factorW = dScreenMToWorldVM * invScreenW / scale;
		const double factorH = dScreenMToWorldVM * invScreenH / scale;
		return {
			viewPortPos.x * factorW,
			viewPortPos.y * factorH,
			viewPortPos.z * factorW
		};
    }

	olc::Sprite* CreateLeftCrossTextMapImage(
		std::string left, std::string top,
		std::string front, std::string bottom, 
		std::string right, std::string back)
	{
		/*
		*  __________
		* |   TP     |
		* | LTFRRTBK |
		* |   BM     |
		*  ----------
		*/
		olc::Sprite* sprleft = new olc::Sprite(left);
		olc::Sprite* sprtop = new olc::Sprite(top);
		olc::Sprite* sprfront = new olc::Sprite(front);
		olc::Sprite* sprbottom = new olc::Sprite(bottom);
		olc::Sprite* sprright = new olc::Sprite(right);
		olc::Sprite* sprback = new olc::Sprite(back);

		olc::Sprite* skyCube = new olc::Sprite(sprleft->width * 4, sprleft->height * 3);
		SetDrawTarget(skyCube);
		Clear(olc::BLANK);
		
		// Left
		DrawSprite(0, sprleft->height, sprleft);
		// Top
		DrawSprite(sprtop->width, 0, sprtop);
		// Front
		DrawSprite(sprleft->width, sprleft->height, sprfront);
		// Bottom
		DrawSprite(sprbottom->width, sprleft->height * 2, sprbottom);
		// Right
		DrawSprite(sprleft->width * 2, sprleft->height, sprright);
		// Back
		DrawSprite(sprleft->width * 3, sprleft->height, sprback);
		
		return skyCube;
	}

public:
	// Some functions to help with the physics

	/* 2D Light Rays*/
	/*
	* The geodesicRHS function computes the right-hand side of the geodesic equations for a given ray in a Schwarzschild spacetime, 
	* updating the provided rhs array with derivatives of the ray's position and angular momentum.
	*/
	void geodesicRHS2D(const Ray2D& ray, double rhs[4], double rs) 
	{
		double r = ray.r;
		double dr = ray.dr;
		double dphi = ray.dphi;
		double E = ray.E;

		double f = 1.0 - rs / r;

		// dr/dλ = dr
		rhs[0] = dr;
		// dφ/dλ = dphi
		rhs[1] = dphi;

		// d²r/dλ² from Schwarzschild null geodesic:
		double dt_d = E / f;
		rhs[2] =
			-(rs / (2 * r * r)) * f * (dt_d * dt_d)
			+ (rs / (2 * r * r * f)) * (dr * dr)
			+ (r - rs) * (dphi * dphi);

		// d²φ/dλ² = -2*(dr * dphi) / r
		rhs[3] = -2.0 * dr * dphi / r;
	}

	void addState2D(const double a[4], const double b[4], double factor, double out[4])
	{
		for (int i = 0; i < 4; i++)
			out[i] = a[i] + b[i] * factor;
	}

	/*
	* The rk4Step function implements a single step of the Runge-Kutta 4th order method to update the state of a Ray object 
	* based on its current properties and a given time step.
	*/
	void rk4Step2D(Ray2D& ray, double d, double rs) 
	{
		double y0[4] = { ray.r, ray.phi, ray.dr, ray.dphi };
		double k1[4], k2[4], k3[4], k4[4], temp[4];

		geodesicRHS2D(ray, k1, rs);
		addState2D(y0, k1, d / 2.0, temp);
		Ray2D r2 = ray; r2.r = temp[0]; r2.phi = temp[1]; r2.dr = temp[2]; r2.dphi = temp[3];
		geodesicRHS2D(r2, k2, rs);

		addState2D(y0, k2, d / 2.0, temp);
		Ray2D r3 = ray; r3.r = temp[0]; r3.phi = temp[1]; r3.dr = temp[2]; r3.dphi = temp[3];
		geodesicRHS2D(r3, k3, rs);

		addState2D(y0, k3, d, temp);
		Ray2D r4 = ray; r4.r = temp[0]; r4.phi = temp[1]; r4.dr = temp[2]; r4.dphi = temp[3];
		geodesicRHS2D(r4, k4, rs);

		ray.r += (d / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
		ray.phi += (d / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
		ray.dr += (d / 6.0) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
		ray.dphi += (d / 6.0) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
	}

	/* End 2D Light Rays*/



	/* 3D Light Rays*/
	/*
	* The geodesicRHS function computes the right-hand side of the geodesic equations for a given ray in a Schwarzschild spacetime,
	* updating the provided rhs array with derivatives of the ray's position and angular momentum.
	*/
	void geodesicRHS3D(const Ray3D& ray, double rhs[4], double rs)
	{
		double r = ray.r;
		double dr = ray.dr;
		double dphi = ray.dphi;
		double E = ray.E;

		double f = 1.0 - rs / r;

		// dr/dλ = dr
		rhs[0] = dr;
		// dφ/dλ = dphi
		rhs[1] = dphi;

		// d²r/dλ² from Schwarzschild null geodesic:
		double dt_d = E / f;
		rhs[2] =
			-(rs / (2 * r * r)) * f * (dt_d * dt_d)
			+ (rs / (2 * r * r * f)) * (dr * dr)
			+ (r - rs) * (dphi * dphi);

		// d²φ/dλ² = -2*(dr * dphi) / r
		rhs[3] = -2.0 * dr * dphi / r;
	}

	void addState3D(const double a[4], const double b[4], double factor, double out[4])
	{
		for (int i = 0; i < 4; i++)
			out[i] = a[i] + b[i] * factor;
	}

	/*
	* The rk4Step function implements a single step of the Runge-Kutta 4th order method to update the state of a Ray object
	* based on its current properties and a given time step.
	*/
    // Updated RK4 step for 3D geodesics (r, theta, phi, dr, dtheta, dphi)
    void rk4Step3D(Ray3D& ray, double d, double rs)
    {
        // State vector: [r, theta, phi, dr, dtheta, dphi]
        double y0[6] = { ray.r, ray.theta, ray.phi, ray.dr, ray.dtheta, ray.dphi };
        double k1[6], k2[6], k3[6], k4[6], temp[6];

        geodesicRHS3D(ray, k1, rs);
        for (int i = 0; i < 6; ++i) temp[i] = y0[i] + k1[i] * (d / 2.0);
        Ray3D r2 = ray;
        r2.r = temp[0]; r2.theta = temp[1]; r2.phi = temp[2];
        r2.dr = temp[3]; r2.dtheta = temp[4]; r2.dphi = temp[5];
        geodesicRHS3D(r2, k2, rs);

        for (int i = 0; i < 6; ++i) temp[i] = y0[i] + k2[i] * (d / 2.0);
        Ray3D r3 = ray;
        r3.r = temp[0]; r3.theta = temp[1]; r3.phi = temp[2];
        r3.dr = temp[3]; r3.dtheta = temp[4]; r3.dphi = temp[5];
        geodesicRHS3D(r3, k3, rs);

        for (int i = 0; i < 6; ++i) temp[i] = y0[i] + k3[i] * d;
        Ray3D r4 = ray;
        r4.r = temp[0]; r4.theta = temp[1]; r4.phi = temp[2];
        r4.dr = temp[3]; r4.dtheta = temp[4]; r4.dphi = temp[5];
        geodesicRHS3D(r4, k4, rs);

        for (int i = 0; i < 6; ++i)
            y0[i] += (d / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);

        ray.r = y0[0];
        ray.theta = y0[1];
        ray.phi = y0[2];
        ray.dr = y0[3];
        ray.dtheta = y0[4];
        ray.dphi = y0[5];
    }

	/* End 3D Light Rays*/

	/*
	* Returns the center point of a cube given its bottom-left corner and side length.
	*/
	olc::vf3d getCubeCornerFromCenter(olc::vf3d centerpos, float sideLength)
	{
		olc::vf3d corner;
		corner.x = centerpos.x + sideLength / 2.0f;
		corner.y = centerpos.y + sideLength / 2.0f;
		corner.z = centerpos.z + sideLength / 2.0f;
		return corner;
	}



public:

	// Load any 3D objects here
	void Load3DObjects()
	{
		// Load any 3D objects here
		mf4dWorld.identity();
		mf4dView.identity();

		auto t = olc::utils::hw3d::LoadObj("assets/objectfiles/mountains.obj");
		if (t.has_value())
		{
			meshSpaceGrid = *t;
		}
		
		// Create required matrices
		meshSphere = olc::utils::hw3d::CreateSphere();								// Default sphere
		meshEventHorizon = olc::utils::hw3d::Create3DTorus(1.0f, 0.1f, 64, 32);		// Default Event Horizon
		meshBlackHole = olc::utils::hw3d::Create2DCircle(1.0f, 128, olc::BLACK);	// Default Black Hole
		meshBackGround = olc::utils::hw3d::CreateSphere();							// Default sphere for background
		//meshGravityGrid = olc::utils::hw3d::CreateGrid(1.0f, 30);					// Default Grid
		meshGravityGrid = olc::utils::hw3d::CreateGrid(25.0f, 50);


		// Load any textures here
		renStar.Load("assets/images/NASA_2020_4k.jpg");
		renEventHorizon.Load("assets/images/NASA_2020_4k.jpg");
		renSkyCube.Load("assets/images/spacetexture.png");
		renBlackHoleDecal = CreateBlackHoleEventHorizon(1.0f);
		renBackGround.Load("assets/images/starmap_2020_4k_updated.png");

		/*auto skyCubeImage = CreateLeftCrossTextMapImage(
			"assets/images/skybox_left.png",
			"assets/images/skybox_top.png",
			"assets/images/skybox_front.png",
			"assets/images/skybox_bottom.png",
			"assets/images/skybox_right.png",
			"assets/images/skybox_back.png"
		);
		renSkyCube.Create(skyCubeImage->width, skyCubeImage->height);
		renSkyCube.Sprite()->pColData.swap(skyCubeImage->pColData);
		renSkyCube.Decal()->Update();
		delete skyCubeImage;*/
		
		// Load Properties for Renderables
		vf3dCubeBLCorner = getCubeCornerFromCenter(vf3dCubeBLCorner, 10.0f);
	}

	/*
	* Updates the cam position by user input
	* Mouse/Touch/Keyboard
	*/
	void UpdateCamByUserInput(float fElapsedTime)
	{
		// Handle Camera
		// Touch zeros (single touch) handles Camera look direction
		if (GetMouse(0).bHeld)
		{

			// We know the Right Center point we need to compare our positions
			// Looking Right
			if ((float)GetMousePos().x > (((float)centreScreenPos.x / 100) * 130))
			{
				fTheta -= fThetaRoC * fElapsedTime;


			}

			// Looking Left
			if ((float)GetMousePos().x < (((float)centreScreenPos.x / 100) * 70))
			{
				fTheta += fThetaRoC * fElapsedTime;


			}

			// Looking Up
			if ((float)GetMousePos().y < (((float)centreScreenPos.y / 100) * 70))
			{
				fYaw -= fYawRoC * fElapsedTime;
				if (fYaw < -1.0f) fYaw = -1.0f;
			}

			// Looking Down
			if ((float)GetMousePos().y > (((float)centreScreenPos.y / 100) * 130))
			{
				fYaw += fYawRoC * fElapsedTime;
				if (fYaw > 1.0f) fYaw = 1.0f;
			}

		}
		else
		{
			// Move the camera back to center, stops the dizzies!
			if (fYaw > -0.01f && fYaw < 0.01f)
			{
				fYaw = 0.0f;
			}
			if (fYaw >= 0.01)
			{
				fYaw -= fYawRoC * fElapsedTime;

			}
			if (fYaw <= -0.01)
			{
				fYaw += fYawRoC * fElapsedTime;

			}

		}

		// Handle movement
		// Moving Forward
		if (GetKey(olc::Key::UP).bHeld || GetMouse(1).bHeld)
		{
			vf3dCamera += vf3dForward;
		}

		// Moving Backward
		if (GetKey(olc::Key::DOWN).bHeld)
		{
			vf3dCamera -= vf3dForward;
		}

		// Moving Left (Strife)
		if (GetKey(olc::Key::LEFT).bHeld)
		{
			vf3dCamera.x -= cos(fTheta) * fStrifeRoC * fElapsedTime;
			vf3dCamera.z -= sin(fTheta) * fStrifeRoC * fElapsedTime;
		}


		// Moving Right (Strife)
		if (GetKey(olc::Key::RIGHT).bHeld)
		{
			vf3dCamera.x += cos(fTheta) * fStrifeRoC * fElapsedTime;
			vf3dCamera.z += sin(fTheta) * fStrifeRoC * fElapsedTime;

		}


		// Moving UP
		if (GetKey(olc::Key::U).bHeld)
		{
			fJump += fJumpRoC * fElapsedTime;
			vf3dCamera.y = fJump;
		}
		else if (GetKey(olc::Key::B).bHeld)
		{
			fJump -= fJumpRoC * fElapsedTime;
			vf3dCamera.y = fJump;

		}
		else
		{
			/* if (fJump > (vf3dOffset.y - 0.01f) && fJump < (vf3dOffset.y + 0.01f))
			 {
				 fJump = vf3dOffset.y;
				 vf3dCamera.y = fJump;
			 }
			 if (fJump >= (vf3dOffset.y + 0.01))
			 {
				 fJump -= 4.0f * fElapsedTime;
				 vf3dCamera.y = fJump;
			 }
			 if (fJump <= (vf3dOffset.y - 0.01))
			 {
				 fJump += 4.0f * fElapsedTime;
				 vf3dCamera.y = fJump;
			 }*/
		}

	}

	/*
	* Displays messages on the screen
	*/
	void DisplayMessages()
	{
		nFrameCount = GetFPS();

		std::string sMessage = "OneLoneCoder.com";
		vecMessages.push_back(sMessage);

		sMessage = sAppName + " - FPS: " + std::to_string(nFrameCount);
		vecMessages.push_back(sMessage);


		sMessage = "---";
		vecMessages.push_back(sMessage);

		fStep = 10;
		vf2MessPos.y = fStep;
		for (auto& s : vecMessages)
		{
			DrawStringDecal(vf2MessPos, s);
			vf2MessPos.y += fStep;
		}
		vecMessages.clear();


	}

public:
	bool OnUserCreate() override
	{

		// Setup up worldview parameters
		WorldX = double((ScreenWidth() * 2.0) / dKMtoMeters)* dScreenMToWorldVM;	// Width of the WorldView in meters
		WorldY = double((ScreenHeight() * 2.0) / dKMtoMeters) * dScreenMToWorldVM;	// Height of the WorldView in meters
		WorldZ = WorldX; // a little hacky but we what our graviry grid to be square, hence WorldX = WorldZ

		// Setup 3D Camera
		float fAspect = float(GetScreenSize().x) / float(GetScreenSize().y); // Width / height 
		float S = 1.0f / (tan(3.14159f * 0.25f));
		float f = 1000.0f;
		float n = 0.1f;

		Cam3D.SetScreenSize(GetScreenSize());
		Cam3D.SetClippingPlanes(n, f);
		Cam3D.SetFieldOfView(S);

		centreScreenPos = GetScreenSize();
		centreScreenPos.x = centreScreenPos.x / 2;
		centreScreenPos.y = centreScreenPos.y / 2;

		// Load 3D objects
		Load3DObjects();

		// Load any sprites, decals or renderables here
		//renOLCPGEMobLogo.Load("assets/images/olcpgemob.png");  //TODO add logo
		

		/*
			Sagittarius A*, the supermassive black hole at the center of our Milky Way galaxy, and has an estimated mass of approximately 4.154 million solar masses.
			That’s roughly:
				- 8 × 10³⁶ kilograms — an almost unfathomable amount of mass!
				- Equivalent to over 4 million times the mass of our Sun.
				- Its Schwarzschild radius is about 12 million kilometers (or about 7.5 million miles), which is roughly 17 times the radius of our Sun.
				- Sagittarius A* is located about 26,000 light-years away from Earth

			Create a black hole with the mass of Sagittarius A* at the origin
		*/
		SagittariusA = PGEBlackHole({ 0.0, 0.0, 0.0 }, dSagittariusAMass);

		rays2D.push_back(Ray2D(vd2dLoopyLoop, vd2dConstLightDir, SagittariusA));

		// Create the Black Hole Sphere

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		SetDrawTarget(nullptr);
		Clear(olc::BLACK);
			

		// 3D Render section
		olc::mf4d mRotationX, mRotationY, mRotationZ;  // Rotation Matrices
		olc::mf4d mCubeTrans, mCubeScale;
		olc::mf4d mf4dSkyCubeTrans, mf4dSkyCubeScale, mf4dSkyCubeRotationX, mf4dSkyCubeRotationY, mf4dSkyCubeRotationZ;
		olc::mf4d mSphereTrans, mSphereScale, mSphereRotationX, mSphereRotationY, mSphereRotationZ;
		olc::mf4d mEventHozTrans, mEventHozScale, mEventHozRotationX, mEventHozRotationY, mEventHozRotationZ;
		olc::mf4d mPosition, mCollision;
		olc::mf4d mMovement, mOffset;
		olc::mf4d mBackGroundTrans, mBackGroundScale, mBackGroundRotationX, mBackGroundRotationY, mBackGroundRotationZ;
		olc::mf4d mGravityGridTrans, mGravityGridScale, mGravityGridRotationX, mGravityGridRotationY, mGravityGridRotationZ;


		// Setup Event Horizon
		mEventHozTrans.translate(vf3dEventHorizonLocation);
		mEventHozScale.scale(vf3dEventHorizonScale);
		mEventHozRotationY.rotateY(fTheta);
		mEventHozRotationX.rotateX(fYaw);
		
		mf4dEventHorizon = mEventHozTrans * mEventHozScale * mEventHozRotationY; // Rotate the Sphere into the correct North/South pole position
		mf4dEventHorizon = mf4dEventHorizon * mEventHozRotationX;
		mEventHozRotationZ.rotateZ(fTheta);
		mf4dEventHorizon = mf4dEventHorizon * mEventHozRotationZ;

		// Setup Grid
		mGravityGridTrans.translate(vf3dGravityGridLocation);
		mGravityGridScale.scale(vf3dGravityGridScale);
		// As the grid is flat we only need to rotate on the X axis
		mGravityGridRotationX.rotateX(0.17079633f);

		mf4dGravityGrid = mGravityGridTrans * mGravityGridScale;// *mGravityGridRotationX;

		// Setup Camera
		olc::vf3d vf3dTarget = { 0,0,1 };

		mRotationY.rotateY(fTheta);  // Left/Right
		mRotationX.rotateX(fYaw);    // Up/Down

		vf3dLookDir = mRotationY * mRotationX * vf3dTarget;   // Left-Right * Up-Down
		vf3dTarget = vf3dCamera + vf3dLookDir;

		Cam3D.SetPosition(vf3dCamera);
		Cam3D.SetTarget(vf3dTarget);
		Cam3D.Update();
		mf4dWorld = Cam3D.GetViewMatrix();

		// Manage forward / backwards
		vf3dForward = vf3dLookDir * (fForwardRoC * fElapsedTime);


		// Setup World Background
		mBackGroundTrans.translate(Cam3D.GetPosition());
		mBackGroundScale.scale(vf3dBackGroundScale);
		mBackGroundRotationZ.rotateZ(1.606f);
		mBackGroundRotationY.rotateY(-0.546f);

		mf4dBackGround = mBackGroundTrans * mBackGroundScale * mBackGroundRotationY * mBackGroundRotationZ;

		// Set our projection matrix
		HW3D_Projection(Cam3D.GetProjectionMatrix().m);

		// Draw the Background Shere
		HW3D_DrawObject((mf4dWorld * mf4dBackGround).m, renBackGround.Decal(), meshBackGround.layout, meshBackGround.pos, meshBackGround.uv, meshBackGround.col);
				

		// Draw the black hole
		HW3D_DrawObject((mf4dWorld * mf4dEventHorizon).m, nullptr, meshBlackHole.layout, meshBlackHole.pos, meshBlackHole.uv, meshBlackHole.col);

		// Draw the Event Horizon
		HW3D_DrawObject((mf4dWorld * mf4dEventHorizon).m, renStar.Decal(), meshEventHorizon.layout, meshEventHorizon.pos, meshEventHorizon.uv, meshEventHorizon.col);


		// Draw the Gravity Grid
		HW3D_DrawObject((mf4dWorld * mf4dGravityGrid).m, nullptr, meshGravityGrid.layout, meshGravityGrid.pos, meshGravityGrid.uv, meshGravityGrid.col);


		// dRAW SOME LINES AND BOXES FOR DEBUGGING
		HW3D_DrawLine((mf4dWorld).m, { 0.0f, 0.0f, 0.0f }, { 100.0f, 100.0f, 100.0f }, olc::RED);

		HW3D_DrawLineBox((mf4dWorld).m, { -vf3dCubeBLCorner.x, -vf3dCubeBLCorner.y, -vf3dCubeBLCorner.z }, { 10.0f, 10.0f, 10.0f }, olc::YELLOW);


		olc::vf2d vCenterPos = { float(ScreenWidth()) / 2.0f, float(ScreenHeight()) / 2.0f };
		//DrawDecal(vCenterPos - olc::vf2d(renBlackHoleDecal.Sprite()->width / 2.0f, renBlackHoleDecal.Sprite()->height / 2.0f), renBlackHoleDecal.Decal());

		if (GetKey(olc::Key::R).bPressed)
		{
			rays3D.clear();
			rays3D.push_back(Ray3D(vd2dLoopyLoop, ConvertWorldViewPosToViewPortPos(vd2dLoopyLoop), vd2dConstLightDirZ, SagittariusA, olc::YELLOW.n));
			//rays3D.push_back(Ray3D(vd2dLoopyLoop, ConvertWorldViewPosToViewPortPos(vd2dLoopyLoop), vd2dConstLightDirY, SagittariusA, olc::RED.n));
			//rays3D.push_back(Ray3D(vd2dLoopyLoop, ConvertWorldViewPosToViewPortPos(vd2dLoopyLoop), vd2dConstLightDirZ, SagittariusA));
		}
		if (GetKey(olc::Key::SPACE).bPressed)
		{
			// creates 10,000 rays 
			for (size_t i = 0; i < 30000; i++)
			{
				for (auto& ray : rays3D) {
					RayStep3D(ray, 1.0f, SagittariusA.r_s);
				}
			}

			RemoveTrailRays3D(rays3D);
    			
		}

		// DrawRays3Ds_Threaded(rays3D);

		DrawFinalRayPoints();

		if (GetKey(olc::Key::ESCAPE).bPressed)
		{
			return false;
		}


		// Update Camera by user input
		UpdateCamByUserInput(fElapsedTime);

		// Display Messages
		DisplayMessages();

		return true;
	}



};

int main()
{
	PGEBlackHoleDemo demo;
	if (demo.Construct(1280, 720, 1, 1))
		demo.Start();
	return 0;
}
