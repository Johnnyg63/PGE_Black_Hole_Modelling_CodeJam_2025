
#define OLC_GFX_OPENGL33
#define OLC_PGE_APPLICATION
#define OLC_IMAGE_STB
#include "olcUTIL_Hardware3D.h"
#include "olcPixelGameEngine.h"
#include <immintrin.h>			
#define _USE_MATH_DEFINES 
#include <cmath>				
#include <vector>				
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
olc::vd3d vd2dLoopyLoop = { -1e+11, 3.13106302719999999e+10, 0.0 };

olc::vd3d vd2dConstLightDir = { C, 0.0, 0.0 };// Const Initial direction of the light ray (pointing right along the x-axis)

const double dKMtoMeters = 1e+3;		// Conversion factor from kilometers to meters
const double dMetersToKM = 1e-3;		// Conversion factor from meters to kilometers
const double dScreenMToWorldVM = 1e+11;	// Conversion factor from Screen meters to WorldView Meters(1e11 meters)
double WorldWidth = 0;					// Width of the viewport in meters  
double WorldHeight = 0;					// Height of the viewport in meters

                     

#define OLC_PGEX_SPLASHSCREEN		// Manages the GPL-3.0 Licence requirements 
#include "olcPGEX_SplashScreen.h"


// Override base class with your custom functionality
class PGEBlackHoleDemo : public olc::PixelGameEngine
{

public:
	olc::SplashScreen olcSplashScreen;

	// In Example's constructor, initialize PBH_SagittariusA after the class definition
	PGEBlackHoleDemo() {
		sAppName = "PGE Black Hole Modelling CodeJam2025";

	}

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

	struct Ray {
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

		Ray(olc::vd3d position, olc::vd3d direction, PGEBlackHole blackhole)
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
	

	std::vector<Ray> rays;

	PGEBlackHole SagittariusA = PGEBlackHole({ 0.0, 0.0, 0.0 }, dSagittariusAMass); // Sagittarius A* black hole


public:

	// Some drawing functions

	void DrawRays(const std::vector<Ray>& rays) {
		// draw current ray positions as points
		int32_t screenX = 0;
		int32_t screenY = 0;
		float alpha = 1.0f;

		for (const auto& ray : rays) {
			screenX = int32_t((ray.Position.x / WorldWidth + 0.5) * ScreenWidth());
			screenY = int32_t((ray.Position.y / WorldHeight + 0.5) * ScreenHeight());
			Draw(screenX, screenY, olc::WHITE);
		}

		// draw each trail with fading alpha
		for (const auto& ray : rays) {
			size_t N = ray.trail.size();
			if (N < 2) continue;

			for (size_t i = 0; i < N; ++i) {
				// older points (i=0) get alpha≈0, newer get alpha≈1
				alpha = float(i) / float(N - 1);
				// convert world coords to screen coords
				screenX = int32_t((ray.trail[i].x / WorldWidth + 0.5) * ScreenWidth());
				screenY = int32_t((ray.trail[i].y / WorldHeight + 0.5) * ScreenHeight());
				Draw(screenX, screenY, olc::PixelF(1.0f, 1.0f, 1.0f, std::max(alpha, 0.05f)));

			}

		}

	}
	void RayStep(Ray& ray, double d, double rs) {
		// 1) integrate (r,φ,dr,dφ)
		if (ray.r <= rs) return; // stop if inside the event horizon
		rk4Step(ray, d, rs);

		// 2) convert back to cartesian x,y, TODO rework this to use 2D vectors properly
		ray.Position.x = ray.r * cos(ray.phi);
		ray.Position.y = ray.r * sin(ray.phi);

		// 3) record the trail
		ray.trail.push_back(ray.Position);
	}


	// Some functions to help with the physics

	/*
	* The geodesicRHS function computes the right-hand side of the geodesic equations for a given ray in a Schwarzschild spacetime, 
	* updating the provided rhs array with derivatives of the ray's position and angular momentum.
	*/
	void geodesicRHS(const Ray& ray, double rhs[4], double rs) 
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

	void addState(const double a[4], const double b[4], double factor, double out[4])
	{
		for (int i = 0; i < 4; i++)
			out[i] = a[i] + b[i] * factor;
	}

	/*
	* The rk4Step function implements a single step of the Runge-Kutta 4th order method to update the state of a Ray object 
	* based on its current properties and a given time step.
	*/
	void rk4Step(Ray& ray, double d, double rs) 
	{
		double y0[4] = { ray.r, ray.phi, ray.dr, ray.dphi };
		double k1[4], k2[4], k3[4], k4[4], temp[4];

		geodesicRHS(ray, k1, rs);
		addState(y0, k1, d / 2.0, temp);
		Ray r2 = ray; r2.r = temp[0]; r2.phi = temp[1]; r2.dr = temp[2]; r2.dphi = temp[3];
		geodesicRHS(r2, k2, rs);

		addState(y0, k2, d / 2.0, temp);
		Ray r3 = ray; r3.r = temp[0]; r3.phi = temp[1]; r3.dr = temp[2]; r3.dphi = temp[3];
		geodesicRHS(r3, k3, rs);

		addState(y0, k3, d, temp);
		Ray r4 = ray; r4.r = temp[0]; r4.phi = temp[1]; r4.dr = temp[2]; r4.dphi = temp[3];
		geodesicRHS(r4, k4, rs);

		ray.r += (d / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
		ray.phi += (d / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
		ray.dr += (d / 6.0) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
		ray.dphi += (d / 6.0) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
	}




public:
	bool OnUserCreate() override
	{

		// Setup up worldview parameters
		WorldWidth = double((ScreenWidth() * 2.0) / dKMtoMeters)* dScreenMToWorldVM;	// Width of the WorldView in meters
		WorldHeight = double((ScreenHeight() * 2.0) / dKMtoMeters) * dScreenMToWorldVM;	// Height of the WorldView in meters

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

		rays.push_back(Ray(vd2dLoopyLoop, vd2dConstLightDir, SagittariusA));

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		Clear(olc::BLACK);
		olc::vf2d vCenterPos = { float(ScreenWidth()) / 2.0f, float(ScreenHeight()) / 2.0f };
		float fRadus = std::min(ScreenWidth(), ScreenHeight()) / 2.0f * 0.15f;
		FillCircle(vCenterPos, fRadus, olc::DARK_YELLOW);
		FillCircle(vCenterPos, fRadus - 0.5, olc::BLACK);

		if (GetKey(olc::Key::R).bPressed)
		{
			rays.clear();
			rays.push_back(Ray(vd2dLoopyLoop, vd2dConstLightDir, SagittariusA));
		}
		if (GetKey(olc::Key::SPACE).bHeld)
		{
			for (auto& ray : rays) {
				RayStep(ray, 1.0f, SagittariusA.r_s);
				DrawRays(rays);
			}

		}

		if (GetKey(olc::Key::ESCAPE).bPressed)
		{
			return false;
		}
			

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
