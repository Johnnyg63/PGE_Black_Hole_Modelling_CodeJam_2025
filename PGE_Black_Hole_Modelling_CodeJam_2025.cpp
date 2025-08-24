
#define OLC_GFX_OPENGL33
#define OLC_PGE_APPLICATION
#define OLC_IMAGE_STB
#include "olcUTIL_Hardware3D.h"
#include "olcPixelGameEngine.h"
#include <immintrin.h>			// For AVX/SSE
#define _USE_MATH_DEFINES 
#include <cmath>				// Maths lots of maths
#include <vector>				// Vectors lots of vectors
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // M_PI

double C = 299792458;			// Speed of light in m/s
double G = 6.67430e-11;			// Gravitational constant
double dAU = 1.496e+11;			// Astronomical Unit in meters
double dPC = 3.086e+16;			// Parsec in meters
double dLY = 9.461e+15;			// Light Year in meters
double dSolarMass = 1.989e+30;	// Solar mass in kg
double dEarthMass = 5.972e+24;	// Earth mass in kg
double dEarthRadius = 6.371e+6; // Earth radius in meters
double dSunMass = dSolarMass;	// Sun mass in kg (1 Solar mass = our sun mass :))
double dSunRadius = 6.9634e+8;	// Sun radius in meters

double dSagittariusAMass = 4.1e6 * dSolarMass; // Mass of Sagittarius A* in kg (approximately 4.154 million solar masses)

double dArbitraryfactor = 2.5;	// Arbitrary factor for visualization of event horizon

float WorldWidth = 100000000000.0f * 2.0f; // Width of the viewport in meters
float WorldHeight = 75000000000.0f * 2.0f; // Height of the viewport in meters

#define OLC_PGEX_SPLASHSCREEN		// Manages the GPL-3.0 Licence requirements 
#include "olcPGEX_SplashScreen.h"


// Override base class with your custom functionality
class Example : public olc::PixelGameEngine
{

public:
	//olc::SplashScreen olcSplashScreen;

	// In Example's constructor, initialize PBH_SagittariusA after the class definition
	Example() {
		sAppName = "PGE Black Hole Modelling CodeJam2025";

	}

public:
	// Black hole structure
	struct PGEBlackHole
	{
		olc::vd3d vPosition = { 0.0, 0.0, 0.0 };
		double dMass;			// The mass of the black hole in kilograms (kg).... these are big numbers!
		double dr_s;			// the Schwarzschild radius (r_s) can be calculated using the formula: \[r_s = \frac{ 2GM }{c\ ^ 2} \]
		double dEventHorizon;	// Typically set at 2.5 times the Schwarzschild radius for visualization

		PGEBlackHole(olc::vf3d vf3dPos, double mass) : vPosition(vf3dPos), dMass(mass)
		{
			dr_s = 2 * G * dMass / (C * C);				/* Schwarzschild radius in meters r_s = \frac{ 2GM }{c\ ^ 2} \ */
			dEventHorizon = dr_s * dArbitraryfactor;	// Arbitrary factor for visualization
		}
	};


	// Light ray structure
	struct Ray {
		// -- cartesian coords -- //
		double x;   double y;
		// -- polar coords -- //
		double r;   double phi;
		double dr;  double dphi;
		std::vector<olc::vd2d> trail; // trail of points
		double E, L;             // conserved quantities

		Ray(olc::vd2d pos, olc::vd2d dir, PGEBlackHole blackhole) : x(pos.x), y(pos.y), r(pos.mag()), phi(atan2(pos.y, pos.x)), dr(dir.x), dphi(dir.y) {
			// step 1) get polar coords (r, phi) :
			this->r = sqrt(x * x + y * y);
			this->phi = atan2(y, x);
			// step 2) seed velocities :
			dr = dir.x * cos(phi) + dir.y * sin(phi); // m/s
			dphi = (-dir.x * sin(phi) + dir.y * cos(phi)) / r;
			// step 3) store conserved quantities
			L = r * r * dphi;
			double f = 1.0 - blackhole.dr_s / r;
			double dt_d = sqrt((dr * dr) / (f * f) + (r * r * dphi * dphi) / f);
			E = f * dt_d;
			// step 4) start trail :
			trail.push_back({ x, y });
		}

	};
	std::vector<Ray> rays;

	PGEBlackHole SagittariusA = PGEBlackHole({ 0.0, 0.0, 0.0 }, dSagittariusAMass); // Sagittarius A* black hole


public:

	// Some drawing functions

	void DrawRays(const std::vector<Ray>& rays) {
		// draw current ray positions as points
		double screenX = 0.0;
		double screenY = 0.0;
		float alpha = 1.0f;

		for (const auto& ray : rays) {
			screenX = (ray.x / WorldWidth + 0.5) * ScreenWidth();
			screenY = (ray.y / WorldHeight + 0.5) * ScreenHeight();
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
				screenX = (ray.trail[i].x / WorldWidth + 0.5) * ScreenWidth();
				screenY = (ray.trail[i].y / WorldHeight + 0.5) * ScreenHeight();
				Draw(screenX, screenY, olc::PixelF(1.0f, 1.0f, 1.0f, std::max(alpha, 0.05f)));

			}

		}

	}
	void RayStep(Ray& ray, double d, double rs) {
		// 1) integrate (r,φ,dr,dφ)
		if (ray.r <= rs) return; // stop if inside the event horizon
		rk4Step(ray, d, rs);

		// 2) convert back to cartesian x,y
		ray.x = ray.r * cos(ray.phi);
		ray.y = ray.r * sin(ray.phi);

		// 3) record the trail
		ray.trail.push_back({ float(ray.x), float(ray.y) });
	}


	// Some functions to help with the physics
	void geodesicRHS(const Ray& ray, double rhs[4], double rs) {
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

	void addState(const double a[4], const double b[4], double factor, double out[4]) {
		for (int i = 0; i < 4; i++)
			out[i] = a[i] + b[i] * factor;
	}

	void rk4Step(Ray& ray, double d, double rs) {
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
		//rays.push_back(Ray({ 0.0, 0.0 }, { 1.0, 0.0 }, SagittariusA));

		rays.push_back(Ray({ -1e11, 3.27606302719999999e10 }, { C, 0.0 }, SagittariusA));

		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		Clear(olc::BLACK);
		olc::vf2d vCenterPos = { float(ScreenWidth()) / 2.0f, float(ScreenHeight()) / 2.0f };
		float fRadus = std::min(ScreenWidth(), ScreenHeight()) / 2.0f * 0.1f;
		FillCircle(vCenterPos, fRadus, olc::DARK_YELLOW);
		FillCircle(vCenterPos, fRadus - 0.5, olc::BLACK);
		if (GetKey(olc::Key::SPACE).bHeld)
		{
			for (auto& ray : rays) {
				RayStep(ray, 1.0f, SagittariusA.dr_s);
				DrawRays(rays);
			}

		}
			
		
		
		return true;
	}



};

int main()
{
	Example demo;
	if (demo.Construct(800, 600, 1, 1))
		demo.Start();
	return 0;
}
