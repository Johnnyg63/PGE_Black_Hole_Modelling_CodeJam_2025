
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


#define OLC_PGEX_SPLASHSCREEN		// Manages the GPL-3.0 Licence requirements 
#include "olcPGEX_SplashScreen.h"


// Override base class with your custom functionality
class Example : public olc::PixelGameEngine
{
public:
	//olc::SplashScreen olcSplashScreen;

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

	/*
	Sagittarius A*, the supermassive black hole at the center of our Milky Way galaxy, and has an estimated mass of approximately 4.154 million solar masses. 
	That’s roughly:
		- 8 × 10³⁶ kilograms — an almost unfathomable amount of mass!
		- Equivalent to over 4 million times the mass of our Sun.
		- Its Schwarzschild radius is about 12 million kilometers (or about 7.5 million miles), which is roughly 17 times the radius of our Sun.
		- Sagittarius A* is located about 26,000 light-years away from Earth

	Create a black hole with the mass of Sagittarius A* at the origin
	*/
	PGEBlackHole PBH_SagittariusA = PGEBlackHole({ 0.0, 0.0, 0.0 }, dSagittariusAMass);

public:
	Example()
	{
		// Name your application
		sAppName = "PGE Black Hole Modelling CodeJam2025";
	}

public:
	bool OnUserCreate() override
	{
		// Called once at the start, so create things here
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		Clear(olc::BLACK);
		olc::vf2d vCenterPos = { float(ScreenWidth()) / 2.0f, float(ScreenHeight()) / 2.0f };
		float fRadus = std::min(ScreenWidth(), ScreenHeight()) / 2.0f * 0.3f;
		FillCircle(vCenterPos, int(fRadus), olc::RED);
		
		return true;
	}
};

int main()
{
	Example demo;
	if (demo.Construct(640, 480, 2, 2))
		demo.Start();
	return 0;
}
