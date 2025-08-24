#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

#define OLC_PGEX_SPLASHSCREEN		// Manages the GPL-3.0 Licence requirements 
#include "olcPGEX_SplashScreen.h"


// Override base class with your custom functionality
class Example : public olc::PixelGameEngine
{
public:
	olc::SplashScreen olcSplashScreen;

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
		// Called once per frame, draws random coloured pixels
		for (int x = 0; x < ScreenWidth(); x++)
			for (int y = 0; y < ScreenHeight(); y++)
				Draw(x, y, olc::Pixel(rand() % 256, rand() % 256, rand() % 256));
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
