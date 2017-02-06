//
//  GUICode.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//


#include <vector>
#include <fstream>

#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapOverlay.h"
#include "TemplateAStar.h"
#include "MM.h"
#include "BOBA.h"
#include "SVGUtil.h"
//#include "WeightedHeuristic.h"

#include "GUICode.h"


//#define COMPARE_TO_MM

#define COMPARE_TO_ASTAR

#define HEUR_MAP

//#define HEUR_ZERO

const double WEIGHT = 1.0;


Map *map = 0;
MapEnvironment *me = 0;
MapOverlay *mo;
xyLoc start, goal;

std::vector<int> counts;


TemplateAStar<xyLoc, tDirection, MapEnvironment> forward;
TemplateAStar<xyLoc, tDirection, MapEnvironment> backward;

ZeroHeuristic<xyLoc> *z = new ZeroHeuristic<xyLoc>;
WeightedHeuristic<xyLoc> *wh = 0;


MM<xyLoc, tDirection, MapEnvironment> mm;


TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;

BOBA<xyLoc, tDirection, MapEnvironment> bobacompare;


std::vector<xyLoc> path;
std::vector<xyLoc> goalPath;

bool mouseTracking = false;
bool mouseTracked = false;
void SetupMapOverlay();

int gStepsPerFrame = 1;


#ifdef COMPARE_TO_MM
bool mmSearchRunning = false;
#endif

#ifdef COMPARE_TO_ASTAR
bool astarSearchRunning = false;
#endif

bool bobacompareSearchRunning = false;
bool searchRan = false;

bool recording = false;

std::fstream svgFile;
bool saveSVG = false;

enum bibfs {
	XX = 0,
	NN = 1,
	NF = 2,
	NR = 3,
	FN = 4,
	FF = 5,
	FR = 6,
	RN = 7,
	RF = 8,
	RR = 9
};

const char *bibfs_desc[10] = {
	"", "NN", "NF", "NR", "FN", "FF", "FR", "RN", "RF", "RR"
};



void InstallHandlers()
{
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler);
	InstallKeyboardHandler(MyKeyboardHandler, "Save SVG", "Export graphics to SVG File", kNoModifier, 's');
	InstallKeyboardHandler(MyKeyboardHandler, "Record", "Start/stop recording movie", kNoModifier, 'r');
	InstallKeyboardHandler(MyKeyboardHandler, "Single Viewport", "Set to use a single viewport", kNoModifier, '1');
	InstallKeyboardHandler(MyKeyboardHandler, "Two Viewports", "Set to use two viewports", kNoModifier, '2');
	InstallKeyboardHandler(MyKeyboardHandler, "Slower", "Slow down visualization", kNoModifier, '[');
	InstallKeyboardHandler(MyKeyboardHandler, "Faster", "Speed up visualization", kNoModifier, ']');
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		mouseTracking = false;
		delete map;
		delete me;
		map = 0;
		me = 0;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		delete map;
		delete me;
		//map = new Map("/Users/nathanst/hog2/maps/dao/lak308d.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/w_woundedcoast.map");
		
		//map = new Map("/Users/nathanst/hog2/maps/random/random512-35-6.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/lt_backalley_g.map");
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		map = new Map("/home/jingwei/Desktop/Shared/hog2/maps/bg512/AR0012SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz107d.map");
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		me->SetDiagonalCost(1.414213562373);
		wh = new WeightedHeuristic<xyLoc>(me, WEIGHT);
	}
	
}

void MyKeyboardHandler(unsigned long windowID, tKeyboardModifier, char key)
{
	switch (key)
	{
		case 's':
		{
			svgFile.open("/Users/nathanst/Desktop/test.svg", std::fstream::out | std::fstream::trunc);
			saveSVG = true;
			break;
		}
		case 'r':
		{
			recording = !recording;
			break;
		}
		case '1':
		{
			SetNumPorts(windowID, 1);
			break;
		}
		case '2':
		{
			SetNumPorts(windowID, 2);
			break;
		}
		case '[':
		{
			gStepsPerFrame /= 2;
			break;
		}
		case ']':
		{
			if (gStepsPerFrame == 0)
				gStepsPerFrame = 1;
			else
				gStepsPerFrame *= 2;
			break;
		}
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (saveSVG && viewport == 0)
	{
		svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(10*map->GetMapWidth()+10)+"\" height = \""+std::to_string(10*map->GetMapHeight()+10+100)+"\">";
		recColor black = {0, 0, 0};
		recColor white = {1, 1, 1};
		//svgFile << SVGDrawRect(0, 0, map->GetMapWidth(), map->GetMapHeight()+10+1, white);
		if (mo)
		{
			svgFile << mo->SVGDraw();

			for (int x = 1; x < 10; x++)
			{
				recColor r = mo->GetValueColor(x);
				glColor3f(r.r, r.g, r.b);
				char num[16];
				sprintf(num, "%d", counts[x]);
				
				svgFile << SVGDrawRect(x*map->GetMapWidth()/11+0, map->GetMapHeight()+2, 5, 5, r);
				svgFile << SVGDrawText(x*map->GetMapWidth()/11+6, map->GetMapHeight()+4, bibfs_desc[x], black, 3);
				svgFile << SVGDrawText(x*map->GetMapWidth()/11+6, map->GetMapHeight()+7, num, black, 3);
			}

			svgFile << SVGDrawText(start.x+1, start.y+2, "start", black, 5);
			svgFile << SVGDrawText(goal.x+1, goal.y+2, "goal", black, 5);
		}
		else {
			svgFile << me->SVGDraw();
		}
		
		svgFile << "</svg>";


		saveSVG = false;
		svgFile.close();
	}

	map->OpenGLDraw();
	if (mo)
	{
		mo->OpenGLDraw();
		for (int x = 1; x < 10; x++)
		{
			recColor r = mo->GetValueColor(x);
			glColor3f(r.r, r.g, r.b);
			DrawBox(-1+0.2*x-1.0/40.0, -1-1.0/40.0, 0, 1.0/40.0);
			glColor3f(1.0, 1.0, 1.0);
			DrawText(-1+0.2*x+1.0/40.0, -1-1.0/40.0, -0.01, 1.0/10.0, bibfs_desc[x]);
			char num[16];
			sprintf(num, "%d", counts[x]);
			DrawText(-1+0.2*x+1.0/40.0, -1+1.0/40.0, -0.01, 1.0/10.0, num);
		}
	}
	if (mouseTracking)
	{
		me->SetColor(1.0, 0, 0);
		glLineWidth(3.0);
		me->GLDrawLine(start, goal);
		glLineWidth(1.0);
		me->SetColor(1.0, 1.0, 1.0);
		me->GLDrawLine(start, goal);
	}
	if (mouseTracked && 0)
	{
		me->SetColor(1.0, 1.0, 1.0);
		glLineWidth(1.0);
		me->GLLabelState(start, "start", map->GetMapHeight()/8.0);
		me->GLLabelState(goal, "goal", map->GetMapHeight()/8.0);
		glLineWidth(1.0);
	}

	for (int x = 0; x < gStepsPerFrame*2; x++)
	{
#ifdef COMPARE_TO_MM
		if (mmSearchRunning)
		{
			mmSearchRunning = !mm.DoSingleSearchStep(path);
			if (!mmSearchRunning)
				printf("MM: %llu nodes expanded\n", mm.GetNodesExpanded());
		}
#endif

#ifdef COMPARE_TO_ASTAR
		if (astarSearchRunning)
		{
			astarSearchRunning = !astar.DoSingleSearchStep(path);
			if (!astarSearchRunning)
			{
				printf("A*: %llu nodes expanded const %1.1f\n", astar.GetNodesExpanded(), me->GetPathLength(path));
			}
		}
#endif
	}
	for (int x = 0; x < gStepsPerFrame; x++)
	{
		if (bobacompareSearchRunning)
		{
			bobacompareSearchRunning = !bobacompare.ExpandAPair(path);
			if (!bobacompareSearchRunning)
			{
				printf("boba: %llu nodes expanded const %1.1f\n", bobacompare.GetNodesExpanded(), me->GetPathLength(path));
			}
		}
	}
	if (searchRan)
	{
		if (viewport == 0)
#ifdef COMPARE_TO_MM
			mm.OpenGLDraw();
#endif

#ifdef COMPARE_TO_ASTAR
		astar.OpenGLDraw();
#endif
		else if (viewport == 1)
			bobacompare.OpenGLDraw();
	}

	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		//TODO...
		sprintf(fname, "/home/jingwei/Desktop/Shared/Movie/tmp/BI-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
	if (goalPath.size() > 0)
	{
		goal = goalPath.back();
		goalPath.pop_back();
		//SetupMapOverlay();
	}
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	//	return false;
	static point3d startLoc;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kRightButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			delete mo;
			mo = 0;
			
			int x, y;
			map->GetPointFromCoordinate(loc, x, y);
			start.x = x; start.y = y;
			goal = start;
			mouseTracking = true;
			mouseTracked = true;
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			map->GetPointFromCoordinate(loc, x, y);
			goal.x = x; goal.y = y;
			mouseTracking = true;
			mouseTracked = true;
			return true;
		}
		case kMouseUp:
			if (mouseTracking)
			{
				int x, y;
				map->GetPointFromCoordinate(loc, x, y);
				goal.x = x; goal.y = y;

				////test case 188
				//start.x = 139;
				//start.y = 272;
				//goal.x = 201;
				//goal.y = 54;

				////test case 1
				//start.x = 322; 
				//start.y = 410 ;
				//goal.x = 314 ;
				//goal.y = 266;

				////test case 2
				//start.x = 175 ;
				//start.y =386;
				//goal.x =  275;
				//goal.y =  392;

				////test case 3
				//start.x = 228 ;
				//start.y =446;
				//goal.x =  198;
				//goal.y =  199;

				//test case 5
				start.x = 247  ;
				start.y = 230;
				goal.x =  380;
				goal.y =  316;

				//start.x = 27;
				//start.y = 73;
				//goal.x = 91;
				//goal.y = 120;


				//start.x = 68;
				//start.y = 29;
				//goal.x = 125;
				//goal.y = 87;

//				start.x = 394;
//				start.y = 158;
//				goal.x = 313;
//				goal.y = 167;

//				start.x = 389;
//				start.y = 278;
//				goal.x = 510;
//				goal.y = 208;
				
				//start.x = 37;
				//start.y = 61;
				//goal.x = 101;//78;
				//goal.y = 101;//132-10;
				
//				forward.GetPath(me,
//								{static_cast<uint16_t>(goal.x-17),
//									static_cast<uint16_t>(goal.y+22)}, goal, goalPath);
//				recording = true;
				
				std::cout << "Doing map search from " << start << " to " << goal << "\n";
				mouseTracking = false;
				//SetupMapOverlay();
				SetNumPorts(windowID, 2);
				//bdscompare.SetHeuristic(me);
#ifdef HEUR_MAP
				bobacompare.InitializeSearch(me, start, goal,wh,wh, path);
#ifdef COMPARE_TO_MM
				mm.InitializeSearch(me, start, goal, wh, wh, path);
#endif

#ifdef COMPARE_TO_ASTAR
				astar.SetHeuristic(wh);
				astar.InitializeSearch(me, start, goal, path);
#endif

#endif

#ifdef HEUR_ZERO
				bobacompare.InitializeSearch(me, start, goal, z, z, path);
#ifdef COMPARE_TO_MM
				mm.InitializeSearch(me, start, goal, z, z, path);
#endif
#ifdef COMPARE_TO_ASTAR
				astar.SetHeuristic(z);
				astar.InitializeSearch(me, start, goal, path);
#endif
#endif // HEUR_ZERO

#ifdef COMPARE_TO_MM
				mmSearchRunning = true;
#endif
#ifdef COMPARE_TO_ASTAR
				astarSearchRunning = true;
#endif
				bobacompareSearchRunning = true;
				searchRan = true;
				return true;
			}
	}
	return false;
}

bibfs GetLocationClassification(xyLoc l, double optimal)
{
	double startDist, goalDist;
	forward.GetClosedListGCost(l, startDist);
	backward.GetClosedListGCost(l, goalDist);

	// Only Show NEAR in each direction
//	if (startDist < optimal/2)
//		return NF;
//	if (goalDist < optimal/2)
//		return FN;
//	return RR;

	// Only show N*, F*
//	if (startDist < optimal/2)
//		return NR;
//	if (startDist < optimal)
//		return NF;
//	if (startDist == optimal/2)
//		return NN;
//	return RR;

	if (0) // Draw MM & A* heuristic regions that don't overlap
	{
		bool MM = false;
		bool ASTAR = false;
		if (startDist <= optimal && wh->HCost(l, goal)+startDist <= optimal)
			ASTAR = true;
		if ((startDist <= optimal/2 && wh->HCost(l, goal)+startDist <= optimal) ||
			(goalDist <= optimal/2 && wh->HCost(l, start)+goalDist <= optimal))
			MM = true;
		if (ASTAR && MM)
			return XX;
//			return NF;
		if (MM)
			return RN;
		if (ASTAR)
			return FN;
		else return XX;
	}
	
	if (0) // Draw MM heuristic regions
	{
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			if (wh->HCost(l, goal)+startDist > optimal || wh->HCost(l, start)+goalDist > optimal)
				return NN;
			return RN;
			//return XX;
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NN;
			return RN;
			return XX;
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NN;
			return RN;
			return XX;
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			if (wh->HCost(l, start)+goalDist > optimal)
				return NN;
			return RN;
			return XX;
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			return XX;
			return FF;
		}
		else if (startDist <= optimal)
		{
			return XX;
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			if (wh->HCost(l, start)+goalDist > optimal)
				return NN;
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return XX;
			return RF;
		}
		else {
			return XX;
			return RR;
		}
	}
	
	if (0) // Draw A* heuristic regions
	{
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NN;
			return RN;
			//return XX;
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return FN;
			return XX;
			return FF;
		}
		else if (startDist <= optimal)
		{
			if (wh->HCost(l, goal)+startDist > optimal)
				return NR;
			return FN;
			return XX;
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			return XX;
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return XX;
			return RF;
		}
		else {
			return XX;
			return RR;
		}
	}
	
	if (0) // DRAW just BFS regions
	{
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			return FN;
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			return FN;
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			return FN;
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			return FN;
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			return FN;
			return FF;
		}
		else if (startDist <= optimal)
		{
			return FN;
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			return XX;
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return XX;
			return RF;
		}
		else {
			return XX;
			return RR;
		}
	}

	if (1)
	{
		// DRAW all regions
		if (startDist <= optimal/2 && goalDist <= optimal/2)
		{
			return NN;
		}
		else if (startDist <= optimal/2 && goalDist <= optimal)
		{
			return NF;
		}
		else if (startDist <= optimal/2)
		{
			return NR;
		}
		else if (startDist <= optimal && goalDist <= optimal/2)
		{
			return FN;
		}
		else if (startDist <= optimal && goalDist <= optimal)
		{
			return FF;
		}
		else if (startDist <= optimal)
		{
			return FR;
		}
		else if (goalDist <= optimal/2)
		{
			return RN;
		}
		else if (goalDist <= optimal)
		{
			return RF;
		}
		else {
			return RR;
		}
	}
}

void SetupMapOverlay()
{
	if (start.x >= map->GetMapWidth() || start.x < 0 || start.y >= map->GetMapHeight() || start.y < 0)
	{
		std::cout << "Invalid path: " << start << " to " << goal << "\n";
		return;
	}
	std::cout << "Doing map overlay from " << start << " to " << goal << "\n";
	counts.resize(0);
	counts.resize(10);
	delete mo;
	mo = new MapOverlay(map);
	mo->SetColorMap(-1);

	mo->SetColor(XX, colors::black);
	mo->SetColor(NF, colors::orange);
	mo->SetColor(NR, colors::purple);
	mo->SetColor(NN, colors::cyan);
	mo->SetColor(FN, colors::red);
	mo->SetColor(RN, colors::yellow);
	mo->SetColor(FF, colors::red);
	mo->SetColor(FR, colors::red); // colors::orange
	mo->SetColor(RF, colors::green);
	mo->SetColor(RR, colors::gray);
	
	forward.SetStopAfterGoal(false);
	backward.SetStopAfterGoal(false);
	std::vector<xyLoc> path;
	forward.GetPath(me, start, goal, path);
	backward.GetPath(me, goal, start, path);
	
	double optimal;
	forward.GetClosedListGCost(goal, optimal);
	for (int x = 0; x < map->GetMapWidth(); x++)
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			if (map->GetTerrainType(x, y) == kGround)
			{
				xyLoc l(x, y);
				bibfs i = GetLocationClassification(l, optimal);
				counts[i]++;
				mo->SetOverlayValue(x, y, i);
			}
		}
	}
//	mo->SetD
	mo->SetTransparentValue(XX);
	mo->SetOverlayValue(start.x, start.y, 10);
	mo->SetOverlayValue(goal.x, goal.y, 10);
	for (int x = 0; x < counts.size(); x++)
	{
		switch (x)
		{
			case 1: printf("NN: %d\n", counts[x]); break;
			case 2: printf("NF: %d\n", counts[x]); break;
			case 3: printf("NR: %d\n", counts[x]); break;
			case 4: printf("FN: %d\n", counts[x]); break;
			case 5: printf("FF: %d\n", counts[x]); break;
			case 6: printf("FR: %d\n", counts[x]); break;
			case 7: printf("RN: %d\n", counts[x]); break;
			case 8: printf("RF: %d\n", counts[x]); break;
			case 9: printf("RR: %d\n", counts[x]); break;
			default: break;
		}
	}
}

void AnalyzeProblem(Map *m, Experiment e)
{
	forward.SetStopAfterGoal(false);
	backward.SetStopAfterGoal(false);
	std::vector<xyLoc> path;
	start.x = e.GetStartX();
	start.y = e.GetStartY();
	goal.x = e.GetGoalX();
	goal.y = e.GetGoalY();
	

	forward.GetPath(me, start, goal, path);
	backward.GetPath(me, goal, start, path);

	double optimal;
	forward.GetClosedListGCost(goal, optimal);

	bobacompare.GetPath(me, start, goal,wh,wh, path);
	printf("boba path length: %1.2f\t", me->GetPathLength(path));
#ifdef COMPARE_TO_MM
	mm.GetPath(me, start, goal, wh, wh, path);
	printf("MM path length: %1.2f\n", me->GetPathLength(path));
#endif

#ifdef COMPARE_TO_ASTAR
	astar.GetPath(me, start, goal, path);
	printf("A* path length: %1.2f\t", me->GetPathLength(path));
#endif

#ifdef COMPARE_TO_MM
	printf("boba: %llu\tMM: %llu\t", bobacompare.GetNodesExpanded(), mm.GetNodesExpanded());
#endif
	// full state space
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			for (int y = 0; y < m->GetMapHeight(); y++)
			{
				if (m->GetTerrainType(x, y) == kGround)
				{
					counts[GetLocationClassification(xyLoc(x, y), optimal)]++;
				}
			}
		}
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
				case 1: printf("NN: %d\t", counts[x]); break;
				case 2: printf("NF: %d\t", counts[x]); break;
				case 3: printf("NR: %d\t", counts[x]); break;
				case 4: printf("FN: %d\t", counts[x]); break;
				case 5: printf("FF: %d\t", counts[x]); break;
				case 6: printf("FR: %d\t", counts[x]); break;
				case 7: printf("RN: %d\t", counts[x]); break;
				case 8: printf("RF: %d\t", counts[x]); break;
				case 9: printf("RR: %d\t", counts[x]); break;
				default: break;
			}
		}
	}
	// BDS
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < bobacompare.GetNumForwardItems(); x++)
		{
			if (bobacompare.GetForwardItem(x).where == kClosed)
				counts[GetLocationClassification(bobacompare.GetForwardItem(x).data, optimal)]++;
		}
		for (int x = 0; x < bobacompare.GetNumBackwardItems(); x++)
		{
			if (bobacompare.GetBackwardItem(x).where == kClosed)
				counts[GetLocationClassification(bobacompare.GetBackwardItem(x).data, optimal)]++;
		}
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
				case 1: printf("NN: %d\t", counts[x]); break;
				case 2: printf("NF: %d\t", counts[x]); break;
				case 3: printf("NR: %d\t", counts[x]); break;
				case 4: printf("FN: %d\t", counts[x]); break;
				case 5: printf("FF: %d\t", counts[x]); break;
				case 6: printf("FR: %d\t", counts[x]); break;
				case 7: printf("RN: %d\t", counts[x]); break;
				case 8: printf("RF: %d\t", counts[x]); break;
				case 9: printf("RR: %d\t", counts[x]); break;
				default: break;
			}
		}
	}
#ifdef COMPARE_TO_MM
	// MM
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < mm.GetNumForwardItems(); x++)
		{
			if (mm.GetForwardItem(x).where == kClosedList)
				counts[GetLocationClassification(mm.GetForwardItem(x).data, optimal)]++;
		}
		for (int x = 0; x < mm.GetNumBackwardItems(); x++)
		{
			if (mm.GetBackwardItem(x).where == kClosedList)
				counts[GetLocationClassification(mm.GetBackwardItem(x).data, optimal)]++;
		}
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
				case 1: printf("NN: %d\t", counts[x]); break;
				case 2: printf("NF: %d\t", counts[x]); break;
				case 3: printf("NR: %d\t", counts[x]); break;
				case 4: printf("FN: %d\t", counts[x]); break;
				case 5: printf("FF: %d\t", counts[x]); break;
				case 6: printf("FR: %d\t", counts[x]); break;
				case 7: printf("RN: %d\t", counts[x]); break;
				case 8: printf("RF: %d\t", counts[x]); break;
				case 9: printf("RR: %d\t", counts[x]); break;
				default: break;
			}
		}
	}
#endif
#ifdef COMPARE_TO_ASTAR
	// A*
	{
		counts.resize(0);
		counts.resize(10);
		for (int x = 0; x < astar.GetNumItems(); x++)
		{
			if (astar.GetItem(x).where == kClosedList)
				counts[GetLocationClassification(astar.GetItem(x).data, optimal)]++;
		}
		for (int x = 0; x < counts.size(); x++)
		{
			switch (x)
			{
			case 1: printf("NN: %d\t", counts[x]); break;
			case 2: printf("NF: %d\t", counts[x]); break;
			case 3: printf("NR: %d\t", counts[x]); break;
			case 4: printf("FN: %d\t", counts[x]); break;
			case 5: printf("FF: %d\t", counts[x]); break;
			case 6: printf("FR: %d\t", counts[x]); break;
			case 7: printf("RN: %d\t", counts[x]); break;
			case 8: printf("RF: %d\t", counts[x]); break;
			case 9: printf("RR: %d\t", counts[x]); break;
			default: break;
			}
		}
	}
#endif

	printf("\n");
}

void AnalyzeMap(const char *map, const char *scenario)
{
	printf("Loading %s with scenario %s\n", map, scenario);
	ScenarioLoader s(scenario);
	Map *m = new Map(map);
	me = new MapEnvironment(m);
	me->SetDiagonalCost(1.5);
	for (int x = 0; x < s.GetNumExperiments(); x++)
	{
		AnalyzeProblem(m, s.GetNthExperiment(x));
	}
}
