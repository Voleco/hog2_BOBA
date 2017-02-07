
#include <cassert>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unordered_map>
#include <set>
#include <unordered_set>

#include "GUICode.h"
#include "Timer.h"

#include <iostream>
#include <vector>
#include <time.h>
#include <cstring>

#include "IDAStar.h"
#include "ParallelIDAStar.h"

#include "TOH.h"
#include "PancakePuzzle.h"
#include "ScenarioLoader.h"
#include "Map2DEnvironment.h"
#include "MapOverlay.h"
#include "TemplateAStar.h"
#include "MM.h"
#include "BOBA.h"
//#include "WeightedHeuristic.h"

#define SQUARE_ROOT_OF2 1.414213562373

using std::cout;


namespace GRIDMAPTEST {
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

	bool LoadBenchmark(std::vector<int>& group, std::vector<int>& startx, std::vector<int>& starty, std::vector<int>& goalx, std::vector<int>& goaly,
		std::vector<double>& expectedCost, std::string fileName);

	void run(std::string fileName, double weight, int teststart, int testend);

}


namespace TOHTEST {
	template <int N>
	void TestTOH(Heuristic<TOHState<N>> *f, int mode, int first, int last);
	void TOHTest(int mode, int first = 0, int last = 50);

	const int M = 3;
	const int numDisks = 14; // [disks - 2] (4^14 - 256 million)
}

namespace PANCAKETEST {
	const int LENGTH = 20;
	enum instanceType
	{
		s5,
		l9
	};
	template<int N>
	void GetInstance(instanceType type, PancakePuzzleState<N> &s);
	
	template<int N>
	void pancakeTest(instanceType type, int alg);
}


int main(int argc, char** argv)
{
	if (argc > 2 && strcmp(argv[1], "-gridMapTest") == 0)
	{
		using namespace GRIDMAPTEST;
		std::string fileName = argv[2];
		double weight = 1.0;
		if (argc > 3)
			weight = std::atof(argv[3]);

		int start = 1;
		int end = 1000000;
		if (argc > 4)
			start = std::atoi(argv[4]);
		if (argc > 5)
			end = std::atoi(argv[5]);
		if (weight<2)
			run(fileName, weight, start, end);
		else
		{
			for (double w = 0.0; w <= 1; w = w + 0.1)
			{
				run(fileName, w, start, end);
			}
		}
	}

	else if (argc > 1 && strcmp(argv[1], "-gridMapGUI") == 0){
		InstallHandlers();
		RunHOGGUI(argc, argv);
	}

	else if (argc > 1 && strcmp(argv[1], "-tohTest") == 0  )
	{

		using namespace TOHTEST;
		int mode = 1;
		int first = 0;
		int last = 50;
		
		if (argc > 2)
			mode = std::atoi(argv[2]);
		
		if (argc > 3)
			first = std::atoi(argv[3]);
		if (argc > 4)
			last = std::atoi(argv[4]);
		TOHTest(mode,first,last);
	}

	else if (argc > 2 && strcmp(argv[1], "-pancakeTest") == 0)
	{

		using namespace PANCAKETEST;
		
		instanceType type;
		if (strcmp(argv[2], "s5") == 0)
			type = s5;
		else if (strcmp(argv[2], "l9") == 0)
			type = l9;

		int alg = 1;
		if (argc > 3)
			alg = std::atoi(argv[3]);
		pancakeTest<LENGTH>(type,alg);
	}

	else
	{
		std::cout << "Usage: \n" 
			<<"1: "<< argv[0] << " -gridMapTest <filename> [weight] [teststart] [testend]\n"
			<<"2: "<< argv[0] << " -gridMapGUI\n"
			<<"3: " << argv[0] << " -tohTest [mode] [first] [last]\n"
			<< "4: " << argv[0] << " -pancakeTest <instanceType> [alg]\n";
	}


	return 0;
}





bool GRIDMAPTEST::LoadBenchmark(std::vector<int>& group, std::vector<int>& startx, std::vector<int>& starty, std::vector<int>& goalx, std::vector<int>& goaly,
	std::vector<double>& expectedCost, std::string fileName)
{
	std::ifstream fin;
	fin.open(fileName);
	if (!fin.is_open())
	{
		std::cout << "fail to load benchmark file: " << fileName << "\n";
		return false;
	}

	startx.resize(0);
	starty.resize(0);
	goalx.resize(0);
	goaly.resize(0);
	expectedCost.resize(0);
	std::string str;
	int grp;
	//get rid of "version 1"
	fin >> str;
	fin >> str;
	while (!fin.eof())
	{
		//first 4 should be "group", "mapname", "map width", "map height"
		//which are not helpful for this assignment
		fin >> str;
		grp = std::stoi(str);
		group.push_back(grp);

		fin >> str;
		fin >> str;
		fin >> str;

		if (fin.eof())
			break;

		fin >> str;
		startx.push_back(std::stoi(str));
		fin >> str;
		starty.push_back(std::stoi(str));
		fin >> str;
		goalx.push_back(std::stoi(str));
		fin >> str;
		goaly.push_back(std::stoi(str));
		fin >> str;
		expectedCost.push_back(std::stod(str));
	}
	return true;
}

void GRIDMAPTEST::run(std::string fileName, double weight, int teststart, int testend)
{
	std::cout << "file_name: " << fileName << "\n";
	std::cout << "weight: " << weight << "\n";

	std::string mapName = "/home/jingwei/Desktop/Shared/hog2/maps/";
	std::string scenName = "/home/jingwei/Desktop/Shared/hog2/scenarios/";

	mapName = mapName + fileName + ".map";
	scenName = scenName + fileName + ".map.scen";
	map = new Map(mapName.c_str());

	me = new MapEnvironment(map);
	me->SetDiagonalCost(SQUARE_ROOT_OF2);


	wh = new WeightedHeuristic<xyLoc>(me, weight);


	//load the benchmark
	std::vector<int> group, startx, starty, goalx, goaly;
	std::vector<double> expectedCost;
	if (!LoadBenchmark(group, startx, starty, goalx, goaly, expectedCost, scenName.c_str()))
		return;


	clock_t startTime;
	clock_t endTime;
	clock_t clockTicksTaken;
	double timeInSeconds;

	double astarTime;
	double mmTime;
	double bobaTime;

	double solutionCost;

	for (int i = teststart - 1; i < std::min(testend, (int)(startx.size())); i++)
	{
		start.x = startx[i];
		start.y = starty[i];
		goal.x = goalx[i];
		goal.y = goaly[i];

		std::cout << "********************************\n"
			<< "test_case " << i + 1 << "\n"
			<< "group_number " << group[i] << "\n"
			<< "start: " << start
			<< " goal: " << goal << "\n";

		std::vector<xyLoc> correctPath;
		startTime = clock();
		astar.SetHeuristic(wh);
		astar.InitializeSearch(me, start, goal, correctPath);
		astar.GetPath(me, start, goal, correctPath);
		endTime = clock();
		clockTicksTaken = endTime - startTime;
		astarTime = clockTicksTaken / (double)CLOCKS_PER_SEC;


		startTime = clock();
		mm.InitializeSearch(me, start, goal, wh, wh, path);
		mm.GetPath(me, start, goal, wh, wh, path);
		endTime = clock();
		clockTicksTaken = endTime - startTime;
		mmTime = clockTicksTaken / (double)CLOCKS_PER_SEC;


		startTime = clock();
		bobacompare.InitializeSearch(me, start, goal, wh, wh, path);
		bobacompare.GetPath(me, start, goal, wh, wh, path);
		endTime = clock();
		clockTicksTaken = endTime - startTime;
		bobaTime = clockTicksTaken / (double)CLOCKS_PER_SEC;

		if (!fequal(bobacompare.GetSolutionCost(),expectedCost[i]))
		{
			std::cout << "error solution cost:\t expected cost\n";
			std::cout << bobacompare.GetSolutionCost() << "\t" << expectedCost[i] << "\n";
			double d;
			for (auto x : correctPath)
			{
				astar.GetClosedListGCost(x, d);
				auto t = bobacompare.GetNodeForwardLocation(x);
				auto u = bobacompare.GetNodeBackwardLocation(x);
				std::cout << x << " is on " << t << " and " << u << "\n";
				std::cout << "True g: " << d;
				if (t != kUnseen)
					std::cout << " forward g: " << bobacompare.GetNodeForwardG(x);
				if (u != kUnseen)
					std::cout << " backward g: " << bobacompare.GetNodeBackwardG(x);
				std::cout << "\n";
			}

			return;
		}

		cout << "nodes:(A*,MM,BOBA,BOBAties) \t" << astar.GetNodesExpanded() << "\t"
			<< mm.GetNodesExpanded() << "\t" << bobacompare.GetNodesExpanded() << "\t" << bobacompare.GetNecessaryExpansions()<< "\n";
		cout << "time:(A*,MM,BOBA) \t" << astarTime << "\t"
			<< mmTime << "\t" << bobaTime << "\t" << "\n";
	}
}

template <int N>
void TOHTEST::TestTOH(Heuristic<TOHState<N>> *f, int mode, int first, int last)
{
	//const int M = 2;

	TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
	BOBA<TOHState<N>, TOHMove, TOH<N>> boba;
	MM<TOHState<N>, TOHMove, TOH<N>> mm;

	TOH<N> ts;
	TOHState<N> s;
	TOHState<N> g;
	std::vector<TOHState<N>> thePath;
	std::vector<TOHMove> actionPath;
	ts.StoreGoal(g);
	ZeroHeuristic<TOHState<N>> z;

	int table[] = { 52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885 };
	int table2[] = { 145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743 };
	for (int count = first; count < last; count++)
	{
		printf("count: %d\n", count);
		printf("Seed: %d\n", table[count & 0xF] ^ table2[(count >> 4) & 0xF]);
		srandom(table[count & 0xF] ^ table2[(count >> 4) & 0xF]);
		for (int x = 0; x < 20000; x++)
		{
			ts.GetActions(s, actionPath);
			ts.ApplyAction(s, actionPath[random() % actionPath.size()]);
		}
		Timer timer;

		if (mode == 0)
		{
			printf("-=-=-A*-=-=-\n");
			astar.SetUseBPMX(1);
			astar.SetHeuristic(f);
			timer.StartTimer();
			astar.GetPath(&ts, s, g, thePath);
			timer.EndTimer();
			printf("%llu nodes expanded\n", astar.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}

		else if (mode == 1)
		{
			//we need to build the backward hueristic for BOBA
			TOH<N - M> absToh2;
			TOHState<N - M> absTohState2;
			TOHPDB<N - M, N> pdb2(&absToh2);

			pdb2.SetGoal(s);
			pdb2.BuildPDB(s, std::thread::hardware_concurrency());

			Heuristic<TOHState<N>> back;

			back.lookups.resize(0);
			back.lookups.push_back({ kAddNode, 1, 2 });
			back.lookups.push_back({ kLeafNode, 0, 0 });

			back.heuristics.resize(0);
			back.heuristics.push_back(&pdb2);

			printf("-=-=-BOBA-=-=-\n");
			timer.StartTimer();

			boba.InitializeSearch(&ts, s, g, f, &back, thePath);
			boba.GetPath(&ts, s, g, f, &back, thePath);
		
			timer.EndTimer();
			printf("%llu nodes expanded\n", boba.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());



		}
		else if (mode == 2)
		{	
			ZeroHeuristic<TOHState<N>> zero;
			printf("-=-=-BOBA no heur-=-=-\n");
			timer.StartTimer();
			boba.InitializeSearch(&ts, s, g, &zero, &zero, thePath);
			boba.GetPath(&ts, s, g, &zero, &zero, thePath);

			timer.EndTimer();
			printf("%llu nodes expanded\n", boba.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}

		else if (mode == 3)
		{
			TOH<N - M> absToh2;
			TOHState<N - M> absTohState2;
			TOHPDB<N - M, N> pdb2(&absToh2);

			pdb2.SetGoal(s);
			pdb2.BuildPDB(s, std::thread::hardware_concurrency());

			Heuristic<TOHState<N>> back;

			back.lookups.resize(0);
			back.lookups.push_back({ kAddNode, 1, 2 });
			back.lookups.push_back({ kLeafNode, 0, 0 });

			back.heuristics.resize(0);
			back.heuristics.push_back(&pdb2);

			printf("-=-=-MM-=-=-\n");
			timer.StartTimer();

			mm.InitializeSearch(&ts, s, g, f, &back, thePath);
			mm.GetPath(&ts, s, g, f, &back, thePath);

			timer.EndTimer();
			printf("%llu nodes expanded\n", mm.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());

		}

		else if (mode == 4)
		{
			ZeroHeuristic<TOHState<N>> zero;
			printf("-=-=-MM0-=-=-\n");
			timer.StartTimer();
			mm.InitializeSearch(&ts, s, g, &zero, &zero, thePath);
			mm.GetPath(&ts, s, g, &zero, &zero, thePath);

			timer.EndTimer();
			printf("%llu nodes expanded\n", mm.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}
	}
}



void TOHTEST::TOHTest(int mode, int first, int last)
{
	printf("--== TOH Test ==--\n");
	
	std::cout << "number of disks: " << numDisks << " PDB size: " << numDisks - M << "\n";
	TOH<numDisks> toh;
	TOHState<numDisks> s, g;

	TOHState<numDisks> goal;
	TOH<numDisks - M> absToh1;

	TOHState<numDisks - M> absTohState1;
	TOHPDB<numDisks - M, numDisks> pdb1(&absToh1);
	TOHPDB<numDisks - M, numDisks> pdb1a(&absToh1);


	goal.Reset();
	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
	
	Heuristic<TOHState<numDisks>> h;

	h.lookups.resize(0);
	h.lookups.push_back({ kAddNode, 1, 2 });
	h.lookups.push_back({ kLeafNode, 0, 0 });

	h.heuristics.resize(0);
	h.heuristics.push_back(&pdb1);

	TestTOH<numDisks>(&h, mode, first, last);

	//printf("Dynamic distribution\n");
	//for (int x = 0; x < 255; x++)
	//	if (h.histogram[x] != 0)
	//		printf("%d\t%llu\n", x, h.histogram[x]);
}

template <int N>
void PANCAKETEST::GetInstance(instanceType type, PancakePuzzleState<N> &s)
{
	int s5_seq[5] = { 1, 3, 5, 2, 4 };
	int l9_seq[9] = { 1, 5, 8, 3, 6, 9, 4, 7, 2 };

	int baseLength = 0;

	int* seq;
	if (type == s5)
	{
		baseLength = 5;
		seq = s5_seq;
	}
	else if (type == l9)
	{
		baseLength = 9;
		seq = l9_seq;
	}

	int repeatedTimes = N / baseLength;

	for (int i = 0; i < repeatedTimes; i++)
	{
		for (int j = 0; j < baseLength;j++)
			//we are using 0-indexing here, so I put -1 in the end
			s.puzzle[i*baseLength + j] = i*baseLength + seq[j] -1;
	}
}

template <int N>
void PANCAKETEST::pancakeTest(instanceType type, int alg)
{
	PancakePuzzle<N> pck;
	PancakePuzzleState<N> start;
	PancakePuzzleState<N> goal;
	goal.Reset();
	GetInstance(type, start);

	TemplateAStar<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> astar;
	BOBA<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> boba;

	std::vector<PancakePuzzleState<N>> thePath;

	std::cout << "Pancake puzzle type: ";
	switch (type)
	{
	case s5:
		std::cout << "s5";
		break;
	case l9:
		std::cout << "l9";
		break;
	default:
		break;
	}
	std::cout << " puzzle size: " << N <<"\n";

	std::cout << " start: " << start << "\n";
	std::cout << " goal: " << goal << "\n";

	Timer timer;
	if (alg == 0)
	{
		printf("-=-=-A*-=-=-\n");
		astar.SetHeuristic(&pck);
		timer.StartTimer();
		astar.GetPath(&pck, start, goal, thePath);
		timer.EndTimer();
		printf("%llu nodes expanded\n", astar.GetNodesExpanded());
		printf("Solution path length %1.0f\n", pck.GetPathLength(thePath));
		printf("%1.2f elapsed\n", timer.GetElapsedTime());
	}
	else if (alg == 1)
	{
		printf("-=-=-BOBA-=-=-\n");
		timer.StartTimer();

		boba.InitializeSearch(&pck, start, goal, &pck, &pck, thePath);
		boba.GetPath(&pck, start, goal, &pck, &pck, thePath);

		timer.EndTimer();
		printf("%llu nodes expanded\n", boba.GetNodesExpanded());
		printf("Solution path length %1.0f\n", pck.GetPathLength(thePath));
		printf("%1.2f elapsed\n", timer.GetElapsedTime());
	}


}