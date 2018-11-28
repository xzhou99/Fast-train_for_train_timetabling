//  Portions Copyright 2013 Lingyun meng (lymeng@bjtu.edu.cn) and Xuesong Zhou (xzhou99@gmail.com)

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html


//    
//    FastTrain is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    FastTrain is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with FastTrain.  If not, see <http://www.gnu.org/licenses/>.
#pragma once

extern std::set<CNode*>		g_NodeSet;
extern std::map<int, CNode*> g_NodeIDMap;
extern std::map<int, int> g_NodeNumbertoIDMap;
extern std::map<int, int> g_NodeIDToNumberMap;

extern std::set<CLink*>		g_LinkSet;
extern std::map<int, CLink*> g_LinkIDMap;

extern	std::map<unsigned long, CLink*> g_NodeIDtoLinkMap;

extern std::vector<CTrain*> g_TrainVector;
extern std::vector<CTrain*> g_TrainVector2;
extern std::map<int, CTrain*> g_TrainIDMap;
extern std::map<int, CTrain*> g_TrainIDMap2;

extern std::set<TrainPaths*> g_TrainPathsSet;
extern std::set<TrainPath*> g_TrainPathSet;
extern std::map<unsigned long,TrainPaths*> g_NodeIDtoTrainPathsMap;
extern std::map<int,TrainPaths*> g_TrainPathsIDMap;
extern std::map<int,TrainPath*> g_TrainPathIDMap;

extern int g_OptimizationHorizon;
extern int g_OptimizationInterval;
extern int g_MaxTrainWaitingTime;
extern int g_MaxNumberOfLRIterations;
extern int g_CurrentLRIterationNumber;
extern int g_SafetyHeadway;

extern int g_PathBasedScheduling;
extern int g_TrainPathID;

extern ofstream g_LogFile;

//for reading data from csv files
int g_read_integer(FILE* f);
int g_read_integer_with_char_O(FILE* f);
float g_read_float(FILE *f);
//for reading scheduling settings
int g_GetPrivateProfileInt( LPCTSTR section, LPCTSTR key, int def_value, LPCTSTR filename);
float g_GetPrivateProfileFloat( LPCTSTR section, LPCTSTR key, float def_value, LPCTSTR filename);

unsigned long g_GetLinkKey(int FromNodeID, int ToNodeID);
CLink* g_FindLinkWithNodeNumbers(int FromNodeNumber, int ToNodeNumber);
CLink* g_FindLinkWithNodeIDs(int FromNodeID, int ToNodeID);

unsigned long g_GetTrainPathsKey(int FromNodeID, int ToNodeID);
TrainPaths* g_FindTrainPathsWithNodeNumbers(int FromNodeNumber, int ToNodeNumber);
TrainPaths* g_FindTrainPathsWithNodeIDs(int FromNodeID, int ToNodeID);
bool g_GenerateTrainPhysicalPaths();
bool g_GeneratePhysicalPaths(int fromnodenumber,int tonodenumber,TrainPaths*pTPs, int direction);
bool g_GenerateTrainForbiddenLinkIDs();
bool g_FilterPossiblePaths(CTrain* pT);

bool g_ReadNodeCSVFile();
bool g_ReadLinkCSVFile();
bool g_ReadMOWCSVFile();
bool g_ReadTrainInfoCSVFile();

bool g_Timetable_Optimization_Lagrangian_Method();
bool g_Timetable_Optimization_Priority_Rule_Method(int *pTrainsToBeScheduled);
bool g_DeduceToProblemFeasibleSolution(int deducingmethod);

bool g_ExportTimetableDataToCSVFile(int iterationnumber,std::vector<CTrain*> trainvector);
bool g_ExportTimetableDataToXMLFile(int iterationnumber,std::vector<CTrain*> trainvector);
bool g_ExportTDLabelCostToTXTFile(NetworkForSP *pN,int departure_time,int* ids);
bool g_ExportResourcePriceToTXTFile();
bool g_ExportResourceUsageCount();
bool g_ExportResourceUsageToTXTFile(CLink*pl,int t);

bool g_UpdateResourceUsageStatus(int LR_Iteration,std::vector<CTrain*> TrainVector,bool updatelastiteratenumber);
bool g_UpdateUpperBoundByTrainVector(std::vector<CTrain*> TrainVector);



