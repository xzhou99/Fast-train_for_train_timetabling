//  Portions Copyright 2013 Lingyun meng (lymeng@bjtu.edu.cn) and Xuesong Zhou (xzhou99@gmail.com)

//   If you help write or modify the code, please also list your names here.
//   The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html


//    This file is part of FastTrain  (Open-source).

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

#include <math.h>
#include <deque>
#include <map>
#include <set>
#include <iostream>
#include <vector>
#include <list>

using namespace std;

#define	MAX_SPLABEL 999999
#define MAX_NODE_SIZE_IN_A_PATH 20000
#define MAX_TRAIN_NUMBER 45
#define MAX_PHYSICAL_NODE_NUMBER 2000
#define MAX_PATH_NUMBER_PER_TRAIN 1000

class CTrain;
class CNode;
class CLink;

//schedule settings
extern int g_MaxTrainWaitingTime;
extern int g_SafetyHeadway;
extern int g_PathBasedScheduling;
extern int g_PathsPerTrain;
extern int g_OptimizationHorizon;
extern int g_OptimizationInterval;
extern int g_MinuteDivision;

extern int g_TrainPathID;

extern std::map<int, int> g_NodeIDToNumberMap;
extern std::map<int,int>g_NodeMPIDToNodeIDMap;
extern std::vector<CTrain*> g_TrainVector;
extern std::set<CNode*>	g_NodeSet;
extern std::set<CLink*>	g_LinkSet;
extern std::map<int, CLink*> g_LinkIDMap;

extern CLink* g_FindLinkWithNodeIDs(int FromNodeID, int ToNodeID);
extern void g_ProgramStop();

template <typename T>
T **AllocateDynamicArray(int nRows, int nCols)
{
	T **dynamicArray;

	dynamicArray = new T*[nRows];

	for( int i = 0 ; i < nRows ; i++ )
	{
		dynamicArray[i] = new T [nCols];

		if (dynamicArray[i] == NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

	}

	return dynamicArray;
}

template <typename T>
void DeallocateDynamicArray(T** dArray,int nRows, int nCols)
{
	for(int x = 0; x < nRows; x++)
	{
		delete[] dArray[x];
	}

	delete [] dArray;

}





struct SearchTreeElement
{
	int CurrentNode;//node id in the g_nodeset
	int PredecessorNode;//node index in the searchtreelist
	int SearchLevel;
	float TravelTime;
};

class CNode//physical network node. it does not mean a station. it means the boundary for block sections
{
public:
	int m_NodeNumber;//node number in physical world
	int m_NodeID; //id in program, starting from zero, continuous sequence
	int m_MPNodeID;//id in program, for multiple path use, the node becomes a virtual one
	CNode()
	{
		m_NodeNumber = 0;
		m_NodeID = 0;
		m_MPNodeID = -1;
		
	};
	~CNode()
	{

	};
	
};

class SResource//for a given link(resource) and time 
{  
public:
	short UsageCount;//already used count
	float Price;//current price for next use
	float tempprice;//for updating and restoring forbidden link usage
	short LastUseIterationNo;//number of last iteration in which the resource was used:relax-and-cut technique
	short *TrainUseFlag;//used to identify whether train occupies this resource or not; 0-not use,1-use
	SResource()
	{
		UsageCount = 0;
		Price = 0;
        tempprice = 0;
		LastUseIterationNo = -100; 
		TrainUseFlag = new short[MAX_TRAIN_NUMBER];
		for (int i=0;i<MAX_TRAIN_NUMBER;i++)
		{
			TrainUseFlag[i]=0;
		}
	}
	~SResource()
	{
		if (TrainUseFlag!=NULL)
		{
			delete []TrainUseFlag;
			TrainUseFlag = NULL;
		}
	}

};

class CapacityReduction//capacity reduction for physical links
{
public:	
	int from_node_id;
	int to_node_id;
	int start_time_in_minutes;
	int end_time_in_minutes;	
	int speed_limit_inMilePerHour;//0 means the link is completely blocked
	float linkcapacity;
	float occurenceprob;//probability of this capacity reduction scenario

	CapacityReduction()
	{
		from_node_id = -1;
		to_node_id = -1;
		start_time_in_minutes = 0;
		end_time_in_minutes = 0;
		speed_limit_inMilePerHour = 0 ;//default- blocked
		linkcapacity = 0;
		occurenceprob = 0;
	}

	~CapacityReduction()
	{

	};

};


class CLink//physical network link, i.e., track section / cell
{
public:	

	int m_LinkID;	//id in program,starting from 0 
	int m_FromNodeID;  // id in program,starting from 0 A_Node_ID
	int m_ToNodeID;    // id in program,starting from 0 B_NodeID
	int m_FromNodeNumber;//name in the physical world
	int m_ToNodeNumber;//name in the physical world
	bool m_BidirectionalFlag;
	float m_Length;  // in miles	
	float m_FTSpeedLimit;//maximum speed limit from from_node_id to to_node_id
	float m_TFSpeedLimit;//maximum speed limit from to_node_id to from_node_id
		                          
	int m_LinkType;  // 0 main track  1 switch 4 siding track
    
	short *m_LinkCapacity;//track capacity 
	std::list<CLink*> m_LinkGroupList;
	SResource *m_ResourceAry;
	std::vector<CapacityReduction> CapacityReductionVector;

	int m_MPLinkID[MAX_PATH_NUMBER_PER_TRAIN];//used for multiple physical paths based scheduling. index in the temporary network combined a number of train paths
	int m_FromMPNodeID[MAX_PATH_NUMBER_PER_TRAIN];
	int m_ToMPNodeID[MAX_PATH_NUMBER_PER_TRAIN];

	CLink(int TimeHorizon)  // TimeHorizon's unit: per min
	{
		m_LinkID=0;
		m_FromNodeID=0;
		m_ToNodeID=0;
		m_FromNodeNumber=0;
		m_ToNodeNumber=0;
		m_BidirectionalFlag = true;
		m_Length=0;
		m_FTSpeedLimit=0;
		m_TFSpeedLimit=0;
		m_LinkType = 0;
		m_LinkCapacity=NULL;
		m_ResourceAry = NULL;					
		
		
		for (int i=0;i<MAX_PATH_NUMBER_PER_TRAIN;i++)
		{
			m_MPLinkID[i] = -1;
			m_FromMPNodeID[i] = -1;
			m_ToMPNodeID[i]=-1;
		}

		m_LinkCapacity = new short[TimeHorizon];
		ResetResourceAry(TimeHorizon);		
	}
	int GetTrainRunningTimeInSeconds(float speedmultiplier, int direction)//in minutes
	{
		ASSERT(speedmultiplier>0&&(direction==1||direction==2));	
		double a=0;
		if (direction==1)
		{
			 a= m_Length*60*60/*change to seconds*//(m_FTSpeedLimit*speedmultiplier);
		}
		if (direction==2)
		{
			 a= m_Length*60*60/*change to seconds*//(m_TFSpeedLimit*speedmultiplier);
		}	
		
		return a;
	}


	int GetTrainRunningTimeInMinuteDivision(float speedmultiplier, int direction)//in time unit according to time division
	{
	
		ASSERT(speedmultiplier>0&&(direction==1||direction==2));	
		double a=0;
		if (direction==1)
		{
			a= m_Length*60/*change to minutes*/*g_MinuteDivision/*change to time unit according to minute division*//(m_FTSpeedLimit*speedmultiplier);
		}
		if (direction==2)
		{
			a= m_Length*60/*change to minutes*/*g_MinuteDivision/*change to time unit*//(m_TFSpeedLimit*speedmultiplier);
		}

		int q=max(1,(int)(a+1.0f));//round up to one more minute division

		return q;
	}

	int GetTrainMaxWaitingTimeInMinuteDivision()
	{
		if (m_LinkType==4)//siding track
		{
			return g_MaxTrainWaitingTime;
		} 
		else
		{
			return 0;
		}
	}
	
	void ResetResourceAry(int OptimizationHorizon)
	{
		if(m_ResourceAry!=NULL)
		{
			delete []m_ResourceAry;
			m_ResourceAry=NULL;
		}

		m_ResourceAry = new SResource[OptimizationHorizon];//note that the price for using the resource is set to be 0 by the 
		                                                   //construction function
	}


	~CLink()
	{
		if(m_ResourceAry!=NULL) 
		{
			delete []m_ResourceAry;
			m_ResourceAry=NULL;
		}
		if (m_LinkCapacity!=NULL)
		{
			delete m_LinkCapacity;
            m_LinkCapacity = NULL;
		}
	};

};




class TrainPath//for path-based formulation
{
public:
	
	int m_PathID;
	int m_OriginNodeNumber;  
	int m_DestinationNodeNumber; 
	int m_OriginNodeID;  
	int m_DestinationNodeID;
	float m_totallength;//in mile
	float m_totalminimumtraveltime;//by maximum speed (speed multiplier = 1) at each link, that is, minimum total travel time
	std::list<CLink*> m_LinkList;//sequenced link list
	TrainPath()
	{
		m_totallength = 0;
		m_totalminimumtraveltime = 0;
		m_PathID=g_TrainPathID;
		g_TrainPathID++;
	};
	
	
};

class TrainPaths
{
public:
	TrainPaths()
	{
		
	};
	int m_PathsID;
	int m_OriginNodeNumber; 
	int m_DestinationNodeNumber;
	int m_OriginNodeID;
	int m_DestinationNodeID;
	std::list <TrainPath*>m_TrainPathList;
};

class STrainNode
{  

public:
int  NodeID; //id of the physical node, rather than the STrainNode
int  LinkID; //id of the physical link, starting from the second element, [i-1,i]

int TaskProcessingTime;  // the task is associated with the previous arc, // first node: origin node release time, other nodes: station entry or exit time, 1, 3, 5,: entry station, 2, 4, 6: exit station
int TaskScheduleWaitingTime;  // to be scheduled by the program
int NodeDepartureTimestamp;  // to be calculated 
int NodeArrivalTimestamp;  // to be calculated 

};




class CTrain  // for train timetabling
{
public:

	int m_TrainID;  //range:
	int m_OriginNodeID;  //in program
	int m_DestinationNodeID;//in program
	int m_OriginNodeNumber;  //in physical world
	int m_DestinationNodeNumber; //in physical world
	int m_OriginMPNodeID;// for multiple paths use
	int m_DestinationMPNodeID;
	int m_Direction;//1-east bound 2-west bound
	int m_EntryTime;// in minutes
	int m_EarlyDepartureTime; // earliest departure time
	int m_AllowableSlackAtDeparture;// maximum
	float m_TOB;//ton of braking
	float m_Length;//0
	bool m_HazMat;//whether loading with hazardous materials
	double m_CostPerUnitTimeStopped;
	double m_CostPerUnitTimeRunning;
	float m_SpeedMultiplier;
    int MaxNumberofTrainPaths;//maximum number of acceptable paths

	int *TraversedLinkIDs;//links traversed by this train
	int *ForbiddenLinkIDs;//links that can not be used by this train
	
	int m_WaitingTime;//m_acutaltraveltime-freerunningtime
	int m_FreeRunningTime;//with m_waitime, for upper bound calculation
	int m_ActualTravelTimeFreeStatus;//normal travel time
	int m_ActualTravelTime; //determined by the schedule , in minutes (arrivaltime at destination-entrytime)
	float m_TripPrice;//
	double m_ResourceCostUsedByThisTrain;
	int m_NodeSize; //initial value could be 0, given from extern input or calculated from shortest path algorithm
	STrainNode *m_aryTN; //node list array of a train path
	float m_DeviationRatio;//actual trip time compared to normal trip time
	std::list <TrainPath*>m_TrainPathList;
	int m_PlannedCompletionTime;
	int m_RealizedCompletionTime;
	
	CTrain()
	{
		m_NodeSize = 0;
		m_AllowableSlackAtDeparture = 5;//in minutes
		m_aryTN = NULL;
		TraversedLinkIDs=NULL;
		ForbiddenLinkIDs = NULL;
		MaxNumberofTrainPaths = g_PathsPerTrain;		
		m_WaitingTime = 0;
		m_FreeRunningTime=0;
		m_ResourceCostUsedByThisTrain=0;
		m_OriginMPNodeID = -1;
		m_DestinationMPNodeID=-1;
		m_PlannedCompletionTime = 0;
		m_RealizedCompletionTime = 0;

	}
	~CTrain()
	{
		if(m_aryTN != NULL)
		{
			delete []m_aryTN;
			m_aryTN=NULL;
		}
		if(TraversedLinkIDs!=NULL)
		{
			delete []TraversedLinkIDs;
			TraversedLinkIDs = NULL;
		}
		if(ForbiddenLinkIDs!=NULL)
		{
			delete []ForbiddenLinkIDs;
			ForbiddenLinkIDs = NULL;
		}
		
	}

	
	int GetFreeRunningTimeAtTraversedLinksInMinuteDivision()//in minute division
	{
		ASSERT(TraversedLinkIDs!=NULL);
		int tttime=0;
		CLink* pl = NULL;
		int size = g_LinkSet.size();
		for (int i=0;i<size;i++)
		{
			if (TraversedLinkIDs[i]==-1)
			{
				break;
			}
			pl = g_LinkIDMap[TraversedLinkIDs[i]];
			ASSERT(pl!=NULL);
			double ltt = pl->GetTrainRunningTimeInMinuteDivision(m_SpeedMultiplier,m_Direction);
			tttime+=ltt;
		}
		return tttime;
	}
};


class NetworkForSP  //mainly for shortest path calculation	
{
public:
	int m_OptimizationIntervalSize;
	int m_NodeSize;
	int m_PhysicalNodeSize;
	int m_OptimizationHorizon;
	int m_OptimizationTimeInveral;
	int m_ListFront;
	int m_ListTail;
	int m_LinkSize;

	int* m_LinkList;  // dimension number of nodes

	int** m_OutboundNodeAry; //Outbound node array 
	int** m_OutboundLinkAry; //Outbound link array
	int* m_OutboundSizeAry;  //Number of outbound links

	int** m_InboundLinkAry; //inbound link array
	int* m_InboundSizeAry;  //Number of inbound links

	int* m_FromIDAry;
	int* m_ToIDAry;

	float** m_LinkTDTimeAry;
	float** m_LinkTDCostAry;
	float* m_LinkTDMaxWaitingTimeAry;

	int* NodeStatusAry;  // Node status array used in KSP;

	float* LabelTimeAry; // label - time
	int* NodePredAry;
	float* LabelCostAry;

	CTrain*pT;//for debug use

	int m_AdjLinkSize;

	//time-dependent cost label and predecessor arrays
	float** TD_LabelCostAry;
	int** TD_NodePredAry;  // pointer to previous NODE INDEX from the current label at current node and time
	int** TD_TimePredAry;  // pointer to previous TIME INDEX from the current label at current node and time

	NetworkForSP(int NodeSize, int LinkSize, int TimeHorizon, int TimeInterval)
	{
		m_AdjLinkSize = 15;//maximum size of adjacent links
		m_NodeSize = NodeSize;
		m_LinkSize = LinkSize;

		m_OptimizationHorizon = TimeHorizon;
		m_OptimizationTimeInveral = TimeInterval;



		m_OutboundSizeAry = new int[m_NodeSize];
		m_InboundSizeAry = new int[m_NodeSize];


		m_OutboundNodeAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_OutboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_InboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);


		m_LinkList = new int[m_NodeSize];

		m_OptimizationIntervalSize = int(m_OptimizationHorizon/m_OptimizationTimeInveral+0.1);  // make sure there is no rounding error
		m_LinkTDTimeAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);
		m_LinkTDMaxWaitingTimeAry    =  new float[m_LinkSize];

		m_LinkTDCostAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);

		for (int i=0;i<m_LinkSize;i++)
		{
			for (int j=0;j<m_OptimizationIntervalSize;j++)
			{
				m_LinkTDCostAry[i][j] = 0;
			}
		}
	
		m_FromIDAry = new int[m_LinkSize];

		m_ToIDAry = new int[m_LinkSize];

		NodeStatusAry = new int[m_NodeSize];                    // Node status array used in KSP;
		NodePredAry = new int[m_NodeSize];
		LabelTimeAry = new float[m_NodeSize];                     // label - time
		LabelCostAry = new float[m_NodeSize];                     // label - cost

		TD_LabelCostAry = AllocateDynamicArray<float>(m_NodeSize,m_OptimizationIntervalSize);
		TD_NodePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);
		TD_TimePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);


		if(m_OutboundSizeAry==NULL || m_LinkList==NULL || m_FromIDAry==NULL || m_ToIDAry==NULL  ||
			NodeStatusAry ==NULL || NodePredAry==NULL || LabelTimeAry==NULL || LabelCostAry==NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

		pT = NULL;

	};

	NetworkForSP();
	void Init(int NodeSize, int LinkSize, int TimeHorizon,int AdjLinkSize)
	{
		m_NodeSize = NodeSize;
		m_LinkSize = LinkSize;

		m_OptimizationHorizon = TimeHorizon;
		m_AdjLinkSize = AdjLinkSize;


		m_OutboundSizeAry = new int[m_NodeSize];
		m_InboundSizeAry = new int[m_NodeSize];

		m_OutboundNodeAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_OutboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);
		m_InboundLinkAry = AllocateDynamicArray<int>(m_NodeSize,m_AdjLinkSize);


		m_LinkList = new int[m_NodeSize];

		m_OptimizationIntervalSize = int(TimeHorizon/m_OptimizationTimeInveral)+1;  // make sure it is not zero
		m_LinkTDTimeAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);
		m_LinkTDCostAry   =  AllocateDynamicArray<float>(m_LinkSize,m_OptimizationIntervalSize);

		for (int i=0;i<m_LinkSize;i++)
		{
			for (int j=0;j<m_OptimizationIntervalSize;j++)
			{
				m_LinkTDCostAry[i][j] = 0;
			}
		}

		m_FromIDAry = new int[m_LinkSize];

		m_ToIDAry = new int[m_LinkSize];

		NodeStatusAry = new int[m_NodeSize];                    // Node status array used in KSP;
		NodePredAry = new int[m_NodeSize];
		LabelTimeAry = new float[m_NodeSize];                     // label - time
		LabelCostAry = new float[m_NodeSize];                     // label - cost


		TD_LabelCostAry =  AllocateDynamicArray<float>(m_NodeSize,m_OptimizationIntervalSize);
		TD_NodePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);
		TD_TimePredAry = AllocateDynamicArray<int>(m_NodeSize,m_OptimizationIntervalSize);


		if(m_OutboundSizeAry==NULL || m_LinkList==NULL || m_FromIDAry==NULL || m_ToIDAry==NULL  ||
			NodeStatusAry ==NULL || NodePredAry==NULL || LabelTimeAry==NULL || LabelCostAry==NULL)
		{
			cout << "Error: insufficent memory.";
			g_ProgramStop();
		}

	};


	~NetworkForSP()
	{
		try
		{
			if(m_OutboundSizeAry!=NULL) 
			{
				delete []m_OutboundSizeAry;
				m_OutboundSizeAry = NULL;
			}
			if(m_InboundSizeAry!=NULL)  
			{
				delete []m_InboundSizeAry;
				m_InboundSizeAry=NULL;
			}

			DeallocateDynamicArray<int>(m_OutboundNodeAry,m_NodeSize, m_AdjLinkSize);
			DeallocateDynamicArray<int>(m_OutboundLinkAry,m_NodeSize, m_AdjLinkSize);
			DeallocateDynamicArray<int>(m_InboundLinkAry,m_NodeSize, m_AdjLinkSize);


			if(m_LinkList!=NULL) 
			{
				delete []m_LinkList;
				m_LinkList = NULL;
			}

			if(m_LinkTDMaxWaitingTimeAry!=NULL)
			{
				delete []m_LinkTDMaxWaitingTimeAry;
				m_LinkTDMaxWaitingTimeAry = NULL;
			}

			DeallocateDynamicArray<float>(m_LinkTDTimeAry,m_LinkSize,m_OptimizationIntervalSize);	   
			DeallocateDynamicArray<float>(m_LinkTDCostAry,m_LinkSize,m_OptimizationIntervalSize);
			DeallocateDynamicArray<float>(TD_LabelCostAry,m_NodeSize,m_OptimizationIntervalSize);
			DeallocateDynamicArray<int>(TD_NodePredAry,m_NodeSize,m_OptimizationIntervalSize);
			DeallocateDynamicArray<int>(TD_TimePredAry,m_NodeSize,m_OptimizationIntervalSize);

			if(m_FromIDAry!=NULL)		
			{ 
				delete []m_FromIDAry;
                m_FromIDAry = NULL;
			}
			if(m_ToIDAry!=NULL)	
			{
				delete []m_ToIDAry;
				m_ToIDAry = NULL;
			}

			if(NodeStatusAry!=NULL) 
			{
				delete []NodeStatusAry;                 // Node status array used in KSP;
                NodeStatusAry = NULL;
			}
			if(NodePredAry!=NULL) 
			{
				delete []NodePredAry;
                NodePredAry = NULL;
			}
			if(LabelTimeAry!=NULL) 
			{
				delete []LabelTimeAry;
				LabelTimeAry = NULL;
			}
			if(LabelCostAry!=NULL) 
			{
				delete []LabelCostAry;
				LabelCostAry = NULL;
			}
		}
		catch(CMemoryException *e)
		{

		}


	};
    
	//for timetabling of one given train by time-dependent shortest path
	void BuildSpaceTimeNetworkForTimetabling(std::set<CNode*>* p_NodeSet, std::set<CLink*>* p_LinkSet, CTrain* pTrain);
	bool OptimalTDLabelCorrecting_DoubleQueue(int origin, int departure_time, int destination, int AllowableSlackAtDeparture, int CostPerUnitTimeStopped,CTrain * pT);
    int FindOptimalSolution(int origin,  int departure_time,  int destination, CTrain* pTrain, float& TripPrice);
	int FindOptimalSolutionMP(int origin,  int departure_time,  int destination, CTrain* pTrain, float& TripPrice);//for multiple path based train scheduling
	    
	// SEList: Scan List implementation: the reason for not using STL-like template is to avoid overhead associated pointer allocation/deallocation
	void SEList_clear()
	{
		m_ListFront= -1;
		m_ListTail= -1;
	}

	void SEList_push_front(int node)
	{
		if(m_ListFront == -1)  // start from empty
		{
			m_LinkList[node] = -1;
			m_ListFront  = node;
			m_ListTail  = node;
		}
		else
		{
			m_LinkList[node] = m_ListFront;
			m_ListFront  = node;
		}

	}
	void SEList_push_back(int node)
	{
		if(m_ListFront == -1)  // start from empty
		{
			m_ListFront = node;
			m_ListTail  = node;
			m_LinkList[node] = -1;
		}
		else
		{
			m_LinkList[m_ListTail] = node;
			m_LinkList[node] = -1;
			m_ListTail  = node;
		}
	}

	bool SEList_empty()
	{
		return(m_ListFront== -1);
	}

	int SEList_front()
	{
		return m_ListFront;
	}

	void SEList_pop_front()
	{
		int tempFront = m_ListFront;
		m_ListFront = m_LinkList[m_ListFront];
		m_LinkList[tempFront] = -1;
	}

	int  GetLinkNoByNodeIndex(int usn_index, int dsn_index);
	
	
	SearchTreeElement*  GenerateSearchTree(int origin_id, int destination_id, int node_size, float TravelTimeBound,int &otreelisttail,int direction)   // Pointer to previous node (node)
	{
		// time -dependent label correcting algorithm with deque implementation
		//  breadth-first search (BFS) is a graph search algorithm that begins at the root node and explores all the neighboring nodes
		// Then for each of those nearest nodes, it explores their unexplored neighbor nodes, and so on, until it finds the goal.
		// compared to BFS, we continue search even a node has been marked before
		// we allow a loop for simplicity
		// we can add a label cost constraint

		int i;

		if(direction==1)
		{
			if(m_OutboundSizeAry[origin_id]== 0)
			{
				otreelisttail = -1;
				return NULL;
			}
		}

		if(direction==2)
		{
			if(m_InboundSizeAry[origin_id]== 0)
			{
				otreelisttail = -1;
				return NULL;
			}
		}

		
		long m_TreeListSize = 5000000;

		SearchTreeElement* m_SearchTreeList = new SearchTreeElement[m_TreeListSize];

		for(i = 0; i < m_TreeListSize; i++)
		{
			m_SearchTreeList[i].CurrentNode = 0;
			m_SearchTreeList[i].SearchLevel = 0;
			m_SearchTreeList[i].TravelTime = 0;
			m_SearchTreeList[i].PredecessorNode = -1;
		}

		m_SearchTreeList[0].CurrentNode  = origin_id;
		m_SearchTreeList[0].SearchLevel = 0;

		int m_TreeListFront = 0;
		int m_TreeListTail = 1;

		int FromID, LinkNo, ToID;

		int PathNo = 0;

		while(m_TreeListTail < m_TreeListSize-1 && m_TreeListFront < m_TreeListTail)
			// not exceed search tree size, and not search to the end of queue/list
		{
			FromID  = m_SearchTreeList[m_TreeListFront].CurrentNode;
			int seefromnumber = g_NodeIDToNumberMap[FromID];

			if(FromID==destination_id || m_SearchTreeList[m_TreeListFront].SearchLevel  >= node_size || m_SearchTreeList[m_TreeListFront].TravelTime   >= TravelTimeBound)
			{

				m_TreeListFront ++; // move to the next front node for breadth first search
				continue;

				// when we finish all search, we should backtrace from a node at position i == destination)
			}
			if (direction==1)
			{			
				for(i=0; i<m_OutboundSizeAry[FromID];  i++)  // for each arc (i,j) belong A(j)
				{
					
					LinkNo = m_OutboundLinkAry[FromID][i];
					ToID = m_OutboundNodeAry[FromID][i];
					int seetonumber = g_NodeIDToNumberMap[ToID];

					if(ToID == origin_id)
					{					
						continue;
					}

					// search if ToID in the path
					bool bToID_inSubPathFlag = false;
					{
						int Pred = m_SearchTreeList[m_TreeListFront].PredecessorNode ;
						while(Pred>0)
						{
							if(m_SearchTreeList[Pred].CurrentNode == ToID)  // in the subpath
							{
								bToID_inSubPathFlag = true;
								break;
							}

							Pred = m_SearchTreeList[Pred].PredecessorNode;
						}

					}

					if(bToID_inSubPathFlag)
						continue;

					m_SearchTreeList[m_TreeListTail].CurrentNode = ToID;
					m_SearchTreeList[m_TreeListTail].PredecessorNode = m_TreeListFront;
					m_SearchTreeList[m_TreeListTail].SearchLevel = m_SearchTreeList[m_TreeListFront].SearchLevel + 1;

					float FFTT = m_LinkTDTimeAry[LinkNo][0];
					m_SearchTreeList[m_TreeListTail].TravelTime  = m_SearchTreeList[m_TreeListFront].TravelTime + FFTT;

					m_TreeListTail++;

					if(m_TreeListTail >= m_TreeListSize-1)
						break;  // the size of list is exceeded

				}// end of for each link
			}


			if (direction==2)
			{			
				for(i=0; i<m_InboundSizeAry[FromID];  i++)  // for each arc (i,j) belong A(j)
				{
					
					LinkNo = m_InboundLinkAry[FromID][i];
					CLink*pl = g_LinkIDMap[LinkNo];
					ToID = pl->m_FromNodeID;
									

					if(ToID == origin_id)
					{
						continue;
					}

					// search if ToID in the path
					bool bToID_inSubPathFlag = false;
					{
						int Pred = m_SearchTreeList[m_TreeListFront].PredecessorNode ;
						while(Pred>0)
						{
							if(m_SearchTreeList[Pred].CurrentNode == ToID)  // in the subpath
							{
								bToID_inSubPathFlag = true;
								break;
							}

							Pred = m_SearchTreeList[Pred].PredecessorNode;
						}

					}

					if(bToID_inSubPathFlag)
						continue;

					m_SearchTreeList[m_TreeListTail].CurrentNode = ToID;
					m_SearchTreeList[m_TreeListTail].PredecessorNode = m_TreeListFront;
					m_SearchTreeList[m_TreeListTail].SearchLevel = m_SearchTreeList[m_TreeListFront].SearchLevel + 1;

					float FFTT = m_LinkTDTimeAry[LinkNo][0];
					m_SearchTreeList[m_TreeListTail].TravelTime  = m_SearchTreeList[m_TreeListFront].TravelTime + FFTT;

					m_TreeListTail++;

					if(m_TreeListTail >= m_TreeListSize-1)
						break;  // the size of list is exceeded

				}// end of for each link
			}

			m_TreeListFront ++; // move to the next front node for breadth first search

		} // end of while

	
		
		if(m_TreeListFront ==  m_TreeListTail && m_TreeListTail < m_TreeListSize-1)
		{
			otreelisttail = m_TreeListTail;
			return m_SearchTreeList;
		}
		else
		{
			if(m_SearchTreeList!=NULL)
			{
				delete []m_SearchTreeList;		
				m_SearchTreeList = NULL;
			}
			otreelisttail = -1;
			return NULL;  // has not be enumerated.
		}
	}
};


#pragma warning(disable:4244)
