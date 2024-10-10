#pragma once
#include "TPG.hpp"

#include <set>
#include <unordered_map>
#include <chrono>

class BTPG : public TPG
{
private:
    int numBiPairs;
    int mode;//算法模式，0表示BTPG-n，1表示BTPG-o
    std::vector<BiPair *> BiPairs;//保存所有双向对
    std::vector<Type2EdgeGroup *> Type2EdgeGroups;
    

    int naiveNegativeCase = 0;
    // Helpers
    void CheckSingleton(type2Edge *candidateEdge);
    bool CheckSingletonValidity(type2Edge *candidateEdge);
    bool BidirectionalDFS(Node *currNode_, Node *endNode_, std::set<Node *> &VisitedStack_, std::vector<std::set<Node *>> &RevisitedNodes_, std::vector<Node *> &RecursionPath_, std::set<Node *> &RecursionPathSet_,
                          std::unordered_map<int, int> &AgentEdgeMap_, std::unordered_map<int, int> &AgentEdgeMapLeave_, bool hasTYpe1Edge_);

public:
    // using TPG::TPG;
    bool finish = false;
    // fileName: mapf路线的输入文件名
    BTPG(std::string fileName, int mode, int timeInterval);

    int getNumBiPairs();
    void addBiPair(BiPair *biPair);
    BiPair *getBiPair(int biPairId);

    int getNumType2EdgeGroups();
    void addType2EdgeGroup(Type2EdgeGroup *type2EdgeGroup);
    Type2EdgeGroup *getType2EdgeGroup(int type2EdgeGroupId);
};