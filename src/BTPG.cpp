#include "BTPG.hpp"
#include <boost/property_tree/ptree.hpp>
#include "Sim.hpp"

const int BTPG_n = 0;
const int BTPG_o = 1;

// 初始化TPG以及对type2边进行分组
BTPG::BTPG(std::string fileName, int mode, int timeInterval)
    : TPG(fileName)
{
    this->mode = mode;
    this->numBiPairs = 0;
    this->naiveNegativeCase = 0;
    // Grouping 5.1节的最后提到的分组部分实现
#ifdef DEBUG
    std::cout << "| BTPG constructor |" << std::endl;
    std::cout << "Start Grouping ..." << std::endl;
#endif
    // 遍历每个agent，找到从这个节点出发的type2边
    for (int i = 0; i < getNumAgents(); i++)
    {
        Agent *agent = getAgent(i);
        Node *node = agent->Type1Next;

        while (node != NULL)
        {
            if (node->Type2Next.size() > 0)
            {
                // node对应图5的上方的C
                // 如果存在从这个节点出发的type2边
                // 遍历每个type2边
                for (auto &edge : node->Type2Next)
                {
                    if (edge->isGrouped)
                        // 如果该边已经被分组，则跳过
                        continue;
                    // edge->nodeTo对应图5的下方的B
                    int toId = edge->nodeTo->robotId;//这个边指向的节点的机器人id //对应图5的下方的机器人
                    int timeStep = edge->nodeTo->timeStep;//这个边指向的节点的时间步，对应图5的下方B的时间步
                    Node *fromNode = node;//记录这个type2边的起点，对应(v_(i+1)^m)，
                    Type2EdgeGroup *group = new Type2EdgeGroup();
                    int groupId = getNumType2EdgeGroups();
                    group->fromId = fromNode->robotId;
                    fromNode = fromNode->Type1Next;//对应图5的上方的D
                    group->type2Edges.push_back(edge);
                    group->toId = toId;//表示下方的机器人
                    edge->isGrouped = true;
                    edge->groupId = groupId;

                    // 下面循环主要是对type2边进行分组
                    while (fromNode != NULL)
                    {
                        bool found = false;
                        for (auto &edge2 : fromNode->Type2Next)
                        {
                            if (edge2->isGrouped)
                                continue;
                            // 如果这个边到达的节点的机器人id与group中的toId相同，且时间步相差1或-1，则加入到group中
                            // edge2->nodeTo对应图5的下方的C，满足下方条件
                            if (edge2->nodeTo->robotId == toId && (edge2->nodeTo->timeStep == timeStep + 1 || edge2->nodeTo->timeStep == timeStep - 1))
                            {
                                group->type2Edges.push_back(edge2);
                                edge2->isGrouped = true;
                                edge2->groupId = groupId;
                                timeStep = edge2->nodeTo->timeStep;
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                            break;
                        fromNode = fromNode->Type1Next;
                    }

                    addType2EdgeGroup(group);
                }
            }
            node = node->Type1Next;
        }
    }

#ifdef DEBUG
    std::cout << "End Grouping." << std::endl;
    std::cout << "******** Grouping Info ********" << std::endl;
    std::cout << "Number of groups: " << getNumType2EdgeGroups() << std::endl;
    std::cout << "Start BTPG ..." << std::endl;
#endif
    // count time
    auto start = std::chrono::high_resolution_clock::now();
    int type2EdgeSigleton = 0;
    int addMorePairs = 1;//记录了这次循环新增了多少个pair
    while (addMorePairs != 0)//直到没有新增pair，则循环结束
    {
        addMorePairs = 0;
        type2EdgeSigleton = 0;
        // 遍历每个group，只处理不能形成group的type2边，论文5.1节的最后一行
        for (int i = 0; i < getNumType2EdgeGroups(); i++)
        {
            Type2EdgeGroup *group = getType2EdgeGroup(i);
            if (group->type2Edges.size() > 1)
            {
                // CheckGroup(group) //TODO: Next Step
                continue;
            }
            else
            {
                // 找到只有1条type2边的group，也就是不能形成group的type2边
                auto endAnytime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endAnytime - start).count();
                if (timeInterval != 0 && duration > timeInterval)
                {

#ifdef DEBUG
                    std::cout << "End BTPG." << std::endl;
                    std::cout << "******** BTPG Info ********" << std::endl;
                    std::cout << "Number of type-2 edge sigleton: " << type2EdgeSigleton << std::endl;
                    std::cout << "Number of BiPairs: " << getNumBiPairs() << std::endl;
                    std::cout << "Number of naive negative cases: " << this->naiveNegativeCase << std::endl;
                    std::cout << "******** ***** ********" << std::endl;
#endif
                    return;
                }
                type2EdgeSigleton++;
                int biPairNum = getNumBiPairs();//记录当前bidirectional pair的数量
                if (group->type2Edges[0]->isBidirectional)
                    continue;
                CheckSingleton(group->type2Edges[0]);//检测单向type2边是否可以形成bidirectional pair
                // 如果增加了新的bidirectional pair，则addMorePairs计数加1
                if (biPairNum != getNumBiPairs())
                {
                    addMorePairs++;
                }
            }
        }
#ifdef DEBUG
        std::cout << "current loop:" << getNumBiPairs() << std::endl;
#endif
        if (this->mode == 0)
        {
            addMorePairs = 0;
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    this->finish = true;
#ifdef DEBUG
    std::cout << "End BTPG." << std::endl;
    std::cout << "******** BTPG Info ********" << std::endl;
    std::cout << "Number of type-2 edge sigleton: " << type2EdgeSigleton << std::endl;
    std::cout << "Number of BiPairs: " << getNumBiPairs() << std::endl;
    std::cout << "Number of naive negative cases: " << this->naiveNegativeCase << std::endl;
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    std::cout << "******** ***** ********" << std::endl;
#endif
// output the BTPG into a json file using boost library
#ifdef DEBUG
    std::cout << "Start output BTPG ..." << std::endl;
#endif
}

int BTPG::getNumType2EdgeGroups()
{
    return this->Type2EdgeGroups.size();
}

void BTPG::addType2EdgeGroup(Type2EdgeGroup *group)
{
    this->Type2EdgeGroups.push_back(group);
}

Type2EdgeGroup *BTPG::getType2EdgeGroup(int groupId)
{
    return this->Type2EdgeGroups[groupId];
}

// 添加pair到BiPairs中
void BTPG::addBiPair(BiPair *pair)
{
    this->BiPairs.push_back(pair);
}

int BTPG::getNumBiPairs()
{
    return this->BiPairs.size();
}

BiPair *BTPG::getBiPair(int biPairId)
{
    return this->BiPairs[biPairId];
}

// Helper functions
// candidateEdge 类似于论文中的e=(v^m_(i+1), v^(n)_(j))
void BTPG::CheckSingleton(type2Edge *candidateEdge)
{
    // 如果边原来已经是bidirectional，则直接返回
    if (candidateEdge->isBidirectional)
        return;
    if (CheckSingletonValidity(candidateEdge))
    {
        // 对应论文中算法1的第8行，可以形成bidirectional pair，则将candidateEdge设置为bidirectional，并添加双向对
        // set candidateEdge to be bidirectional
        candidateEdge->isBidirectional = true;

        // Add another type-2 edge 新建一条反向的type2边
        type2Edge *newType2Edge = new type2Edge();
        newType2Edge->nodeFrom = candidateEdge->nodeTo;//v^(n)_(j)，TODO:为什么这里不是v^(n)_(j+1)??与下面的不一样
        newType2Edge->nodeTo = candidateEdge->nodeFrom;//v^m_i+1, TODO:为什么这里不是v^m_i??与下面的不一样
        newType2Edge->edgeId = getNumTypeTwoEdges();
        newType2Edge->isBidirectional = true;
        addTypeTwoEdge(newType2Edge);

        // Add new edge to the node
        candidateEdge->nodeTo->Type1Next->Type2Next.push_back(newType2Edge);//将边添加到v^(n)_(j+1)节点
        candidateEdge->nodeFrom->Type1Prev->Type2Prev.push_back(newType2Edge);//将边添加到v^m_i节点

        // Add BiPair 算法1第8行左边的步骤，添加双向对
        BiPair *newBiPair = new BiPair(candidateEdge->edgeId, newType2Edge->edgeId);
        newBiPair->id = getNumBiPairs();
        addBiPair(newBiPair);

        // Update two edges
        candidateEdge->biPairId = newBiPair->id;
        newType2Edge->biPairId = newBiPair->id;
    }
    return;
}

/**
 * @brief 这个才是BTPG的核心，论文中的算法1，
 * 
 * @param candidateEdge 为原来的type2边(（(v^m_i+1, v^(n)_(j))）)
 * @return true 可以形成bidirectional pair
 * @return false 不能形成bidirectional pair
 */
bool BTPG::CheckSingletonValidity(type2Edge *candidateEdge)
{
    // std::cout << candidateEdge->nodeFrom->robotId << " " << candidateEdge->nodeFrom->timeStep << " -> " << candidateEdge->nodeTo->robotId << " " << candidateEdge->nodeTo->timeStep << std::endl;
    // 1. Initialization
    // 1a. Initialize the start and end nodes 
    Node *startNode = candidateEdge->nodeFrom->Type1Prev; //startNode = v^m_i
    Node *endNode = candidateEdge->nodeTo->Type1Next; //endNode = v^n_(j+1)
    // edge cases
    if (endNode == NULL || startNode->timeStep == 0)
    {
        this->naiveNegativeCase++;
        return false;
    }

    // Temporarily set the edge to be bidirectional
    candidateEdge->isBidirectional = true;
    // Temporarily add a new edge to type-2 edge 算法1第4行，新建一条反向的type2边（(v^n_j+1, v^m_i)）
    type2Edge *newType2Edge = new type2Edge();
    newType2Edge->nodeFrom = endNode;
    newType2Edge->nodeTo = startNode;
    newType2Edge->edgeId = getNumTypeTwoEdges();
    newType2Edge->isBidirectional = true;
    addTypeTwoEdge(newType2Edge);//算法1第5行，添加到type2边的集合中

    // 1b. Initialize the visit stacks for nodes 记录所有已经访问过的节点
    std::set<Node *> visitedNodes;

    // 1b'. Initialize the visit stacks for current node
    Node *setVisitedNode = endNode->Type1Prev;//v_(j)^n
    while (setVisitedNode != NULL)
    {
        visitedNodes.insert(setVisitedNode);
        setVisitedNode = setVisitedNode->Type1Prev;
    }

    // 1c. Initialize the stacks for revisit nodes
    std::vector<std::set<Node *>> revisitedNodes;
    for (int i = 0; i < getNumTypeTwoEdges(); i++)
    {
        std::set<Node *> temp;
        revisitedNodes.push_back(temp);
    }

    // 1d. stack for current recursion path
    std::vector<Node *> recursionPath;//记录已经访问过的节点
    std::set<Node *> recursionPathSet;

    // 1e. stack for edges in the current recursion path
    std::set<int> recursionEdgePath;

    // 1f. dictionary for agents that how to get to the end node
    std::unordered_map<int, int> agentEdgeMap;//记录从那条边进入到这个agent的<robotId, edgeId>
    for (int i = 0; i < getNumAgents(); i++)
    {
        agentEdgeMap[i] = -1;
    }

    // 1g. dictionary for agents that how to leave to the end node
    std::unordered_map<int, int> agentEdgeMapLeave;//记录从那条边离开到这个agent的
    for (int i = 0; i < getNumAgents(); i++)
    {
        agentEdgeMapLeave[i] = -1;
    }

    agentEdgeMap[startNode->robotId] = candidateEdge->edgeId;
    // std::cout << "StartNode: " << startNode->robotId << " " << startNode->timeStep << std::endl;
    // check if the startnode has a type-2 neighbor is 12 99
    // for (auto &edge : startNode->Type2Next)
    // {
    //     std::cout << "edge: " << edge->nodeTo->robotId << " " << edge->nodeTo->timeStep << std::endl;
    // }

    agentEdgeMap[startNode->robotId] = newType2Edge->edgeId;
    // 2. Start the search 算法1第7行
    if (BidirectionalDFS(startNode, endNode, visitedNodes, revisitedNodes, recursionPath, recursionPathSet, agentEdgeMap, agentEdgeMapLeave, false))
    {
        // 对应算法1第8行，存在NRNS, 无法形成bidirectional pair
        candidateEdge->isBidirectional = false;
        // delete the new edge 删除新加入的边
        removeTypeTwoEdge(newType2Edge);//（(v^n_j+1, v^m_i)）
        delete newType2Edge;
        return false;
    }
    else
    {
        candidateEdge->isBidirectional = false;
        // delete the new edge 删除新加入的边
        removeTypeTwoEdge(newType2Edge);
        delete newType2Edge;
        return true;
    }
}

/**
 * @brief 深度搜索DFS, 论文中的算法2 hasCycle函数，判断是否存在NRNS cycle
 * 
 * @param currNode_ 当前节点
 * @param endNode_ 目标节点
 * @param VisitedStack_ 
 * @param RevisitedNodes_ 
 * @param RecursionPath_  记录已经访问过的边，对应论文中的E_vis
 * @param RecursionPathSet_ 记录了遍历的所有节点
 * @param AgentEdgeMap_ 记录从那条边进入到这个agent的<robotId, edgeId>
 * @param AgentEdgeMapLeave_ 记录从哪条边离开这个agent的<robotId, edgeId>
 * @param hasTYpe1Edge_ 上边遍历的边是否含有type1边
 * @return true 包含NRNS cycle
 * @return false 不包含NRNS cycle
 */
bool BTPG::BidirectionalDFS(Node *currNode_, Node *endNode_, std::set<Node *> &VisitedStack_, std::vector<std::set<Node *>> &RevisitedNodes_, std::vector<Node *> &RecursionPath_, std::set<Node *> &RecursionPathSet_,
                            std::unordered_map<int, int> &AgentEdgeMap_, std::unordered_map<int, int> &AgentEdgeMapLeave_, bool hasTYpe1Edge_)
{
    // std::cout << "currNode: " << currNode_->robotId << " " << currNode_->timeStep << std::endl;
    // !: Base Case
    // 1. check if reach the end node
    if (currNode_ == endNode_)
    {
        // 如果设置了hasTYpe1Edge_，表示这个cycle中带有type1边，则不是rotation cycle,属于non-rotation cycle，则返回true
        // rotaion cycle定义是只含有type2边，而且边的数目大于2。
        if (hasTYpe1Edge_ || RecursionPath_.size() > 2)
        {
            // print out the recursion path
            // std::cout << "Recursion Path: ";
            // for (auto &node : RecursionPath_)
            // {
            //     std::cout << node->robotId << " " << node->timeStep << " -> ";
            // }
            // std::cout << std::endl;
            // std::cout << "endNode: " << endNode_->robotId << " " << endNode_->timeStep << std::endl;
            return true;//TODO:论文中，算法2第3行，这里应该返回false才对，但是这里返回了true，为什么相反了？？
        }
        else
        {
            // revisit all the nodes in the recursion path
            for (auto node = RecursionPath_.rbegin(); node != RecursionPath_.rend() - 1; node++)
            {
                // ??: should set unvisit directly
                VisitedStack_.erase(*node);
            }
            return false;
        }
    }

    // Update
    VisitedStack_.insert(currNode_);
    RecursionPath_.push_back(currNode_);//记录当前迭代的路径
    RecursionPathSet_.insert(currNode_);

    // !: Traverse Type-2 Edge 遍历从currNode_出发的所有type2边
    for (auto &edge : currNode_->Type2Next)
    {
        // 2. Checking if need to keep going on
        // 判断edge->nodeTo节点是否已经访问过（判断edge->nodeTo是否在VisitedStack_和RecursionPathSet_中）算法2第8行
        if (VisitedStack_.find(edge->nodeTo) == VisitedStack_.end() && RecursionPathSet_.find(edge->nodeTo) == RecursionPathSet_.end())
        {
            // Need keep visiting next node
            // 2a. if so, checking if the edge is bidirectional 如果没有访问过，则判断是否是bidirectional pair
            if (edge->isBidirectional)//论文中算法2第7行，如果这条边已经属于bidirectional pair
            {
                if (this->mode == 0 && AgentEdgeMap_[currNode_->robotId] != -1)
                {
                    // Only check if the previous edge is the same bidirectional edge
                    Node *prevNode = getTypeTwoEdge(AgentEdgeMap_[currNode_->robotId])->nodeTo;
                    if (edge->biPairId == getTypeTwoEdge(AgentEdgeMap_[currNode_->robotId])->biPairId)
                    {
                        // 如果这条边跟之前的边属于同一个bidirectional pair
                        // !: if reach current Node by a type-1 edge, then we should only revisit current node
                        // 如果通过type1边到达当前节点
                        if (RecursionPath_.back()->robotId == currNode_->robotId)
                        {
                            RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(currNode_);
                        }
                        else
                        {
                            // !: if reach current Node by a type-2 edge,
                            // !then we should revisit all the nodes in the recursion path back to the prevNode
                            for (auto renode = RecursionPath_.rbegin(); renode != RecursionPath_.rend(); renode++)
                            {
                                RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(*renode);
                                if (*renode == prevNode)
                                {
                                    break;
                                }
                            }
                        }
                        continue;
                    }
                }
                // 2a.1: Check if we should visit the edge (BTPG-o)
                if (this->mode == 1 && AgentEdgeMap_[currNode_->robotId] != -1)
                {
                    // Have visited the same agent
                    // std::cout << "current number of type-2:" << getNumTypeTwoEdges() << std::endl;
                    // std::cout << "current number of type-2:" << AgentEdgeMap_[currNode_->robotId] << std::endl;
                    Node *prevNode = getTypeTwoEdge(AgentEdgeMap_[currNode_->robotId])->nodeTo;
                    // 2a.1.1: Check if prevNode is in the same robot's path and has smaller time step
                    if (prevNode->timeStep < currNode_->timeStep)
                    {
                        // !: if reach current Node by a type-1 edge, then we should only revisit current node
                        if (RecursionPath_.back()->robotId == currNode_->robotId)
                        {
                            RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(currNode_);
                        }
                        else
                        {
                            // !: if reach current Node by a type-2 edge,
                            // !then we should revisit all the nodes in the recursion path back to the prevNode
                            for (auto renode = RecursionPath_.rbegin(); renode != RecursionPath_.rend(); renode++)
                            {
                                RevisitedNodes_[AgentEdgeMap_[currNode_->robotId]].insert(*renode);
                                if (*renode == prevNode)
                                {
                                    break;
                                }
                            }
                        }

                        continue;
                    }
                }
            }

            // 2a.2: Check if we should visit the edge (BTPG-o)
            if (this->mode == 1 && AgentEdgeMapLeave_[edge->nodeTo->robotId] != -1)
            {
                type2Edge *prevEdge = getTypeTwoEdge(AgentEdgeMapLeave_[edge->nodeTo->robotId]);
                Node *prevFromNode = prevEdge->nodeFrom;
                if (prevFromNode->timeStep > edge->nodeTo->timeStep && prevEdge->isBidirectional)
                {
                    for (auto renode = RecursionPath_.rbegin(); renode != RecursionPath_.rend(); renode++)
                    {
                        RevisitedNodes_[AgentEdgeMapLeave_[edge->nodeTo->robotId]].insert(*renode);
                        if (*renode == prevEdge->nodeTo)
                        {
                            break;
                        }
                    }
                    continue;
                }
            }

            // 2b. keep visiting next node 继续递归下一个节点

            if (edge->nodeTo->robotId != -1)
            {
                // 2b.1: update agentEdgeMap (enter) 记录从edge->edgeId进入到edge->nodeTo的robotId
                AgentEdgeMap_[edge->nodeTo->robotId] = edge->edgeId;
            }
            // 2b.2: update agentEdgeMap (leave) 记录从edge->edgeId离开currNode_的robotId
            AgentEdgeMapLeave_[currNode_->robotId] = edge->edgeId;

            // !: c.Recursion
            if (BidirectionalDFS(edge->nodeTo, endNode_, VisitedStack_, RevisitedNodes_, RecursionPath_, RecursionPathSet_, AgentEdgeMap_, AgentEdgeMapLeave_, hasTYpe1Edge_))
            {
                // 如果能找到NRNS cycle，则返回true
                return true;
            }

            if (edge->nodeTo->robotId != -1)
            {
                // 2b.3: update agentEdgeMap (leave), without the edge in the recursion path
                AgentEdgeMap_[edge->nodeTo->robotId] = -1;
            }
            // 2b.4: update agentEdgeMap (leave), without the edge in the recursion path
            AgentEdgeMapLeave_[currNode_->robotId] = -1;

            // revisit relevant nodes
            for (auto &node : RevisitedNodes_[edge->edgeId])
            {
                VisitedStack_.erase(node);
            }
            RevisitedNodes_[edge->edgeId].clear();
        }
        else
        {
            // 如果这个节点已经访问过，则继续下一个边
            continue;
        }
    }

    // !: Traverse Type-1 Edge 遍历从currNode_出发的所有type1边
    if (currNode_->Type1Next != NULL)
    {
        // 1. check if need to keep going on
        if (VisitedStack_.find(currNode_->Type1Next) == VisitedStack_.end() && RecursionPathSet_.find(currNode_->Type1Next) == RecursionPathSet_.end())
        {
            // 如果currNode_->Type1Next不存在于VisitedStack_和RecursionPathSet_中，则继续迭代
            // !: c.Recursion
            if (BidirectionalDFS(currNode_->Type1Next, endNode_, VisitedStack_, RevisitedNodes_, RecursionPath_, RecursionPathSet_, AgentEdgeMap_, AgentEdgeMapLeave_, true))
            {
                // 如果在迭代中找到NRNS cycle，则返回true
                return true;
            }
        }
    }

    // Update
    RecursionPath_.pop_back();
    RecursionPathSet_.erase(currNode_);

    return false;
}