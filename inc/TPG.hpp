#include "util.hpp"

class TPG
{
private:
    int numAgents; ///<@brief 机器人数量
    int numTypeTwoEdges; ///<@brief type2边的数量
    std::vector<Agent *> agents;
    std::vector<type2Edge *> type2Edges;///<@brief 记录type2边的集合

public:
    /**
     * @brief Construct a new TPG object
     * 
     * @param fileName mapf路径的文件名
     */
    TPG(std::string fileName);
    // ~TPG();

    int getNumAgents();
    // 获取type2边的数量
    int getNumTypeTwoEdges();
    void addRobot(Agent *agent);
    void addTypeTwoEdge(type2Edge *edge);
    void removeTypeTwoEdge(type2Edge *edge);

    Agent *getAgent(int robotId);
    // 通过边的id获取type2边
    type2Edge *getTypeTwoEdge(int edgeId);
};