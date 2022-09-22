#ifndef ASTAR_H
#define ASTAR_H
#include "input.h"
#include <chrono>

class AStar
{   ClosedList closed;
    OpenList open;
public:
    AStar() {}
    SearchResult findPath(Input input);
    std::pair<std::list<Node>,int> reconstructPath(Node current);

    double getHValue(Node current, Node goal, bool dma); //dma - diagonal moves allowed
};

#endif // ASTAR_H
