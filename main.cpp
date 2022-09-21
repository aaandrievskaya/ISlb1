#include <iostream>
#include "dijkstra.h"
#include "astar.h"
#include "structures.h"

int main(int argc, const char *argv[])
{
    Dijkstra planner;
    bool diagonal_moves_allowed = false;
    //AStar planner;
    std::string input_name = "C:\\Users\\EVGENY\\Desktop\\lb1\\Data\\7.map";
    if(argc > 1)
        input_name = argv[1];
    Input input(input_name, diagonal_moves_allowed);
    SearchResult result = planner.findPath(input);
    if(result.pathfound)
    {
        for(auto it = result.path.begin(); it != --result.path.end(); it++)
            std::cout<<"("<<it->x<<","<<it->y<<")->";
        std::cout<<"("<<(--result.path.end())->x<<","<<(--result.path.end())->y<<")";
    }
    else
        std::cout<<"PATH NOT FOUND!";

    std::cout<<"\nCost:"<<result.cost
             <<"\nRuntime: "<<result.runtime
             <<"\nSteps: "<<result.steps
             <<"\nNodes Created: "<<result.createdNodes
             <<"\nTrue Cardinal Cost:"<<input.true_cardinal_cost
             <<"\nTrue Diagonal Cost:"<<input.true_diagonal_cost<<"\n";
    return 0;
}

/*
 * start.g = 0
 * OPEN = {start}
 * CLOSED = {}
 * while OPEN is not empty
 *  current = node from OPEN with minimal g-value
 *  move current from OPEN to CLOSED
 *  if current is goal
 *      return reconstructPath(current)
 *  neighbors = get neighbors of current node
 *  for each neighbor in neighbors:
 *      if neighbor in CLOSED:
 *          continue
 *      neighbor.g = current.g + cost(neighbor, current)
 *      neighbor.parent = current
 *      insert or update neighbor into OPEN
 * return path not found
 */
