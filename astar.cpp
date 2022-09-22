#include "astar.h"
#include "dijkstra.h"

SearchResult AStar::findPath(Input input) //найдем ответ с помощью A*
{
    auto t = std::chrono::high_resolution_clock::now(); //фиксируем текущее время
    SearchResult result; //заведем пустой список для ответа
    input.start.f=0; // лучшая крутость1 для начальной вершины (расстояние от начальной вершины до текущей) 
    input.start.g = getHValue(input.start, input.goal, input.map.diagonal_moves_allowed); 
    // крутость2 для начальной вершины (расстояние от текущей вершины до конечной по прямой)
    open.addNode(input.start); //добавляем начальную вершину в список для обработки

    while (open.getSize()) { //пока список обработки не пуст
        auto current = open.getMin(); //берем первый элемент из списка обработки(он самый крутой)
        closed.addClose(current);//добавляем в список обработанных сразу, чтоб не забыть и потом забрать указатель на него
        open.popMin(); // удаляем этот минимум, чтоб не мешался
        auto tol = input.map.getValidMoves(current); // получить список доступных вершин для текущей
        for (auto b: tol) { //перебираем всех соседей для текущей вершины
            if (!closed.inClose(b.x, b.y)) { //если вершина ещё не обработана
                b.f = input.map.getCost(current, b) + current.f ;
                // вычисляем крутость1 данного соседа 
                b.g = getHValue(b,input.goal, input.map.diagonal_moves_allowed);
                // вычисляем крутость2 данного соседа
               
                b.parent = closed.getPointer(current.x, current.y); // забираем указатель на родителя
                open.addNode(b); // в список на обработку добавляется текущий сосед
            }
        }
        if (current.x == input.goal.x && current.y == input.goal.y) { //если текущая вершина является конечной
            result.pathfound = true; //путь найден
            auto answer = reconstructPath(current); //конструируем путь и вычисляем количество шагов
            result.path = answer.first; //записываем путь в результат
            result.steps = answer.second; //записываем количество шагов в результат
            result.cost = current.f; //записываем стоимость
            break;
        }
    }

    result.createdNodes = closed.getSize() + open.getSize(); //вычисляем количество созданных вершин
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count();
    // вычисляем время работы
    return result; //возвращаем результат
}
double AStar::getHValue(Node current, Node goal, bool dma) // вычисляем расстояние от точки до конечной по прямой
{
    if (dma) //если можно ходить по диагонали

        return sqrt(pow((goal.x - current.x),2) + pow((goal.y-current.y),2)); //вычисляем евклидово расстояние между точками
    return abs(goal.x - current.x)+ abs(goal.y-current.y); //вычисляем манхэттенское расстояние между точками
}

std::pair<std::list<Node>, int> AStar::reconstructPath(Node current) //восстановление пути, подсчет числа шагов
{
    std::list<Node> path; //создаем список нодов для хранения пути
    int Stepscount = 0;
    while(current.parent != nullptr) // пока текущий предок не пустой указатель(он у первого элемента)
    {
        path.push_front(current); //добавляем текущий элемент в начало списка
        current = *current.parent; // делаем родителя текущим элементом
        ++Stepscount;
    }
    path.push_front(current); //добавляем начальгый элемент отдельно
    return std::make_pair(path, Stepscount);  //возвращаем список нодов готовый :)
}

