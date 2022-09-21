#include "dijkstra.h"

SearchResult Dijkstra::findPath(Input input) 
{
    auto t = std::chrono::high_resolution_clock::now(); //записать время начала работы
    SearchResult result;   // создать пустой результат
    input.start.f=0;  // крутость первой вершины лучшая (0)
    open.addNode(input.start); // добавить стартовую вершину в список на обработку

    while (open.getSize()) {   
        auto current = open.getMin();  //берем первый элемент из списка обработки (он самый крутой)
        closed.addClose(current); //добавляем в список обработанных сразу, чтоб не забыть и потом забрать указатель на него
        open.popMin(); // удаляем этот минимум, чтоб не мешался
        auto tol = input.map.getValidMoves(current); // получить список доступных вершин для текущей
        for (auto b: tol) {  //перебираем всех соседей для текущей вершины
            if (!closed.inClose(b.x, b.y)) {  //если вершина ещё не обработана
                b.f = input.map.getCost(current, b) + current.f;  // ??  обновляем крутость данного соседа 
                b.parent = closed.getPointer(current.x, current.y); // забираем указатель на родителя
                open.addNode(b);  // в список на обработку добавляется текущий сосед
            }
        }
        if (current.x == input.goal.x && current.y == input.goal.y) {
            //closed.addClose(current); // ?
            result.pathfound = true;  // нашелся ответ!
            auto answer = reconstructPath(current);
            result.path = answer.first;
            result.steps = answer.second;
            result.cost = current.f; // вернуть итоговую крутость финишной вершины
            break;
        }
    }

    result.createdNodes = closed.getSize() + open.getSize(); //сумма длин обоих списков вершин = итоговое количество созданных вершин
    result.runtime = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::high_resolution_clock::now() - t).count(); //время работы - текущее минус начальное
    return result;
}

std::pair<std::list<Node>, int> Dijkstra::reconstructPath(Node current)
{
    std::list<Node> path;  //создаем список нодов для хранения пути
    int stepsCount = 0;
    while(current.parent != nullptr) // пока текущий предок не пустой указатель(он у первого элемента)
    {
        path.push_front(current); //добавляем текущий элемент в начало списка
        current = *current.parent; // делаем родителя текущим элементом
        ++stepsCount;
    }
    path.push_front(current); //добавляем начальгый элемент отдельно
    return make_pair(path, stepsCount); //возвращаем список нодов готовый :)
}
