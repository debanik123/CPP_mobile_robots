#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;


//                up (-1,0)
//                  |
//                  |
//                  |
//(0,-1)left <-------------> right(0,1)
//                  |
//                  |
//                  |
//                down(1,0)
class Map
{
public:
    const static int mapWidth=6;
    const static int mapHeight=5;
    vector <vector<int>> grid = {
        {0,1,0,0,0,0},
        {0,1,0,0,0,0},
        {0,1,0,0,0,0},
        {0,1,0,0,0,0},
        {0,0,0,1,1,0}
    };

    template <typename T>
    void print2DVector(T vec);

};

template <typename T>
void Map::print2DVector(T vec)
{
    for (int i=0; i<vec.size();i++) //5
    {
        for(int j=0; j<vec[0].size(); j++) //6
        {
            cout<< vec[i][j]<<' ';
        }
        cout<<endl;
    }
    // cout<<"okkkkkkkkkkkkkkkkkk"<<endl;
}

class Planner : Map
{
public:
    int start[2] = {0,0};
    int goal[2] = {mapHeight-1, mapWidth-1}; //{4,5}
    int cost =1;

    string actions[4] = {"^","<",">","v"};
    vector<vector<int>> move{
        {-1,0},
        {0,-1},
        {1,0},
        {0,1}
    };
};


int main()
{
    Map map;
    map.print2DVector(map.grid);
    Planner planner;
    cout<<"Start: "<<planner.start[0]<<" , "<<planner.start[1]<<endl;
    cout<<"Goal: "<<planner.goal[0]<<" , "<<planner.goal[1]<<endl;
    cout<<"Robot actions are: "<<planner.actions[0]<<" , "<<planner.actions[1]<<" , "<<planner.actions[2]<<" , "<<planner.actions[3]<<endl;
    
    return 0;
}
