#include <iostream>
#include <vector>
#include <bits/stdc++.h>
using namespace std;

int main()
{
    vector<int> v = {0,2,5,7,8,10};
    int k=10;
    int sum=0;

    // vector index and element access
    cout<<"vector index and element access"<<endl;
    for(int i=0; i<v.size();i++)
    {
        cout<<i<<' '<<v.at(i)<<endl;
    }
    cout<<"-------------------------------"<<endl;

    // vector element access from a loop
    cout<<"vector element access from a loop"<<endl;
    for(int n:v)
    {
        cout<<n<<endl;
    }
    cout<<"-------------------------------"<<endl;

    // vector element access by pointer
    cout<<"vector element access by pointer"<<endl;
    for(auto it=v.begin(); it !=v.end(); it++)
    {
        cout<<"---->"<<*it<<endl;
    }
    cout<<"-------------------------------"<<endl;

    // Iterating vector elements in the Reverse Direction
    cout<<"Iterating vector elements in the Reverse Direction"<<endl;
    for(auto it=v.rbegin(); it !=v.rend(); it++)
    {
        cout<<"---->"<<*it<<endl;
    }
    cout<<"-------------------------------"<<endl;

    // vector element sum and find a specific sum value with condition
    cout<<"vector element sum and find a specific sum value with condition"<<endl;
    for(int i=0; i<v.size();i++)
    {
        for(int j=0; j<v.size();j++)
        {
            sum = v.at(i) + v.at(j);
            if(sum == 7)
            {
                cout<<'['<<i<<','<<j<<']'<<"----->>"<<i<<' '<<j<<"--->"<<sum<<endl;
            }
        }
    }
    cout<<"-------------------------------"<<endl;
    return 0;
}
