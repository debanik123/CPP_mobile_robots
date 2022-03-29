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
    for(int i=0; i<v.size();i++)
    {
        cout<<i<<' '<<v.at(i)<<endl;
    }
    cout<<"-------------------------------";

    // vector element access from a loop
    for(int n:v)
    {
        cout<<n<<endl;
    }
    cout<<"-------------------------------";

    // vector element sum and find a specific sum value with condition
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

        cout<<"-------------------------------";
    }
    return 0;
}
