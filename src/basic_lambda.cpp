// Generalized Lambda Expressions in C++14
// [](int a, int b) -> int { return a + b; }
// [](double a, double b) -> double { return a + b; }
// sort(container.begin(), container.end(), [](auto i, auto j) -> bool { return i > j; }
// template<typename T> [](T a, T b) -> T { return a + b };

#include<iostream>
#include<vector>
#include<string>
#include<algorithm>

using namespace std;

int main()
{
    auto sum = [](auto a, auto b){return a+b;};
    cout<<"sum(1,2)-->"<<sum(1,2)<<endl;

    auto multiple = [](auto a, auto b){return a*b;};
    cout<<"multiple(1,2)-->"<<multiple(1,2)<<endl;

    auto subtraction = [](auto a, auto b){return a-b;};
    cout<<"subtraction(1,2)-->"<<subtraction(1,2)<<endl;

    auto division = [](auto a, auto b) {return a/b;};
    cout<<"division(1,2)-->"<<division(4.0,2.0)<<endl;

    auto greater = [](auto a, auto b) -> bool {return a>b;};


    // append vector elements into vec
    vector<int> v;
    for(int i=0; i<10; i++)
    {
        v.push_back(i*5);
    }

    cout<<"v vector-->";
    for(auto c:v)
    {
        cout<<c<<" ";
    }
    cout<<endl;

    sort(v.begin(), v.end(), greater);

    cout<<"sort of the vector-->";
    for(auto c:v)
    {
        cout<<c<<" ";
    }
    cout<<endl;


    // double vector
    vector<double> vd{1, 4, 2, 1, 6, 62, 636};
    cout<<"vd vector-->";
    for(auto c:vd)
    {
        cout<<c<<" ";
    }
    cout<<endl;

    sort(vd.begin(), vd.end(), greater);
    
    cout<<"sort of the vector-->";
    for(auto c:vd)
    {
        cout<<c<<" ";
    }
    cout<<endl;

    //by value
    int x = 2;
    auto mul = [=](auto y){return x*y*2;};
    cout<<"mul_val-->"<<mul(2)<<endl;

    //by reference
    int y = 2;
    auto mul_ref = [&](auto x){y=y+1;return x*y;};
    cout<<"mul_ref-->"<<&y<<" "<<mul_ref(2)<<endl;

    auto mul_ = [&x](auto y){return x*y;};
    cout<<"mul_(&x, &y)-->"<<mul_(5)<<endl;


}