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

    cout <<"lambda_function single augument--->"<<
    [](int x){return x+2;}(2)<< endl;

    cout <<"lambda_function double auguments--->"<<
    [](int x, int y){return x+y;}(2, 4)<< endl;

    cout <<"lambda_function tree auguments--->"<<
    [](int x, int y, int z){return x+y+z;}(2, 4, 2)<< endl;

    cout <<"lambda_function four auguments--->"<<
    [](int x, int y, int z, int g){return x+y+z+g;}(2, 4, 2, 5)<< endl;

    auto sum_ = [](int x, int y){return x+y;};
    cout<<"sum of x+y --->"<<sum_(2,6)<<endl;

    auto sum_three = [](int x, int y, int z){return x+y+z;};
    cout<<"sum of x+y+z --->"<<sum_three(2,6,5)<<endl;

    auto sum_four = [](int x, int y, int z, int g){return x+y+z+g;};
    cout<<"sum of x+y+z+g --->"<<sum_four(2,6,5,6)<<endl;

    vector<int> v_ {2, 3, 5, 6, 7, 8, 43, 8};
    vector<int> v_mix {2,-3, 5, 6,-7, 8, 43, -8};
    auto cond = [](int x){return x > 0;};
    cout<<"all_of-->"<<all_of(v_.begin(), v_.end(), cond)<<endl;  //all are true
    cout<<"all_of-->"<<all_of(v_mix.begin(), v_mix.end(), cond)<<endl;

    cout<<"any_of-->"<<any_of(v_.begin(), v_.end(), cond)<<endl;  //any of true
    cout<<"any_of-->"<<any_of(v_mix.begin(), v_mix.end(), cond)<<endl;

    cout<<"none_of-->"<<none_of(v_.begin(), v_.end(), cond)<<endl;  //none of true
    cout<<"none_of-->"<<none_of(v_mix.begin(), v_mix.end(), cond)<<endl;

    vector<int> g {4, 2, 6, 8};
    auto cond_ = [](int x){return x%2==0;};
    cout<<"all_of-->"<<all_of(g.begin(), g.end(), cond_)<<endl;
    cout<<"any_of-->"<<any_of(g.begin(), g.end(), cond_)<<endl;
    cout<<"none_of-->"<<none_of(g.begin(), g.end(), cond_)<<endl;

}