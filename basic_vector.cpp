#include <iostream>
#include <vector>
#include <bits/stdc++.h>
using namespace std;

int main()
{
    vector<int> v {0,2,5,7,8,10};
    vector<int>::iterator it;
    int k=8;
    int sum=0;
    int size_of_v = v.size();

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

    // find an element index in a vector
    cout<<"find an element in a vector"<<endl;
    it = find(v.begin(),v.end(),k);
    cout<<"item found->>"<<*it<<"index at-->"<<size_of_v-(v.end()-it)<<endl;
    cout<<"-------------------------------"<<endl;

    // find an element index in a vector with auto
    cout<<"find an element index in a vector with auto"<<endl;
    auto it_ = find(v.begin(),v.end(),k);
    cout<<"item found->>"<<*it<<"index at-->"<<size_of_v-(v.end()-it_)<<endl;
    cout<<"-------------------------------"<<endl;

    
    vector<int> vec;
    // append vector elements into vec
    for(int i=0; i<10; i++)
    {
        vec.push_back(i*4);
    }

    cout << "Output of begin and end: ";
    for(auto it=vec.begin(); it!=vec.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "\nOutput of cbegin and cend: ";
    for(auto it=vec.cbegin(); it!=vec.cend();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "\nOutput of rbegin and rend: ";
    for(auto it=vec.rbegin(); it!=vec.rend();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "\nOutput of crbegin and crend : ";
    for(auto it=vec.crbegin(); it!=vec.crend();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "Size : " <<vec.size()<<endl;
    cout << "\nCapacity : " <<vec.capacity()<<endl;
    cout << "\nMax_Size : " <<vec.max_size()<<endl;

    vec.resize(6);
    cout << "resized vec Size : " <<vec.size()<<endl;

    cout << "resized vec Size iter : ";
    for(auto it=vec.begin(); it!=vec.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    if (vec.empty() == false)
        cout << "\nVector is not empty"<<endl;
    else
        cout << "\nVector is empty"<<endl;
    
    // Shrinks the vector
    vec.shrink_to_fit();
    cout << "Shrinks vec Size iter : ";
    for(auto it=vec.begin(); it!=vec.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "\nReference operator [vec] : vec[2] = " << vec[2]<<endl;
    cout << "\nat : vec.at(4) = " << vec.at(4)<<endl;
    cout << "\nfront() : vec.front() = " << vec.front()<<endl;
    cout << "\nback() : vec.back() = " << vec.back()<<endl;

    // pointer to the first element
    auto p = vec.data();
    cout << "\nThe first element memory address is " << p <<endl;
    cout << "\nThe first element is " << *p <<endl;
    cout << "\nThe second element is " << *(p+1) <<endl;
    cout << "\nThe third element is " << *(p+2) <<endl;

    // Modifiers
    // Assign vector
    vector<int> vect;

    // fill the array with 20 five times
    vect.assign(5,20);

    cout << "Output of begin and end: ";
    for(auto it=vect.begin(); it!=vect.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    int n = vect.size();
    cout << "\nThe last element is: " << vect[n - 1]<<endl;

    // removes last element
    vect.pop_back();

    cout << "Output of begin and end: ";
    for(auto it=vect.begin(); it!=vect.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    // inserts at the beginning
    vect.insert(vect.begin(), 10);
    cout << "\nThe first element is: " << vect[0]<<endl;

    // Inserts 20 at the end
    vect.emplace_back(40);
    int o = vect.size();
    cout << "\nThe last element is: " << vect[o - 1]<<endl;

    // erases the vector
    vect.clear();
    cout << "\nVector size after erase(): " << vect.size()<<endl;

    // two vector to perform swap
    vector<int> v1 {4,5,7}, v2 {8,5,9};
    cout << "\nVector v1 size : " <<v1.size()<<", Vector v2 size : "<<v2.size()<<endl;

    cout << "\n\nVector 1: ";
    for(auto it=v1.begin(); it!=v1.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "\nVector 2: ";
    for(auto it=v2.begin(); it!=v2.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    v1.swap(v2);

    cout << "\nAfter Swap \nVector 1: ";
    for(auto it=v1.begin(); it!=v1.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    cout << "\nVector 2: ";
    for(auto it=v2.begin(); it!=v2.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    // Vector iterators are used to point to the memory address of a vector element. In some ways, they act like pointers in C++.
    vector<int> l;

    for(int i=0; i<10; i++)
    {
        l.push_back(5*i);
    }

    cout << "\nVector l: ";
    for(auto it=l.begin(); it!=l.end();it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    cout<<"-------------------------------"<<endl;

    vector<int>::iterator ip;

    // iter points to l[0]
    ip = l.begin();
    cout<<"iter points to l[0]-->"<<*ip<<endl;
    cout<<"iter points to l[1]-->"<<*(ip+1)<<endl;
    cout<<"iter points to l[2]-->"<<*(ip+2)<<endl;
    cout<<"iter points to l[3]-->"<<*(ip+3)<<endl;
    cout<<"iter points to l[4]-->"<<*(ip+4)<<endl;
    ip = l.begin()+5;
    cout<<"iter points to l[5]-->"<<*ip<<endl;
    ip = l.begin()+6;
    cout<<"iter points to l[6]-->"<<*ip<<endl;
    ip = l.begin()+7;
    cout<<"iter points to l[7]-->"<<*ip<<endl;
    ip = l.begin()+8;
    cout<<"iter points to l[8]-->"<<*ip<<endl;

    // iter points to the last element of num
    ip = l.end()-1;
    cout<<"iter points to l[n-1]-->"<<*ip<<endl;

    // use iterator with for loop
    for(auto it=l.begin(); it!=l.end(); it++)
    {
        cout<<*it<<" ";
    }
    cout<<endl;
    
    return 0;
}
