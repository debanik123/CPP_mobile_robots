#ifndef Basic_class_H
#define Basic_class_H
#include <iostream>
#include <vector>
#include <bits/stdc++.h>
#include <numeric>
using namespace std;

class Basic_class
{
public:
    // int find_element(vector<int>& v, int element);
    template<typename dtype>
    dtype sum_vector_element(vector<dtype>& v);

    template<typename dtype>
    void find_odd(vector<dtype>& v);

    template<typename dtype>
    void find_even(vector<dtype>& v);

    // float sum_of_elems = 0;
};

#endif