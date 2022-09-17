#include "basic_class.h"

// int Basic_class::find_element(vector<int>& v, int element)
// {
//     auto find = [element]()
// }

int Basic_class::sum_vector_element(vector<int>& v)
{
    sum_of_elems = accumulate(v.begin(), v.end(), 0);
    return sum_of_elems;
}

int main()
{
    Basic_class obj;

    vector<int> v {0,2,5,7,8,10,30};

    cout<<"sum of vector elements are-->"<<obj.sum_vector_element(v)<<endl;

    v.clear();

    return 0;
}