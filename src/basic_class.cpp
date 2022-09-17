#include "basic_class.h"

// float Basic_class::find_element(vector<float>& v, float element)
// {
//     auto find = [element]()
// }

float Basic_class::sum_vector_element(vector<float>& v)
{
    sum_of_elems = accumulate(v.begin(), v.end(), 0.0f);
    return sum_of_elems;
}

int main()
{
    Basic_class obj;

    vector<float> v {0.0, 2.1, 5.4 , 7.6, 8.0, 10.8 , 30.9};

    cout<<"sum of vector elements are-->"<<obj.sum_vector_element(v)<<endl;

    v.clear();

    return 0;
}