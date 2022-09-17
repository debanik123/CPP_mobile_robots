#include "basic_class.h"

// float Basic_class::find_element(vector<float>& v, float element)
// {
//     auto find = [element]()
// }

template<typename dtype>
dtype Basic_class::sum_vector_element(vector<dtype>& v)
{
    return accumulate(v.begin(), v.end(), 0.0f);
}

int main()
{
    Basic_class obj;

    vector<int> v {0,2,5,7,8,10};
    cout<<"sum of int vector elements are-->"<<obj.sum_vector_element<int>(v)<<endl;

    vector<float> vec {0.0, 2.1, 5.4 , 7.6, 8.0, 10.8 , 30.9};
    cout<<"sum of float vector elements are-->"<<obj.sum_vector_element<float>(vec)<<endl;


    v.clear();

    return 0;
}