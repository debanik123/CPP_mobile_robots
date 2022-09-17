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

template<typename dtype>
void Basic_class::find_odd(vector<dtype>& v)
{
    auto is_odd = [](dtype x){return x%2;};
    // std::vector<dtype>::iterator it_odd;
    auto it_odd = find_if(v.begin(), v.end(), is_odd);
    std::cout << "The first odd value is " << *it_odd << endl;

}

template<typename dtype>
void Basic_class::find_even(vector<dtype>& v)
{
    auto is_even = [](dtype x){return x%2;};
    // std::vector<dtype>::iterator it_even;
    auto it_even = find_if_not(v.begin(), v.end(), is_even);
    std::cout << "The first even value is " << *it_even << endl;
    
}

int main()
{
    Basic_class obj;

    vector<int> v {2,5,7,8,10};
    cout<<"sum of int vector elements are-->"<<obj.sum_vector_element<int>(v)<<endl;
    obj.find_odd(v);
    obj.find_even(v);

    vector<float> vec {2.1, 5.4 , 7.6, 8.0, 10.8 , 30.9};
    cout<<"sum of float vector elements are-->"<<obj.sum_vector_element<float>(vec)<<endl;

    v.clear();

    return 0;
}