#include <iostream>
#include <thread>
#include <string>

void foo()
{
    // printf("Hello from foo - \n", std::this_thread::get_id());
    std::cout<<"Hello from foo "<< std::this_thread::get_id()<<std::endl;

}

class callable_class {
    public:
        void operator()()
        {
            // printf("Hello form callable class - \n", std::this_thread::get_id());
            std::cout<<"Hello from callable_class "<< std::this_thread::get_id()<<std::endl;
        }
};

int main(int argc, char **argv)
{
    std::thread thread1(foo);

    callable_class obj1;
    std::thread thread2(obj1);

    std::thread thread3( [] {
        // printf("Hello forom lambda function - \n", std::this_thread::get_id());
    std::cout<<"Hello from lambda function "<< std::this_thread::get_id()<<std::endl;
    });

    thread1.join();
    thread2.join();
    thread3.join();



    // printf("Hello from main - \n", std::this_thread::get_id()); 
    std::cout<<"Hello from main "<< std::this_thread::get_id()<<std::endl;
    return 0;
}