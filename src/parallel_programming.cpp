#include <iostream>
#include <thread>

void foo()
{
    printf("Hello from foo \n");
}

int main(int argc, char **argv)
{
    std::thread thread1(foo);
    thread1.join();

    printf("Hello from main \n"); 
    return 0;
}