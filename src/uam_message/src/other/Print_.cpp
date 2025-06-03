#include "other/Print_.h"
void Print_green(std::string str){
    std::cout<<"\033[32m"<<str<<"\033[0m"<<std::endl;
}
void Print_red(std::string str){
    std::cout<<"\033[31m"<<str<<"\033[0m"<<std::endl;
}
void Print_pink(std::string str){
    std::cout<<"\033[35m"<<str<<"\033[0m"<<std::endl;
}