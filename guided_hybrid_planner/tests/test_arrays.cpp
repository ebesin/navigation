/*
 * @Author       : dwayne
 * @Date         : 2023-05-25
 * @LastEditTime : 2023-05-25
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include <iostream>

void test(int **array){
    array[0][0] = 1;
}

int main(){
    int **array = new int*[2];
    for(int i=0;i<2;i++){
        array[i] = new int[2];
    }
    test(array);
    std::cout<<"array[0][0]--->"<<array[0][0]<<std::endl;
}