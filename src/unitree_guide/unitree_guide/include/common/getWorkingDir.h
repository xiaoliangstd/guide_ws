#ifndef GETUSER_H
#define GETUSER_H

/*
https://blog.csdn.net/taiyang1987912/article/details/79303296
C++ 获取linux和windows系统的用户名
*/

#include <iostream>
#include <string>
#include <unistd.h>

inline std::string getWorkingDir()
{
    char currentPath[256];
    if (getcwd(currentPath, sizeof(currentPath)) != nullptr)
    {
    }
    else
    {
        perror("Error getting working dir");
    }
    std::string currentPathStr(currentPath);
    return currentPathStr;
}

#endif // BIANLIB_GETUSER_H