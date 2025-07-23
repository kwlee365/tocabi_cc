#pragma once

#include<string>

enum class TaskType { Position, Orientation };

struct TaskInfo {
    std::string link_name;
    TaskType type;
};