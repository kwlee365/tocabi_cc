#pragma once

#include<string>

enum class TaskType { Position, Orientation };

struct TaskInfo {
    std::string link_name;
    TaskType type;
};

enum class ContactIndicator {
    DoubleSupport,
    LeftSingleSupport,
    RightSingleSupport
};
