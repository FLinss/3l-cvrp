#pragma once

#include "ContainerLoading/Model/Container.h"

#include <utility>
#include <vector>

namespace VehicleRouting
{
namespace Model
{

class Vehicle
{
  public:
    int InternId = 0;
    double Volume = 0.0;
    std::vector<ContainerLoading::Model::Container> Containers;

    Vehicle() = default;
    Vehicle(int id, std::vector<ContainerLoading::Model::Container> containers) : InternId(id), Containers(std::move(containers)) {}
};

}
}