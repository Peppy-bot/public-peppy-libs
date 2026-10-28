// The reference program of the ROS reference harness. It reads the scenario at argv[1],
// runs the reference that the scenario's kind names, writes the reference output to
// argv[2], and appends the path of each file it read, the scenario among them, to the
// list at argv[4]. The scenario's path and the paths inside it are relative to the
// repository root, argv[3].

#include <exception>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>

#include <nlohmann/json.hpp>

#include "forward_kinematics.hpp"
#include "scenario.hpp"

namespace {

using Reference = nlohmann::json (*)(const nlohmann::json&, ros_reference::Inputs&);

const std::map<std::string, Reference> references = {
    {"forward_kinematics", ros_reference::forward_kinematics},
};

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: ros_reference <scenario.json> <output.json> <repository root> "
                 "<inputs list>\n";
    return 2;
  }
  try {
    ros_reference::Inputs inputs(argv[3]);
    const auto scenario = nlohmann::json::parse(inputs.read(argv[1]));
    const auto kind = scenario.at("kind").get<std::string>();
    const auto reference = references.find(kind);
    if (reference == references.end()) {
      std::cerr << argv[1] << ": no reference runs the kind " << kind << '\n';
      return 2;
    }
    ros_reference::write_json(argv[2], reference->second(scenario, inputs));
    inputs.append_to(argv[4]);
  } catch (const std::exception& error) {
    std::cerr << argv[1] << ": " << error.what() << '\n';
    return 1;
  }
  return 0;
}
