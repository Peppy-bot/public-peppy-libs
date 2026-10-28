#include "scenario.hpp"

#include <bit>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ros_reference {

Inputs::Inputs(std::filesystem::path repository_root)
    : repository_root_(std::move(repository_root)) {}

std::string Inputs::read(const std::string& path) {
  std::ifstream file(repository_root_ / path);
  if (!file) {
    throw std::runtime_error("cannot read " + path);
  }
  std::ostringstream text;
  text << file.rdbuf();
  paths_.push_back(path);
  return text.str();
}

void Inputs::append_to(const std::filesystem::path& list) const {
  std::ofstream file(list, std::ios::app);
  for (const auto& path : paths_) {
    file << path << '\n';
  }
  file.close();
  if (!file) {
    throw std::runtime_error("cannot write " + list.string());
  }
}

void write_json(const std::filesystem::path& path, const nlohmann::json& document) {
  std::ofstream file(path);
  file << document.dump(2) << '\n';
  file.close();
  if (!file) {
    throw std::runtime_error("cannot write " + path.string());
  }
}

std::string bits(double x) {
  char text[19];
  std::snprintf(text, sizeof text, "0x%016" PRIx64, std::bit_cast<std::uint64_t>(x));
  return text;
}

}  // namespace ros_reference
