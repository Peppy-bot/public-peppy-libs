#include "scenario.hpp"

#include <algorithm>
#include <bit>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <ostream>
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

namespace {

// Whether `value` is a scalar or an array of scalars.
bool is_flat(const nlohmann::json& value) {
  return value.is_primitive() ||
         (value.is_array() &&
          std::all_of(value.begin(), value.end(),
                      [](const nlohmann::json& member) { return member.is_primitive(); }));
}

// Whether `value` goes on one line: a scalar, an array of scalars, or a record, an object
// whose members are each a scalar or an array of scalars.
bool is_one_line(const nlohmann::json& value) {
  return value.is_object() ? std::all_of(value.begin(), value.end(), is_flat) : is_flat(value);
}

// Writes `value` indented by `indent`: a value of one line on its line, and every other
// array or object over several lines, one member a line.
void write_value(std::ostream& out, const nlohmann::json& value, const std::string& indent) {
  if (is_one_line(value)) {
    out << value.dump();
    return;
  }
  const auto inner = indent + "  ";
  const bool object = value.is_object();
  out << (object ? "{\n" : "[\n");
  bool first = true;
  for (const auto& [key, member] : value.items()) {
    out << (first ? "" : ",\n") << inner;
    if (object) {
      out << nlohmann::json(key).dump() << ": ";
    }
    write_value(out, member, inner);
    first = false;
  }
  out << '\n' << indent << (object ? '}' : ']');
}

}  // namespace

void write_json(const std::filesystem::path& path, const nlohmann::json& document) {
  std::ofstream file(path);
  write_value(file, document, "");
  file << '\n';
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
