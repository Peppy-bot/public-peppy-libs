// Reading the inputs of a reference and writing its output: what every reference of the
// harness shares.
#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace ros_reference {

// The files of the repository that a reference reads, its inputs, by their paths from
// the repository root.
class Inputs {
 public:
  explicit Inputs(std::filesystem::path repository_root);

  // The text of the file at `path`, relative to the repository root, which becomes an
  // input.
  std::string read(const std::string& path);

  // Appends the path of each input, one a line, to the file at `list`.
  void append_to(const std::filesystem::path& list) const;

 private:
  std::filesystem::path repository_root_;
  std::vector<std::string> paths_;
};

// Writes `document` to `path`, indented by two spaces, with a final newline.
void write_json(const std::filesystem::path& path, const nlohmann::json& document);

// The IEEE-754 bit pattern of `x` as 0x and 16 hexadecimal digits: the form in which a
// reference output carries every number, so a parity test reads the reference's bits.
std::string bits(double x);

}  // namespace ros_reference
