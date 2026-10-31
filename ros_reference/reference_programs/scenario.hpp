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

  // Reads the file at `path`, relative to the repository root, through `read`, and gives
  // its file:// URI, by which a ROS library, MoveIt loading a mesh among them, reads it.
  std::string file_uri(const std::string& path);

  // Appends the path of each input, one a line, to the file at `list`.
  void append_to(const std::filesystem::path& list) const;

 private:
  std::filesystem::path repository_root_;
  std::vector<std::string> paths_;
};

// Writes `document` to `path`, with a final newline. A scalar, an array of scalars and a
// record, an object whose members are each a scalar or an array of scalars, go on one
// line; every other array or object spreads over lines, one member a line, indented by
// two spaces. Every float goes as the shortest decimal that reads back to its bits, as
// std::to_chars writes it, a JSON number, so a reference output carries each number as
// the reference's own double. Throws on a NaN or an infinity, which JSON has no number
// for.
void write_json(const std::filesystem::path& path, const nlohmann::json& document);

// Throws unless the writer of a float of write_json writes each double at the edges of
// the format as the decimal that the reader test of the ros_reference crate reads back
// to it, writes each of a seeded draw of bit patterns as a JSON number that reads back to
// its bits, and refuses each NaN and infinity.
void check_shortest_decimals();

}  // namespace ros_reference
