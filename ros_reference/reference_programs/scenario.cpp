#include "scenario.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <ostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
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

std::string Inputs::file_uri(const std::string& path) {
  read(path);
  return "file://" + std::filesystem::absolute(repository_root_ / path).string();
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

// The shortest decimal that reads back to the bits of `x`, as std::to_chars writes it, a
// JSON number. Throws on a NaN or an infinity, which JSON has no number for, and on a
// decimal that std::from_chars reads back to other bits.
std::string shortest_decimal(double x) {
  if (!std::isfinite(x)) {
    throw std::domain_error("a reference output holds the non-finite value " +
                            std::to_string(x) + ", which JSON has no number for");
  }
  // The longest shortest decimal of a double: a sign, 17 digits, a point, and an exponent
  // of a sign and 3 digits, as -2.2250738585072014e-308.
  constexpr std::size_t longest = 24;
  std::array<char, longest> text;
  const auto written = std::to_chars(text.data(), text.data() + text.size(), x);
  if (written.ec != std::errc()) {
    throw std::runtime_error("std::to_chars writes the double " + std::to_string(x) +
                             " in more than " + std::to_string(longest) + " characters");
  }
  const std::string decimal(text.data(), written.ptr);
  double read = 0.0;
  const auto parsed = std::from_chars(decimal.data(), decimal.data() + decimal.size(), read);
  if (parsed.ec != std::errc() || parsed.ptr != decimal.data() + decimal.size() ||
      std::bit_cast<std::uint64_t>(read) != std::bit_cast<std::uint64_t>(x)) {
    throw std::runtime_error("the decimal " + decimal + " does not read back to its double");
  }
  return decimal;
}

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

// Writes `value` on one line with no spaces, as nlohmann::json dumps it, each float as its
// shortest decimal.
void write_one_line(std::ostream& out, const nlohmann::json& value) {
  if (value.is_number_float()) {
    out << shortest_decimal(value.get<double>());
    return;
  }
  if (value.is_primitive()) {
    out << value.dump();
    return;
  }
  const bool object = value.is_object();
  out << (object ? '{' : '[');
  bool first = true;
  for (const auto& [key, member] : value.items()) {
    out << (first ? "" : ",");
    if (object) {
      out << nlohmann::json(key).dump() << ':';
    }
    write_one_line(out, member);
    first = false;
  }
  out << (object ? '}' : ']');
}

// Writes `value` indented by `indent`: a value of one line on its line, and every other
// array or object over several lines, one member a line.
void write_value(std::ostream& out, const nlohmann::json& value, const std::string& indent) {
  if (is_one_line(value)) {
    write_one_line(out, value);
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

namespace {

// Doubles at the edges of the format, by their bits, and the decimal that std::to_chars
// writes for each: the smallest and the largest subnormal, the smallest normal, the
// largest finite value and its negation, and both zeros. The reader test of the
// ros_reference crate reads each decimal back to its bits.
constexpr std::array<std::pair<std::uint64_t, std::string_view>, 7> edge_decimals = {{
    {0x0000000000000001, "5e-324"},
    {0x000fffffffffffff, "2.225073858507201e-308"},
    {0x0010000000000000, "2.2250738585072014e-308"},
    {0x7fefffffffffffff, "1.7976931348623157e+308"},
    {0xffefffffffffffff, "-1.7976931348623157e+308"},
    {0x0000000000000000, "0"},
    {0x8000000000000000, "-0"},
}};

// The seed and the size of the draw of bit patterns that check_shortest_decimals writes.
constexpr std::uint64_t drawn_patterns_seed = 20261031;
constexpr int drawn_patterns = 1 << 16;

// Throws unless shortest_decimal refuses `x`.
void require_refused(double x) {
  try {
    shortest_decimal(x);
  } catch (const std::domain_error&) {
    return;
  }
  throw std::runtime_error("shortest_decimal writes the non-finite value " + std::to_string(x));
}

}  // namespace

void check_shortest_decimals() {
  for (const auto& [pattern, decimal] : edge_decimals) {
    const auto written = shortest_decimal(std::bit_cast<double>(pattern));
    if (written != decimal) {
      throw std::runtime_error("std::to_chars writes " + written + " for the double of " +
                               std::string(decimal));
    }
  }
  std::mt19937_64 draws(drawn_patterns_seed);
  for (int i = 0; i < drawn_patterns; ++i) {
    const auto x = std::bit_cast<double>(draws());
    if (!std::isfinite(x)) {
      require_refused(x);
      continue;
    }
    const auto decimal = shortest_decimal(x);
    if (!nlohmann::json::parse(decimal).is_number()) {
      throw std::runtime_error("the decimal " + decimal + " is not a JSON number");
    }
  }
  for (const double x : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity()}) {
    require_refused(x);
  }
}

}  // namespace ros_reference
