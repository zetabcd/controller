#pragma once

#include <filesystem>
#include <stdexcept>

namespace uav_utils {

// anchor is the current package's share directory, never the shell's cwd.
inline std::filesystem::path projectRoot(const std::filesystem::path &anchor)
{
  auto candidate = std::filesystem::weakly_canonical(anchor);
  while (!candidate.empty()) {
    if (std::filesystem::is_regular_file(
        candidate / "src/realflight_modules/px4ctrl/package.xml")) {
      return candidate;
    }
    const auto parent = candidate.parent_path();
    if (parent == candidate) break;
    candidate = parent;
  }
  throw std::runtime_error(
    "Cannot locate the controller workspace above " + anchor.string() +
    "; keep install/ inside the workspace or configure an absolute data path");
}

inline std::filesystem::path projectPath(
  const std::filesystem::path &value, const std::filesystem::path &anchor)
{
  return std::filesystem::weakly_canonical(
    value.is_absolute() ? value : projectRoot(anchor) / value);
}

}  // namespace uav_utils
