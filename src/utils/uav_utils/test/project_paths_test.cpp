#include <uav_utils/project_paths.h>

#include <chrono>
#include <fstream>
#include <iostream>

int main()
{
  namespace fs = std::filesystem;
  const auto temporary = fs::temp_directory_path() / (
    "controller-path-test-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto original = temporary / "original";
  const auto root = temporary / "renamed project";
  const auto cwd = fs::current_path();
  int result = 0;
  try {
    fs::create_directories(original / "src/realflight_modules/px4ctrl");
    std::ofstream(original / "src/realflight_modules/px4ctrl/package.xml") << "<package/>";
    fs::rename(original, root);
    fs::current_path(temporary);
    for (const auto *layout : {"install/px4ctrl/share/px4ctrl", "install/share/px4ctrl",
                              "src/realflight_modules/px4ctrl"}) {
      const auto anchor = root / layout;
      fs::create_directories(anchor);
      for (const auto *path : {"datalog/omtraj/omtraj_optimized.csv", "datalog/flightlog"}) {
        if (uav_utils::projectPath(path, anchor) != root / path) {
          throw std::runtime_error("relative path did not follow relocated workspace");
        }
      }
    }
    const auto link = root / "install/source_share";
    fs::create_directory_symlink(root / "src/realflight_modules/px4ctrl", link);
    if (uav_utils::projectRoot(link) != root) throw std::runtime_error("symlink anchor failed");
    if (uav_utils::projectPath(temporary / "absolute", temporary) != temporary / "absolute") {
      throw std::runtime_error("absolute override failed");
    }
    fs::current_path(root);
    bool rejected = false;
    try {
      uav_utils::projectPath("datalog/omtraj/omtraj_optimized.csv", temporary);
    } catch (const std::runtime_error &) {
      rejected = true;
    }
    if (!rejected) throw std::runtime_error("unrelated cwd was used as workspace");
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  fs::current_path(cwd);
  fs::remove_all(temporary);
  return result;
}
