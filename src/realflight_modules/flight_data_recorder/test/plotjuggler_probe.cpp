// Exercise the installed, unmodified PlotJuggler ULog plugin (optional check).
// Usage: plotjuggler_probe <libDataLoadULog.so> <log.ulg>
#include <QApplication>
#include <QPluginLoader>
#include <PlotJuggler/dataloader_base.h>
#include <cmath>
#include <array>
#include <iostream>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  if (argc != 3) return 2;
  QPluginLoader plugin(argv[1]);
  auto loader = qobject_cast<PJ::DataLoader*>(plugin.instance());
  if (!loader)
  {
    std::cerr << plugin.errorString().toStdString() << '\n';
    return 3;
  }
  PJ::FileLoadInfo info;
  info.filename = argv[2];
  PJ::PlotDataMapRef destination;
  if (!loader->readDataFromFile(&info, destination)) return 4;
  bool debug = false, late = false;
  std::array<bool, 12> motors{};
  for (const auto& entry : destination.numeric)
  {
    const auto& name = entry.first;
    const auto& series = entry.second;
    if (name.find("ros/debugPx4/ctrl") != std::string::npos &&
        name.find("msg_thr2acc") != std::string::npos)
    {
      debug = series.size() == 400;
      for (size_t i = 0; i < series.size(); ++i)
        debug = debug && series.at(i).y == 1.0 + double(i) / 1024.0;
      std::cout << name << " samples=" << series.size() << '\n';
    }
    for (size_t channel = 0; channel < motors.size(); ++channel)
    {
      const auto suffix = std::string("msg_control.") + (channel < 10 ? "0" : "") +
                          std::to_string(channel);
      if (name.find("ros/fmu/in/actuator_motors/" + suffix) != std::string::npos)
      {
        motors[channel] = series.size() == 400;
        for (size_t i = 0; i < series.size(); ++i)
          motors[channel] = motors[channel] &&
                            series.at(i).y == double(i) / 1024.0 + double(channel) / 32.0;
      }
    }
    if (name.find("ros/recorder_test/late") != std::string::npos &&
        name.find("msg_data") != std::string::npos)
      late = series.size() == 1 && series.at(0).y == 3.25;
  }
  const auto valid_motors = std::count(motors.begin(), motors.end(), true);
  std::cout << "numeric series=" << destination.numeric.size()
            << " thr2acc=" << debug << " motor_channels=" << valid_motors
            << " dynamic_topic=" << late << '\n';
  return debug && valid_motors == 12 && late ? 0 : 5;
}
