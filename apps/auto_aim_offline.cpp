#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "io/command.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

using json = nlohmann::json;

namespace
{
struct Arguments
{
  std::filesystem::path dataset;
  std::filesystem::path output;
  std::filesystem::path auto_config;
};

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments args;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    if (key == "--dataset") args.dataset = argv[i + 1];
    else if (key == "--output") args.output = argv[i + 1];
    else if (key == "--auto-config") args.auto_config = argv[i + 1];
    else throw std::runtime_error("unknown argument: " + key);
  }
  if (args.dataset.empty() || args.output.empty() || args.auto_config.empty()) {
    throw std::runtime_error("usage: auto_aim_offline --dataset PATH --output PATH --auto-config PATH");
  }
  return args;
}

std::vector<json> read_jsonl(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open " + path.string());
  std::vector<json> rows;
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty()) rows.push_back(json::parse(line));
  }
  return rows;
}

void write_jsonl_row(std::ofstream & stream, const json & row) { stream << row.dump() << '\n'; }

std::vector<double> eigen_vector(const Eigen::VectorXd & value)
{
  return std::vector<double>(value.data(), value.data() + value.size());
}

std::vector<double> eigen_vector4(const Eigen::Vector4d & value)
{
  return {value[0], value[1], value[2], value[3]};
}

std::string frame_key(std::int64_t frame_id) { return std::to_string(frame_id); }

auto_aim::Armor make_armor(const json & row, int image_width, int image_height)
{
  std::vector<cv::Point2f> points;
  for (const auto & point : row.at("corners_px")) {
    points.emplace_back(point[0].get<float>(), point[1].get<float>());
  }
  const auto & bbox = row.at("bbox_xywh");
  cv::Rect rect{
    static_cast<int>(std::floor(bbox[0].get<double>())),
    static_cast<int>(std::floor(bbox[1].get<double>())),
    std::max(1, static_cast<int>(std::ceil(bbox[2].get<double>()))),
    std::max(1, static_cast<int>(std::ceil(bbox[3].get<double>()))) };
  auto_aim::Armor armor(row.at("class_id").get<int>(), row.at("confidence").get<float>(), rect, points);
  armor.center_norm = {
    armor.center.x / static_cast<float>(image_width), armor.center.y / static_cast<float>(image_height)};
  armor.priority = auto_aim::ArmorPriority::first;
  armor.duplicated = false;
  return armor;
}
}  // namespace

int main(int argc, char ** argv)
try {
  const auto args = parse_arguments(argc, argv);
  std::filesystem::create_directories(args.output);
  std::filesystem::create_directories("logs");
  std::ifstream metadata_stream(args.dataset / "metadata.yaml");
  if (!metadata_stream) throw std::runtime_error("cannot open metadata.yaml");
  json metadata;
  metadata_stream >> metadata;
  const int image_width = metadata["camera"]["image_width"].get<int>();
  const int image_height = metadata["camera"]["image_height"].get<int>();
  const auto frames = read_jsonl(args.dataset / "frames.jsonl");
  const auto observation_rows = read_jsonl(args.dataset / "observations.jsonl");
  std::unordered_map<std::string, json> observations;
  for (const auto & row : observation_rows) {
    observations.emplace(frame_key(row["frame_id"].get<std::int64_t>()), row);
  }

  auto_aim::Solver solver(args.auto_config.string());
  auto_aim::Tracker tracker(args.auto_config.string(), solver, 1);  // current project: 1 selects blue
  auto_aim::Aimer aimer(args.auto_config.string());
  auto_aim::Shooter shooter(args.auto_config.string());
  std::ofstream algorithm_stream(args.output / "algorithm_output.jsonl");
  std::ofstream solver_stream(args.output / "solver_output.jsonl");
  std::ofstream shot_stream(args.output / "shots.jsonl");
  io::Command previous_command{false, false, 0.0, 0.0};
  std::int64_t shot_id = 0;

  for (const auto & frame : frames) {
    const auto frame_id = frame["frame_id"].get<std::int64_t>();
    const auto timestamp_ns = frame["timestamp_ns"].get<std::int64_t>();
    const auto timestamp = std::chrono::steady_clock::time_point{std::chrono::nanoseconds(timestamp_ns)};
    const auto & q = frame["imu_q_wxyz"];
    solver.set_R_gimbal2world(Eigen::Quaterniond{
      q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()});
    std::list<auto_aim::Armor> armors;
    const auto observation_it = observations.find(frame_key(frame_id));
    if (observation_it != observations.end()) {
      for (const auto & observation : observation_it->second["armors"]) {
        if (!observation.value("valid", true)) continue;
        // Consume the recorded observation exactly as supplied. Ground Truth is
        // deliberately unavailable to this executable.
        auto armor = make_armor(observation, image_width, image_height);
        auto solver_copy = armor;
        solver.solve(solver_copy);
        write_jsonl_row(
          solver_stream,
          {{"frame_id", frame_id}, {"timestamp_ns", timestamp_ns},
           {"observation_id", observation["observation_id"]},
           {"target_hint_id", observation["target_hint_id"]},
           {"armor_hint_id", observation["armor_hint_id"]},
           {"position", {solver_copy.xyz_in_world[0], solver_copy.xyz_in_world[1], solver_copy.xyz_in_world[2]}},
           {"yaw", solver_copy.ypr_in_world[0]}, {"valid", solver_copy.xyz_in_world.allFinite()}});
        armors.push_back(std::move(armor));
      }
    }

    const auto start = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, timestamp);
    const auto after_tracker = std::chrono::steady_clock::now();
    auto command = aimer.aim(targets, timestamp, frame["bullet_speed"].get<double>(), io::both_shoot, false);
    const auto after_aimer = std::chrono::steady_clock::now();
    const Eigen::Vector3d simulated_gimbal{previous_command.yaw, previous_command.pitch, 0.0};
    command.shoot = shooter.shoot(command, aimer, targets, simulated_gimbal);
    const auto finish = std::chrono::steady_clock::now();
    if (command.control) previous_command = command;

    json tracker_json = {
      {"state", tracker.state()}, {"has_target", !targets.empty()},
      {"last_armor_id", targets.empty() ? json(nullptr) : json(targets.front().last_id)},
      {"ekf_state", targets.empty() ? json(nullptr) : json(eigen_vector(targets.front().ekf_x()))},
      {"nis", targets.empty() ? json(nullptr) : json(targets.front().ekf().data.at("nis"))},
      {"nees_internal", targets.empty() ? json(nullptr) : json(targets.front().ekf().data.at("nees"))}};
    const bool aimer_valid = command.control && !targets.empty();
    json aimer_json = {
      {"valid", aimer_valid},
      {"aim_xyza", aimer_valid ? json(eigen_vector4(aimer.debug_aim_point.xyza)) : json(nullptr)},
      {"target_id", !targets.empty() ? json(1) : json(nullptr)},
      {"armor_id", aimer_valid ? json(aimer.debug_aim_point.armor_id) : json(nullptr)},
      {"tracker_last_armor_id", !targets.empty() ? json(targets.front().last_id) : json(nullptr)},
      {"delay_time_s", aimer_valid ? json(aimer.debug_delay_time) : json(nullptr)},
      {"flight_time_s", aimer_valid ? json(aimer.debug_fly_time) : json(nullptr)},
      {"impact_timestamp_ns", nullptr}};
    if (aimer_valid) {
      const auto prediction_ns = static_cast<std::int64_t>(
        std::llround(aimer.debug_prediction_time * 1e9));
      aimer_json["impact_timestamp_ns"] = timestamp_ns + prediction_ns;
    }
    auto milliseconds = [](auto end, auto begin) {
      return std::chrono::duration<double, std::milli>(end - begin).count();
    };
    write_jsonl_row(
      algorithm_stream,
      {{"frame_id", frame_id}, {"timestamp_ns", timestamp_ns}, {"tracker", tracker_json},
       {"aimer", aimer_json},
       {"command", {{"control", command.control}, {"shoot", command.shoot}, {"yaw", command.yaw}, {"pitch", command.pitch}}},
       {"timing_ms", {{"solver_tracker", milliseconds(after_tracker, start)}, {"aimer", milliseconds(after_aimer, after_tracker)}, {"shooter", milliseconds(finish, after_aimer)}, {"total", milliseconds(finish, start)}}},
       {"backend", "cpp"}});
    if (command.shoot) {
      // Python evaluator/hit model fills hit semantics once future GT is available.
      write_jsonl_row(
        shot_stream,
        {{"shot_id", shot_id++}, {"frame_id", frame_id},
         {"command_timestamp_ns", timestamp_ns},
         {"muzzle_timestamp_ns", timestamp_ns + static_cast<std::int64_t>(std::llround(aimer.debug_delay_time * 1e9))},
         {"impact_timestamp_ns", aimer_json["impact_timestamp_ns"]},
         {"bullet_speed", frame["bullet_speed"]}, {"command_yaw", command.yaw}, {"command_pitch", command.pitch},
         {"target_id", 1}, {"intended_armor_id", aimer_json["armor_id"]},
         {"hit", false}, {"hit_armor_id", nullptr}, {"miss_distance", 1e9},
         {"model", "pending_python_hit_check"}});
    }
  }
  return 0;
} catch (const std::exception & error) {
  std::cerr << "auto_aim_offline: " << error.what() << '\n';
  return 1;
}
