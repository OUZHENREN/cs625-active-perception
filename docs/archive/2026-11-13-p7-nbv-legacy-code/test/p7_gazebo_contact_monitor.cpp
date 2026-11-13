// Low-overhead Gazebo contact counter for P7 view and grasp phases.
#include <gz/msgs/contacts.pb.h>
#include <gz/transport/Node.hh>

#include <chrono>
#include <csignal>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void Stop(int) { stop_requested = 1; }
std::string JsonEscape(const std::string &value) {
  std::ostringstream out;
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') out << '\\';
    out << ch;
  }
  return out.str();
}

std::string UtcNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t stamp = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&stamp, &utc);
  std::ostringstream out;
  out << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return out.str();
}

bool IsRobot(const std::string &name) { return name.rfind("cs::", 0) == 0; }
int FingerTarget(const std::pair<std::string, std::string> &pair) {
  const std::string &robot = IsRobot(pair.first) ? pair.first : pair.second;
  const std::string &other = IsRobot(pair.first) ? pair.second : pair.first;
  if (other.rfind("target_object::", 0) != 0) return 0;
  if (robot.rfind("cs::gripper_left_finger_link::", 0) == 0) return 1;
  if (robot.rfind("cs::gripper_right_finger_link::", 0) == 0) return 2;
  return 0;
}
bool Unexpected(const std::pair<std::string, std::string> &pair, bool grasp_mode) {
  if (!IsRobot(pair.first) && !IsRobot(pair.second)) return false;
  const bool support =
      (pair.first.find("cs::base_link::") == 0 && pair.second.find("ground_plane::") == 0) ||
      (pair.second.find("cs::base_link::") == 0 && pair.first.find("ground_plane::") == 0);
  return !support && !(grasp_mode && FingerTarget(pair));
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 3 || argc > 4) {
    std::cerr << "usage: p7_gazebo_contact_monitor DURATION_SEC OUTPUT_JSON [view|grasp]\n";
    return 64;
  }
  const double duration_sec = std::stod(argv[1]);
  const std::string output_path = argv[2];
  const std::string mode = argc == 4 ? argv[3] : "view";
  if (mode != "view" && mode != "grasp") return 64;
  const bool grasp_mode = mode == "grasp";
  if (duration_sec <= 0.0) return 64;

  const std::vector<std::string> topics = {
      "/p7/contacts/ground", "/p7/contacts/occluder", "/p7/contacts/target"};
  std::mutex mutex;
  std::map<std::string, std::size_t> message_counts;
  std::map<std::string, double> first_stamps, last_stamps;
  std::map<std::pair<std::string, std::string>, std::size_t> pair_counts;
  std::map<std::pair<std::string, std::string>, double> pair_first_stamps, pair_last_stamps;
  std::size_t bilateral_contact_messages = 0;
  double bilateral_first_stamp = 0.0, bilateral_last_stamp = 0.0;
  gz::transport::Node node;
  std::vector<std::string> advertised_topics;
  node.TopicList(advertised_topics);
  const std::set<std::string> advertised(advertised_topics.begin(), advertised_topics.end());
  for (const auto &topic : topics) {
    if (!advertised.count(topic)) {
      std::cerr << "contact topic unavailable: " << topic << '\n';
      return 2;
    }
  }
  bool subscribed = true;
  for (const auto &topic : topics) {
    message_counts[topic] = 0;
    std::function<void(const gz::msgs::Contacts &)> callback =
        [&, topic](const gz::msgs::Contacts &message) {
          std::lock_guard<std::mutex> lock(mutex);
          ++message_counts[topic];
          const double stamp = message.header().stamp().sec() + message.header().stamp().nsec() * 1e-9;
          if (!first_stamps.count(topic)) first_stamps[topic] = stamp;
          last_stamps[topic] = stamp;
          int finger_mask = 0;
          for (const auto &contact : message.contact()) {
            std::string first = contact.collision1().name();
            std::string second = contact.collision2().name();
            if (second < first) std::swap(first, second);
            const auto pair = std::make_pair(first, second);
            ++pair_counts[pair];
            if (!pair_first_stamps.count(pair)) pair_first_stamps[pair] = stamp;
            pair_last_stamps[pair] = stamp;
            finger_mask |= FingerTarget(pair);
          }
          if (topic == "/p7/contacts/target" && finger_mask == 3) {
            if (bilateral_contact_messages == 0) bilateral_first_stamp = stamp;
            ++bilateral_contact_messages;
            bilateral_last_stamp = stamp;
          }
        };
    const bool ok = node.Subscribe<gz::msgs::Contacts>(topic, callback);
    subscribed = subscribed && ok;
  }
  if (!subscribed) {
    std::cerr << "failed to subscribe to one or more contact topics\n";
    return 2;
  }

  const std::string started_at_utc = UtcNow();
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  const auto started = std::chrono::steady_clock::now();
  bool ready = false;
  while (std::chrono::steady_clock::now() - started < std::chrono::seconds(8)) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      ready = message_counts[topics[0]] > 0;
    }
    if (ready) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!ready) {
    std::cerr << "ground contact heartbeat absent\n";
    return 2;
  }
  std::cout << "P7_CONTACT_MONITOR_READY" << std::endl;
  while (!stop_requested && std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() < duration_sec)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();

  std::lock_guard<std::mutex> lock(mutex);
  bool unexpected = false;
  for (const auto &[pair, count] : pair_counts) {
    (void)count;
    unexpected = unexpected || Unexpected(pair, grasp_mode);
  }
  std::ofstream output(output_path);
  if (!output) {
    std::cerr << "cannot open output: " << output_path << '\n';
    return 3;
  }
  output << "{\n"
         << "  \"schema\": \"p7_gazebo_contact_capture_v1\",\n"
         << "  \"captured_at_utc\": \"" << UtcNow() << "\",\n"
         << "  \"started_at_utc\": \"" << started_at_utc << "\",\n"
         << "  \"duration_sec\": " << elapsed << ",\n"
         << "  \"monitor_available\": true,\n"
         << "  \"monitor_implementation\": \"native_gz_transport_counter\",\n"
         << "  \"phase_policy\": \"" << mode << "\",\n"
         << "  \"finger_target_contact_allowed\": " << (grasp_mode ? "true" : "false") << ",\n"
         << "  \"classification_scope\": \"robot contacts with ground, occluder and target in severe-v5; pedestal-ground support allowed; self-contact is outside this sensor scope\",\n"
         << "  \"topics\": {\n";
  for (std::size_t index = 0; index < topics.size(); ++index) {
    const auto &topic = topics[index];
    output << "    \"" << JsonEscape(topic) << "\": {\"message_count\": "
           << message_counts[topic] << ", \"first_sim_stamp_sec\": " << std::setprecision(12)
           << first_stamps[topic] << ", \"last_sim_stamp_sec\": " << last_stamps[topic] << "}";
    output << (index + 1 == topics.size() ? "\n" : ",\n");
  }
  output << "  },\n  \"collision_pairs\": [\n";
  std::size_t pair_index = 0;
  for (const auto &[pair, count] : pair_counts) {
    const bool robot_contact = Unexpected(pair, grasp_mode);
    output << "    {\"collision_pair\": [\"" << JsonEscape(pair.first)
           << "\", \"" << JsonEscape(pair.second) << "\"], "
           << "\"record_count\": " << count << ", "
           << "\"first_sim_stamp_sec\": " << pair_first_stamps[pair] << ", "
           << "\"last_sim_stamp_sec\": " << pair_last_stamps[pair] << ", "
           << "\"finger_target_contact\": " << (FingerTarget(pair) ? "true" : "false") << ", "
           << "\"unexpected_robot_contact\": "
           << (robot_contact ? "true" : "false") << "}";
    output << (++pair_index == pair_counts.size() ? "\n" : ",\n");
  }
  output << "  ],\n"
         << "  \"bilateral_finger_contact\": {\"observed\": "
         << (bilateral_contact_messages ? "true" : "false")
         << ", \"simultaneous_message_count\": " << bilateral_contact_messages
         << ", \"first_sim_stamp_sec\": " << bilateral_first_stamp
         << ", \"last_sim_stamp_sec\": " << bilateral_last_stamp << "},\n"
         << "  \"unexpected_collision\": "
         << (unexpected ? "true" : "false") << ",\n"
         << "  \"claim_boundary\": \"Gazebo physical-contact observation only; MoveIt collision rejection is reported separately\"\n"
         << "}\n";
  output.close();
  std::cout << "P7_CONTACT_MONITOR_DONE unexpected_collision="
            << (unexpected ? "true" : "false") << std::endl;
  return unexpected ? 1 : 0;
}
