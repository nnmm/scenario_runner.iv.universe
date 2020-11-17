#ifndef SCENARIO_RUNNER_SCENARIO_TERMINATOR_H_INCLUDEED
#define SCENARIO_RUNNER_SCENARIO_TERMINATOR_H_INCLUDEED

#include <rclcpp/rclcpp.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace scenario_runner
{
class ScenarioTerminator
{
public:
  ScenarioTerminator(const char * host, int port);

  void sendTerminateRequest(int);

  void update_mileage(double = 0);
  void update_duration(double = 0);

private:
  XmlRpc::XmlRpcClient client_;
};
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER_SCENARIO_TERMINATOR_H_INCLUDEED
