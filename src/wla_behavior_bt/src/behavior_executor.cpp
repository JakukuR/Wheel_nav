// Application-level BT. All callbacks and ticks run on one ROS executor thread.
// Navigation terminal includes confirmed stop, enforced by the gateway.
#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <stdexcept>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <wla_behavior_interfaces/action/behavior_task.hpp>
#include <wla_behavior_interfaces/srv/resolve_target.hpp>
#include <wla_behavior_interfaces/msg/task_lease.hpp>

using Task = wla_behavior_interfaces::action::BehaviorTask;
using Resolve = wla_behavior_interfaces::srv::ResolveTarget;
using ServerHandle = rclcpp_action::ServerGoalHandle<Task>;
using ClientHandle = rclcpp_action::ClientGoalHandle<Task>;
using Clock = std::chrono::steady_clock;

struct Session {
  Task::Goal goal;
  bool cancel = false;
  bool fault = false;
  std::string reason;
  std::string state = "resolving";
  float remaining = -1;
};

class ResolveBoundTarget : public BT::StatefulActionNode {
public:
  ResolveBoundTarget(const std::string& name, const BT::NodeConfig& cfg,
                    rclcpp::Node::SharedPtr node, std::shared_ptr<Session>& session)
    : StatefulActionNode(name, cfg), session_(session) {
    client_ = node->create_client<Resolve>("/r680_nav/resolve_target");
  }
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    current_ = session_; done_ = false; ok_ = false;
    deadline_ = Clock::now() + std::chrono::seconds(12);
    if (!client_->service_is_ready()) {
      current_->reason = "resolver_unavailable"; return BT::NodeStatus::FAILURE;
    }
    auto req = std::make_shared<Resolve::Request>();
    req->map_id = current_->goal.map_id; req->feature_id = current_->goal.feature_id;
    auto captured = current_;
    client_->async_send_request(req, [this, captured](rclcpp::Client<Resolve>::SharedFuture f) {
      if (current_ != captured) return;
      try {
        auto r = f.get();
        ok_ = r->success && r->map_revision == captured->goal.map_revision &&
              r->target_fingerprint == captured->goal.target_fingerprint;
        if (!ok_) captured->reason = r->reason.empty() ? "target_binding_changed" : r->reason;
      } catch (const std::exception& e) { captured->reason = e.what(); ok_ = false; }
      done_ = true;
    });
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (current_->cancel) return BT::NodeStatus::FAILURE;
    if (done_) return ok_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    if (Clock::now() >= deadline_) {
      current_->reason = "resolve_timeout"; return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override { current_.reset(); }
private:
  std::shared_ptr<Session>& session_;
  std::shared_ptr<Session> current_;
  rclcpp::Client<Resolve>::SharedPtr client_;
  bool done_ = false, ok_ = false;
  Clock::time_point deadline_;
};

class NavigateOwnedTask : public BT::StatefulActionNode {
public:
  NavigateOwnedTask(const std::string& name, const BT::NodeConfig& cfg,
                   rclcpp::Node::SharedPtr node, std::shared_ptr<Session>& session)
    : StatefulActionNode(name, cfg), session_(session) {
    client_ = rclcpp_action::create_client<Task>(node, "/r680_nav/execute_navigation_task");
  }
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override {
    current_ = session_; handle_.reset(); done_ = false; ok_ = false; cancel_sent_ = false;
    if (current_->cancel) return BT::NodeStatus::FAILURE;
    if (!client_->action_server_is_ready()) {
      current_->reason = "navigation_gateway_unavailable"; return BT::NodeStatus::FAILURE;
    }
    auto captured = current_;
    rclcpp_action::Client<Task>::SendGoalOptions opts;
    opts.goal_response_callback = [this, captured](ClientHandle::SharedPtr h) {
      if (current_ != captured) { if (h) client_->async_cancel_goal(h); return; }
      handle_ = h;
      if (!h) { done_ = true; captured->reason = "gateway_rejected"; }
    };
    opts.feedback_callback = [this, captured](ClientHandle::SharedPtr,
                                             const std::shared_ptr<const Task::Feedback> f) {
      if (current_ != captured) return;
      captured->state = f->state; captured->reason = f->reason;
      captured->remaining = f->distance_remaining;
    };
    opts.result_callback = [this, captured](const ClientHandle::WrappedResult& r) {
      if (current_ != captured) return;
      done_ = true;
      ok_ = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->state == "succeeded";
      captured->reason = r.result ? r.result->reason : "missing_gateway_result";
    };
    client_->async_send_goal(current_->goal, opts);
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    if (current_->cancel && handle_ && !cancel_sent_ && !done_) {
      client_->async_cancel_goal(handle_); cancel_sent_ = true;
    }
    // Never declare cancellation complete before the gateway's terminal+stop result.
    if (done_) return ok_ && !current_->cancel ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
  }
  void onHalted() override {
    if (handle_ && !done_ && !cancel_sent_) client_->async_cancel_goal(handle_);
    current_.reset();
  }
private:
  std::shared_ptr<Session>& session_;
  std::shared_ptr<Session> current_;
  rclcpp_action::Client<Task>::SharedPtr client_;
  ClientHandle::SharedPtr handle_;
  bool done_ = false, ok_ = false, cancel_sent_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wla_behavior_executor");
  auto tree_path = node->declare_parameter<std::string>("tree_path", "");
  if (tree_path.empty()) throw std::runtime_error("trusted tree_path parameter required");
  std::shared_ptr<Session> session;
  std::shared_ptr<ServerHandle> handle;
  bool reserved = false;
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ResolveBoundTarget>("ResolveBoundTarget", node, std::ref(session));
  factory.registerNodeType<NavigateOwnedTask>("NavigateOwnedTask", node, std::ref(session));
  auto tree = factory.createTreeFromFile(tree_path);
  auto lease_pub = node->create_publisher<wla_behavior_interfaces::msg::TaskLease>(
    "/wla_behavior/executor_lease", 10);
  auto server = rclcpp_action::create_server<Task>(node, "/wla_behavior/execute_task",
    [&](const rclcpp_action::GoalUUID&, std::shared_ptr<const Task::Goal> g) {
      if (reserved || g->request_id.empty() || g->feature_id.empty() || g->owner.empty())
        return rclcpp_action::GoalResponse::REJECT;
      reserved = true; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&](std::shared_ptr<ServerHandle> h) {
      if (h != handle || !session) return rclcpp_action::CancelResponse::REJECT;
      session->cancel = true; return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](std::shared_ptr<ServerHandle> h) {
      handle = h; session = std::make_shared<Session>(); session->goal = *h->get_goal();
    });
  auto timer = node->create_wall_timer(std::chrono::milliseconds(50), [&]() {
    if (!handle || !session) return;
    wla_behavior_interfaces::msg::TaskLease lease;
    lease.request_id = session->goal.request_id; lease.owner = session->goal.owner;
    lease.generation = session->goal.generation;
    lease.allowed = !session->cancel && !session->fault;
    lease_pub->publish(lease);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    try { if (!session->fault) status = tree.tickOnce(); }
    catch (const std::exception& e) {
      // An exception cannot prove gateway terminal+stop. Revoke immediately and
      // retain ownership until operator reconciliation instead of inventing a result.
      session->reason = e.what(); session->fault = true; session->state = "fault";
      lease.allowed = false; lease_pub->publish(lease);
      try { tree.haltTree(); } catch (const std::exception&) {}
    }
    auto feedback = std::make_shared<Task::Feedback>();
    feedback->request_id = session->goal.request_id; feedback->generation = session->goal.generation;
    feedback->state = session->state; feedback->reason = session->reason;
    feedback->distance_remaining = session->remaining; handle->publish_feedback(feedback);
    if (status == BT::NodeStatus::RUNNING) return;
    auto result = std::make_shared<Task::Result>();
    result->request_id = session->goal.request_id; result->generation = session->goal.generation;
    result->reason = session->reason;
    if (session->cancel) { result->state = "canceled"; handle->canceled(result); }
    else if (status == BT::NodeStatus::SUCCESS) { result->state = "succeeded"; handle->succeed(result); }
    else { result->state = "failed"; handle->abort(result); }
    lease.allowed = false; lease_pub->publish(lease);
    tree.haltTree(); handle.reset(); session.reset(); reserved = false;
  });
  rclcpp::spin(node);
  tree.haltTree(); rclcpp::shutdown(); return 0;
}
