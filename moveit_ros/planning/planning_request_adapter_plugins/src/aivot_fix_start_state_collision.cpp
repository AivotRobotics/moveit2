#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection_acl/collision_env_acl.h>

namespace default_planner_request_adapters
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.fix_start_state_collision");

/** \brief Helper param for getting a parameter using a namespace **/
template <typename T>
T getParam(const rclcpp::Node::SharedPtr& node, const rclcpp::Logger& logger, const std::string& parameter_namespace,
           const std::string& parameter_name, T default_value = {})
{
    std::string full_name = parameter_namespace.empty() ? parameter_name : parameter_namespace + "." + parameter_name;
    T value;
    if (!node->get_parameter(full_name, value))
    {
        RCLCPP_INFO(logger, "Param '%s' was not set. Using default value: %s", full_name.c_str(),
                    std::to_string(default_value).c_str());
        return default_value;
    }
    else
    {
        RCLCPP_INFO(logger, "Param '%s' was set to %s", full_name.c_str(), std::to_string(default_value).c_str());
        return value;
    }
}

class AivotFixStartStateCollision : public planning_request_adapter::PlanningRequestAdapter
{
public:
    static const std::string DT_PARAM_NAME;

    AivotFixStartStateCollision()
            : planning_request_adapter::PlanningRequestAdapter()
    {}

    void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override {
        RCLCPP_INFO(LOGGER, "Initializing '%s'", getDescription().c_str());

        max_dt_offset_ = getParam(node, LOGGER, parameter_namespace, DT_PARAM_NAME, 0.5);
        RCLCPP_INFO_STREAM(LOGGER, "Param '" << DT_PARAM_NAME << "' was set to " << max_dt_offset_);
    }

    std::string getDescription() const override
    {
        return "(Aivot) Fix Start State In Collision";
    }

    bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                      const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res) const override
    {
        RCLCPP_INFO(LOGGER, "Running '%s'", getDescription().c_str());

        // get the specified start state
        moveit::core::RobotState start_state = planning_scene->getCurrentState();
        moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

        bool invokeStuckPlanner = false;

        // check if start state is in collision
        collision_detection::CollisionRequest creq;
        creq.group_name = req.group_name;
        creq.verbose = true;
        creq.contacts = true;
        collision_detection::CollisionResult cres;
        planning_scene->checkCollision(creq, cres, start_state);
        if (cres.collision) {
            // Rerun in verbose mode
            collision_detection::CollisionRequest vcreq = creq;
            collision_detection::CollisionResult vcres;
            vcreq.verbose = true;
            planning_scene->checkCollision(vcreq, vcres, start_state);

            if (creq.group_name.empty()) {
                RCLCPP_INFO(LOGGER, "Start state appears to be in collision");
            } else {
                RCLCPP_INFO_STREAM(LOGGER, "Start state appears to be in collision with respect to group "
                        << creq.group_name);
            }

            invokeStuckPlanner = true;
        }

        if (!invokeStuckPlanner) {
            // check if start state satisfies constraints
            if (!planning_scene->isStateValid(start_state, req.path_constraints, req.group_name)) {
                RCLCPP_INFO(LOGGER, "Path constraints not satisfied for start state...");
                // Rerun in verbose mode
                planning_scene->isStateValid(start_state, req.path_constraints, req.group_name, true);

                invokeStuckPlanner = true;
            }
        }

        if (invokeStuckPlanner) {
            const collision_detection::CollisionEnvACL* pCollisionEnvAcl =
                dynamic_cast<const collision_detection::CollisionEnvACL*>(planning_scene->getCollisionEnv().get());
            assert(pCollisionEnvAcl != nullptr && "Expected collision env of type CollisionEnvACL");

            std::vector<moveit::core::RobotState> prefixStates = pCollisionEnvAcl->getUnstuckPath(start_state);
            // if prefixStates is non-empty, first node is the start state and last node is the un-collided state
            if (prefixStates.size() > 1) {
                // set updated start state to last prefix entry
                start_state = prefixStates.back();
                prefixStates.pop_back();

                planning_interface::MotionPlanRequest req2 = req;
                moveit::core::robotStateToRobotStateMsg(start_state, req2.start_state);
                bool solved = planner(planning_scene, req2, res);
                RCLCPP_INFO_STREAM(LOGGER, "Planning with updated start state returned " <<
                    std::boolalpha << solved);

                if (solved && !res.trajectory->empty())
                {
                    size_t initialWayPointCount = res.trajectory->getWayPointCount();
                    // heuristically decide a duration offset for the trajectory (induced by the additional point added as a
                    // prefix to the computed trajectory)
                    res.trajectory->setWayPointDurationFromPrevious(
                        0, std::min(max_dt_offset_, res.trajectory->getAverageSegmentDuration()));
                    // TODO: there's an opportunity to optimize these vector operations
                    for (auto prefix_state : prefixStates) {
                        res.trajectory->addPrefixWayPoint(prefix_state, 0.0);
                    }
                    RCLCPP_INFO_STREAM(LOGGER, "Planning with updated start state" <<
                        ", initialWaypoints: " << initialWayPointCount <<
                        ", finalWayPoints: " << res.trajectory->getWayPointCount());
                }
                return solved;
            } else {
                RCLCPP_WARN_STREAM(LOGGER, "Callback returned " << prefixStates.size() <<
                    " prefix states. Passing the original planning request to the planner.");
                return planner(planning_scene, req, res);
            }
        }
        else
        {
            if (creq.group_name.empty())
                RCLCPP_INFO(LOGGER, "Start state is valid");
            else
                RCLCPP_INFO_STREAM(LOGGER, "Start state is valid with respect to group " << creq.group_name);
            return planner(planning_scene, req, res);
        }
    }

private:
    double max_dt_offset_;
};

const std::string AivotFixStartStateCollision::DT_PARAM_NAME = "start_state_max_dt";
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AivotFixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter);
