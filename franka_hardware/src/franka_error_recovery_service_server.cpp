#include <franka_hardware/franka_error_recovery_service_server.hpp>

namespace franka_hardware{

FrankaErrorRecoveryServiceServer::FrankaErrorRecoveryServiceServer(const rclcpp::NodeOptions & options,
                                                                    std::shared_ptr<Robot> robot, std::string prefix)
    : rclcpp::Node(prefix+"error_recovery_service_server", options), robot_(std::move(robot))
{
    error_recovery_service_ = create_service<franka_msgs::srv::ErrorRecovery>(
        "~/error_recovery",
        std::bind(
            &FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery, this, 
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    RCLCPP_INFO(get_logger(), "Error recovery service started");
}

void FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery(const franka_msgs::srv::ErrorRecovery::Request::SharedPtr& /*request*/,
                                                                const franka_msgs::srv::ErrorRecovery::Response::SharedPtr& response)
{
    if(!this->robot_->hasError()){
        RCLCPP_INFO(this->get_logger(), "No errors detected; error recovery is not necessary.");
        response->error = "No errors";
        response->success = false;
    }
    else{
        try{

            this->robot_->doAutomaticErrorRecovery();
            this->robot_->setError(false);
            this->robot_->stopRobot();
            // this->robot_->initializeContinuousReading(); triggering this causes an error. Better just make it manual.
            RCLCPP_INFO(this->get_logger(), "Successfully recovered from error.");
            if(!this->robot_->getInitParamsSet()){
                RCLCPP_INFO(this->get_logger(), "Setting default params");
                this->robot_->setDefaultParams();
            }
            auto current_cm = robot_->getControlMode();
            RCLCPP_INFO_STREAM(this->get_logger(), "Restarting the control loop. Current cm: " << current_cm);

            std::lock_guard<std::mutex> lock(robot_->read_mutex_);
            if(current_cm == ControlMode::None){
                robot_->initializeContinuousReading();
            }
            else if(current_cm == ControlMode::JointTorque){
                robot_->initializeTorqueControl();
            }
            else if(current_cm == ControlMode::JointPosition){
                robot_->initializeJointPositionControl();
            }
            else if(current_cm == ControlMode::JointVelocity){
                robot_->initializeJointVelocityControl();
            }
            else if(current_cm == ControlMode::CartesianPose){
                robot_->initializeCartesianPositionControl();
            }
            else if(current_cm == ControlMode::CartesianVelocity){
                robot_->initializeCartesianVelocityControl();
            }
            response->success = true;
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(this->get_logger(), "Error recovery failed: %s", e.what());
            response->error = e.what();
            response->success = false;
        }
        catch(franka::NetworkException& e){
            RCLCPP_ERROR(this->get_logger(), "Error recovery timed out. :%s", e.what());
            response->error = e.what();
            response->success = false;
        }
    }
};


} // namespace franka_hardware