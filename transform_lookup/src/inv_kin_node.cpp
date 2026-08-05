#include <transform_lookup/inv_kin_node.h>

InvKinNode::InvKinNode() : Node("inv_kin_node")
{
    // setup subscriber for robot description
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot_description", 
        rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
        std::bind(&InvKinNode::robotDescriptionCallback, this, std::placeholders::_1));

    // publisher for the joint_states object
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        
    // frame names
    frame_names_[0] = "target_frame_1";
    frame_names_[1] = "target_frame_2";
    
    // setup joint states message
    joint_state_msg_.name = {"joint1", "joint2", "joint3"};
    joint_state_msg_.position.resize(joint_state_msg_.name.size());

    // setup transform buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}
bool InvKinNode::solveNextPose()
{
    // return false if done
    if (pose_index_ >= frame_names_.size())
        return false;
    
    KDL::Frame frame;
    if (lookupFrame(frame_names_[pose_index_], frame))
        solveIK(frame);
    
    // return true if not finished
    return true;
}

void InvKinNode::robotDescriptionCallback(const std_msgs::msg::String& msg)
{
    // Construct KDL tree from URDF
    const std::string urdf = msg.data;
    kdl_parser::treeFromString(urdf, tree_);

    // Get kinematic chain of the robot
    tree_.getChain("world", "ee_link", chain_);

    q_ = KDL::JntArray(chain_.getNrOfJoints());

    // Create IK solver
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
}

void InvKinNode::solveIK(const KDL::Frame & frame)
{
    if (ik_solver_ != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Trying pose number %i", pose_index_);
        
        // initial guess of IK solution
        KDL::JntArray q_init(chain_.getNrOfJoints());
        q_init(0) = 0.0; q_init(1) = 0.0; q_init(2) = 0.0;

        // solve IK 
        int error = ik_solver_->CartToJnt(q_init, frame, q_);

        if (error == KDL::ChainIkSolverPos_LMA::E_NOERROR) // IK succeeded
        {
            RCLCPP_INFO(this->get_logger(), "IK succeeded");

            // set the header time stamp
            joint_state_msg_.header.stamp = this->get_clock()->now();
            
            // copy the joint angles 
            std::copy(q_.data.begin(), q_.data.end(), joint_state_msg_.position.begin());

            // publish
            joint_pub_->publish(joint_state_msg_);

        }
        else // IK failed
        {
            RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed!");
        }

        // increase the index (only if it worked)
        ++pose_index_;
    }
}

bool InvKinNode::lookupFrame(std::string & frame_name, KDL::Frame & output_frame)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("world", frame_name, tf2::TimePointZero);
        
        // set translation of output frame
        output_frame.p.x(transform.transform.translation.x); 
        output_frame.p.y(transform.transform.translation.y); 
        output_frame.p.z(transform.transform.translation.z);

        // set translation of output frame
        output_frame.M = KDL::Rotation::Quaternion(
            transform.transform.rotation.x, 
            transform.transform.rotation.y, 
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );

        RCLCPP_INFO(this->get_logger(), "Found frame: %s", frame_name.data());
        
        return true;
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(this->get_logger(), "lookup failed! Reason: %s", ex.what());

        return false;
    }
}

int main(int argc, char** argv)
{
    // initialize the node
    rclcpp::init(argc, argv);
    
    // create instance of class
    auto node = std::make_shared<InvKinNode>();

    // Set loop rate
    rclcpp::Rate rate = rclcpp::Rate(1); // Hz
    
    while (node->solveNextPose())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}