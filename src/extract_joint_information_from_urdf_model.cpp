bool TreeKinematics::readJoints(urdf::Model &robot_model,
                                KDL::Tree &kdl_tree,
                                std::string &tree_root_name,
                                unsigned int &nr_of_jnts,
                                KDL::JntArray &joint_min, 
                                KDL::JntArray &joint_max,
                                KDL::JntArray &joint_vel_max)
{
  KDL::SegmentMap segmentMap;
  segmentMap = kdl_tree.getSegments();
  tree_root_name = kdl_tree.getRootSegment()->second.segment.getName();
  nr_of_jnts = kdl_tree.getNrOfJoints();
  ROS_INFO( "the tree's number of joints: [%d]", nr_of_jnts );
  joint_min.resize(nr_of_jnts);
  joint_max.resize(nr_of_jnts);
  joint_vel_max.resize(nr_of_jnts);
  info_.joint_names.resize(nr_of_jnts);
  info_.limits.resize(nr_of_jnts);

  // walks through all tree segments, extracts their joints except joints of KDL::None and extracts   
  // the information about min/max position and max velocity of the joints not of type urdf::Joint::UNKNOWN or
  // urdf::Joint::FIXED
  ROS_DEBUG("Extracting all joints from the tree, which are not of type KDL::Joint::None.");
  boost::shared_ptr<const urdf::Joint> joint;
  for (KDL::SegmentMap::const_iterator seg_it = segmentMap.begin(); seg_it != segmentMap.end(); ++seg_it)
  {
    if (seg_it->second.segment.getJoint().getType() != KDL::Joint::None)
    {
      joint = robot_model.getJoint(seg_it->second.segment.getJoint().getName().c_str());
      // check, if joint can be found in the URDF model of the robot
      if (!joint)
      {
        ROS_FATAL("Joint '%s' has not been found in the URDF robot model! Aborting ...", joint->name.c_str());
        return false;
      }
      // extract joint information
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
      {
        ROS_INFO( "getting information about joint: [%s]", joint->name.c_str() );
        double lower = 0.0, upper = 0.0, vel_limit = 0.0;
        unsigned int has_pos_limits = 0, has_vel_limits = 0;
        
        if ( joint->type != urdf::Joint::CONTINUOUS )
        {
          ROS_DEBUG("joint is not continuous.");
          lower = joint->limits->lower;
          upper = joint->limits->upper;
          has_pos_limits = 1;
          if (joint->limits->velocity)
          {
            has_vel_limits = 1;
            vel_limit = joint->limits->velocity;
            ROS_DEBUG("joint has following velocity limit: %f", vel_limit);
          }
          else
          {          
            has_vel_limits = 0;
            vel_limit = 0.0;
            ROS_DEBUG("joint has no velocity limit.");
          }
        }
        else
        {
          ROS_DEBUG("joint is continuous.");
          lower = -M_PI;
          upper = M_PI;
          has_pos_limits = 0;
          if(joint->limits && joint->limits->velocity)
          {             
            has_vel_limits = 1;
            vel_limit = joint->limits->velocity;
            ROS_DEBUG("joint has following velocity limit: %f", vel_limit);
          }
          else
          {          
            has_vel_limits = 0;
            vel_limit = 0.0;
            ROS_DEBUG("joint has no velocity limit.");
          }
        }
        
        joint_min(seg_it->second.q_nr) = lower;
        joint_max(seg_it->second.q_nr) = upper;
        joint_vel_max(seg_it->second.q_nr) = vel_limit;
        ROS_INFO("pos_min = %f, pos_max = %f, vel_max = %f", lower, upper, vel_limit);
        
        info_.joint_names[seg_it->second.q_nr] = joint->name;
        info_.limits[seg_it->second.q_nr].joint_name = joint->name;
        info_.limits[seg_it->second.q_nr].has_position_limits = has_pos_limits;
        info_.limits[seg_it->second.q_nr].min_position = lower;
        info_.limits[seg_it->second.q_nr].max_position = upper;
        info_.limits[seg_it->second.q_nr].has_velocity_limits = has_vel_limits;
        info_.limits[seg_it->second.q_nr].max_velocity = vel_limit;        
      }
    }
  }  
  return true;
}
