/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_joint_tracker/joint_tracker.h>

#include <ow_core/trajectory/trajectories.h>

namespace ow_joint_tracker
{

  JointTracker::JointTracker() : Base("JointTracker"),
                                 enabled_(false),
                                 opposite_signs_(false),
                                 joint_id_left_(0),
                                 joint_id_right_(0),
                                 groin_id_(0),
                                 ankle_id_(0),
                                 max_groin_offset_(0.0),
                                 max_ankle_offset_(0.0),
                                 max_offset_time_(0.0),
                                 alpha_(0.0),
                                 time_left_(0.0),
                                 time_right_(0.0),
                                 q_cmd_(ow::JointState::Zero()),
                                 left_ankle_offset_(0.0),
                                 left_groin_offset_(0.0),
                                 right_ankle_offset_(0.0),
                                 right_groin_offset_(0.0)
  {
  }

  bool JointTracker::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // load module parameter
    parameter_.add<bool>("enabled");
    parameter_.add<bool>("oposite_leg_sign");
    parameter_.add<ow::Scalar>("offset_time");
    parameter_.add<ow::Scalar>("max_groin_offset");
    parameter_.add<ow::Scalar>("max_ankle_offset");
    parameter_.add<size_t>("groin_index");
    parameter_.add<size_t>("ankle_index");
    if (!parameter_.load(nh, "joint_tracker"))
    {
      ROS_ERROR("%s::initialize: Config loading failed.", Base::name().c_str());
      return false;
    }

    // init variables
    parameter_.get("enabled", enabled_);
    parameter_.get("oposite_leg_sign", opposite_signs_);
    parameter_.get("groin_index", groin_id_);
    parameter_.get("ankle_index", ankle_id_);
    parameter_.get("offset_time", max_offset_time_);
    parameter_.get("max_groin_offset", max_groin_offset_);
    parameter_.get("max_ankle_offset", max_ankle_offset_);
    alpha_ = 2 * M_PI / max_offset_time_;
    time_left_ = 0;
    time_right_ = 0;

    return true;
  }

  void JointTracker::start(
      ow::Scalar spline_duration,
      size_t joint_id_left,
      size_t joint_id_right,
      const ow::JointPosition &q_home,
      const ros::Time &time)
  {
    // set inputs
    total_time_ = spline_duration;
    joint_id_left_ = joint_id_left;
    joint_id_right_ = joint_id_right;
    q_home_ = q_home;
  }

  void JointTracker::update(
      const ow::JointState &q,
      ow::Flags &flags,
      const ros::Time &time,
      const ros::Duration &dt)
  {
    // default
    q_cmd_ = q;
    
    // homing started
    if(flags.eventOut() == ow::Flags::HOMEING_START)
    {
      // create spline to homing pose
      ow::JointPosition zero = ow::JointPosition::Zero();
      spline.reset(new JointSpline(ow_core::Polynomial5Order(
        total_time_,
        q.pos(), zero, zero,
        q_home_, zero, zero)));
      time_spline_ = time;
    }

    // homing
    if(flags.state() == ow::Flags::HOMEING)
    {
      ow::Scalar elapsed = (time - time_spline_).toSec();
      if(elapsed > total_time_)
      {
        // signal homing done
        flags.eventIn() = ow::Flags::REACHED_HOME;
      }
      q_cmd_ = spline->evaluate(elapsed);
    }

    // robot is walking
    if(flags.state() == ow::Flags::WALKING)
    {
      // is enabled and ground contact
      if(enabled_ && flags.hasGroundContact())
      {
        // in single support add offset to the supporting leg
        if(flags.walkingPhase() == ow::Flags::SINGLE_SUPPORT)
        {
          if(flags.supportFoot() == ow::FootId::LEFT)
          {
            // left is support, integrate time
            if (time_left_ < max_offset_time_)
              time_left_ += dt.toSec();
          }
          else
          {
            // right is support, integrate time
            if (time_right_ < max_offset_time_)
              time_right_ += dt.toSec();
          }
        }

        // in double support integrate offset away
        if(flags.walkingPhase() == ow::Flags::DOUBLE_SUPPORT)
        {
          if (time_left_ > 0.0)
            time_left_ -= dt.toSec();

          if (time_right_ > 0.0)
            time_right_ -= dt.toSec();
        }

        // apply offsets
        left_ankle_offset_ = smoothOffset(max_ankle_offset_,
                                          alpha_, time_left_);
        left_groin_offset_ = smoothOffset(max_groin_offset_,
                                          alpha_, time_left_);

        if (opposite_signs_)
        {
          right_ankle_offset_ = smoothOffset(-max_ankle_offset_,
                                              alpha_, time_right_);
          right_groin_offset_ = smoothOffset(-max_groin_offset_,
                                              alpha_, time_right_);
        }
        else
        {
          right_ankle_offset_ = smoothOffset(max_ankle_offset_,
                                              alpha_, time_right_);
          right_groin_offset_ = smoothOffset(max_groin_offset_,
                                              alpha_, time_right_);
        }

        q_cmd_.pos()[joint_id_left_ + ankle_id_] += left_ankle_offset_;
        q_cmd_.pos()[joint_id_left_ + groin_id_] += left_groin_offset_;
        q_cmd_.pos()[joint_id_right_ + ankle_id_] += right_ankle_offset_;
        q_cmd_.pos()[joint_id_right_ + groin_id_] += right_groin_offset_;
      }
    }
  }

  const ow::JointState &JointTracker::jointState() const
  {
    return q_cmd_;
  }

  ow::Scalar JointTracker::leftAnkleOffset() const
  {
    return left_ankle_offset_;
  }

  ow::Scalar JointTracker::leftGroinOffset() const
  {
    return left_groin_offset_;
  }

  ow::Scalar JointTracker::rightAnkleOffset() const
  {
    return right_ankle_offset_;
  }

  ow::Scalar JointTracker::rightGroinOffset() const
  {
    return right_groin_offset_;
  }

  ow::Scalar JointTracker::smoothOffset(ow::Scalar max,
                                        ow::Scalar alpha,
                                        ow::Scalar time)
  {
    return 0.5 * max * (1 + std::tanh(alpha * time - M_PI));
  }

} // namespace ow_joint_tracker
