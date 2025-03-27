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

#ifndef OPEN_WALKER_JOINT_TRACKER_H
#define OPEN_WALKER_JOINT_TRACKER_H

#include <ow_core/interfaces/i_joint_tracker.h>
#include <ow_core/trajectory/state_trajectory.h>
#include <ow_core/trajectory/trajectories.h>

/*!
 * \brief Open Walker joint tracker module namespace. These classes implement
 * controllers in joint space to coup with different hardware interfaces.
 */
namespace ow_joint_tracker
{

/*!
 * \brief The JointTracker class.
 *
 * This Class applies offsets to the commanded joint values from the inverse
 * kinematics to track them with the real joint states. These offsets compensate
 * for tracking errors in the low level hardware controllers of the motors.
 *
 * Additionally it computed the jointspace spline for inital homing motion.
 */
class JointTracker : 
  public ow::IJointTracker
{
public:
  typedef ow::IJointTracker Base;
  typedef ow_core::StateTrajectory<ow::JointState> JointSpline;

protected:
  ow::Parameter parameter_;   //!< configuration
  bool enabled_;              //!< Foot compliance enable flag.
  bool opposite_signs_; //!< Apply the joint offset with opposite signs

  // homing
  ow::Scalar total_time_;               //!< Total time of homming motion. 
  ros::Time time_spline_;

  // joint indices in complete robot state
  size_t joint_id_left_;      //!< First joint id of left leg in full state.
  size_t joint_id_right_;     //!< First joint id of right leg in full state.
  size_t groin_id_;           //!< Groin joint id in leg state vector.
  size_t ankle_id_;           //!< Ankle joint id in leg state vector.
  ow::JointPosition q_home_;  //!< Homing pose

  // offsets
  ow::Scalar max_groin_offset_;   //!< Maximum groin offset value.
  ow::Scalar max_ankle_offset_;   //!< Maximum ankle offset value.
  ow::Scalar max_offset_time_;    //!< Rise time of joint offsets.

  ow::Scalar alpha_;        //!< Internal angle value of offset function.
  ow::Scalar time_left_;    //!< Joint offset timer for left leg.
  ow::Scalar time_right_;   //!< Joint offset timer for right leg.
  ow::JointState q_cmd_;    //!< Output joint state.

  ow::Scalar left_ankle_offset_;    //!< Output left ankle offset.
  ow::Scalar left_groin_offset_;    //!< Output left groin offset.
  ow::Scalar right_ankle_offset_;   //!< Output right ankle offset.
  ow::Scalar right_groin_offset_;   //!< Output right groin offset.

  // homing spline trajectory
  std::unique_ptr<JointSpline> spline;

public:

  /*!
  * \brief Constructor with home pose.
  * \param freq
  */
  JointTracker();

  /** 
  * \brief start the controller, called befor update
  *
  * \param starting time
  */
  void start(
    ow::Scalar spline_duration,
    size_t joint_id_left,
    size_t joint_id_right,
    const ow::JointPosition& q_home,
    const ros::Time& time);

  /** 
  * \brief performs update step of the controller, called periodically
  *
  * \param current jointstate
  * \param current time
  * \param current deltatime since last call
  */
  void update(
    const ow::JointState& q,
    ow::Flags& flags,
    const ros::Time& time, 
    const ros::Duration& dt);

  /*!
   * \brief Output port function.
   *
   * \return
   *    JointState of the robot computed by tracker.
   */
  virtual const ow::JointState& jointState() const;

  /*!
   * \brief Output port function.
   *
   * \return
   *    left ankle offset.
   */
  ow::Scalar leftAnkleOffset() const;

  /*!
   * \brief Output port function.
   *
   * \return
   *    left groin offset.
   */
  ow::Scalar leftGroinOffset() const;

  /*!
   * \brief Output port function.
   *
   * \return
   *    right ankle offset.
   */
  ow::Scalar rightAnkleOffset() const;

  /*!
   * \brief Output port function.
   *
   * \return
   *    right groin offset.
   */
  ow::Scalar rightGroinOffset() const;

protected:
  /** 
  * \brief init interal\parameter
  *
  * \param ros nh for namespace
  */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

private:

  /*!
   * \brief Smooth offset function.
   *
   * \param max
   *    Maximum offset value.
   *
   * \param alpha
   *    Time constant of rising offset.
   *
   * \param time
   *    cCurrent time.
   *
   * \return
   *    Computed offset.
   */
  ow::Scalar smoothOffset(ow::Scalar max, ow::Scalar alpha, ow::Scalar time);
};

}

#endif // OPEN_WALKER_WALKING_CONTROLLER_H
